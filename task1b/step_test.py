"""step_test.py -- yaw step-response identification (Stage 1a).

Runs INSTEAD of the controller on the Linux box, in an open area. It
publishes wheel commands itself and logs every tick through RunLogger,
so analyze_run.py reads the output like any other run (trial markers
go to events.csv).

Per trial: 0.5 s still (measures gyro bias), 2.0 s constant spin
command, 0.5 s zero (coast). Aborts the trial if any valid ToF < 0.08.

Matrix: --speeds (default 1.0,1.5,2,3,4,5,6,8) x {left,right} x
--repeats (default 3). Prints a per-speed summary and writes
summary.json into the run dir:
  steady yaw rate, gain k = gyro/(2w), rise time, time to 90 deg,
  coast-down time + coast angle, flatten flag (k >15% below the
  low-speed mean). Recommendations: YAW_GAIN_K, SPIN_SPEED (highest w
  with no flatten and coast < 3 deg), TURN_MIN_W.

Usage (three terminals, sim already up):
    mosquitto
    ./task_1b_launch          # robot placed in an OPEN area first
    python3 step_test.py --label s1_step
"""
import argparse
import json
import math
import os
import statistics
import sys
import time

import paho.mqtt.client as mqtt

from runlog import RunLogger
from sensing import Tof

MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"

STILL_S = 0.5
STEP_S = 2.0
COAST_S = 0.5
SETTLE_S = 1.0
ABORT_DIST = 0.08
MAX_RANGE = 2.0


def summarize(trials):
    """Per-speed rollup over repeats and directions (sign-normalized)."""
    by_w = {}
    for t in trials:
        by_w.setdefault(t["w"], []).append(t)
    low_ks = [s["k"] for w in (1.0, 1.5, 2.0)
              for t in by_w.get(w, []) for s in [t["sum"]] if s["k"]]
    low_mean = statistics.mean(low_ks) if low_ks else None
    out = {}
    for w, ts in sorted(by_w.items()):
        ok = [t["sum"] for t in ts if t["sum"] and not t["aborted"]]
        if not ok:
            out[w] = {"n": 0, "note": "all aborted"}
            continue
        k = statistics.mean(s["k"] for s in ok)
        out[w] = {
            "n": len(ok),
            "steady_yaw": statistics.mean(s["steady"] for s in ok),
            "k": k,
            "rise_s": statistics.mean(s["rise"] for s in ok),
            "time_to_90_s": statistics.mean(s["t90"] for s in ok),
            "coast_s": statistics.mean(s["coast_t"] for s in ok),
            "coast_deg": statistics.mean(s["coast_deg"] for s in ok),
            "flatten": bool(low_mean and k < 0.85 * low_mean),
        }
    return out, low_mean


def recommend(summary):
    good = [(w, s) for w, s in summary.items()
            if isinstance(s.get("k"), float) and not s["flatten"]
            and s["coast_deg"] < 3.0]
    ks = [s["k"] for _, s in good] or \
        [s["k"] for s in summary.values() if isinstance(s.get("k"), float)]
    k = statistics.mean(ks) if ks else 0.13
    spin = max([w for w, _ in good]) if good else 3.0
    lows = sorted(w for w, s in summary.items()
                  if isinstance(s.get("k"), float) and not s["flatten"]
                  and s["steady_yaw"] > 0.2)
    return {"YAW_GAIN_K": round(k, 4), "SPIN_SPEED": spin,
            "TURN_MIN_W": lows[0] if lows else 1.5}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="s1_step")
    ap.add_argument("--log-dir", default=None)
    ap.add_argument("--speeds", default="1.0,1.5,2,3,4,5,6,8")
    ap.add_argument("--repeats", type=int, default=3)
    args = ap.parse_args()
    speeds = [float(v) for v in args.speeds.split(",") if v.strip()]

    base = (args.log_dir or
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs"))
    logger = RunLogger(base, label=args.label, constants={
        "speeds": speeds, "repeats": args.repeats,
        "still_s": STILL_S, "step_s": STEP_S, "coast_s": COAST_S,
        "abort_dist": ABORT_DIST,
    })

    tofs = {k: Tof(MAX_RANGE, 0.1, 0.05) for k in ("fl", "fr", "sl", "sr")}
    latest = {}
    got = False

    def on_message(client, userdata, msg):
        nonlocal got
        latest.clear()
        latest.update(json.loads(msg.payload.decode()))
        got = True

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT)
    client.subscribe(TOPIC_SENSORS)
    client.loop_start()

    def wait_for_first():
        while not got:
            time.sleep(0.005)

    def pub(L, R):
        client.publish(TOPIC_WHEEL_VEL,
                       json.dumps({"left": float(L), "right": float(R)}))

    def sense(dt):
        d = latest
        vals = {}
        for k in tofs:
            vals[k], _ = tofs[k].update(d.get(k), dt)
        gyro = (d.get("gyro") or [0, 0, 0])[2]
        return vals, gyro

    def log_tick(vals, gyro, dt, L, R, state):
        raw = tuple(latest.get(k, 0.0) for k in ("fl", "fr", "sl", "sr"))
        logger.tick(raw=raw,
                    filt=tuple(vals[k] for k in ("fl", "fr", "sl", "sr")),
                    gyro_z=gyro, dt_rep=dt, e_lat=float("nan"),
                    e_front=float("nan"), steer=0.0, L=L, R=R,
                    state=state, extra={"trial": trial_tag[0]})

    trials = []
    trial_tag = [""]
    try:
        wait_for_first()
        dt = float(latest.get("dt", 0.02)) or 0.02
        for w in speeds:
            for direction in ("left", "right"):
                sgn = 1.0 if direction == "left" else -1.0
                for rep in range(args.repeats):
                    trial_tag[0] = f"w{w}_{direction}_r{rep}"
                    logger.event("IDLE", "TRIAL", trial_tag[0])
                    trial = {"w": w, "dir": direction, "rep": rep,
                             "aborted": False, "sum": None}
                    # phases: (name, duration, L, R)
                    phases = [("STILL", STILL_S, 0.0, 0.0),
                              ("STEP", STEP_S, -sgn * w, sgn * w),
                              ("COAST", COAST_S, 0.0, 0.0)]
                    samples = []  # (t, gyro, yaw_int)
                    yaw_int, t = 0.0, 0.0
                    bias_s = []
                    aborted = False
                    for name, dur, L, R in phases:
                        tph = 0.0
                        while tph < dur:
                            pub(L, R)
                            vals, gyro = sense(dt)
                            if any(v is not None and v < ABORT_DIST
                                   for v in vals.values()):
                                aborted = True
                                break
                            yaw_int += gyro * dt
                            samples.append((t, name, gyro, yaw_int))
                            if name == "STILL":
                                bias_s.append(gyro)
                            log_tick(vals, gyro, dt, L, R, name)
                            tph += dt
                            t += dt
                            time.sleep(max(0.0, dt - 0.002))
                        if aborted:
                            break
                    pub(0.0, 0.0)
                    if aborted:
                        trial["aborted"] = True
                        logger.event("TRIAL", "IDLE", trial_tag[0] + ":abort<0.08")
                    else:
                        bias = statistics.mean(bias_s) if bias_s else 0.0
                        step = [(tt, g - bias, y) for tt, n, g, y in samples
                                if n == "STEP"]
                        coast = [(tt, g - bias, y) for tt, n, g, y in samples
                                 if n == "COAST"]
                        steady = statistics.mean(g for _, g, _ in step[-50:])
                        tgt = 0.9 * steady
                        rise = next((tt - step[0][0] for tt, g, _ in step
                                     if abs(g) >= abs(tgt)), STEP_S)
                        y0 = step[0][2]
                        t90 = next((tt - step[0][0] for tt, g, y in step
                                    if abs(y - y0) >= math.pi / 2), STEP_S)
                        cy0 = coast[0][2] if coast else 0.0
                        c_end = next((i for i, (tt, g, y) in enumerate(coast)
                                      if abs(g) < 0.05), len(coast) - 1)
                        trial["sum"] = {
                            "k": abs(steady) / (2 * w) if w else 0.0,
                            "steady": sgn * steady,
                            "rise": rise, "t90": t90,
                            "coast_t": coast[c_end][0] - coast[0][0] if coast else 0.0,
                            "coast_deg": abs(math.degrees(
                                (coast[c_end][2] - cy0))) if coast else 0.0,
                        }
                        logger.event("TRIAL", "IDLE", trial_tag[0] + ":done")
                    trials.append(trial)
                    # settle between trials
                    tph = 0.0
                    while tph < SETTLE_S:
                        pub(0.0, 0.0)
                        vals, gyro = sense(dt)
                        log_tick(vals, gyro, dt, 0.0, 0.0, "SETTLE")
                        tph += dt
                        time.sleep(max(0.0, dt - 0.002))
    finally:
        pub(0.0, 0.0)
        client.loop_stop()
        client.disconnect()

    summary, low_mean = summarize(trials)
    rec = recommend(summary)
    with open(os.path.join(logger.run_dir, "summary.json"), "w") as f:
        json.dump({"low_speed_k_mean": low_mean, "per_speed": summary,
                   "recommend": rec,
                   "aborted": sum(t["aborted"] for t in trials),
                   "n_trials": len(trials)}, f, indent=2)
    print(json.dumps({"recommend": rec, "per_speed": summary}, indent=2))
    logger.close()


if __name__ == "__main__":
    main()
