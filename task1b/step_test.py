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

import paho.mqtt.client as mqtt

from phaser import Sampler, StallTimeout
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
    """Per-speed rollup over repeats and directions (sign-normalized).
    Aborted trials (sum None) are skipped, never crash the summary."""
    by_w = {}
    for w in {t["w"] for t in trials}:
        vals = [t["sum"] for t in trials
                if t["w"] == w and t["sum"] is not None]
        by_w[w] = vals
    low_ks = [s["k"] for w in (1.0, 1.5, 2.0) for s in by_w.get(w, [])
              if s["k"]]
    low_mean = statistics.mean(low_ks) if low_ks else None
    out = {}
    for w in sorted(by_w):
        ok = by_w[w]
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
    sampler = Sampler()

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = sampler.on_message
    client.connect(MQTT_HOST, MQTT_PORT)
    client.subscribe(TOPIC_SENSORS)
    client.loop_start()

    def pub(L, R):
        client.publish(TOPIC_WHEEL_VEL,
                       json.dumps({"left": float(L), "right": float(R)}))

    def sense(data, dt):
        vals = {}
        for k in tofs:
            vals[k], _ = tofs[k].update(data.get(k), dt)
        gyro = (data.get("gyro") or [0, 0, 0])[2]
        return vals, gyro

    def log_tick(data, vals, gyro, dt, L, R, state):
        raw = tuple(data.get(k, 0.0) for k in ("fl", "fr", "sl", "sr"))
        logger.tick(raw=raw,
                    filt=tuple(vals[k] for k in ("fl", "fr", "sl", "sr")),
                    gyro_z=gyro, dt_rep=dt, e_lat=float("nan"),
                    e_front=float("nan"), steer=0.0, L=L, R=R,
                    state=state, extra={"trial": trial_tag[0]})

    def run_phase(L, R, state, dur, collect, fresh_only=True):
        """Fresh samples only (STILL/SETTLE pace on wall when parked).
        Appends (t, gyro, yaw_int, vals) to collect; returns outcome."""
        t = 0.0
        pub(L, R)
        while t < dur:
            try:
                data, dt, _ = sampler.next(fresh_only=fresh_only)
            except StallTimeout:
                return "sim_stall"
            vals, gyro = sense(data, dt)
            if any(v is not None and v < ABORT_DIST
                   for v in vals.values()):
                return "abort"
            collect["yaw"] += gyro * dt
            collect["rows"].append((t, state, gyro, collect["yaw"]))
            if state == "STILL":
                collect["bias"].append(gyro)
            log_tick(data, vals, gyro, dt, L, R, state)
            t += dt
            pub(L, R)
        return "done"

    trials = []
    trial_tag = [""]
    try:
        sampler.next(fresh_only=True)  # drain: first fresh sample
        for w in speeds:
            for direction in ("left", "right"):
                sgn = 1.0 if direction == "left" else -1.0
                for rep in range(args.repeats):
                    trial_tag[0] = f"w{w}_{direction}_r{rep}"
                    logger.event("IDLE", "TRIAL", trial_tag[0])
                    trial = {"w": w, "dir": direction, "rep": rep,
                             "aborted": False, "sum": None}
                    collect = {"yaw": 0.0, "rows": [], "bias": []}
                    outcome = None
                    for name, dur, L, R in [
                            ("STILL", STILL_S, 0.0, 0.0),
                            ("STEP", STEP_S, -sgn * w, sgn * w),
                            ("COAST", COAST_S, 0.0, 0.0)]:
                        outcome = run_phase(
                            L, R, name, dur, collect,
                            fresh_only=(name == "STEP" or name == "COAST"))
                        if outcome != "done":
                            break
                    pub(0.0, 0.0)
                    samples = collect["rows"]
                    if outcome != "done":
                        trial["aborted"] = True
                        trial["abort_why"] = outcome
                        logger.event("TRIAL", "IDLE",
                                     trial_tag[0] + f":{outcome}")
                    else:
                        bias = (statistics.mean(collect["bias"])
                                if collect["bias"] else 0.0)
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
                    # settle between trials (static: wall-paced is fine)
                    run_phase(0.0, 0.0, "SETTLE", SETTLE_S,
                              {"yaw": 0.0, "rows": [], "bias": []},
                              fresh_only=False)
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
