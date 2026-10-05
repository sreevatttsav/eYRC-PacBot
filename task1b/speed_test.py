"""speed_test.py -- linear-speed identification (Stage 2a).

Runs INSTEAD of the controller on the Linux box, facing a wall with
room to back up. Per wheel speed w, it:
  1. reverses until both front sensors read >= 0.299 (or 10 s timeout),
  2. holds still 0.5 s,
  3. drives forward at constant w until min front <= 0.12 (or 10 s),
  4. commands zero for 2 s and records coast distance.

Speed is the least-squares slope of front range vs time while the
front reads in [0.30, 0.15]. Output per w: K_LIN (m/s per wheel
rad/s) and coast distance. K_LIN replaces WHEEL_R_EST in RECOVER
distance and runlog dead-reckoning.

Caveat: if the front rays splay, range understates travel by
cos(splay) (~3% at 15 deg splay -- unverified). Coast check for s2c:
require FRONT_STOP_DIST >= coast + 0.05 at the tested cruise speed.

Aborts a trial if any valid side ToF < 0.08.

Usage:
    python3 speed_test.py --label s2_speed
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

BACK_TARGET = 0.299
STILL_S = 0.5
FWD_END = 0.12
COAST_S = 2.0
SETTLE_S = 1.0
PHASE_TIMEOUT = 10.0
WIN_LO, WIN_HI = 0.15, 0.30
ABORT_DIST = 0.08
MAX_RANGE = 2.0


def slope(txs):
    """Least-squares d(range)/dt; speed = -slope (range shrinks)."""
    n = len(txs)
    if n < 5:
        return None
    mt = statistics.mean(t for t, _ in txs)
    mx = statistics.mean(x for _, x in txs)
    den = sum((t - mt) ** 2 for t, _ in txs)
    if den <= 0:
        return None
    return sum((t - mt) * (x - mx) for t, x in txs) / den


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="s2_speed")
    ap.add_argument("--log-dir", default=None)
    ap.add_argument("--speeds", default="2,3,4,6,8")
    args = ap.parse_args()
    speeds = [float(v) for v in args.speeds.split(",") if v.strip()]

    base = (args.log_dir or
            os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs"))
    logger = RunLogger(base, label=args.label, constants={
        "speeds": speeds, "back_target": BACK_TARGET,
        "window": [WIN_LO, WIN_HI], "abort_dist": ABORT_DIST,
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

    def pub(L, R):
        client.publish(TOPIC_WHEEL_VEL,
                       json.dumps({"left": float(L), "right": float(R)}))

    tag = [""]

    def sense(dt):
        vals = {}
        for k in tofs:
            vals[k], _ = tofs[k].update(latest.get(k), dt)
        return vals

    def tick(vals, L, R, state, dt):
        raw = tuple(latest.get(k, 0.0) for k in ("fl", "fr", "sl", "sr"))
        logger.tick(raw=raw,
                    filt=tuple(vals[k] for k in ("fl", "fr", "sl", "sr")),
                    gyro_z=(latest.get("gyro") or [0, 0, 0])[2],
                    dt_rep=dt, e_lat=float("nan"), e_front=float("nan"),
                    steer=0.0, L=L, R=R, state=state,
                    extra={"trial": tag[0]})

    def run_phase(L, R, state, dur, dt, stop=None):
        """Publish for up to dur s; stop early if stop(vals) is True.
        Returns (timed_out, samples)."""
        t = 0.0
        samples = []
        while t < dur:
            pub(L, R)
            vals = sense(dt)
            if any(v is not None and v < ABORT_DIST
                   for v in (vals["sl"], vals["sr"])):
                return "abort", samples
            samples.append((t, vals))
            tick(vals, L, R, state, dt)
            if stop is not None and stop(vals):
                return "stop", samples
            t += dt
            time.sleep(max(0.0, dt - 0.002))
        return "timeout", samples

    def front_min(vals):
        xs = [v for v in (vals["fl"], vals["fr"]) if v is not None]
        return min(xs) if xs else None

    trials = []
    try:
        while not got:
            time.sleep(0.005)
        dt = float(latest.get("dt", 0.02)) or 0.02
        for w in speeds:
            tag[0] = f"w{w}"
            logger.event("IDLE", "SPEED", tag[0])
            # 1. back up for room
            r, _ = run_phase(-2.0, -2.0, "BACK", PHASE_TIMEOUT, dt,
                             stop=lambda v: (front_min(v) is not None
                                             and front_min(v) >= BACK_TARGET))
            # 2. still
            run_phase(0.0, 0.0, "STILL", STILL_S, dt)
            # 3. forward
            r3, fwd = run_phase(w, w, "FWD", PHASE_TIMEOUT, dt,
                                stop=lambda v: (front_min(v) is not None
                                                and front_min(v) <= FWD_END))
            # 4. coast
            _, coast = run_phase(0.0, 0.0, "COAST", COAST_S, dt)
            pub(0.0, 0.0)
            win = [(t, front_min(v)) for t, v in fwd
                   if front_min(v) is not None
                   and WIN_LO <= front_min(v) <= WIN_HI]
            m = slope(win)
            k_lin = (-m / w) if m is not None else None
            c0 = front_min(coast[0][1]) if coast else None
            c1 = front_min(coast[-1][1]) if coast else None
            trials.append({
                "w": w, "phase": r, "fwd_end": r3,
                "n_win": len(win),
                "k_lin": k_lin,
                "coast_m": (c0 - c1) if c0 is not None and c1 is not None else None,
            })
            logger.event("SPEED", "IDLE", f"{tag[0]}:done")
            run_phase(0.0, 0.0, "SETTLE", SETTLE_S, dt)
    finally:
        pub(0.0, 0.0)
        client.loop_stop()
        client.disconnect()

    ks = [t["k_lin"] for t in trials if t["k_lin"]]
    summary = {"per_speed": trials,
               "k_lin_mean": statistics.mean(ks) if ks else None,
               "coast_max": max((t["coast_m"] or 0) for t in trials),
               "note": "speed understated by cos(splay) if rays splay"}
    with open(os.path.join(logger.run_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))
    logger.close()


if __name__ == "__main__":
    main()
