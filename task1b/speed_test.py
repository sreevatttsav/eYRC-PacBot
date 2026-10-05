"""speed_test.py -- linear-speed identification (Stage 2a).

Runs INSTEAD of the controller on the Linux box, facing a wall with
room to back up. Per wheel speed w, it:
  1. reverses until the front opens: numeric >= 0.299, OR sat/none
     sustained 1 s (open space -- the spawn entrance opens to exactly
     0.300 = sat, which can never satisfy a numeric acceptance),
  2. holds still 0.5 s,
  3. drives forward at constant w until min front <= 0.12 (or 10 s),
  4. commands zero for 2 s and records coast distance.

Trials where the robot provably did not move (BACK rear-blocked:
front unchanged and no opening) are SKIPPED as "no_room", not
measured -- driving into the spawn wall yields n_win=0, k_lin=null.

Speed is the least-squares slope of front range vs time while the
front reads in [0.30, 0.15]; a window spanning < 0.10 m of range is
rejected. Output per w: K_LIN (m/s per wheel rad/s) and coast
distance. K_LIN replaces WHEEL_R_EST in RECOVER distance and runlog
dead-reckoning.

Sampling: every processed row is a FRESH payload (phaser.Sampler);
phase clocks advance per-sample, so durations are honest even though
the bridge publishes slower than the control loop iterates.

Caveat: if the front rays splay, range understates travel by
cos(splay) (~3% at 15 deg splay -- unverified). Coast check for s2c:
require FRONT_STOP_DIST >= coast + 0.05 at the tested cruise speed.

Usage:
    python3 speed_test.py --label s2_speed
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

BACK_TARGET = 0.299
OPEN_SUSTAIN_S = 1.0   # sat/none this long = open space, stop backing
STILL_S = 0.5
FWD_END = 0.12
COAST_S = 2.0
SETTLE_S = 1.0
PHASE_TIMEOUT = 10.0
WIN_LO, WIN_HI = 0.15, 0.30
WIN_SPAN_MIN = 0.10    # window must cover this much range to fit k
MOVE_MIN = 0.03        # BACK must change the front by this much
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
        "fresh_samples_only": True,
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

    tag = [""]

    def sense(data, dt):
        vals, stats = {}, {}
        for k in tofs:
            vals[k], stats[k] = tofs[k].update(data.get(k), dt)
        return vals, stats

    def tick(data, vals, L, R, state, dt):
        raw = tuple(data.get(k, 0.0) for k in ("fl", "fr", "sl", "sr"))
        logger.tick(raw=raw,
                    filt=tuple(vals[k] for k in ("fl", "fr", "sl", "sr")),
                    gyro_z=(data.get("gyro") or [0, 0, 0])[2],
                    dt_rep=dt, e_lat=float("nan"), e_front=float("nan"),
                    steer=0.0, L=L, R=R, state=state,
                    extra={"trial": tag[0]})

    def run_phase(L, R, state, dur, stop=None, fresh_only=True):
        """One fresh sample per loop; phase clock advances per-sample dt.
        stop(vals, stats, dt) may accumulate time. Returns (outcome,
        samples[(t, vals, stats)])."""
        t = 0.0
        samples = []
        pub(L, R)
        while t < dur:
            try:
                data, dt, _ = sampler.next(fresh_only=fresh_only)
            except StallTimeout:
                return "sim_stall", samples
            vals, stats = sense(data, dt)
            if any(v is not None and v < ABORT_DIST
                   for v in (vals["sl"], vals["sr"])):
                return "abort", samples
            samples.append((t, vals, stats))
            tick(data, vals, L, R, state, dt)
            if stop is not None and stop(vals, stats, dt):
                return "stop", samples
            t += dt
            pub(L, R)
        return "timeout", samples

    def front_min(vals):
        xs = [v for v in (vals["fl"], vals["fr"]) if v is not None]
        return min(xs) if xs else None

    trials = []
    try:
        # 0. drain: wait for the first fresh sample
        sampler.next(fresh_only=True)
        for w in speeds:
            tag[0] = f"w{w}"
            logger.event("IDLE", "SPEED", tag[0])
            # 1. back up for room. Test Tofs run numeric (no sat
            # cap): 0.300 reads as distance and satisfies the target.
            # Sustained unknown front also counts as open.
            back_start = None
            open_t = 0.0

            def back_stop(vals, stats, dt):
                nonlocal open_t
                fm = front_min(vals)
                if fm is not None and fm >= BACK_TARGET:
                    return True
                if fm is None:
                    open_t += dt
                else:
                    open_t = 0.0
                return open_t >= OPEN_SUSTAIN_S

            r, back = run_phase(-2.0, -2.0, "BACK", PHASE_TIMEOUT,
                                stop=back_stop)
            f0 = [front_min(v) for _, v, _ in back if front_min(v) is not None]
            moved = (r == "stop" and f0
                     and (max(f0) - min(f0) > MOVE_MIN or max(f0) >= BACK_TARGET))
            # 2. still
            run_phase(0.0, 0.0, "STILL", STILL_S, fresh_only=False)
            if not moved:
                trials.append({"w": w, "phase": r, "skip": "no_room",
                               "note": "front never opened: rear-blocked spawn?"})
                logger.event("SPEED", "IDLE", f"{tag[0]}:no_room")
                run_phase(0.0, 0.0, "SETTLE", SETTLE_S, fresh_only=False)
                continue
            # 3. forward
            r3, fwd = run_phase(w, w, "FWD", PHASE_TIMEOUT,
                                stop=lambda v, s, d: (front_min(v) is not None
                                                      and front_min(v) <= FWD_END))
            # 4. coast
            _, coast = run_phase(0.0, 0.0, "COAST", COAST_S)
            pub(0.0, 0.0)
            win = [(t, front_min(v)) for t, v, s in fwd
                   if front_min(v) is not None
                   and WIN_LO <= front_min(v) <= WIN_HI]
            span = (max(x for _, x in win) - min(x for _, x in win)) if win else 0.0
            m = slope(win) if span >= WIN_SPAN_MIN else None
            k_lin = (-m / w) if m is not None else None
            c = [front_min(v) for _, v, s in coast if front_min(v) is not None]
            trials.append({
                "w": w, "phase": r, "fwd_end": r3,
                "n_win": len(win), "win_span": round(span, 4),
                "k_lin": k_lin,
                "coast_m": (c[0] - c[-1]) if c else None,
            })
            logger.event("SPEED", "IDLE", f"{tag[0]}:done")
            run_phase(0.0, 0.0, "SETTLE", SETTLE_S, fresh_only=False)
    finally:
        pub(0.0, 0.0)
        client.loop_stop()
        client.disconnect()

    ks = [t["k_lin"] for t in trials if t.get("k_lin")]
    summary = {"per_speed": trials,
               "k_lin_mean": statistics.mean(ks) if ks else None,
               "coast_max": max((t.get("coast_m") or 0) for t in trials),
               "skipped": sum(1 for t in trials if t.get("skip")),
               "note": "speed understated by cos(splay) if rays splay"}
    with open(os.path.join(logger.run_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))
    logger.close()


if __name__ == "__main__":
    main()
