"""speed_test.py -- linear-speed identification (Stage 2a).

Runs INSTEAD of the controller on the Linux box. Spawn sits ~0.10 m
in front of the entrance wall with open space behind (see
SIM_NOTES.md), so per wheel speed w, each trial:
  1. reverses at -w until the front opens: numeric >= 0.299, OR
     unknown front sustained 1 s (open space),
  2. holds still 0.5 s,
  3. drives forward at w until min front <= 0.12 (COAST data only),
  4. commands zero for 2 s and records coast distance.

K_LIN is fit on the BACK opening slope (known command, translation
along the facing axis). The forward sweep is NOT used for speed: in
the tight slot the rays sweep across walls as the robot yaws, so
forward range-rate measures rotation, not translation (s2_speed_v2:
k = -0.07..0.17 garbage vs geometric 0.017).

Fits need window span >= 0.05 m and r^2 >= 0.8, else k_lin is null
with a reason -- never commit a contaminated fit. K_LIN itself
defaults to the binary-MJCF geometric value (0.017); this script
CONFIRMS it, it does not discover it.

Sampling: every processed row is a FRESH payload (phaser.Sampler);
phase clocks advance per-sample, so durations are honest even though
the bridge publishes slower than the control loop iterates.

Caveat: front rays splay 20 deg (SIM_NOTES.md), so range understates
travel by cos20 ~= 0.94 (~6%) when facing squarely. Coast check for
s2c: require FRONT_STOP_DIST >= coast + 0.05 at cruise speed.

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
OPEN_SUSTAIN_S = 1.0   # unknown front this long = open space, stop backing
GYRO_STRAIGHT_MAX = 0.15  # BACK samples yawing faster are rotation, not
# translation: rays sweeping across walls fake the slope. Gate the fit.
STILL_S = 0.5
FWD_END = 0.12
COAST_S = 2.0
SETTLE_S = 1.0
PHASE_TIMEOUT = 10.0
BACK_SPAN_MIN = 0.05   # BACK fit needs this much opening range
BACK_R2_MIN = 0.8      # ... and this fit quality, else null, not garbage
ABORT_DIST = 0.08
MAX_RANGE = 2.0


def slope_r2(txs):
    """Least-squares slope + r^2. None if <5 samples or no time spread."""
    n = len(txs)
    if n < 5:
        return None, None
    mt = statistics.mean(t for t, _ in txs)
    mx = statistics.mean(x for _, x in txs)
    den = sum((t - mt) ** 2 for t, _ in txs)
    if den <= 0:
        return None, None
    m = sum((t - mt) * (x - mx) for t, x in txs) / den
    ss = sum((x - mx) ** 2 for _, x in txs)
    r2 = 1.0 - sum((x - (mx + m * (t - mt))) ** 2 for t, x in txs) / ss if ss > 0 else 0.0
    return m, max(0.0, min(r2, 1.0))


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
        "back_span_min": BACK_SPAN_MIN, "back_r2_min": BACK_R2_MIN,
        "abort_dist": ABORT_DIST, "fit": "BACK opening slope",
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
        samples[(t, vals, stats, gyro)])."""
        t = 0.0
        samples = []
        pub(L, R)
        while t < dur:
            try:
                data, dt, _ = sampler.next(fresh_only=fresh_only)
            except StallTimeout:
                return "sim_stall", samples
            vals, stats = sense(data, dt)
            gyro = (data.get("gyro") or [0, 0, 0])[2]
            if any(v is not None and v < ABORT_DIST
                   for v in (vals["sl"], vals["sr"])):
                return "abort", samples
            raw = (data.get("fl"), data.get("fr"))
            samples.append((t, vals, stats, gyro, raw))
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

            r, back = run_phase(-w, -w, "BACK", PHASE_TIMEOUT,
                                 stop=back_stop)
            f0 = [front_min(v) for _, v, _, _, _ in back if front_min(v) is not None]
            moved = (r == "stop" and f0
                     and (max(f0) - min(f0) > BACK_SPAN_MIN
                          or max(f0) >= BACK_TARGET))
            # K_LIN from the BACK opening slope on RAW samples (not the
            # EMA-filtered values: the filter slews smoothly through
            # ray flicker and fabricates high-r2 ramps out of a pinned
            # robot). Straight samples only (rotation sweeps rays).
            def raw_min(rr):
                xs = [x for x in rr if x is not None and x == x
                      and 0.0 < x <= MAX_RANGE]
                return min(xs) if xs else None
            bwin = [(t, raw_min(rr)) for t, _, _, g, rr in back
                    if raw_min(rr) is not None and abs(g) < GYRO_STRAIGHT_MAX]
            bspan = (max(x for _, x in bwin) - min(x for _, x in bwin)) if bwin else 0.0
            m, r2 = slope_r2(bwin) if bspan >= BACK_SPAN_MIN else (None, None)
            if m is not None and (r2 is None or r2 < BACK_R2_MIN):
                m, r2 = None, r2  # contaminated fit: null, not garbage
            k_lin = (m / w) if m is not None else None
            # 2. still
            run_phase(0.0, 0.0, "STILL", STILL_S, fresh_only=False)
            if not moved:
                trials.append({"w": w, "phase": r, "skip": "no_room",
                               "k_lin": None,
                               "note": "front never opened: rear-blocked spawn?"})
                logger.event("SPEED", "IDLE", f"{tag[0]}:no_room")
                run_phase(0.0, 0.0, "SETTLE", SETTLE_S, fresh_only=False)
                continue
            # 3. forward (COAST data only -- FWD range-rate in the slot
            # measures ray sweep, not translation; never fit k here)
            r3, fwd = run_phase(w, w, "FWD", PHASE_TIMEOUT,
                                stop=lambda v, s, d: (front_min(v) is not None
                                                      and front_min(v) <= FWD_END))
            # 4. coast
            _, coast = run_phase(0.0, 0.0, "COAST", COAST_S)
            pub(0.0, 0.0)
            c = [front_min(v) for _, v, _, _, _ in coast if front_min(v) is not None]
            trials.append({
                "w": w, "phase": r, "fwd_end": r3,
                "n_back": len(bwin), "back_span": round(bspan, 4),
                "back_r2": round(r2, 4) if r2 is not None else None,
                "back_raw": sum(1 for _, v, _, _, _ in back
                                if front_min(v) is not None),
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
               "note": ("front range understates travel by cos20 ~= 0.94; "
                        "K_LIN=0.017 geometric default stands unless a "
                        "clean r2>=0.8 BACK fit says otherwise")}
    with open(os.path.join(logger.run_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary, indent=2))
    logger.close()


if __name__ == "__main__":
    main()
