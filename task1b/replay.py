"""replay.py -- validate sensing offline from a logged run (Phase 0).

Feeds raw ToF from logs/<run>/ticks.csv through the CURRENT sensing
code (sensing.Tof + the validity gating in the controller) and reports
per-sensor valid/held/none rates plus old-vs-new e_front/e_lat stats.

Sensing only: the replay cannot react to what the robot would have
done, so closed-loop behavior is NOT validated here -- record a sim
run for that. Use --t0/--t1 to window on t_wall (e.g. a clean straight).

Usage:
    python3 replay.py logs/20250101T000000Z_baseline/ticks.csv
    python3 replay.py logs/<run>/ticks.csv --t0 3 --t1 12

Stuck regression (Stage 0 gate -- exit code 1 on failure):
    python3 replay.py <rerun>/ticks.csv --stuck-expect 5.0:7.0
    python3 replay.py <first>/ticks.csv --stuck-expect 16.5:38.5

Recomputes yaw_cmd = YAW_GAIN_K*(R-L) and runs the CURRENT
StuckDetector over the whole log, then asserts:
  (a) every 1 s bin of each --stuck-expect window contains a flag
      (known pins/jams must be caught), and
  (b) zero flags on steady TURN ticks (1 s spin-up transient after
      each TURN entry is excluded -- a fixed 1 s window cannot judge
      a step change until it fills with post-step data).
"""
import argparse
import csv
import statistics
import sys
import types

# stub paho (not installed on mac) to import controller constants
_paho = types.ModuleType("paho")
_paho.__path__ = []
_mqtt = types.ModuleType("paho.mqtt")
_mqtt.__path__ = []
_mc = types.ModuleType("paho.mqtt.client")
_mc.Client = lambda *a, **k: None
_mc.CallbackAPIVersion = types.SimpleNamespace(VERSION2=0)
_paho.mqtt = _mqtt
_mqtt.client = _mc
sys.modules["paho"] = _paho
sys.modules["paho.mqtt"] = _mqtt
sys.modules["paho.mqtt.client"] = _mc

import task_1b_boilerplate as B  # noqa: E402
from sensing import StuckDetector, Tof  # noqa: E402

# Stage-1b defines these in the controller; fall back until then.
YAW_GAIN_K = getattr(B, "YAW_GAIN_K", 0.13)
STUCK_CMD_MIN = getattr(B, "STUCK_CMD_MIN", 0.3)
STUCK_RATIO = getattr(B, "STUCK_RATIO", 0.25)
STUCK_WIN_S = getattr(B, "STUCK_WIN_S", 1.0)


def legacy_ema(prev, new, dt):
    a = dt / (B.FILTER_TAU + dt) if dt and dt > 0 else 1.0
    return new if prev is None else prev + a * (new - prev)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ticks_csv")
    ap.add_argument("--t0", type=float, default=None)
    ap.add_argument("--t1", type=float, default=None)
    ap.add_argument("--stuck-expect", action="append", default=[],
                    metavar="A:B",
                    help="t_wall window that must be flagged (repeatable)")
    ap.add_argument("--stuck-quiet", action="append", default=["TURN"],
                    metavar="STATE",
                    help="states where zero flags are allowed "
                         "(1 s post-entry excluded)")
    a = ap.parse_args()

    tofs = {k: Tof(B.MAX_RANGE, B.TOF_HOLD_S, B.FILTER_TAU)
            for k in ("fl", "fr", "sl", "sr")}
    n = 0
    status_counts = {k: {"valid": 0, "held": 0, "none": 0}
                     for k in tofs}
    old_front, new_front, old_lat, new_lat = [], [], [], []
    leg = {k: None for k in tofs}

    with open(a.ticks_csv) as f:
        for row in csv.DictReader(f):
            tw = float(row["t_wall"])
            if a.t0 is not None and tw < a.t0:
                continue
            if a.t1 is not None and tw > a.t1:
                continue
            dt = float(row["dt_rep"])
            raw = {k: float(row[k]) for k in tofs}
            vals, stats = {}, {}
            for k in tofs:
                vals[k], stats[k] = tofs[k].update(raw[k], dt)
                status_counts[k][stats[k]] += 1
            # legacy: clip + unconditional EMA (pre-fix behavior)
            for k in tofs:
                c = max(0.0, min(raw[k], B.MAX_RANGE))
                leg[k] = legacy_ema(leg[k], c, dt)
            old_front.append(leg["fl"] - leg["fr"])
            old_lat.append(leg["sl"] - leg["sr"])
            # new validity-gated rules (mirror controller logic)
            if vals["fl"] is not None and vals["fr"] is not None:
                new_front.append(vals["fl"] - vals["fr"])
            if vals["sl"] is not None and vals["sr"] is not None:
                new_lat.append(vals["sl"] - vals["sr"])
            n += 1

    print(f"rows: {n}" +
          (f" (t_wall {a.t0}..{a.t1})" if a.t0 or a.t1 else ""))
    for k in tofs:
        c = status_counts[k]
        tot = max(1, sum(c.values()))
        print(f"{k}: valid {c['valid']/tot:.1%} "
              f"held {c['held']/tot:.1%} none {c['none']/tot:.1%}")

    def show(name, xs):
        if not xs:
            print(f"{name}: n/a (unknown throughout window)")
            return
        print(f"{name}: n={len(xs)} mean={statistics.mean(xs):+.4f} "
              f"std={statistics.pstdev(xs):.4f}")

    print("-- e_front --")
    show("old (legacy filter)", old_front)
    show("new (validity-gated)", new_front)
    print("-- e_lat --")
    show("old (legacy filter)", old_lat)
    show("new (validity-gated)", new_lat)

    if a.stuck_expect:
        sys.exit(stuck_regress(a.ticks_csv, a.stuck_expect, a.stuck_quiet))


def stuck_regress(path, expects, quiet_states):
    """Recompute stuck flags with current constants; assert coverage of
    known spans and silence on steady quiet-state ticks."""
    rows = list(csv.DictReader(open(path)))
    sd = StuckDetector(STUCK_WIN_S, STUCK_CMD_MIN, STUCK_RATIO)
    flags = []  # (t_wall, flagged, state)
    for r in rows:
        yc = YAW_GAIN_K * (float(r["R"]) - float(r["L"]))
        f = sd.update(yc, float(r["gyro_z"]), float(r["dt_rep"]))
        flags.append((float(r["t_wall"]), bool(f), r.get("state", "?")))
    print(f"-- stuck regress (K={YAW_GAIN_K} min={STUCK_CMD_MIN} "
          f"ratio={STUCK_RATIO} win={STUCK_WIN_S}s) --")
    ok = True
    for spec in expects:
        lo, hi = (float(v) for v in spec.split(":"))
        # every 1 s bin of the window must contain a flag
        b = lo
        missing = []
        while b < hi:
            be = min(b + 1.0, hi)
            if not any(f and b <= t < be for t, f, _ in flags):
                missing.append((b, be))
            b = be
        status = "COVERED" if not missing else f"MISSED {missing}"
        if missing:
            ok = False
        print(f"  expect {lo:.1f}-{hi:.1f}s: {status}")
    # quiet states: TURN entry times for the spin-up exclusion
    entries = []
    prev = None
    for t, f, s in flags:
        if s == "TURN" and prev != "TURN":
            entries.append(t)
        prev = s
    bad = [(t, s) for t, f, s in flags
           if f and s in quiet_states
           and not any(e <= t < e + 1.0 for e in entries)]
    if bad:
        ok = False
        print(f"  steady-{quiet_states} flags: {len(bad)} "
              f"first at t={bad[0][0]:.2f}s FAIL")
    else:
        print(f"  steady-{quiet_states} flags: 0 PASS")
    print("STUCK-REGRESS " + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    main()
