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
from sensing import Tof  # noqa: E402


def legacy_ema(prev, new, dt):
    a = dt / (B.FILTER_TAU + dt) if dt and dt > 0 else 1.0
    return new if prev is None else prev + a * (new - prev)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ticks_csv")
    ap.add_argument("--t0", type=float, default=None)
    ap.add_argument("--t1", type=float, default=None)
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


if __name__ == "__main__":
    main()
