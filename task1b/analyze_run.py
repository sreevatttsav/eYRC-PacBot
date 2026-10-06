"""analyze_run.py -- pass/fail report from a Phase-0 run log (Stage 0).

Reads columns BY HEADER NAME: old 27-col logs (no status/yaw_cmd/stuck/
turn_* columns) and new logs both work; missing data is reported n/a,
never judged by eye and never silently invented.

Usage:
    python3 analyze_run.py logs/<timestamp>_<label>/
    python3 analyze_run.py logs/A logs/B --compare   # diff two runs

Report:
    turn table   target, final angle 0.3 s after exit (gyro integral,
                 coast included), duration, abort reason, reversals
    stuck        episodes with flag-to-clear time
    transitions  state-transition counts
    sensors      status fractions per ToF
    wheels       mean |wheel| speed by state
    lateral      (sl-sr)/2 over ticks both-valid & below cap + coverage

Turn PASS/FAIL follows Stage 1d: final err within +/-5 deg, zero
turn_timeout / no_progress / overshoot / (TURN|BRAKE)->BACKUP, <=1
reversal (within 8 deg of target only), sign(left)=+yaw.
"""
import argparse
import csv
import json
import math
import os
import statistics
import sys

CAP_M = 0.300          # lateral metric needs both sides below the cap
FINAL_ERR_DEG = 5.0
REVERSAL_BAND_DEG = 8.0
HEADING_FLAG_RAD = math.radians(20.0)
HEADING_FLAG_DWELL_S = 5.0
STEER_COMPONENT_MIN = 1.0
STEER_TOTAL_NEAR_ZERO = 0.10
REPEAT_SIGNATURE_TOL_M = 0.01
REPEAT_SIGNATURE_MIN = 3


def _f(row, key, default=None):
    v = row.get(key, "")
    if v is None or v == "":
        return default
    try:
        return float(v)
    except (ValueError, TypeError):
        return default


def load_run(run_dir):
    ticks = list(csv.DictReader(open(os.path.join(run_dir, "ticks.csv"))))
    ev_path = os.path.join(run_dir, "events.csv")
    events = list(csv.DictReader(open(ev_path))) if os.path.exists(ev_path) else []
    meta_path = os.path.join(run_dir, "meta.json")
    meta = json.load(open(meta_path)) if os.path.exists(meta_path) else {}
    result_path = os.path.join(run_dir, "result.json")
    result = json.load(open(result_path)) if os.path.exists(result_path) else None
    # gyro-integral heading on sim-time (turn ground truth, coast incl.)
    th = 0.0
    for r in ticks:
        g = _f(r, "gyro_z", 0.0) or 0.0
        d = _f(r, "dt_rep", 0.02) or 0.0
        th += g * d
        r["_th"] = th
    return {"dir": run_dir, "ticks": ticks, "events": events,
            "meta": meta, "result": result}


def turn_episodes(rep):
    """Contiguous TURN runs, extended through a trailing BRAKE (the
    brake is part of the turn: rotation stop + angle settle). A
    standalone BRAKE (no preceding TURN) starts its own episode."""
    ticks = rep["ticks"]
    eps, cur = [], None
    for i, r in enumerate(ticks):
        s = r.get("state")
        if s == "TURN" and cur is None:
            cur = {"entry": i, "brake": False}
        elif s == "BRAKE" and cur is None:
            cur = {"entry": i, "brake": True}
        elif s == "BRAKE" and cur is not None:
            cur["brake"] = True
        elif s != "TURN" and cur is not None:
            cur["exit"] = i - 1
            eps.append(cur)
            cur = None
    if cur is not None:
        cur["exit"] = len(ticks) - 1
        eps.append(cur)
    return eps


def exit_reason(rep, ep):
    """Abort reason from the events log (transition out of TURN/BRAKE).
    runlog stamps event.i one past the transition tick row, so the
    matching row is exit+2."""
    for e in rep["events"]:
        try:
            if (int(e["i"]) == ep["exit"] + 2
                    and e["from"] in ("TURN", "BRAKE")):
                return f"{e['to']}:{e.get('reason', '')}"
        except (ValueError, KeyError):
            pass
    # fallback: next state in ticks
    nxt = rep["ticks"][ep["exit"] + 1].get("state") if ep["exit"] + 1 < len(rep["ticks"]) else "END"
    return f"{nxt}:(no-event-row)"


def analyze_turns(rep):
    ticks = rep["ticks"]
    out = []
    for n, ep in enumerate(turn_episodes(rep)):
        e0, e1 = ep["entry"], ep["exit"]
        seg = ticks[e0:e1 + 1]
        t0 = _f(ticks[e0], "t_wall", 0.0) or 0.0
        # final angle 0.3 s after exit (coast included)
        t_end = (_f(ticks[e1], "t_wall", 0.0) or 0.0) + 0.3
        th_exit = ticks[e1]["_th"]
        th_end = th_exit
        for r in ticks[e1 + 1:]:
            th_end = r["_th"]
            if (_f(r, "t_wall", 0.0) or 0.0) >= t_end:
                break
        target = _f(seg[0], "turn_target", None)
        tgt_note = "logged"
        if target is None:
            # gap/lost turns log one arming tick (state TURN, zeros out)
            # before the servo initializes: take the first non-empty
            # target in the episode, not the arming row.
            for r in seg[1:]:
                target = _f(r, "turn_target", None)
                if target is not None:
                    break
        if target is None:  # old log: infer magnitude, sign from motion
            target = math.copysign(math.pi / 2,
                                   (th_exit - ticks[e0]["_th"]) or 1.0)
            tgt_note = "inferred-90deg"
        turned = th_end - ticks[e0]["_th"]
        err_deg = math.degrees(turned - target)
        # reversals: sign flips of commanded differential WITHIN TURN
        # ticks only. The TURN->BRAKE brake opposition flip is the
        # brake doing its job, not a direction reversal -- excluding
        # BRAKE rows keeps re-trim flips countable.
        flips, flip_at = 0, []
        prev = None
        for r in seg:
            if r.get("state") != "TURN":
                continue
            d = (_f(r, "R", 0.0) or 0.0) - (_f(r, "L", 0.0) or 0.0)
            s = 1 if d > 0 else (-1 if d < 0 else 0)
            if s and prev and s != prev:
                flips += 1
                flip_at.append(math.degrees(r["_th"] - ticks[e0]["_th"]))
            if s:
                prev = s
        reason = exit_reason(rep, ep)
        aborted = ("BACKUP" in reason or "timeout" in reason
                   or "progress" in reason or "unverified" in reason
                   or "overshoot" in reason or "retrim" in reason)
        ok = (abs(err_deg) <= FINAL_ERR_DEG and not aborted and flips <= 1
              and all(abs(a - math.degrees(abs(target))) <= REVERSAL_BAND_DEG
                      for a in flip_at))
        out.append({
            "n": n, "t0": t0, "target_deg": math.degrees(target),
            "target_src": tgt_note, "final_err_deg": err_deg,
            "dur_s": (_f(ticks[e1], "t_wall", 0.0) or 0.0) - t0,
            "reason": reason, "reversals": flips,
            "flip_at_deg": flip_at, "pass": ok,
        })
    return out


def analyze_stuck(rep):
    ticks = rep["ticks"]
    if "stuck" not in (ticks[0].keys() if ticks else []):
        return None  # old log: no stuck column
    eps, cur = [], None
    for i, r in enumerate(ticks):
        if r.get("stuck") == "1" and cur is None:
            cur = {"start_i": i, "start_t": _f(r, "t_wall", 0.0) or 0.0,
                   "state": r.get("state", "?")}
        if cur is not None and r.get("stuck") != "1":
            cur["clear_t"] = _f(r, "t_wall", 0.0) or 0.0
            cur["hold_s"] = cur["clear_t"] - cur["start_t"]
            eps.append(cur)
            cur = None
    if cur is not None:
        cur["clear_t"] = _f(ticks[-1], "t_wall", 0.0) or 0.0
        cur["hold_s"] = cur["clear_t"] - cur["start_t"]
        cur["unterminated"] = True
        eps.append(cur)
    return eps


def analyze_sensors(rep):
    ticks = rep["ticks"]
    keys = ticks[0].keys() if ticks else []
    out = {}
    for s in ("fl", "fr", "sl", "sr"):
        col = f"{s}_s"
        if col not in keys:
            out[s] = "n/a (old log)"
            continue
        tot = len(ticks)
        out[s] = {st: sum(1 for r in ticks if r.get(col) == st) / tot
                  for st in ("valid", "sat", "held", "none")}
    return out


def analyze_wheels(rep):
    by_state = {}
    for r in rep["ticks"]:
        by_state.setdefault(r.get("state", "?"), []).append(
            (abs(_f(r, "L", 0.0) or 0.0) + abs(_f(r, "R", 0.0) or 0.0)) / 2.0)
    return {s: statistics.mean(v) for s, v in by_state.items()}


def analyze_transitions(rep):
    counts = {}
    if rep["events"]:
        for e in rep["events"]:
            k = (e.get("from", "?"), e.get("to", "?"), e.get("reason", ""))
            counts[k] = counts.get(k, 0) + 1
    else:  # derive from state column
        prev = None
        for r in rep["ticks"]:
            s = r.get("state", "?")
            if prev is not None and s != prev:
                k = (prev, s, "")
                counts[k] = counts.get(k, 0) + 1
            prev = s
    return counts


def analyze_lateral(rep):
    ticks = rep["ticks"]
    keys = ticks[0].keys() if ticks else []
    has_status = "sl_s" in keys
    vals = []
    for r in ticks:
        sl, sr = _f(r, "sl_f"), _f(r, "sr_f")
        if sl is None or sr is None or sl >= CAP_M or sr >= CAP_M:
            continue
        if has_status and (r.get("sl_s") not in ("valid", "held")
                           or r.get("sr_s") not in ("valid", "held")):
            continue  # sat/unknown sides are not centerable
        vals.append((sl - sr) / 2.0)
    if not vals:
        return {"mean_abs": None, "coverage": 0.0, "n": 0}
    return {"mean_abs": statistics.mean(abs(v) for v in vals),
            "coverage": len(vals) / len(ticks), "n": len(vals)}


def analyze_heading_drift(rep):
    ticks = rep["ticks"]
    if not ticks or "e_heading" not in ticks[0]:
        return None
    episodes, start, elapsed = [], None, 0.0
    for r in ticks:
        e = _f(r, "e_heading")
        dt = _f(r, "dt_rep", 0.0) or 0.0
        if e is not None and abs(e) > HEADING_FLAG_RAD:
            if start is None:
                start = _f(r, "t_wall", 0.0) or 0.0
                elapsed = 0.0
            elapsed += dt
        elif start is not None:
            episodes.append({"start_t": start, "duration_s": elapsed})
            start, elapsed = None, 0.0
    if start is not None:
        episodes.append({"start_t": start, "duration_s": elapsed,
                         "unterminated": True})
    flagged = [e for e in episodes if e["duration_s"] > HEADING_FLAG_DWELL_S]
    return {"episodes": episodes, "flagged": flagged,
            "max_duration_s": max((e["duration_s"] for e in episodes),
                                   default=0.0)}


def analyze_steer_cancellation(rep):
    ticks = rep["ticks"]
    keys = ticks[0].keys() if ticks else []
    components = ("steer_lat", "steer_front", "steer_heading")
    if not ticks or not all(k in keys for k in components) or "steer" not in keys:
        return None
    rows = []
    for r in ticks:
        vals = [_f(r, k) for k in components]
        total = _f(r, "steer")
        if None in vals or total is None:
            continue
        magnitude = sum(abs(v) for v in vals)
        if (magnitude >= STEER_COMPONENT_MIN
                and abs(total) <= STEER_TOTAL_NEAR_ZERO):
            rows.append({"t": _f(r, "t_wall", 0.0) or 0.0,
                         "state": r.get("state", "?"),
                         "component_abs_sum": magnitude,
                         "total": total})
    return {"count": len(rows), "fraction": len(rows) / len(ticks),
            "examples": rows[:5]}


def analyze_repeated_state_signatures(rep):
    ticks = rep["ticks"]
    if not ticks:
        return None
    sig_keys = ("fl_f", "fr_f", "sl_f", "sr_f")
    if not all(k in ticks[0] for k in sig_keys):
        sig_keys = ("fl", "fr", "sl", "sr")
    clusters = []
    prev_state = None
    pending_signature = None
    for i, r in enumerate(ticks):
        state = r.get("state")
        if state == "FRONT_BACKOUT" and prev_state != "FRONT_BACKOUT":
            sig = tuple(_f(r, k) for k in sig_keys)
            pending_signature = sig if all(v is not None for v in sig) else None
        elif (pending_signature is not None
              and prev_state == "FRONT_BACKOUT" and state == "FOLLOW"):
            sig = pending_signature
            if all(v is not None for v in sig):
                if (clusters and max(abs(a - b) for a, b in
                                     zip(sig, clusters[-1]["signature"]))
                        <= REPEAT_SIGNATURE_TOL_M):
                    clusters[-1]["count"] += 1
                    clusters[-1]["last_t"] = _f(r, "t_wall", 0.0) or 0.0
                else:
                    clusters.append({
                        "count": 1,
                        "first_t": _f(r, "t_wall", 0.0) or 0.0,
                        "last_t": _f(r, "t_wall", 0.0) or 0.0,
                        "signature": sig,
                    })
            pending_signature = None
        elif pending_signature is not None and state != "FRONT_BACKOUT":
            pending_signature = None
        prev_state = state
    repeated = [c for c in clusters if c["count"] >= REPEAT_SIGNATURE_MIN]
    return {"clusters": clusters, "repeated": repeated}


def report(rep):
    turns = analyze_turns(rep)
    stuck = analyze_stuck(rep)
    trans = analyze_transitions(rep)
    sens = analyze_sensors(rep)
    wheels = analyze_wheels(rep)
    lat = analyze_lateral(rep)
    heading = analyze_heading_drift(rep)
    steer_cancel = analyze_steer_cancellation(rep)
    cycles = analyze_repeated_state_signatures(rep)
    return {"turns": turns, "stuck": stuck, "trans": trans,
            "sens": sens, "wheels": wheels, "lat": lat,
            "heading": heading, "steer_cancel": steer_cancel,
            "cycles": cycles, "result": rep.get("result"),
            "nticks": len(rep["ticks"]),
            "label": rep["meta"].get("label", "?"),
            "commit": (rep["meta"].get("git_commit") or "?")[:7]}


def print_report(r):
    print(f"== {r['label']} commit={r['commit']} ticks={r['nticks']}")
    print("-- turns --")
    if not r["turns"]:
        print("  (none)")
    for t in r["turns"]:
        print(f"  #{t['n']} t={t['t0']:.1f}s target={t['target_deg']:+.0f}deg"
              f"({t['target_src']}) final_err={t['final_err_deg']:+.1f}deg "
              f"dur={t['dur_s']:.2f}s exit={t['reason']} "
              f"rev={t['reversals']}{t['flip_at_deg']} "
              f"{'PASS' if t['pass'] else 'FAIL'}")
    print("-- stuck --")
    if r["stuck"] is None:
        print("  n/a (old log, no stuck column)")
    elif not r["stuck"]:
        print("  zero episodes")
    for s in r["stuck"] or []:
        flag = " CLEAR>4s-FAIL" if s["hold_s"] > 4.0 else ""
        print(f"  t={s['start_t']:.1f}s @{s['state']} hold={s['hold_s']:.2f}s{flag}")
    print("-- transitions --")
    for (f, t, why), n in sorted(r["trans"].items()):
        print(f"  {f}->{t} [{why}] x{n}")
    print("-- sensors (valid/sat/held/none) --")
    for s, v in r["sens"].items():
        print(f"  {s}: {v}" if isinstance(v, str) else
              f"  {s}: " + " ".join(f"{k}={v[k]:.1%}" for k in ("valid", "sat", "held", "none")))
    print("-- mean |wheel| by state --")
    for s, v in sorted(r["wheels"].items()):
        print(f"  {s}: {v:.2f} rad/s")
    lat = r["lat"]
    print(f"-- lateral: mean|offset|={lat['mean_abs']} "
          f"coverage={lat['coverage']:.1%} n={lat['n']}")
    result = r.get("result")
    if result is None:
        print("-- simulator verdict -- n/a (no result.json)")
    else:
        payloads = [m.get("payload") for m in result.get("messages", [])]
        print(f"-- simulator verdict -- {result.get('status')}"
              + (f" payload={payloads[-1]}" if payloads else ""))
    heading = r.get("heading")
    if heading is None:
        print("-- heading drift -- n/a (old log)")
    else:
        print(f"-- heading drift -- max>{math.degrees(HEADING_FLAG_RAD):.0f}deg "
              f"duration={heading['max_duration_s']:.2f}s "
              f"episodes>{HEADING_FLAG_DWELL_S:.0f}s="
              f"{len(heading['flagged'])}"
              + (" FLAG" if heading["flagged"] else ""))
    cancel = r.get("steer_cancel")
    if cancel is None:
        print("-- steer cancellation -- n/a (old log)")
    else:
        print(f"-- steer cancellation -- {cancel['count']} ticks "
              f"({cancel['fraction']:.2%})"
              + (" FLAG" if cancel["count"] else ""))
    cycles = r.get("cycles")
    if cycles is None:
        print("-- repeated state/signature cycles -- n/a")
    else:
        print(f"-- repeated state/signature cycles -- "
              f"{len(cycles['repeated'])} cluster(s) of >="
              f"{REPEAT_SIGNATURE_MIN} similar backouts"
              + (" FLAG" if cycles["repeated"] else ""))


def compare(a, b):
    print(f"== compare {a['label']} vs {b['label']}")
    ta, tb = len(a["turns"]), len(b["turns"])
    pa = sum(t["pass"] for t in a["turns"])
    pb = sum(t["pass"] for t in b["turns"])
    print(f"  turns: {ta} ({pa} pass) -> {tb} ({pb} pass)")
    for key in ("dur_s", "final_err_deg"):
        va = [abs(t[key]) for t in a["turns"]] or [0]
        vb = [abs(t[key]) for t in b["turns"]] or [0]
        print(f"  mean|{key}|: {statistics.mean(va):.3f} -> {statistics.mean(vb):.3f}")
    sa = len(a["stuck"] or [])
    sb = len(b["stuck"] or [])
    print(f"  stuck episodes: {sa} -> {sb}")
    print(f"  lateral mean|off|: {a['lat']['mean_abs']} -> {b['lat']['mean_abs']} "
          f"(cov {a['lat']['coverage']:.1%} -> {b['lat']['coverage']:.1%})")
    for s in sorted(set(a["wheels"]) | set(b["wheels"])):
        print(f"  wheel[{s}]: {a['wheels'].get(s, float('nan')):.2f} -> "
              f"{b['wheels'].get(s, float('nan')):.2f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--compare", action="store_true")
    args = ap.parse_args()
    reps = [report(load_run(d)) for d in args.runs]
    if args.compare and len(reps) == 2:
        compare(reps[0], reps[1])
    else:
        for r in reps:
            print_report(r)


if __name__ == "__main__":
    main()
