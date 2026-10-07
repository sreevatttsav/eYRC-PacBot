"""Mac-local tuning harness for Task 1B. No Linux sim / MQTT needed.

Models the meshes you have: chassis ~92x97mm, wheel R~17mm.
Diff-drive: v = R*(Vl+Vr)/2, omega = R*(Vr-Vl)/L.

Run:
    python3 tune_local.py --run offset,angle,noise,junction,front
    python3 tune_local.py --sweep            # grid over KP_LAT x KD_YAW
    python3 tune_local.py --run offset --plot out.png --verbose

Tune order: KP_LAT up till slight oscillation -> KD_YAW up till damped
-> KP_FRONT up till nose stops wiggling.
"""
import argparse
import csv
import math
import os
import random
import sys
import types
from datetime import datetime, timezone

# --- stub paho so we can import the real controller class unmodified ---
_paho = types.ModuleType("paho")
_paho.__path__ = []
_mqtt = types.ModuleType("paho.mqtt")
_mqtt.__path__ = []
_mqtt_client_mod = types.ModuleType("paho.mqtt.client")
_mqtt_client_mod.Client = lambda *a, **k: None
_mqtt_client_mod.CallbackAPIVersion = types.SimpleNamespace(VERSION2=0)
_paho.mqtt = _mqtt
_mqtt.client = _mqtt_client_mod
sys.modules["paho"] = _paho
sys.modules["paho.mqtt"] = _mqtt
sys.modules["paho.mqtt.client"] = _mqtt_client_mod
sys.path.insert(0, ".")
sys.path.insert(0, "task1b")

from task_1b_boilerplate import (  # noqa: E402
    BASE_SPEED,
    FRONT_STOP_DIST,
    KD_YAW,
    KP_FRONT,
    KP_LAT,
    MAX_RANGE,
    CenteringController,
)

# Robot scale from meshes/roda_sim.stl + chassis_sim.stl
WHEEL_R = 0.017   # m
WHEEL_L = 0.078   # m track width from simulator MJCF
ROBOT_HALF_W = 0.043
CORRIDOR_W = 0.17  # m wall-to-wall from the documented maze geometry
FRONT_RAY_ANGLE = math.radians(20.0)
DT = 0.02
TOF_SIGMA = 0.005  # m
GYRO_SIGMA = 0.02  # rad/s
OMEGA_TAU = 0.12   # yaw inertia lag (s) -- makes KD_YAW meaningful

# Turn plant for Stage 1c logic tests: first-order lag + yaw gain.
# Plant default (0.18) is the SIM surrogate: the log measured -1.09 rad/s
# at w=3, i.e. k = 1.09/6 = 0.18. The CONTROLLER's YAW_GAIN_K stays at
# the plan's conservative 0.13 until step_test replaces it -- the gap
# between the two is exactly what the slow-plant test probes.
# Coast tau models passive spin-down (step_test: ~10 deg coast from
# w=3 at 0.66 rad/s -> tau ~= 0.28 s); the drive tau stays fast.
TURN_PLANT_TAU = 0.03
TURN_PLANT_COAST_TAU = 0.28
TURN_PLANT_GAIN = 0.18

CSV_HEADER = ["timestamp", "mode", "scenario", "kp_lat", "kp_front",
              "kd_yaw", "gyro_bias", "seed",
              "rms", "max", "final", "hit", "spun"]


def log_csv(path, rows):
    """Append rows (list of dicts) to CSV, writing header if file is new."""
    if not path or not rows:
        return
    new_file = not os.path.exists(path) or os.path.getsize(path) == 0
    with open(path, "a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_HEADER)
        if new_file:
            w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in CSV_HEADER})
    print(f"logged {len(rows)} row(s) -> {path}")


class Corridor:
    def __init__(self, length=6.0, end_wall_x=5.0,
                 dropout=None, gyro_bias=0.0, seed=0):
        self.half = CORRIDOR_W / 2
        self.end_x = end_wall_x
        self.length = length
        self.dropout = dropout  # (x0, x1) where RIGHT wall missing
        self.gyro_bias = gyro_bias
        self.rng = random.Random(seed)

    def sense(self, x, y, theta, true_omega):
        c = max(0.2, math.cos(theta))
        sl = (self.half - y) / c
        sr = (self.half + y) / c
        if self.dropout and self.dropout[0] < x < self.dropout[1]:
            sr = MAX_RANGE  # junction: right wall gone
        ahead = max(0.0, (self.end_x - x) / c)
        fl = ahead / math.cos(FRONT_RAY_ANGLE)
        fr = ahead / math.cos(-FRONT_RAY_ANGLE)
        # clip + noise like real ToF
        def n(v):
            v = max(0.0, min(v, MAX_RANGE))
            return max(0.0, v + self.rng.gauss(0, TOF_SIGMA))
        yaw = true_omega + self.gyro_bias + self.rng.gauss(0, GYRO_SIGMA)
        return {"fl": n(fl), "fr": n(fr), "sl": n(sl), "sr": n(sr),
                "yaw_rate": yaw}


def step_pose(x, y, th, om_prev, Vl, Vr, dt):
    v = WHEEL_R * (Vl + Vr) / 2.0
    om_cmd = WHEEL_R * (Vr - Vl) / WHEEL_L
    # first-order yaw lag: body + motor inertia (else KD_YAW does nothing)
    om = om_prev + (om_cmd - om_prev) * dt / (OMEGA_TAU + dt)
    th += om * dt
    x += v * math.cos(th) * dt
    y += v * math.sin(th) * dt
    return x, y, th, om, v


class TurnPlant:
    """First-order yaw plant for turn-logic tests (Stage 1c).

    omega_dot = (gain*(R-L) - omega)/tau, with a SLOW coast tau when
    the command is ~zero (passive spin-down, step_test-measured) and a
    fast drive tau otherwise. stall=True pins omega at 0 (expect
    no_progress abort ~1.0 s into the turn, ~1.5 s from block).
    """

    def __init__(self, gain=TURN_PLANT_GAIN, tau=TURN_PLANT_TAU,
                 coast_tau=TURN_PLANT_COAST_TAU,
                 stall=False, noise=0.02, seed=0):
        self.gain = gain
        self.tau = tau
        self.coast_tau = coast_tau
        self.stall = stall
        self.noise = noise
        self.rng = random.Random(seed)
        self.om = 0.0

    def step(self, L, R, dt):
        cmd = 0.0 if self.stall else self.gain * (R - L)
        tau = self.coast_tau if abs(cmd) < 0.05 else self.tau
        self.om += (cmd - self.om) * dt / (tau + dt)
        meas = self.om + self.rng.gauss(0, self.noise)
        return self.om, meas


def run_turn_test(name, target_deg, plant_gain=TURN_PLANT_GAIN,
                  stall=False, seed=0, T=14.0):
    """Drive a fresh controller: blocked front until the turn passes 60%
    of target, then open. Returns outcome dict."""
    ctl = CenteringController()
    want_left = target_deg > 0
    mag = abs(target_deg)
    if mag > 100:  # dead-end 180: both sides close (but above wedge)
        sl0, sr0 = 0.10, 0.10
    elif want_left:
        # The left branch is open; the right wall remains present.
        sl0, sr0 = 0.30, 0.12
    else:
        # The right branch is open; the left wall remains present.
        sl0, sr0 = 0.12, 0.30
    if mag > 100:
        # This harness isolates 180-degree execution. In navigation, the
        # controller first performs a left 90-degree dead-end probe.
        ctl.deadend_probe_used = True
    plant = TurnPlant(gain=plant_gain, stall=stall, seed=seed)
    turned = 0.0
    aborts = []
    prev_state = None
    states_seen = set()
    n = int(T / DT)
    om_meas = 0.0
    first_abort = None
    for i in range(n):
        frac = abs(turned) / math.radians(mag)
        front = 0.05 if frac < 0.6 else 1.0
        L, R, el, ef, st = ctl.update(front, front, sl0, sr0,
                                      om_meas, DT)
        om, om_meas = plant.step(L, R, DT)
        turned += om * DT
        states_seen.add(ctl.state)
        if ctl.state != prev_state:
            aborts.append((i * DT, prev_state, ctl.state,
                           ctl.state_reason))
            if (first_abort is None and ctl.state == "BACKUP"
                    and ctl.abort_reason):
                first_abort = (i * DT, ctl.abort_reason)
            prev_state = ctl.state
        if ctl.state == "FOLLOW" and i * DT > 1.0:
            break
    err_deg = abs(math.degrees(abs(turned) - math.radians(mag)))
    return {"name": name, "target_deg": target_deg,
            "turned_deg": math.degrees(turned), "final_err_deg": err_deg,
            "abort_reason": ctl.abort_reason,
            "first_abort": first_abort,
            "states": sorted(states_seen),
            "state": ctl.state, "transitions": aborts,
            "time_s": i * DT}


def turn_tests(plant_gain=TURN_PLANT_GAIN, seed=0):
    cases = [
        ("left90", 90.0, plant_gain, False),
        ("right90", -90.0, plant_gain, False),
        ("deadend180", 180.0, plant_gain, False),
        ("stalled", 90.0, plant_gain, True),
        ("slow060", 90.0, 0.6 * plant_gain, False),
    ]
    results = []
    for name, tgt, gain, stall in cases:
        r = run_turn_test(name, tgt, plant_gain=gain, stall=stall,
                          seed=seed)
        braked = "BRAKE" in r["states"]
        if name.startswith("stalled"):
            fa = r["first_abort"]
            r["pass"] = (fa is not None and fa[1] == "no_progress"
                         and 1.2 <= fa[0] <= 2.0)
        elif name.startswith("slow"):
            r["pass"] = (r["abort_reason"] == "" and r["state"] == "FOLLOW"
                         and r["final_err_deg"] <= 5.0 and braked)
        else:
            r["pass"] = (r["abort_reason"] == "" and r["state"] == "FOLLOW"
                         and r["final_err_deg"] <= 5.0 and braked)
        results.append(r)
        first = (f"first_abort={r['first_abort']}" if r["first_abort"]
                 else "no-abort")
        print(f"{r['name']:10s} turned={r['turned_deg']:+7.1f}deg "
              f"err={r['final_err_deg']:.1f}deg t={r['time_s']:.2f}s "
              f"abort={r['abort_reason'] or '-':12s} "
              f"end={r['state']} brake={braked} {first} "
              f"{'PASS' if r['pass'] else 'FAIL'}")
    print("TURN-TESTS " +
          ("PASS" if all(r["pass"] for r in results) else "FAIL"))
    return results


def run_scenario(name, gains, T=12.0, seed=1, gyro_bias=0.0,
                 verbose=False):
    import task_1b_boilerplate as B
    old = (B.KP_LAT, B.KP_FRONT, B.KD_YAW)
    B.KP_LAT, B.KP_FRONT, B.KD_YAW = gains
    try:
        ctl = CenteringController()
        y0, th0, x0 = 0.0, 0.0, 0.0
        dropout = None
        end_x = 999.0  # no front wall unless scenario says so
        if name == "offset":
            y0 = 0.10
        elif name == "angle":
            th0 = math.radians(15)
        elif name == "noise":
            y0, th0 = 0.03, math.radians(-8)
        elif name == "junction":
            y0 = 0.02
            dropout = (2.0, 3.0)
        elif name == "front":
            end_x = 1.5
        cor = Corridor(end_wall_x=end_x, dropout=dropout,
                       gyro_bias=gyro_bias, seed=seed)
        x, y, th = x0, y0, th0
        om_actual = 0.0
        n = int(T / DT)
        ys, ths, sts = [], [], []
        hit, spun = False, False
        for i in range(n):
            s = cor.sense(x, y, th, om_actual)
            L, R, el, ef, st = ctl.update(
                s["fl"], s["fr"], s["sl"], s["sr"], s["yaw_rate"], DT)
            if abs(L) == abs(R) and L < 0 < R or L < 0 < R:
                spun = True
            x, y, th, om_actual, v = step_pose(x, y, th, om_actual, L, R, DT)
            ys.append(y)
            ths.append(th)
            sts.append(st)
            if abs(y) > cor.half - ROBOT_HALF_W:
                hit = True
            if verbose and i % 25 == 0:
                print(f"  t={i*DT:5.2f} y={y:+.3f} th={math.degrees(th):+6.1f} "
                      f"e_lat={el:+.3f} steer={st:+.3f} L={L:+.2f} R={R:+.2f}")
            if end_x < 900 and x > end_x - 0.05:
                break
        rms = math.sqrt(sum(v * v for v in ys[-200:]) / min(200, len(ys)))
        return {"rms": rms, "max": max(abs(v) for v in ys),
                "final": abs(ys[-1]), "hit": hit, "spun": spun,
                "ys": ys, "ths": ths, "sts": sts}
    finally:
        B.KP_LAT, B.KP_FRONT, B.KD_YAW = old


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default="offset,angle,noise,junction",
                    help="comma list: offset,angle,noise,junction,front")
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--kp-lat", type=float, default=KP_LAT)
    ap.add_argument("--kp-front", type=float, default=KP_FRONT)
    ap.add_argument("--kd-yaw", type=float, default=KD_YAW)
    ap.add_argument("--gyro-bias", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--plot", default=None)
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--csv", default=None,
                    help="append per-scenario metrics to this CSV file")
    ap.add_argument("--turn-tests", action="store_true",
                    help="run turn-logic tests on the yaw plant (Stage 1c)")
    ap.add_argument("--plant-gain", type=float, default=TURN_PLANT_GAIN,
                    help="yaw gain for the turn plant (from step_test)")
    a = ap.parse_args()
    ts = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")

    if a.turn_tests:
        turn_tests(plant_gain=a.plant_gain, seed=a.seed)
        return

    if a.sweep:
        print(f"{'KP':>5} {'KD':>5} | {'offset':>6} {'angle':>6} "
              f"{'noise':>6} {'junct':>6} | hit?")
        best, best_cost = None, 1e9
        csv_rows = []
        for kp in [1.5, 3.0, 5.0, 7.0]:
            for kd in [0.0, 0.4, 0.8, 1.5]:
                gains = (kp, a.kp_front, kd)
                row, hits = [], 0
                for sc in ["offset", "angle", "noise", "junction"]:
                    r = run_scenario(sc, gains, seed=a.seed,
                                     gyro_bias=a.gyro_bias)
                    row.append(r["rms"])
                    hits += r["hit"]
                    if a.csv:
                        csv_rows.append({
                            "timestamp": ts, "mode": "sweep",
                            "scenario": sc, "kp_lat": kp,
                            "kp_front": a.kp_front, "kd_yaw": kd,
                            "gyro_bias": a.gyro_bias,
                            "seed": a.seed, "rms": f"{r['rms']:.4f}",
                            "max": f"{r['max']:.4f}",
                            "final": f"{r['final']:.4f}",
                            "hit": r["hit"], "spun": r["spun"]})
                cost = sum(row) + 10 * hits
                flag = "HIT" if hits else ""
                print(f"{kp:5.1f} {kd:5.1f} | " +
                      " ".join(f"{v:6.3f}" for v in row) + f" | {flag}")
                if cost < best_cost:
                    best, best_cost = (kp, kd), cost
        print(f"best: KP_LAT={best[0]} KD_YAW={best[1]} cost={best_cost:.3f}")
        log_csv(a.csv, csv_rows)
        return

    gains = (a.kp_lat, a.kp_front, a.kd_yaw)
    print(f"gains KP_LAT={gains[0]} KP_FRONT={gains[1]} "
          f"KD_YAW={gains[2]} bias={a.gyro_bias}")
    results = {}
    csv_rows = []
    for sc in a.run.split(","):
        sc = sc.strip()
        if not sc:
            continue
        r = run_scenario(sc, gains, seed=a.seed,
                         gyro_bias=a.gyro_bias, verbose=a.verbose)
        results[sc] = r
        print(f"{sc:8s} rms={r['rms']:.3f} max={r['max']:.3f} "
              f"final={r['final']:.3f} hit={r['hit']} spun={r['spun']}")
        if a.csv:
            csv_rows.append({
                "timestamp": ts, "mode": "run", "scenario": sc,
                "kp_lat": gains[0], "kp_front": gains[1],
                "kd_yaw": gains[2],
                "gyro_bias": a.gyro_bias, "seed": a.seed,
                "rms": f"{r['rms']:.4f}", "max": f"{r['max']:.4f}",
                "final": f"{r['final']:.4f}",
                "hit": r["hit"], "spun": r["spun"]})
    log_csv(a.csv, csv_rows)

    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        names = list(results)
        fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        for sc in names:
            ys = results[sc]["ys"]
            t = [i * DT for i in range(len(ys))]
            axes[0].plot(t, ys, label=sc)
            axes[1].plot(t, results[sc]["sts"], label=sc)
        axes[0].axhline(CORRIDOR_W / 2 - ROBOT_HALF_W, c="r", ls="--")
        axes[0].axhline(-(CORRIDOR_W / 2 - ROBOT_HALF_W), c="r", ls="--")
        axes[0].set_ylabel("lateral y (m)")
        axes[1].set_ylabel("steer (rad/s)")
        axes[1].set_xlabel("time (s)")
        axes[0].legend()
        axes[0].set_title(f"KP={gains[0]} KF={gains[1]} KD={gains[2]} KI={gains[3]}")
        fig.tight_layout()
        fig.savefig(a.plot)
        print(f"saved {a.plot}")


if __name__ == "__main__":
    main()
