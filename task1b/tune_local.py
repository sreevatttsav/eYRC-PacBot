"""Mac-local tuning harness for Task 1B. No Linux sim / MQTT needed.

Models the meshes you have: chassis ~92x97mm, wheel R~17mm.
Diff-drive: v = R*(Vl+Vr)/2, omega = R*(Vr-Vl)/L.

Run:
    python3 tune_local.py --run offset,angle,noise,junction,front
    python3 tune_local.py --sweep            # grid over KP_LAT x KD_YAW
    python3 tune_local.py --run offset --plot out.png --verbose

Tune order: KP_LAT up till slight oscillation -> KD_YAW up till damped
-> KP_FRONT up till nose stops wiggling -> KI_LAT last, tiny.
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
    FRONT_SLOW_DIST,
    FRONT_STOP_DIST,
    KD_YAW,
    KI_LAT,
    KP_FRONT,
    KP_LAT,
    MAX_RANGE,
    CenteringController,
)

# Robot scale from meshes/roda_sim.stl + chassis_sim.stl
WHEEL_R = 0.017   # m
WHEEL_L = 0.08    # m track width (tunable guess)
ROBOT_HALF_W = 0.05
CORRIDOR_W = 0.40  # m wall-to-wall
DT = 0.02
TOF_SIGMA = 0.005  # m
GYRO_SIGMA = 0.02  # rad/s
OMEGA_TAU = 0.12   # yaw inertia lag (s) -- makes KD_YAW meaningful

CSV_HEADER = ["timestamp", "mode", "scenario", "kp_lat", "kp_front",
              "kd_yaw", "ki_lat", "gyro_bias", "seed",
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
        fl = ahead / math.cos(0.26)  # ~15 deg splay each side
        fr = ahead / math.cos(-0.26)
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


def run_scenario(name, gains, T=12.0, seed=1, gyro_bias=0.0,
                 verbose=False):
    import task_1b_boilerplate as B
    old = (B.KP_LAT, B.KP_FRONT, B.KD_YAW, B.KI_LAT)
    B.KP_LAT, B.KP_FRONT, B.KD_YAW, B.KI_LAT = gains
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
        B.KP_LAT, B.KP_FRONT, B.KD_YAW, B.KI_LAT = old


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default="offset,angle,noise,junction",
                    help="comma list: offset,angle,noise,junction,front")
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--kp-lat", type=float, default=KP_LAT)
    ap.add_argument("--kp-front", type=float, default=KP_FRONT)
    ap.add_argument("--kd-yaw", type=float, default=KD_YAW)
    ap.add_argument("--ki-lat", type=float, default=KI_LAT)
    ap.add_argument("--gyro-bias", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--plot", default=None)
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--csv", default=None,
                    help="append per-scenario metrics to this CSV file")
    a = ap.parse_args()
    ts = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")

    if a.sweep:
        print(f"{'KP':>5} {'KD':>5} | {'offset':>6} {'angle':>6} "
              f"{'noise':>6} {'junct':>6} | hit?")
        best, best_cost = None, 1e9
        csv_rows = []
        for kp in [1.5, 3.0, 5.0, 7.0]:
            for kd in [0.0, 0.4, 0.8, 1.5]:
                gains = (kp, a.kp_front, kd, a.ki_lat)
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
                            "ki_lat": a.ki_lat, "gyro_bias": a.gyro_bias,
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

    gains = (a.kp_lat, a.kp_front, a.kd_yaw, a.ki_lat)
    print(f"gains KP_LAT={gains[0]} KP_FRONT={gains[1]} "
          f"KD_YAW={gains[2]} KI_LAT={gains[3]} bias={a.gyro_bias}")
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
                "kd_yaw": gains[2], "ki_lat": gains[3],
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
