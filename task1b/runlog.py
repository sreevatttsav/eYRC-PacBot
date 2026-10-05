"""Per-run file logging for Task 1B controller debugging (Phase 0).

Each run gets its own folder: <log_dir>/<UTC-timestamp>_<label>/
  meta.json  - git commit, every gain/constant, run label, wheel estimates.
               Two runs are only comparable if you know what produced them.
  ticks.csv  - one row per on_message callback (see TICKS_HEADER).
  events.csv - controller state transitions (FOLLOW -> TURN, wedge, give-up).

Buffered writers flush every FLUSH_EVERY rows so disk I/O never stalls
the MQTT callback. Pose (x_est, y_est, th_est) is DEAD-RECKONED from gyro
heading + commanded wheel speeds -- it drifts, treat as estimated only.
True pose is logged if the sim payload ever provides it (see `extra`).

Usage (from task_1b_boilerplate.on_message):
    logger = RunLogger("logs", label="baseline", constants={...})
    ...
    logger.tick(raw=..., filt=..., gyro_z=..., dt_rep=..., e_lat=...,
                e_front=..., steer=..., L=..., R=..., state=..., extra={...})
    logger.event(prev_state, new_state, reason)   # on transitions only
    logger.close()                                # on shutdown
"""
import csv
import json
import math
import os
import subprocess
import time
from datetime import datetime, timezone

FLUSH_EVERY = 50

TICKS_HEADER = [
    "i",
    "t_mono",      # s, time.monotonic() at callback (monotonic clock)
    "t_wall",      # s since run start, derived from t_mono (UTC-anchored)
    "dt_rep",      # s, simulator-reported timestep (payload "dt")
    "dt_wall",     # s, measured wall-clock between callbacks
    "fl", "fr", "sl", "sr",          # raw ToF readings
    "fl_f", "fr_f", "sl_f", "sr_f",  # controller-filtered ToF ("" = unknown)
    "fl_s", "fr_s", "sl_s", "sr_s",  # per-sensor status: valid/held/none
    "gyro_z",      # rad/s about z (measured)
    "yaw_cmd",     # rad/s commanded (from wheel outputs + estimates)
    "stuck",       # 0/1 stuck-detector flag
    "e_lat", "e_front", "steer",
    "L", "R",     # commanded wheel velocities (rad/s)
    "state",       # controller state: FOLLOW / REVERSE / TURN / BACKUP / WEDGE
    "turn_target",  # rad, signed (+left). "" when no turn active/old log
    "turn_angle",   # rad turned since turn entry (gyro integral)
    "turn_error",   # rad remaining (target - angle)
    "turn_elapsed",  # s since turn start
    "turn_progress",  # rad gained in current 1 s no-progress window
    "abort_reason",  # "", hard_timeout, no_progress (+ event reasons)
    "lat_mode",    # both / left_only / right_only / blind (Stage 2)
    "x_est", "y_est", "th_est",  # dead-reckoned pose (ESTIMATED, drifts)
    "x_true", "y_true", "th_true",  # sim ground truth if available, else empty
    "extra",       # JSON of unrecognized payload keys (future pose/topics)
]

EVENTS_HEADER = ["i", "t_mono", "t_wall", "from", "to", "reason"]


def _git_commit():
    try:
        out = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            capture_output=True, text=True, timeout=5,
            cwd=os.path.dirname(os.path.abspath(__file__)),
        )
        h = out.stdout.strip()
        return h if h else None
    except Exception:
        return None


def _sanitize_label(label):
    keep = "".join(c if (c.isalnum() or c in "-_") else "_" for c in label)
    return (keep or "run")[:40]


class RunLogger:
    def __init__(self, log_dir, label, constants,
                 wheel_r=0.017, wheel_track=0.08, k_lin=None):
        """constants: dict of gains/limits recorded verbatim into meta.json.
        k_lin (m/s per wheel rad/s) drives dead-reckoning; defaults to
        wheel_r for old callers. Replace with the speed_test value."""
        self.label = _sanitize_label(label)
        self.started_utc = datetime.now(timezone.utc)
        stamp = self.started_utc.strftime("%Y%m%dT%H%M%SZ")
        self.run_dir = os.path.join(log_dir, f"{stamp}_{self.label}")
        os.makedirs(self.run_dir, exist_ok=True)

        self.wheel_r = wheel_r
        self.wheel_track = wheel_track
        self.k_lin = k_lin if k_lin else wheel_r
        meta = {
            "label": label,
            "started_utc": self.started_utc.isoformat(),
            "git_commit": _git_commit(),
            "constants": constants,
            "wheel_estimates": {
                "R_m": wheel_r,
                "track_m": wheel_track,
                "k_lin": self.k_lin,
                "status": "mesh guess -- replace with speed_test K_LIN",
            },
            "pose_note": ("x_est/y_est/th_est are dead-reckoned from gyro "
                          "heading + commanded wheel speeds. They DRIFT. "
                          "Label estimated in every plot."),
        }
        with open(os.path.join(self.run_dir, "meta.json"), "w") as f:
            json.dump(meta, f, indent=2)

        self._ticks_f = open(os.path.join(self.run_dir, "ticks.csv"),
                             "w", newline="")
        self._events_f = open(os.path.join(self.run_dir, "events.csv"),
                              "w", newline="")
        self._ticks_w = csv.writer(self._ticks_f)
        self._events_w = csv.writer(self._events_f)
        self._ticks_w.writerow(TICKS_HEADER)
        self._events_w.writerow(EVENTS_HEADER)

        self._tick_buf = []
        self._event_buf = []
        self._i = 0
        self._mono0 = None
        self._last_mono = None
        # dead-reckoned pose
        self._x = 0.0
        self._y = 0.0
        self._th = 0.0
        print(f"[runlog] logging to {self.run_dir}")

    # -- per-callback ----------------------------------------------------
    def tick(self, raw, filt, gyro_z, dt_rep, e_lat, e_front, steer,
             L, R, state, true_pose=None, extra=None,
             statuses=("none",) * 4, yaw_cmd=0.0, stuck=False,
             turn=None, lat_mode=""):
        """turn: None or dict(target, angle, error, elapsed, progress,
        abort) -- missing keys log as empty (old callers unaffected)."""
        now = time.monotonic()
        if self._mono0 is None:
            self._mono0 = now
            dt_wall = float(dt_rep) if dt_rep and dt_rep > 0 else 0.0
        else:
            dt_wall = now - self._last_mono
        self._last_mono = now
        t_wall = now - self._mono0

        # dead-reckon: heading from gyro, speed from commanded wheels
        self._th += gyro_z * dt_wall
        v = self.k_lin * (L + R) / 2.0
        self._x += v * math.cos(self._th) * dt_wall
        self._y += v * math.sin(self._th) * dt_wall

        if true_pose is None:
            xt, yt, tht = "", "", ""
        else:
            xt, yt, tht = true_pose

        turn = turn or {}
        def tcol(key, fmt):
            v = turn.get(key)
            if v is None or v == "":
                return ""
            try:
                return fmt % float(v)
            except (ValueError, TypeError):
                return str(v)

        self._tick_buf.append([
            self._i, f"{now:.4f}", f"{t_wall:.4f}",
            f"{dt_rep:.4f}", f"{dt_wall:.4f}",
            *[f"{v:.4f}" for v in raw],
            *[(f"{v:.4f}" if v is not None else "") for v in filt],
            *statuses,
            f"{gyro_z:+.4f}", f"{yaw_cmd:+.4f}", int(bool(stuck)),
            (f"{e_lat:+.4f}" if e_lat is not None else ""),
            (f"{e_front:+.4f}" if e_front is not None else ""),
            f"{steer:+.4f}",
            f"{L:+.3f}", f"{R:+.3f}", state,
            tcol("target", "%+.4f"), tcol("angle", "%+.4f"),
            tcol("error", "%+.4f"), tcol("elapsed", "%.3f"),
            tcol("progress", "%+.4f"), str(turn.get("abort", "")),
            str(lat_mode),
            f"{self._x:.4f}", f"{self._y:.4f}", f"{self._th:+.4f}",
            xt, yt, tht,
            json.dumps(extra or {}, separators=(",", ":")),
        ])
        self._i += 1
        if len(self._tick_buf) >= FLUSH_EVERY:
            self.flush()

    def event(self, frm, to, reason):
        now = time.monotonic()
        t_wall = (now - self._mono0) if self._mono0 else 0.0
        self._event_buf.append([self._i, f"{now:.4f}", f"{t_wall:.4f}",
                                frm, to, reason])
        if len(self._tick_buf) + len(self._event_buf) >= FLUSH_EVERY:
            self.flush()

    # -- lifecycle --------------------------------------------------------
    def flush(self):
        if self._tick_buf:
            self._ticks_w.writerows(self._tick_buf)
            self._tick_buf = []
        if self._event_buf:
            self._events_w.writerows(self._event_buf)
            self._event_buf = []
        self._ticks_f.flush()
        self._events_f.flush()
        try:
            os.fsync(self._ticks_f.fileno())
            os.fsync(self._events_f.fileno())
        except OSError:
            pass

    def close(self):
        try:
            self.flush()
        finally:
            self._ticks_f.close()
            self._events_f.close()
        print(f"[runlog] closed {self.run_dir} "
              f"({self._i} ticks)")
