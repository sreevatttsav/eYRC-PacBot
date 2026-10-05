"""Boilerplate for PB Task 1B.

Subscribes to the simulator's sensor topic, logs each reading, and publishes
a wheel velocity command back. Fill in your control logic where marked.

Run (three terminals):
    mosquitto
    ./task_1b_launch
    python3 task_1b_boilerplate.py --label baseline
"""
import argparse
import json
import math
import os
from collections import deque

import paho.mqtt.client as mqtt

from runlog import RunLogger
from sensing import StuckDetector, Tof

MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"      # simulator publishes, this file subscribes
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"  # this file publishes, simulator subscribes

# ---------------------------------------------------------------------------
# Controller tuning. All gains in (wheel rad/s) per unit error.
# Sign convention: steer > 0 = turn LEFT (right wheel faster).
#   left_vel  = base - steer
#   right_vel = base + steer
# ---------------------------------------------------------------------------
BASE_SPEED = 6.0       # cruise wheel speed (rad/s)
MAX_SPEED = 10.0       # hard clamp on each wheel
MAX_STEER = 4.0        # hard clamp on steer correction

KP_LAT = 3.0           # lateral centering: e_lat = sl - sr (meters)
KP_FRONT = 1.5         # heading trim: e_front = fl - fr (meters)
KD_YAW = 0.8           # gyro damping: -KD_YAW * yaw_rate

# Optional leaky-integral for steady-state bias. Default 0.0 = OFF (pure PD).
# You are right to be suspicious of it -- see note in update().
KI_LAT = 0.0
I_MAX = 0.5            # integrator clamp (rad/s)
I_LEAK = 0.995         # per-tick decay (<1.0 = leaky / forgetting)
I_DEADBAND = 0.005     # ignore |e_lat| below this (meters)

FRONT_SLOW_DIST = 0.5  # below this, start slowing down (meters)
FRONT_STOP_DIST = 0.15 # below this, spin in place
SPIN_SPEED = 3.0       # spin-in-place wheel speed

MAX_RANGE = 2.0        # clip ToF readings to this (meters)
FILTER_TAU = 0.05      # low-pass time constant for ToF (seconds)
TOF_HOLD_S = 0.1       # validity hold: coast through dropouts this long

# Single-wall hold target (m). Provisional: sl=sr~=0.275 at the start
# pose IF it began centered (unconfirmed -- Phase 1: verify, then fix).
WALL_TARGET = 0.275

# Wedge hysteresis (fix 4). Enter only after both sides persist low;
# exit only when both sides read high AND dwell has passed.
WEDGE_ENTER = 0.08
WEDGE_EXIT = 0.12
WEDGE_ENTER_T = 0.1
WEDGE_MIN_DWELL = 0.5
WEDGE_MAX_T = 2.0      # then hand off to stuck/give-up logic

# Gyro-terminated turns (fix 3, reworked Stage 1b). Turn rate now comes
# from the MEASURED yaw gain (step test), not wheel-size estimates.
YAW_GAIN_K = 0.0914   # from step_test s1_step summary.json
TURN_KP = 2.0          # P gain on angle error (rad/s per rad)
TURN_MIN_W = 1.5       # minimum wheel speed in a turn (rad/s, step test sets)
TURN_EXIT_ERR = 0.052  # 3 deg; coast adds ~1-2 deg -> final inside +/-5
TURN_TIE_DB = 0.05     # side-openness deadband: tie -> prev turn dir
DEADEND_DIST = 0.15    # both sides below this + blocked front = 180 deg
TURN_VERIFY_DIST = 0.20  # front clearance required after a turn
TURN_MAX_RETRIES = 2
TURN_TIMEOUT_BASE = 0.5    # hard timeout = BASE + PER_RAD*|target|
TURN_TIMEOUT_PER_RAD = 2.5  # 4.4 s per 90 deg, 8.4 s per 180 (provisional;
# NOTE: step_test t90@SPIN3 capped at the 2 s window, so the honest
# recompute (1.5*t90+0.5 = 3.5 s) waits on real s1_turns data. The
# no-progress watchdog is the real stall protection; this stays loose.
NO_PROGRESS_START = 0.5   # begin no-progress watch this far into a turn
NO_PROGRESS_WIN = 1.0     # abort if integrated angle gains < MIN in any window
NO_PROGRESS_MIN = 0.0873  # 5 deg
# Coast braking (step_test: 4-30 deg passive coast kills the +/-5 deg
# budget, so turns end with a closed-loop rotation stop, not a timer).
BRAKE_W = 2.0            # opposing wheel speed to stop rotation
BRAKE_EXIT_GYRO = 0.12   # stopped threshold (rad/s)
BRAKE_TIMEOUT = 0.5      # give up stopping after this (s)
RETRIM_TOL = 0.0873      # 5 deg: accept final error within this
OVERSHOOT_MAX = 0.14     # 8 deg past target: give up, don't re-trim

# Stuck detection + recovery (fix 2, retuned Stage 1b, rescaled for the
# MEASURED gain: 0.3 was calibrated at K=0.13; at K=0.0914 the same
# physical threshold (|R-L| > ~2.3 wheel rad/s sustained) is 0.21.
STUCK_WIN_S = 1.0
STUCK_CMD_MIN = 0.2    # |commanded yaw| must exceed this (rad/s)
STUCK_RATIO = 0.25     # |measured| below this fraction of commanded
STUCK_ESCALATE_S = 3.0  # re-flag within this -> escalate
RECOVER_DIST = 0.10    # back-out distance per recovery (m, estimated)

# Linear-speed layer (Stage 2). Cruise is commanded in m/s and
# converted with K_LIN. K_LIN = 0.017 is GROUND TRUTH from the sim
# binary's embedded MJCF (wheel geom size 0.017, direct hinge drive --
# see SIM_NOTES.md), NOT the speed_test fit: the maze-slot sweeps
# measure ray-sweep across walls (rotation contamination), giving
# 0.05-0.17. Straight-line rolling slip on flat floor is ~1-5%.
# Sanity anchor: yaw gain 0.0914 (measured) vs R/track = 0.218
# kinematic -- the sim loses ~58% in yaw (slip), translation unaffected.
K_LIN = 0.017          # m/s per wheel rad/s (binary MJCF wheel radius)
CRUISE_LINEAR_MPS = 0.04   # s2a cruise (refactor at explicit speed)
MAX_LINEAR_MPS = 0.12      # s2c ceiling (only if scoring rewards speed)

# Saturation model (Stage 2). SAT_MODE="ceiling": a reading at the cap
# is a lower bound ("at least 0.3"), never a distance. "distance":
# legacy numeric interpretation (s2b comparison run).
SENSOR_CAP_M = 0.300
SAT_MODE = "ceiling"   # or "distance"

# Exploration behaviors (maze1 post-mortem: the controller turned
# away from blockages but never INTO openings, so it walked past the
# entrance and cruised 20 m into the void).
FOLLOW_WALL_MAX = 0.25   # single-side numeric below this = following it
FOLLOW_ESTABLISH_S = 1.0  # sustained single-wall follow before gaps count
GAP_OPEN_S = 0.5         # follow wall lost this long -> seek it (90 deg)
BLIND_TURN_S = 15.0      # fully blind this long -> 180 deg turn-back;
# a second consecutive blind stretch latches HOLD (stop, don't wander).

# Heading-aware centering (Stage 3). The reference accumulates each
# COMPLETED turn's intended target (never the measured exit angle --
# that would bake the ~3 deg residual in). Illustration used KP 3.0 /
# 2.0; tuning starts low.
KP_HEADING = 1.0       # heading trim (wheel rad/s per rad), start low
STEER_HEADING_MAX = 1.0  # bound on the heading contribution
# Optional wall-slope re-anchor: heading ~= asin(side_rate / v).
# Needs K_LIN (Stage 2 done). Strict gates; FOLLOW-only so turns
# (entry-relative angles) are never disturbed.
REANCHOR_ENABLE = True
REANCHOR_V_MIN = 0.03    # need forward motion for a slope signal
REANCHOR_MAX_IMPLIED = 0.5  # ignore wild slopes (rad)
REANCHOR_CONSIST_S = 1.0    # consistent slope this long -> correct gyro_th

# Superseded by K_LIN/YAW_GAIN_K (kept for meta compat only).
WHEEL_R_EST = 0.017    # m
WHEEL_TRACK_EST = 0.08  # m

# Set by main(); None with --no-log.
LOGGER = None
_PREV_STATE = None
_FIRST_MSG = True
_DT_EMA = None       # wall-clock dt EWMA for sim/reported mismatch warn
_DT_TICKS = 0


def _dt_warn(dt_rep):
    """Warn (throttled) if wall-clock dt runs >20% above reported dt
    sustained -- i.e. the sim is slower than it claims (Phase 1 issue 3).
    One-sided deliberately: callbacks arriving faster than reported dt
    is burst/replay traffic, not a live mismatch."""
    global _DT_EMA, _DT_TICKS
    import time
    now = time.monotonic()
    if not hasattr(_dt_warn, "last"):
        _dt_warn.last = now
        return
    wall = now - _dt_warn.last
    _dt_warn.last = now
    if not (dt_rep and dt_rep > 0) or wall <= dt_rep or wall >= 1.0:
        return
    _DT_EMA = wall if _DT_EMA is None else 0.95 * _DT_EMA + 0.05 * wall
    _DT_TICKS += 1
    if _DT_TICKS % 250 == 0 and _DT_EMA > 1.20 * dt_rep:
        print(f"[warn] wall dt {_DT_EMA:.4f}s vs reported {dt_rep:.4f}s "
              f"(>20% slow) -- controller timers use sim-time; see Phase 2")


def _controller_constants():
    c = CenteringController
    return {
        "BASE_SPEED": BASE_SPEED, "MAX_SPEED": MAX_SPEED,
        "MAX_STEER": MAX_STEER, "KP_LAT": KP_LAT,
        "KP_FRONT": KP_FRONT, "KD_YAW": KD_YAW, "KI_LAT": KI_LAT,
        "I_MAX": I_MAX, "I_LEAK": I_LEAK, "I_DEADBAND": I_DEADBAND,
        "FRONT_SLOW_DIST": FRONT_SLOW_DIST,
        "FRONT_STOP_DIST": FRONT_STOP_DIST, "SPIN_SPEED": SPIN_SPEED,
        "K_LIN": K_LIN, "CRUISE_LINEAR_MPS": CRUISE_LINEAR_MPS,
        "MAX_LINEAR_MPS": MAX_LINEAR_MPS,
        "SENSOR_CAP_M": SENSOR_CAP_M, "SAT_MODE": SAT_MODE,
        "FOLLOW_WALL_MAX": FOLLOW_WALL_MAX,
        "FOLLOW_ESTABLISH_S": FOLLOW_ESTABLISH_S,
        "GAP_OPEN_S": GAP_OPEN_S, "BLIND_TURN_S": BLIND_TURN_S,
        "KP_HEADING": KP_HEADING, "STEER_HEADING_MAX": STEER_HEADING_MAX,
        "REANCHOR_ENABLE": REANCHOR_ENABLE,
        "REANCHOR_V_MIN": REANCHOR_V_MIN,
        "REANCHOR_MAX_IMPLIED": REANCHOR_MAX_IMPLIED,
        "REANCHOR_CONSIST_S": REANCHOR_CONSIST_S,
        "MAX_RANGE": MAX_RANGE, "FILTER_TAU": FILTER_TAU,
        "TOF_HOLD_S": TOF_HOLD_S, "WALL_TARGET": WALL_TARGET,
        "WEDGE_ENTER": WEDGE_ENTER, "WEDGE_EXIT": WEDGE_EXIT,
        "WEDGE_ENTER_T": WEDGE_ENTER_T, "WEDGE_MIN_DWELL": WEDGE_MIN_DWELL,
        "WEDGE_MAX_T": WEDGE_MAX_T,
        "TURN_KP": TURN_KP, "TURN_MIN_W": TURN_MIN_W,
        "TURN_EXIT_ERR": TURN_EXIT_ERR,
        "TURN_TIE_DB": TURN_TIE_DB, "DEADEND_DIST": DEADEND_DIST,
        "TURN_VERIFY_DIST": TURN_VERIFY_DIST,
        "TURN_MAX_RETRIES": TURN_MAX_RETRIES,
        "TURN_TIMEOUT_BASE": TURN_TIMEOUT_BASE,
        "TURN_TIMEOUT_PER_RAD": TURN_TIMEOUT_PER_RAD,
        "NO_PROGRESS_START": NO_PROGRESS_START,
        "NO_PROGRESS_WIN": NO_PROGRESS_WIN,
        "NO_PROGRESS_MIN": NO_PROGRESS_MIN,
        "BRAKE_W": BRAKE_W, "BRAKE_EXIT_GYRO": BRAKE_EXIT_GYRO,
        "BRAKE_TIMEOUT": BRAKE_TIMEOUT, "RETRIM_TOL": RETRIM_TOL,
        "OVERSHOOT_MAX": OVERSHOOT_MAX,
        "YAW_GAIN_K": YAW_GAIN_K,
        "STUCK_WIN_S": STUCK_WIN_S, "STUCK_CMD_MIN": STUCK_CMD_MIN,
        "STUCK_RATIO": STUCK_RATIO, "STUCK_ESCALATE_S": STUCK_ESCALATE_S,
        "RECOVER_DIST": RECOVER_DIST,
        "BACKUP_S": c.BACKUP_S, "REVERSE_S": c.REVERSE_S,
        "RESUME_DIST": c.RESUME_DIST, "MIN_SPIN_S": c.MIN_SPIN_S,
        "WHEEL_R_EST": WHEEL_R_EST, "WHEEL_TRACK_EST": WHEEL_TRACK_EST,
    }


class CenteringController:
    """PD lateral + front-alignment P + gyro D. No raw I by default."""

    def __init__(self):
        # Validity-filtered sensors (fix 1 + Stage 2 sat). Values are
        # None = unknown; status "sat" = lower bound, never differenced.
        cap = SENSOR_CAP_M if SAT_MODE == "ceiling" else None
        self.tof_fl = Tof(MAX_RANGE, TOF_HOLD_S, FILTER_TAU, cap)
        self.tof_fr = Tof(MAX_RANGE, TOF_HOLD_S, FILTER_TAU, cap)
        self.tof_sl = Tof(MAX_RANGE, TOF_HOLD_S, FILTER_TAU, cap)
        self.tof_sr = Tof(MAX_RANGE, TOF_HOLD_S, FILTER_TAU, cap)
        self.lat_mode = ""
        self.fl_f = None
        self.fr_f = None
        self.sl_f = None
        self.sr_f = None
        self.last_status = ("none",) * 4
        self.i_lat = 0.0
        self.t = 0.0              # controller clock (sim-time, from dt)
        self.gyro_th = 0.0        # integrated heading (turn termination)
        # Heading reference (Stage 3): accumulates intended turn targets.
        self.heading_ref = 0.0
        self.e_heading = 0.0
        self.steer_lat = 0.0
        self.steer_front = 0.0
        self.steer_heading = 0.0
        # wall-slope re-anchor state (Stage 3): 1 s regression buffers
        self._ra_sl = deque()
        self._ra_sr = deque()
        self.reanchor_mag = None  # last correction (for log extra)
        # Escape state
        self.spin_dir = 0.0
        self.backup_ticks = 0
        self.reverse_ticks = 0
        self.flip_next = False   # after a give-up, try the other way first
        self.spin_done_s = 0.0   # committed turn executed this escape
        # Gyro turn state (fix 3, Stage 1b)
        self.turn_active = False
        self.turn_entry = 0.0
        self.turn_target = 0.0   # signed radians (may become a residual)
        self.turn_orig = 0.0     # intended target (heading_ref uses this)
        self.turn_t = 0.0
        self.turn_retries = 0
        self.turn_reversals = 0  # direction flips (cap 1, near target only)
        self.prev_turn_dir = 1.0  # tie-break default: left (fixed rule)
        self.retry_dir = None    # hard-timeout retry: force same dir once
        self.turn_angle = 0.0    # exposed for turn logging
        self.turn_error = 0.0
        self.retry_used = False
        self._from_backup = False
        self.np_t0 = 0.0         # no-progress window start (turn-time)
        self.np_a0 = 0.0         # no-progress window start (angle)
        self.turn_progress = 0.0  # angle gained in current 1 s window
        self.abort_reason = ""   # last turn abort: "", timeout, no_progress
        self.brake_until = None  # turn_t deadline for the BRAKE state
        self.brake_w = 0.0       # signed opposing wheel speed in BRAKE
        self.turn_cause = ""     # why this turn: "", "gap", "lost"
        # Exploration state (maze1 pack): wall follow + blind driving.
        self.follow_side = 0.0   # +1 left / -1 right / 0 none latched
        self.follow_t = 0.0
        self.gap_t = 0.0
        self.last_seen_l = -1e9  # last controller-t with a close left wall
        self.last_seen_r = -1e9
        self.blind_t = 0.0
        self.void_count = 0      # consecutive blind stretches
        self.hold = False        # latched HOLD (parked, see section 5)
        # Wedge hysteresis state (fix 4)
        self.wedge_active = False
        self.wedge_below_t = 0.0
        self.wedge_t = 0.0
        # Stuck recovery state (fix 2)
        self.stuck = StuckDetector(STUCK_WIN_S, STUCK_CMD_MIN, STUCK_RATIO)
        self.stuck_flag = False
        self._stuck_prev = False
        self.last_stuck_clear_t = -1e9
        self.stuck_episodes = 0
        self.recover_active = False
        self.recover_target = RECOVER_DIST
        self.recover_done = 0.0
        self.recover_then_backup = False
        self.yaw_cmd = 0.0
        self.state = "FOLLOW"    # exposed for runlog.py event logging
        self.state_reason = "init"

    # escape-maneuver tuning
    BACKUP_S = 0.5         # how long to reverse before retrying
    REVERSE_S = 0.4        # reverse before rotating once blocked
    RESUME_DIST = 0.20     # front clearance needed to exit escape
    MIN_SPIN_S = 0.6       # committed turn before a clear front may exit spin


    def _set_state(self, state, reason):
        self.state = state
        self.state_reason = reason

    def _start_turn(self, tdir, mag, cause):
        """Arm a turn: fresh-block, gap-seek, and lost-turn entries all
        funnel here so target/retries/reason stay consistent. The TURN
        P servo initializes on the next tick (turn_active False). A
        pre-reverse is the caller's job (escape needs room, gap/lost
        turns start from open space and skip it)."""
        self.spin_dir = tdir
        self.turn_target = tdir * mag
        self.turn_orig = tdir * mag
        self.turn_active = False
        self.turn_retries = 0
        self.turn_reversals = 0
        self.turn_t = 0.0
        self.abort_reason = ""
        self.brake_until = None
        self.reverse_ticks = 0
        self.spin_done_s = 0.0
        self.turn_cause = cause
        self.spin_done_s = 0.0  # new escape -> new committed turn
        self.follow_side = 0.0
        self.follow_t = 0.0
        self.gap_t = 0.0

    @staticmethod
    def _wrap(a):
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def _reanchor(self, dt, v):
        """Wall-slope re-anchor (Stage 3, optional). A valid unsaturated
        side wall gives heading ~= asin(side_rate / v), with the rate
        from a least-squares fit over a 1 s buffer (consecutive
        differences are pure ToF noise at 50 Hz). After a consistent
        window, snap gyro_th toward the implied heading.
        FOLLOW-tail only (no active turn: entry-relative turn math is
        never disturbed). Corrections below ~4.6 deg are ignored: slope
        noise alone reads ~3 deg, while real drift is unbounded and
        always grows past the gate."""
        self.reanchor_mag = None
        if (not REANCHOR_ENABLE or dt <= 0 or v < REANCHOR_V_MIN):
            self._ra_sl.clear()
            self._ra_sr.clear()
            return
        if (self.sl_f is not None
                and self.last_status[2] in ("valid", "held")):
            self._ra_sl.append((self.t, self.sl_f))
        if (self.sr_f is not None
                and self.last_status[3] in ("valid", "held")):
            self._ra_sr.append((self.t, self.sr_f))
        for buf in (self._ra_sl, self._ra_sr):
            while buf and self.t - buf[0][0] > REANCHOR_CONSIST_S:
                buf.popleft()
        implied = []
        for buf, sgn in ((self._ra_sl, -1.0), (self._ra_sr, 1.0)):
            s = self._slope(buf)
            if s is not None:
                implied.append(math.asin(max(-1.0, min(sgn * s / v, 1.0))))
        if not implied:
            return
        if len(implied) == 2 and implied[0] * implied[1] < 0:
            return  # walls disagree: no signal (buffers keep filling)
        avg = sum(implied) / len(implied)
        if abs(avg) > REANCHOR_MAX_IMPLIED:
            return
        # implied lives in the CURRENT corridor frame: compare against
        # (gyro_th - ref), ~0 when parallel. (Bare gyro_th would snap
        # to disaster after the first turn.)
        corr = self._wrap(avg - self._wrap(self.gyro_th - self.heading_ref))
        # Gate at ~4.6 deg: EMA-correlated ToF noise still pushes fitted
        # slopes to ~3 deg tails, so anything smaller is likely noise.
        # Drift is unbounded and start crookedness is typically larger,
        # so real errors always grow past this gate.
        if abs(corr) > 0.08:
            self.gyro_th += corr
            self.reanchor_mag = corr
            self._ra_sl.clear()
            self._ra_sr.clear()
            self._ra_sl.clear()
            self._ra_sr.clear()

    @staticmethod
    def _slope(buf):
        """Least-squares d(val)/dt over buffer; None if <10 samples or
        span < 0.8 s (not enough baseline to beat ToF noise)."""
        if len(buf) < 10 or buf[-1][0] - buf[0][0] < 0.8:
            return None
        n = len(buf)
        mt = sum(t for t, _ in buf) / n
        mv = sum(v for _, v in buf) / n
        den = sum((t - mt) ** 2 for t, _ in buf)
        if den <= 0:
            return None
        return sum((t - mt) * (v - mv) for t, v in buf) / den

    def _finalize(self, L, R, e_lat, e_front, steer, yaw_rate, dt):
        """Single exit path: runs stuck detection (fix 2) on the final
        command. A rising stuck edge during any maneuver switches this
        tick's output to stuck recovery (distance-driven reverse).
        The edge is blanked during turn spin-up (turn_t < 1 window):
        a fixed 1 s window cannot judge a step change until it fills
        with post-step data, and the flag still logs for analysis."""
        yaw_cmd = YAW_GAIN_K * (R - L)
        self.yaw_cmd = yaw_cmd
        stuck_now = self.stuck.update(yaw_cmd, yaw_rate, dt)
        self.stuck_flag = stuck_now
        # Blanking: fixed 1 s windows cannot judge a step change until
        # full (turn spin-up), and BRAKE is open-loop by design (its own
        # timeout + angle check handle a stuck brake). The flag still
        # logs for analysis either way.
        blanked = ((self.state == "TURN" and self.turn_t < STUCK_WIN_S)
                   or self.state == "BRAKE")
        if stuck_now and not self._stuck_prev and not blanked:
            L, R = self._on_stuck(L, R)
        if not stuck_now and self._stuck_prev:
            self.last_stuck_clear_t = self.t
        self._stuck_prev = stuck_now
        return L, R, e_lat, e_front, steer

    def _on_stuck(self, L, R):
        gap = self.t - self.last_stuck_clear_t
        self.stuck_episodes = self.stuck_episodes + 1 if gap < STUCK_ESCALATE_S else 1
        if self.state in ("FOLLOW", "REVERSE", "TURN", "BACKUP", "WEDGE",
                          "RECOVER"):
            if self.stuck_episodes >= 2:
                self.recover_target = RECOVER_DIST * 2.0
                self.recover_then_backup = True
            else:
                self.recover_target = RECOVER_DIST
                self.recover_then_backup = False
            self.recover_active = True
            self.recover_done = 0.0
            self._set_state("RECOVER", "stuck")
            back = BASE_SPEED * 0.5
            return -back, -back
        return L, R

    @staticmethod
    def _clip_range(v):
        if v is None or v != v:  # None / NaN -> treat as no wall
            return MAX_RANGE
        return max(0.0, min(float(v), MAX_RANGE))

    def _filter(self, prev, new, dt):
        alpha = dt / (FILTER_TAU + dt) if dt > 0 else 1.0
        if prev is None:
            return new
        return (1.0 - alpha) * prev + alpha * new

    def update(self, fl, fr, sl, sr, yaw_rate, dt):
        dt = dt if dt and dt > 0 else 0.0
        # 1. Validity filter (fix 1). Each sensor is valid / held / none.
        #    NOTE: a reading of exactly ~0.300 may be a saturation ceiling
        #    ("at least 0.3") rather than a true distance -- the Task 1B
        #    spec has to settle this. Until then it is treated as valid
        #    but never differenced blindly: single-side logic below only
        #    trusts a reading clearly below the ceiling.
        self.fl_f, fl_s = self.tof_fl.update(fl, dt)
        self.fr_f, fr_s = self.tof_fr.update(fr, dt)
        self.sl_f, sl_s = self.tof_sl.update(sl, dt)
        self.sr_f, sr_s = self.tof_sr.update(sr, dt)
        self.last_status = (fl_s, fr_s, sl_s, sr_s)

        # 2. Errors, validity-gated (Stage 2 sat rules). "numeric" =
        #    valid/held; "sat" is a bound, never differenced. Both sides
        #    numeric -> e_lat. One numeric side -> hold WALL_TARGET from
        #    it. Both sat -> blind (NOT centered): steer 0. e_front needs
        #    both front numeric.
        def _num(v, s):
            return v is not None and s in ("valid", "held")

        sl_num = _num(self.sl_f, sl_s)
        sr_num = _num(self.sr_f, sr_s)
        fl_num = _num(self.fl_f, fl_s)
        fr_num = _num(self.fr_f, fr_s)
        e_lat = None
        e_front = None
        single = None  # (+1 side sign, reading) for single-wall hold
        if sl_num and sr_num:
            e_lat = self.sl_f - self.sr_f
            lat_mode = "both"
        elif sl_num:
            single = (1.0, self.sl_f)
            lat_mode = "left_only"
        elif sr_num:
            single = (-1.0, self.sr_f)
            lat_mode = "right_only"
        else:
            lat_mode = "blind"
        self.lat_mode = lat_mode
        if fl_num and fr_num:
            e_front = self.fl_f - self.fr_f
        # Wall memory for the tie-break (maze1 pack): last time each
        # side showed a close wall. Walls are information, void is not.
        if sl_num and self.sl_f < FOLLOW_WALL_MAX:
            self.last_seen_l = self.t
        if sr_num and self.sr_f < FOLLOW_WALL_MAX:
            self.last_seen_r = self.t

        # 3. Leaky integral (OFF by default). Same cautions as before;
        #    additionally gated on e_lat being known.
        if KI_LAT > 0.0 and e_lat is not None and abs(e_lat) > I_DEADBAND:
            self.i_lat = self.i_lat * I_LEAK + e_lat * dt
            self.i_lat = max(-I_MAX, min(self.i_lat, I_MAX))
        else:
            self.i_lat *= I_LEAK

        # 4. Steer composition, by component (Stage 3 logs each).
        #    Single-wall hold: too far from the left wall (sl > target)
        #    steers left (+), and symmetrically right. Heading trim is
        #    added in the FOLLOW tail only (turns own their wheels).
        steer_lat = 0.0
        if e_lat is not None:
            steer_lat = KP_LAT * e_lat
        elif single is not None:
            side, reading = single
            steer_lat = side * KP_LAT * (reading - WALL_TARGET)
        steer_front = KP_FRONT * e_front if e_front is not None else 0.0
        steer = steer_lat + steer_front + KI_LAT * self.i_lat - KD_YAW * yaw_rate
        steer = max(-MAX_STEER, min(steer, MAX_STEER))
        self.steer_lat = steer_lat
        self.steer_front = steer_front
        self.steer_heading = 0.0
        self.e_heading = 0.0

        # 5. Longitudinal + escape. States: FOLLOW / REVERSE / TURN /
        #    BACKUP / WEDGE / RECOVER / HOLD. Every return goes through
        #    _finalize() for stuck detection.
        self.t += dt
        self.gyro_th += yaw_rate * dt

        # --- HOLD latch (anti-void terminal state): parked, zeros out.
        #     Nothing in the maze moves to us, so this never releases;
        #     the run is over for scoring and the operator intervenes.
        if self.hold:
            self._set_state("HOLD", "lost_hold")
            return self._finalize(0.0, 0.0, e_lat, e_front,
                                  steer, yaw_rate, dt)

        # front_clear uses every available reading; a sat side reads the
        # cap (open). Only a NUMERIC reading below STOP blocks -- a sat
        # side never blocks.
        front_vals = [v for v in (self.fl_f, self.fr_f) if v is not None]
        front_clear = min(front_vals) if front_vals else None
        front_known = fl_num and fr_num
        blocked = any(v < FRONT_STOP_DIST
                      for v, s in ((self.fl_f, fl_s), (self.fr_f, fr_s))
                      if v is not None and s in ("valid", "held"))

        # --- stuck recovery finishes first (distance-driven backout) ---
        if self.recover_active:
            back = BASE_SPEED * 0.5
            self.recover_done += abs(K_LIN * back) * dt
            if self.recover_done >= self.recover_target:
                self.recover_active = False
                if self.recover_then_backup:
                    self.recover_then_backup = False
                    self.backup_ticks = max(1, int(self.BACKUP_S / dt)) if dt > 0 else 250
                    self.spin_dir = 0.0
                    self.turn_active = False
                    self.flip_next = True
                    self._set_state("BACKUP", "stuck_escalate")
                    return self._finalize(-back, -back, e_lat, e_front,
                                          steer, yaw_rate, dt)
                self.spin_dir = 0.0  # force a fresh decision below
                self.turn_active = False
                self._set_state("FOLLOW", "recover_done")
                # fall through to normal logic
            else:
                self._set_state("RECOVER", "stuck")
                return self._finalize(-back, -back, e_lat, e_front,
                                      steer, yaw_rate, dt)

        # --- wedge hysteresis (fix 4): enter after 100 ms both-low, exit
        #     only when both-high AND 0.5 s dwell has passed ---
        both_below = (self.sl_f is not None and self.sr_f is not None
                      and self.sl_f < WEDGE_ENTER and self.sr_f < WEDGE_ENTER)
        if not self.wedge_active:
            self.wedge_below_t = self.wedge_below_t + dt if both_below else 0.0
            if (both_below and self.wedge_below_t >= WEDGE_ENTER_T
                    and self.spin_dir == 0.0):
                self.wedge_active = True
                self.wedge_t = 0.0
        if self.wedge_active:
            self.wedge_t += dt
            both_above = (self.sl_f is not None and self.sr_f is not None
                          and self.sl_f > WEDGE_EXIT and self.sr_f > WEDGE_EXIT)
            if both_above and self.wedge_t >= WEDGE_MIN_DWELL:
                self.wedge_active = False
                self.wedge_below_t = 0.0
                # fall through to normal logic
            elif self.wedge_t > WEDGE_MAX_T:
                # doesn't clear -> hand off to give-up backup + flip
                self.wedge_active = False
                self.wedge_below_t = 0.0
                self.backup_ticks = max(1, int(self.BACKUP_S / dt)) if dt > 0 else 250
                self.spin_dir = 0.0
                self.turn_active = False
                self.flip_next = True
                self._set_state("BACKUP", "wedge_giveup")
                back = BASE_SPEED * 0.5
                return self._finalize(-back, -back, e_lat, e_front,
                                      steer, yaw_rate, dt)
            else:
                self._set_state("WEDGE", "wedge_enter")
                back = BASE_SPEED * 0.5
                # same steer mixing as forward: yaws the nose the same way
                wl = max(-MAX_SPEED, min(-back - steer * 0.5, MAX_SPEED))
                wr = max(-MAX_SPEED, min(-back + steer * 0.5, MAX_SPEED))
                return self._finalize(wl, wr, e_lat, e_front,
                                      steer, yaw_rate, dt)

        # Exit escape once the front is clear again -- but NOT while a
        # reverse/backup countdown is still running (those phases create
        # clearance on purpose), NOT before the committed minimum turn
        # has executed, and NEVER mid-turn: an active TURN owns its exit
        # (gyro angle + front verify). Letting a clear front kill the turn
        # is what ended the logged turn at 57° instead of 90°.
        # Unknown front must NOT reset anything.
        in_maneuver = (self.reverse_ticks > 0 or self.backup_ticks > 0
                       or self.wedge_active)
        spin_committed = (self.spin_dir != 0.0
                          and self.spin_done_s < self.MIN_SPIN_S)
        if (front_clear is not None and front_clear > self.RESUME_DIST
                and not in_maneuver and not spin_committed
                and not self.turn_active):
            self.spin_dir = 0.0
            self.turn_active = False

        if self.backup_ticks > 0:
            self.backup_ticks -= 1
            self._from_backup = True  # next fresh block is a retry, not new
            self._set_state("BACKUP", "giveup")
            back = BASE_SPEED * 0.5
            return self._finalize(-back, -back, e_lat, e_front,
                                  steer, yaw_rate, dt)

        if blocked or self.spin_dir != 0.0:
            if self.spin_dir == 0.0 and self.reverse_ticks == 0:
                # fresh block: pick the turn, reverse first for room.
                # A stale same-dir retry from an older obstacle is dropped
                # unless we came straight out of BACKUP.
                if not self._from_backup:
                    self.retry_dir = None
                    self.retry_used = False
                self._from_backup = False
                deadend = (blocked and sl_num and sr_num
                           and self.sl_f < DEADEND_DIST
                           and self.sr_f < DEADEND_DIST)
                mag = math.pi if deadend else math.pi / 2.0
                if self.retry_dir is not None:
                    tdir = self.retry_dir  # hard-timeout retry: same dir
                    self.retry_dir = None
                elif (self.sl_f is not None and self.sr_f is not None
                        and abs(self.sl_f - self.sr_f) > TURN_TIE_DB):
                    tdir = 1.0 if self.sl_f > self.sr_f else -1.0
                elif self.last_seen_l == self.last_seen_r:
                    tdir = self.prev_turn_dir  # never saw a wall: fixed rule
                else:
                    # tie: turn toward the most recently seen wall.
                    # Walls are information, void is not (maze1: the NW
                    # corner tie went west into the void; east had the wall).
                    tdir = (1.0 if self.last_seen_l > self.last_seen_r
                            else -1.0)
                self.spin_dir = tdir
                if self.flip_next:
                    self.spin_dir = -self.spin_dir
                    tdir = self.spin_dir
                    self.flip_next = False
                self._start_turn(tdir, mag, "")
                self.reverse_ticks = max(1, int(self.REVERSE_S / dt)) if dt > 0 else 200
            if self.reverse_ticks > 0:
                self.reverse_ticks -= 1
                self._set_state("REVERSE", "blocked")
                back = BASE_SPEED * 0.5
                return self._finalize(-back, -back, e_lat, e_front,
                                      steer, yaw_rate, dt)
            # --- gyro-terminated turn (fix 3, Stage 1b) + coast brake:
            #     P servo on integrated angle; at |err| <= 3 deg stop the
            #     rotation closed-loop (BRAKE) instead of hoping the ~4-30
            #     deg passive coast lands inside tolerance. ---
            if not self.turn_active:
                self.turn_active = True
                self.turn_entry = self.gyro_th
                self.turn_t = 0.0
                self.np_t0 = 0.0
                self.np_a0 = 0.0
                self.turn_progress = 0.0
            self.turn_t += dt
            turned = self.gyro_th - self.turn_entry
            err = self.turn_target - turned
            finishing = False
            if self.brake_until is not None:
                # in BRAKE: hold opposition until stopped or timed out
                if (abs(yaw_rate) < BRAKE_EXIT_GYRO
                        or self.turn_t > self.brake_until):
                    err = self.turn_target - (self.gyro_th - self.turn_entry)
                    self.brake_until = None
                    finishing = True
                else:
                    w = self.brake_w
                    self.spin_done_s += dt
                    self.turn_angle = turned
                    self.turn_error = err
                    self._set_state("BRAKE", "coast")
                    return self._finalize(-w, w, e_lat, e_front,
                                          steer, yaw_rate, dt)
            timeout = TURN_TIMEOUT_BASE + TURN_TIMEOUT_PER_RAD * abs(self.turn_target)
            if not finishing and self.turn_t > timeout:
                # hard timeout: retry once same dir, then flip
                self.backup_ticks = max(1, int(self.BACKUP_S / dt)) if dt > 0 else 250
                self.spin_dir = 0.0
                self.turn_active = False
                if not self.retry_used:
                    self.retry_used = True
                    self.retry_dir = 1.0 if self.turn_target > 0 else -1.0
                else:
                    self.retry_used = False
                    self.retry_dir = None
                    self.flip_next = True
                self.abort_reason = "hard_timeout"
                self._set_state("BACKUP", "turn_timeout")
                back = BASE_SPEED * 0.5
                return self._finalize(-back, -back, e_lat, e_front,
                                      steer, yaw_rate, dt)
            if not finishing and self.turn_t >= NO_PROGRESS_START:
                if self.turn_t - self.np_t0 >= NO_PROGRESS_WIN:
                    self.turn_progress = turned - self.np_a0
                    if abs(self.turn_progress) < NO_PROGRESS_MIN:
                        # pinned side: reverse out and flip direction
                        self.backup_ticks = max(1, int(self.BACKUP_S / dt)) if dt > 0 else 250
                        self.spin_dir = 0.0
                        self.turn_active = False
                        self.retry_used = False
                        self.retry_dir = None
                        self.flip_next = True
                        self.abort_reason = "no_progress"
                        self._set_state("BACKUP", "no_progress")
                        back = BASE_SPEED * 0.5
                        return self._finalize(-back, -back, e_lat, e_front,
                                              steer, yaw_rate, dt)
                    self.np_t0 = self.turn_t
                    self.np_a0 = turned
            if not finishing and abs(err) <= TURN_EXIT_ERR:
                if abs(yaw_rate) < BRAKE_EXIT_GYRO:
                    finishing = True  # already slow: no brake needed
                else:
                    self.brake_until = self.turn_t + BRAKE_TIMEOUT
                    self.brake_w = -math.copysign(
                        BRAKE_W, yaw_rate if yaw_rate != 0.0 else err)
                    self._set_state("BRAKE", "coast")
                    w = self.brake_w
                    return self._finalize(-w, w, e_lat, e_front,
                                          steer, yaw_rate, dt)
            if finishing:
                verified = (front_clear is None
                            or front_clear > TURN_VERIFY_DIST)
                if verified:
                    if abs(err) <= RETRIM_TOL:
                        self.prev_turn_dir = 1.0 if self.turn_orig > 0 else -1.0
                        self.heading_ref += self.turn_orig
                        self.spin_dir = 0.0
                        self.turn_active = False
                        self.retry_used = False  # completed: no flip
                        self.retry_dir = None
                        self._set_state("FOLLOW", "turn_done" if front_known
                                        else "turn_done_unverified")
                        # fall through to FOLLOW tail below
                    else:
                        # stopped off-target: re-trim the residual
                        flipped = (err > 0) != (self.turn_orig > 0)
                        if flipped:
                            self.turn_reversals += 1
                        if (self.turn_reversals > 1
                                or (flipped and abs(err) > OVERSHOOT_MAX)):
                            self.backup_ticks = max(1, int(self.BACKUP_S / dt)) if dt > 0 else 250
                            self.spin_dir = 0.0
                            self.turn_active = False
                            self.flip_next = True
                            self.abort_reason = "overshoot"
                            self._set_state("BACKUP", "overshoot")
                            back = BASE_SPEED * 0.5
                            return self._finalize(-back, -back, e_lat, e_front,
                                                  steer, yaw_rate, dt)
                        self.turn_retries += 1
                        if self.turn_retries > TURN_MAX_RETRIES:
                            self.backup_ticks = max(1, int(self.BACKUP_S / dt)) if dt > 0 else 250
                            self.spin_dir = 0.0
                            self.turn_active = False
                            self.flip_next = True
                            self.abort_reason = "retrim_exhausted"
                            self._set_state("BACKUP", "retrim_exhausted")
                            back = BASE_SPEED * 0.5
                            return self._finalize(-back, -back, e_lat, e_front,
                                                  steer, yaw_rate, dt)
                        self.turn_target = err  # residual (signed)
                        self.turn_entry = self.gyro_th
                        self.turn_t = 0.0
                        self.np_t0 = 0.0
                        self.np_a0 = 0.0
                        self.turn_progress = 0.0
                        # fall into the P servo below with fresh err
                else:
                    self.turn_retries += 1
                    if self.turn_retries > TURN_MAX_RETRIES:
                        self.backup_ticks = max(1, int(self.BACKUP_S / dt)) if dt > 0 else 250
                        self.spin_dir = 0.0
                        self.turn_active = False
                        self.flip_next = True
                        self._set_state("BACKUP", "turn_unverified")
                        back = BASE_SPEED * 0.5
                        return self._finalize(-back, -back, e_lat, e_front,
                                              steer, yaw_rate, dt)
                    self.turn_entry = self.gyro_th  # retry same target
                    self.turn_t = 0.0
                    self.np_t0 = 0.0
                    self.np_a0 = 0.0
                    self.turn_progress = 0.0
                    err = self.turn_target
            if self.turn_active:
                w = TURN_KP * err
                wmag = min(SPIN_SPEED, max(TURN_MIN_W, abs(w)))
                w = math.copysign(wmag, err) if err != 0.0 else 0.0
                self.spin_done_s += dt
                self.turn_angle = turned   # exposed for turn logging
                self.turn_error = err
                side = "left" if self.turn_target > 0 else "right"
                self._set_state("TURN", (self.turn_cause + "_" if self.turn_cause else "") + side)
                return self._finalize(-w, w, e_lat, e_front,
                                      steer, yaw_rate, dt)
        # FOLLOW tail (also reached after a completed turn). Speed is
        # commanded in m/s and converted with K_LIN. Slowdown ramps
        # within [STOP, CAP-0.02]; a saturated front reads the cap, i.e.
        # full cruise. Unknown front -> cautious half speed.
        if front_clear is not None and not blocked:
            span = (SENSOR_CAP_M - 0.02) - FRONT_STOP_DIST
            scale = (front_clear - FRONT_STOP_DIST) / span if span > 0 else 1.0
            scale = max(0.25, min(1.0, scale))
            v_cmd = CRUISE_LINEAR_MPS * scale
        elif front_clear is None:
            v_cmd = CRUISE_LINEAR_MPS * 0.5
        else:
            v_cmd = CRUISE_LINEAR_MPS * 0.25
        v_cmd = min(v_cmd, MAX_LINEAR_MPS)
        base = v_cmd / K_LIN

        # --- wall follow + gap-seek (maze1 pack): hug a single wall;
        #     when the latched wall opens with front clear, turn INTO it
        #     (the entrance announces itself exactly this way).
        #     Corridors (both sides close) never trigger: drive past.
        L_close = sl_num and self.sl_f < FOLLOW_WALL_MAX
        R_close = sr_num and self.sr_f < FOLLOW_WALL_MAX
        if L_close and R_close:
            self.follow_side = 0.0
            self.follow_t = 0.0
            self.gap_t = 0.0
        elif L_close != R_close:
            side = 1.0 if L_close else -1.0
            if side != self.follow_side:
                self.follow_side = side
                self.follow_t = 0.0
                self.gap_t = 0.0
            else:
                self.follow_t += dt
                self.gap_t = 0.0  # wall present this tick
        elif self.follow_side != 0.0:
            # both open, previously following: wall possibly lost
            self.gap_t += dt
            if (self.follow_t >= FOLLOW_ESTABLISH_S
                    and self.gap_t >= GAP_OPEN_S and not blocked):
                tdir = self.follow_side
                self._start_turn(tdir, math.pi / 2.0, "gap")
                self._set_state("TURN", "gap_" + ("left" if tdir > 0 else "right"))
                return self._finalize(0.0, 0.0, e_lat, e_front,
                                      steer, yaw_rate, dt)
        # NOTE: no latch at all (follow_side == 0, both open) -> nothing
        # to seek; the void logic below owns that case.

        # --- anti-void (maze1 pack): fully blind driving accumulates;
        #     at 15 s turn 180 deg back toward last readings; a second
        #     consecutive blind stretch latches HOLD (park, don't wander).
        #     In the FOLLOW tail blocked is always False by construction.
        front_open = front_clear is None or front_clear > self.RESUME_DIST
        if self.lat_mode == "blind" and front_open:
            self.blind_t += dt
            if self.blind_t >= BLIND_TURN_S:
                self.blind_t = 0.0
                self.void_count += 1
                if self.void_count >= 2:
                    self.hold = True
                    self._set_state("HOLD", "lost_hold")
                    return self._finalize(0.0, 0.0, e_lat, e_front,
                                          steer, yaw_rate, dt)
                self._start_turn(self.prev_turn_dir, math.pi, "lost")
                self._set_state("TURN", "lost_" + ("left" if self.prev_turn_dir > 0 else "right"))
                return self._finalize(0.0, 0.0, e_lat, e_front,
                                      steer, yaw_rate, dt)
        else:
            self.blind_t = 0.0
            self.void_count = 0

        # Heading trim (Stage 3). e_heading closes heading_ref against
        # integrated gyro; bounded contribution, FOLLOW-only (turns own
        # their wheels, maneuvers ignore it).
        self.e_heading = self._wrap(self.heading_ref - self.gyro_th)
        self.steer_heading = max(-STEER_HEADING_MAX,
                                 min(KP_HEADING * self.e_heading,
                                     STEER_HEADING_MAX))
        steer = max(-MAX_STEER, min(steer + self.steer_heading, MAX_STEER))
        self._reanchor(dt, v_cmd)

        left_vel = base - steer
        right_vel = base + steer
        left_vel = max(-MAX_SPEED, min(left_vel, MAX_SPEED))
        right_vel = max(-MAX_SPEED, min(right_vel, MAX_SPEED))
        self._from_backup = False  # driving: next block is a new obstacle
        self._set_state("FOLLOW", "clear" if (
            front_clear is not None
            and front_clear > SENSOR_CAP_M - 0.02) else "approach")
        return self._finalize(left_vel, right_vel, e_lat, e_front,
                              steer, yaw_rate, dt)


CONTROLLER = CenteringController()


def _mqtt_client():
    # paho-mqtt >= 2.0 requires picking a callback API version explicitly.
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()


def on_message(client, userdata, msg):
    global _PREV_STATE, _FIRST_MSG
    data = json.loads(msg.payload.decode())

    if _FIRST_MSG:
        _FIRST_MSG = False
        print(f"[runlog] sensor keys: {sorted(data.keys())}")
        print("[runlog] tip: run `mosquitto_sub -t '#' -v` alongside "
              "to catch pose/encoder topics the controller ignores")

    fl = data["fl"]            # Front-left ToF distance readings
    fr = data["fr"]            # Front-right ToF distance readings
    sl = data["sl"]            # Side-left ToF distance readings 
    sr = data["sr"]            # Side-right ToF distance readings 
    yaw_rate = data["gyro"][2]  # rad/s about z
    dt = data["dt"]            # s, simulator timestep

    print(f"fl={fl:.3f} fr={fr:.3f} sl={sl:.3f} sr={sr:.3f} "
          f"yaw_rate={yaw_rate:+.3f} dt={dt:.4f}")

    # PD lateral centering + front alignment + gyro damping.
    left_vel, right_vel, e_lat, e_front, steer = CONTROLLER.update(
        fl, fr, sl, sr, yaw_rate, dt)
    nan = float("nan")
    e_lat = nan if e_lat is None else e_lat
    e_front = nan if e_front is None else e_front
    print(f"  e_lat={e_lat:+.3f} e_front={e_front:+.3f} "
          f"steer={steer:+.3f} -> L={left_vel:+.2f} R={right_vel:+.2f}")

    if LOGGER is not None:
        # Ground truth if the sim ever publishes it; else dead-reckoned
        # estimate only (labeled estimated in every plot).
        true_pose = None
        if "x" in data and "y" in data:
            th = data.get("theta", data.get("th", data.get("yaw", "")))
            true_pose = (data["x"], data["y"], th)
        extra = {k: v for k, v in data.items()
                 if k not in ("fl", "fr", "sl", "sr", "gyro", "dt",
                              "x", "y", "th", "theta", "yaw")}
        extra["gyro_full"] = data.get("gyro")
        if CONTROLLER.reanchor_mag is not None:
            extra["reanchor"] = CONTROLLER.reanchor_mag
            CONTROLLER.reanchor_mag = None
        in_turn = CONTROLLER.turn_active
        LOGGER.tick(
            raw=(fl, fr, sl, sr),
            filt=(CONTROLLER.fl_f, CONTROLLER.fr_f,
                  CONTROLLER.sl_f, CONTROLLER.sr_f),
            gyro_z=yaw_rate, dt_rep=dt, e_lat=e_lat, e_front=e_front,
            steer=steer, L=left_vel, R=right_vel,
            state=CONTROLLER.state, true_pose=true_pose, extra=extra,
            statuses=CONTROLLER.last_status,
            yaw_cmd=CONTROLLER.yaw_cmd,
            stuck=CONTROLLER.stuck_flag,
            turn={"target": CONTROLLER.turn_target if in_turn else "",
                  "angle": CONTROLLER.turn_angle if in_turn else "",
                  "error": CONTROLLER.turn_error if in_turn else "",
                  "elapsed": CONTROLLER.turn_t if in_turn else "",
                  "progress": CONTROLLER.turn_progress if in_turn else "",
                  "abort": CONTROLLER.abort_reason},
            lat_mode=CONTROLLER.lat_mode,
            heading={"e": CONTROLLER.e_heading,
                     "lat": CONTROLLER.steer_lat,
                     "front": CONTROLLER.steer_front,
                     "heading": CONTROLLER.steer_heading})
        if _PREV_STATE is not None and CONTROLLER.state != _PREV_STATE:
            LOGGER.event(_PREV_STATE, CONTROLLER.state,
                         CONTROLLER.state_reason)
        _PREV_STATE = CONTROLLER.state
        _dt_warn(dt)

    client.publish(TOPIC_WHEEL_VEL, json.dumps({
        "left": float(left_vel), "right": float(right_vel),
    }))


def main():
    global LOGGER
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="run",
                    help="run label; folder is logs/<UTC-timestamp>_<label>/")
    ap.add_argument("--log-dir", default=None,
                    help="parent dir for runs (default: task1b/logs/)")
    ap.add_argument("--no-log", action="store_true",
                    help="disable file logging")
    args = ap.parse_args()

    if not args.no_log:
        base = (args.log_dir or
                os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "logs"))
        LOGGER = RunLogger(base, label=args.label,
                           constants=_controller_constants(),
                           wheel_r=WHEEL_R_EST,
                           wheel_track=WHEEL_TRACK_EST,
                           k_lin=K_LIN)

    client = _mqtt_client()
    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT)
    client.subscribe(TOPIC_SENSORS)
    try:
        client.loop_forever()
    finally:
        if LOGGER is not None:
            LOGGER.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
