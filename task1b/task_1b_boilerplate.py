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
from sensing import StuckDetector, Tof, WallPerception

MQTT_HOST = "localhost"
MQTT_PORT = 1883
TOPIC_SENSORS = "pacbot/sensors"      # simulator publishes, this file subscribes
TOPIC_WHEEL_VEL = "pacbot/wheel_vel"  # this file publishes, simulator subscribes
TOPIC_RESULT = "pacbot/result"        # simulator publishes final score/verdict

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
FRONT_SPEED_TAPER_END = 0.06  # speed remains positive above this range
FRONT_EMERGENCY_DIST = 0.08  # back out before the taper can stall the bot
FRONT_EMERGENCY_RELEASE = 0.12  # hysteresis: keep backing until clear
FRONT_BACKOUT_SPEED = 1.5  # wheel rad/s; no turn while contact is imminent
FOLLOW_STEER_RATIO = 0.75  # FOLLOW keeps both wheels driving forward
SPIN_SPEED = 3.0       # spin-in-place wheel speed
FRONT_RAY_ANGLE = math.radians(20.0)
FRONT_RAY_COS = math.cos(FRONT_RAY_ANGLE)
FRONT_ALIGN_MAX_DIST = 0.5  # only align to a nearby wall seen by both rays
FRONT_ALIGN_SYM_DB = 0.05   # both splayed rays must see nearly same range
FRONT_MIN_SPEED_MPS = 0.02  # keep a useful crawl above emergency threshold
HEADING_PIVOT_THRESHOLD = math.radians(15.0)
HEADING_WATCHDOG_THRESHOLD = math.radians(20.0)
HEADING_WATCHDOG_DWELL_S = 5.0
BACKOUT_REPEAT_TOL_M = 0.01
BACKOUT_REPEAT_LIMIT = 3

MAX_RANGE = 2.0        # clip ToF readings to this (meters)
FILTER_TAU = 0.05      # low-pass time constant for ToF (seconds)
TOF_HOLD_S = 0.1       # validity hold: coast through dropouts this long

# Initial side-sensor clearance from the observed two-wall maze19 segment.
WALL_TARGET = 0.051

# Wedge hysteresis (fix 4). Enter only after both sides persist low;
# exit only when both sides read high AND dwell has passed.
WEDGE_ENTER = 0.08
WEDGE_EXIT = 0.12
WEDGE_ENTER_T = 0.1
WEDGE_MIN_DWELL = 0.5
WEDGE_MAX_T = 2.0      # then hand off to stuck/give-up logic
WEDGE_REPEAT_WINDOW_S = 15.0
WEDGE_TURN_AFTER = 2   # repeated recoveries at one spot -> turn to more room
WEDGE_CLEARANCE_TIE_DB = 0.015

# Gyro-terminated turns (fix 3, reworked Stage 1b). Turn rate now comes
# from the MEASURED yaw gain (step test), not wheel-size estimates.
YAW_GAIN_K = 0.0914   # from step_test s1_step summary.json
TURN_KP = 2.0          # P gain on angle error (rad/s per rad)
TURN_MIN_W = 1.5       # minimum wheel speed in a turn (rad/s, step test sets)
TURN_EXIT_ERR = 0.052  # 3 deg; coast adds ~1-2 deg -> final inside +/-5
TURN_TIE_DB = 0.05     # side-openness deadband: tie -> prev turn dir
TURN_MAX_RETRIES = 2
TURN_TIMEOUT_BASE = 0.5    # hard timeout = BASE + PER_RAD*|target|
TURN_TIMEOUT_PER_RAD = 3.0  # allow the low-speed approach to target to finish
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
# is a lower bound ("at least 0.3"), never a distance. The current
# distance interpretation accepts every finite reading through MAX_RANGE.
SENSOR_CAP_M = 0.300
SAT_MODE = "distance"   # 0.300 m is a measured distance

# Exploration behaviors (maze1 post-mortem: the controller turned
# away from blockages but never INTO openings, so it walked past the
# entrance and cruised 20 m into the void).
FOLLOW_WALL_MAX = 0.25   # single-side numeric below this = following it
FOLLOW_ESTABLISH_S = 1.0  # sustained single-wall follow before gaps count
GAP_OPEN_S = 0.5         # follow wall lost this long -> seek it (90 deg)
FOLLOW_SIDE_PREFERENCE = 1.0  # stable tie-break when both corridor walls are present
BLIND_TURN_S = 15.0      # fully blind this long -> 180 deg turn-back;
# a second consecutive blind stretch latches HOLD (stop, don't wander).
# Gateway probe (maze2/3 spawn: entrance gap ahead BETWEEN the
# splayed rays, posts reading 0.100 on both sides, flanking walls
# at ~0.26). Signature: symmetric close fronts + sides NOT in wedge
# range = narrow passage to squeeze through, NOT a wall (a real
# dead-end wall has sides in wedge range too). Creep centered on
# e_front trim; abort to normal escape on contact approach, timeout,
# or 2nd try.
GATEWAY_FRONT_MAX = 0.15  # both fronts numeric below this...
GATEWAY_SYM_DB = 0.03     # ...symmetric within this...
GATEWAY_SIDE_OPEN = 0.20  # ...with both side readings above this...
GATEWAY_V_FRAC = 1.0      # approach at normal cruise speed...
GATEWAY_ABORT_DIST = 0.06  # ...park if either front approaches contact...
GATEWAY_CLEAR_S = 0.15    # both front rays clear this long -> FOLLOW
GATEWAY_CORRIDOR_MAX = 0.22  # both sides inside this range = passage entered
GATEWAY_CORRIDOR_SYM_DB = 0.06
GATEWAY_CORRIDOR_S = 0.30  # require a stable side-wall signature
GATEWAY_SIDE_WALL_DELTA_M = 0.25  # side range fell from spawn baseline
GATEWAY_SIDE_WALL_DWELL_S = 0.25  # sensor persistence debounce, not mode time
GATEWAY_REARM_CLEAR_DIST = 0.20  # require front sensor evidence to rearm
GATEWAY_FRONT_STEER_MAX = 0.50  # bound front alignment while in the frame
GATEWAY_APPROACH_M = 0.40  # allow a longer straight entrance before handoff
GATEWAY_TIMEOUT = 12.0     # enough time to cover the approach at cruise speed

# Heading-aware centering (Stage 3). The reference accumulates each
# COMPLETED turn's intended target (never the measured exit angle --
# that would bake the ~3 deg residual in). Illustration used KP 3.0 /
# 2.0; tuning starts low.
KP_HEADING = 0.5       # reduced; local ToF steering takes priority
STEER_HEADING_MAX = 1.0  # bound on the heading contribution
HEADING_STEER_FADE_START = 0.15  # begin fading when local ToF correction grows
HEADING_STEER_FADE_FULL = 0.60   # no heading hold above this ToF correction
# Optional wall-slope re-anchor: heading ~= asin(side_rate / v).
# Needs K_LIN (Stage 2 done). Strict gates; FOLLOW-only so turns
# (entry-relative angles) are never disturbed.
REANCHOR_ENABLE = False
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
        "FRONT_STOP_DIST": FRONT_STOP_DIST,
        "FRONT_SPEED_TAPER_END": FRONT_SPEED_TAPER_END,
        "FRONT_EMERGENCY_DIST": FRONT_EMERGENCY_DIST,
        "FRONT_EMERGENCY_RELEASE": FRONT_EMERGENCY_RELEASE,
        "FRONT_BACKOUT_SPEED": FRONT_BACKOUT_SPEED,
        "FRONT_RAY_ANGLE_DEG": math.degrees(FRONT_RAY_ANGLE),
        "FRONT_ALIGN_MAX_DIST": FRONT_ALIGN_MAX_DIST,
        "FRONT_ALIGN_SYM_DB": FRONT_ALIGN_SYM_DB,
        "FRONT_MIN_SPEED_MPS": FRONT_MIN_SPEED_MPS,
        "HEADING_PIVOT_THRESHOLD": HEADING_PIVOT_THRESHOLD,
        "HEADING_WATCHDOG_THRESHOLD": HEADING_WATCHDOG_THRESHOLD,
        "HEADING_WATCHDOG_DWELL_S": HEADING_WATCHDOG_DWELL_S,
        "PROGRESS_WATCHDOG_S": PROGRESS_WATCHDOG_S,
        "PROGRESS_SENSOR_DELTA_M": PROGRESS_SENSOR_DELTA_M,
        "BACKOUT_REPEAT_TOL_M": BACKOUT_REPEAT_TOL_M,
        "BACKOUT_REPEAT_LIMIT": BACKOUT_REPEAT_LIMIT,
        "FOLLOW_STEER_RATIO": FOLLOW_STEER_RATIO,
        "SPIN_SPEED": SPIN_SPEED,
        "K_LIN": K_LIN, "CRUISE_LINEAR_MPS": CRUISE_LINEAR_MPS,
        "MAX_LINEAR_MPS": MAX_LINEAR_MPS,
        "SENSOR_CAP_M": SENSOR_CAP_M, "SAT_MODE": SAT_MODE,
        "FOLLOW_WALL_MAX": FOLLOW_WALL_MAX,
        "FOLLOW_ESTABLISH_S": FOLLOW_ESTABLISH_S,
        "GAP_OPEN_S": GAP_OPEN_S,
        "FOLLOW_SIDE_PREFERENCE": FOLLOW_SIDE_PREFERENCE,
        "BLIND_TURN_S": BLIND_TURN_S,
        "GATEWAY_FRONT_MAX": GATEWAY_FRONT_MAX,
        "GATEWAY_SYM_DB": GATEWAY_SYM_DB,
        "GATEWAY_SIDE_OPEN": GATEWAY_SIDE_OPEN,
        "GATEWAY_V_FRAC": GATEWAY_V_FRAC,
        "GATEWAY_ABORT_DIST": GATEWAY_ABORT_DIST,
        "GATEWAY_CLEAR_S": GATEWAY_CLEAR_S,
        "GATEWAY_CORRIDOR_MAX": GATEWAY_CORRIDOR_MAX,
        "GATEWAY_CORRIDOR_SYM_DB": GATEWAY_CORRIDOR_SYM_DB,
        "GATEWAY_CORRIDOR_S": GATEWAY_CORRIDOR_S,
        "GATEWAY_SIDE_WALL_DELTA_M": GATEWAY_SIDE_WALL_DELTA_M,
        "GATEWAY_SIDE_WALL_DWELL_S": GATEWAY_SIDE_WALL_DWELL_S,
        "GATEWAY_REARM_CLEAR_DIST": GATEWAY_REARM_CLEAR_DIST,
        "GATEWAY_FRONT_STEER_MAX": GATEWAY_FRONT_STEER_MAX,
        "GATEWAY_APPROACH_M": GATEWAY_APPROACH_M,
        "GATEWAY_TIMEOUT": GATEWAY_TIMEOUT,
        "KP_HEADING": KP_HEADING, "STEER_HEADING_MAX": STEER_HEADING_MAX,
        "HEADING_STEER_FADE_START": HEADING_STEER_FADE_START,
        "HEADING_STEER_FADE_FULL": HEADING_STEER_FADE_FULL,
        "REANCHOR_ENABLE": REANCHOR_ENABLE,
        "REANCHOR_V_MIN": REANCHOR_V_MIN,
        "REANCHOR_MAX_IMPLIED": REANCHOR_MAX_IMPLIED,
        "REANCHOR_CONSIST_S": REANCHOR_CONSIST_S,
        "MAX_RANGE": MAX_RANGE, "FILTER_TAU": FILTER_TAU,
        "TOF_HOLD_S": TOF_HOLD_S, "WALL_TARGET": WALL_TARGET,
        "WEDGE_ENTER": WEDGE_ENTER, "WEDGE_EXIT": WEDGE_EXIT,
        "WEDGE_ENTER_T": WEDGE_ENTER_T, "WEDGE_MIN_DWELL": WEDGE_MIN_DWELL,
        "WEDGE_MAX_T": WEDGE_MAX_T,
        "WEDGE_REPEAT_WINDOW_S": WEDGE_REPEAT_WINDOW_S,
        "WEDGE_TURN_AFTER": WEDGE_TURN_AFTER,
        "WEDGE_CLEARANCE_TIE_DB": WEDGE_CLEARANCE_TIE_DB,
        "TURN_KP": TURN_KP, "TURN_MIN_W": TURN_MIN_W,
        "TURN_EXIT_ERR": TURN_EXIT_ERR,
        "TURN_TIE_DB": TURN_TIE_DB,
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
        self.perception = WallPerception()
        self.observation = None
        self.wall_target = WALL_TARGET
        self._target_samples = deque(maxlen=50)
        self.range_trend = 0.0
        self._last_follow_range = None
        self.corridor_heading = 0.0
        self.junction_stage = "none"
        self.junction_distance_est = 0.0
        self.turn_outcome = "none"
        self.i_lat = 0.0
        self.t = 0.0              # controller clock (sim-time, from dt)
        self.gyro_th = 0.0        # integrated heading (turn termination)
        # Heading reference (Stage 3): accumulates intended turn targets.
        self.heading_ref = 0.0
        self.e_heading = 0.0
        self.turn_updates_heading_ref = True
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
        # Gateway probe state (spawn frames).
        self.gateway_active = False
        self.gateway_t = 0.0
        self.gateway_distance = 0.0
        self.gateway_clear_t = 0.0
        self.gateway_corridor_t = 0.0
        self.gateway_side_wall_t = 0.0
        self.gateway_start_sl = None
        self.gateway_start_sr = None
        self.gateway_wall_follow_active = False
        self.gateway_rearm_latched = False
        self.gateway_tries = 0
        self.gateway_failed = False
        self.front_backout_active = False
        self.front_backout_side = 0.0  # +1 left ray / -1 right ray
        self.front_check_t = 0.0
        self.backout_signature = None
        self.backout_signature_count = 0
        self.backout_loop_latched = False
        # Wedge hysteresis state (fix 4)
        self.wedge_active = False
        self.wedge_below_t = 0.0
        self.wedge_t = 0.0
        self.wedge_cycles = deque()
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

    def _start_turn(self, tdir, mag, cause, update_heading_ref=True):
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
        self.turn_outcome = "active"
        self.junction_stage = "none"
        self.turn_updates_heading_ref = update_heading_ref
        self.spin_done_s = 0.0  # new escape -> new committed turn
        self.follow_side = 0.0
        self.follow_t = 0.0
        self.gap_t = 0.0
        self.backout_signature_count = 0

    def _record_backout_signature(self, signature):
        if any(v is None for v in signature):
            self.backout_signature = None
            self.backout_signature_count = 0
            return
        if (self.backout_signature is not None
                and max(abs(a - b) for a, b in
                        zip(signature, self.backout_signature))
                <= BACKOUT_REPEAT_TOL_M):
            self.backout_signature_count += 1
        else:
            self.backout_signature = signature
            self.backout_signature_count = 1
        if self.backout_signature_count >= BACKOUT_REPEAT_LIMIT:
            self.backout_loop_latched = True

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
        """Single exit path. Stuck detection only judges forward progress
        in FOLLOW/GATEWAY. A yaw actuator cannot be classified as stuck
        from a translational reverse/turn/brake window, and mixing those
        modes contaminated the 1 s signed-mean detector."""
        yaw_cmd = YAW_GAIN_K * (R - L)
        self.yaw_cmd = yaw_cmd
        if self.state not in ("FOLLOW", "GATEWAY"):
            self.stuck.reset()
            self.stuck_flag = False
            self._stuck_prev = False
            return L, R, e_lat, e_front, steer
        stuck_now = self.stuck.update(yaw_cmd, yaw_rate, dt)
        self.stuck_flag = stuck_now
        if stuck_now and not self._stuck_prev:
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
        self.t += dt
        self.gyro_th += yaw_rate * dt
        # 1. Validity filter: finite measurements through 2 m are distances.
        #    Held samples preserve control continuity but cannot confirm an
        #    opening or front clearance.
        self.fl_f, fl_s = self.tof_fl.update(fl, dt)
        self.fr_f, fr_s = self.tof_fr.update(fr, dt)
        self.sl_f, sl_s = self.tof_sl.update(sl, dt)
        self.sr_f, sr_s = self.tof_sr.update(sr, dt)
        self.last_status = (fl_s, fr_s, sl_s, sr_s)
        fl_forward = (self.fl_f * FRONT_RAY_COS
                      if self.fl_f is not None else None)
        fr_forward = (self.fr_f * FRONT_RAY_COS
                      if self.fr_f is not None else None)
        self.e_heading = self._wrap(self.corridor_heading - self.gyro_th)
        self.observation = self.perception.update(
            (self.fl_f, self.fr_f, self.sl_f, self.sr_f),
            self.last_status, dt, FRONT_RAY_COS)

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
        if (sl_num and sr_num and self.observation.left == "wall"
                and self.observation.right == "wall"):
            e_lat = ((self.sl_f - self.sr_f) * 0.5
                     )
            if abs(self.sl_f - self.sr_f) < 0.02 and abs(yaw_rate) < 0.15:
                self._target_samples.append((self.sl_f + self.sr_f) * 0.5)
                if len(self._target_samples) >= 20:
                    self.wall_target = max(0.04, min(0.09,
                        sum(self._target_samples) / len(self._target_samples)))
            lat_mode = "both"
        elif sl_num and self.observation.left == "wall":
            single = (1.0, self.sl_f)
            lat_mode = "left_only"
        elif sr_num and self.observation.right == "wall":
            single = (-1.0, self.sr_f)
            lat_mode = "right_only"
        else:
            lat_mode = "blind"
        self.lat_mode = lat_mode
        if fl_num and fr_num:
            e_front = fl_forward - fr_forward
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
            steer_lat = side * KP_LAT * (reading - self.wall_target)
            if self._last_follow_range is not None and dt > 0:
                trend = max(-0.5, min(0.5,
                    (reading - self._last_follow_range) / dt))
                self.range_trend = 0.85 * self.range_trend + 0.15 * trend
                steer_lat += side * 0.25 * self.range_trend
            self._last_follow_range = reading
        front_pair_faces_wall = (
            fl_num and fr_num
            and max(fl_forward, fr_forward) <= FRONT_ALIGN_MAX_DIST
            and abs(fl_forward - fr_forward) <= FRONT_ALIGN_SYM_DB
        )
        steer_front = (
            KP_FRONT * e_front
            if e_front is not None and front_pair_faces_wall else 0.0
        )
        steer = steer_lat + steer_front + KI_LAT * self.i_lat - KD_YAW * yaw_rate
        steer = max(-MAX_STEER, min(steer, MAX_STEER))
        self.steer_lat = steer_lat
        self.steer_front = steer_front
        self.steer_heading = 0.0

        # 5. Longitudinal + escape. States: FOLLOW / REVERSE / TURN /
        #    BACKUP / WEDGE / RECOVER / HOLD. Every return goes through
        #    _finalize() for stuck detection.

        # --- HOLD latch (anti-void terminal state): parked, zeros out.
        #     Nothing in the maze moves to us, so this never releases;
        #     the run is over for scoring and the operator intervenes.
        if self.backout_loop_latched:
            self._set_state("BACKOUT_HOLD", "repeat_front_backout")
            return self._finalize(0.0, 0.0, e_lat, e_front,
                                  0.0, yaw_rate, dt)
        if self.hold:
            self._set_state("HOLD", "lost_hold")
            return self._finalize(0.0, 0.0, e_lat, e_front,
                                  steer, yaw_rate, dt)

        # Use the nearest front ray for cautious speed/steering, but only
        # call it a blocked path when BOTH front rays are close. A single
        # splayed ray can graze a corridor wall and must not trigger escape.
        front_vals = [v for v in (fl_forward, fr_forward) if v is not None]
        front_clear = min(front_vals) if front_vals else None
        front_block_l = (fl_num and fl_forward < FRONT_STOP_DIST)
        front_block_r = (fr_num and fr_forward < FRONT_STOP_DIST)
        blocked = front_block_l and front_block_r
        front_open_l = (fl_s == "valid" and
                        fl_forward > self.RESUME_DIST)
        front_open_r = (fr_s == "valid" and
                        fr_forward > self.RESUME_DIST)
        gateway_path_open = front_open_l or front_open_r
        # Once a side wall has emerged from the open gateway frame, treat
        # the close splayed front rays as entrance geometry until a front
        # ray actually clears. Otherwise FOLLOW immediately re-enters the
        # blocked-front turn loop before it can track the acquired wall.
        if self.gateway_wall_follow_active and gateway_path_open:
            self.gateway_wall_follow_active = False
        blocked = (self.observation.front == "blocked"
                   and not self.gateway_wall_follow_active)
        gateway_corridor = (
            sl_num and sr_num
            and self.sl_f < GATEWAY_CORRIDOR_MAX
            and self.sr_f < GATEWAY_CORRIDOR_MAX
            and abs(self.sl_f - self.sr_f) < GATEWAY_CORRIDOR_SYM_DB
        )
        if self.gateway_failed:
            self._set_state("GATEWAY_HOLD", "gateway_abort")
            return self._finalize(0.0, 0.0, e_lat, e_front,
                                  0.0, yaw_rate, dt)

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

        # --- wedge hysteresis: close side walls qualify only when the
        #     front is also blocked (a narrow straight corridor is not a
        #     wedge). Exit only when both sides clear for the dwell. ---
        both_below = (self.sl_f is not None and self.sr_f is not None
                      and self.sl_f < WEDGE_ENTER and self.sr_f < WEDGE_ENTER)
        if not self.wedge_active:
            self.wedge_below_t = self.wedge_below_t + dt if both_below else 0.0
            if (both_below and self.wedge_below_t >= WEDGE_ENTER_T
                    and blocked
                    and self.spin_dir == 0.0):
                self.wedge_active = True
                self.wedge_t = 0.0
                self.wedge_cycles.append(self.t)
                while (self.wedge_cycles
                       and self.t - self.wedge_cycles[0]
                       > WEDGE_REPEAT_WINDOW_S):
                    self.wedge_cycles.popleft()
        if self.wedge_active:
            self.wedge_t += dt
            both_above = (self.sl_f is not None and self.sr_f is not None
                          and self.sl_f > WEDGE_EXIT and self.sr_f > WEDGE_EXIT)
            if both_above and self.wedge_t >= WEDGE_MIN_DWELL:
                self.wedge_active = False
                self.wedge_below_t = 0.0
                while (self.wedge_cycles
                       and self.t - self.wedge_cycles[0]
                       > WEDGE_REPEAT_WINDOW_S):
                    self.wedge_cycles.popleft()
                if len(self.wedge_cycles) >= WEDGE_TURN_AFTER:
                    # Repeatedly retrying the same narrow approach made no
                    # route progress. We've backed out of the pinch; make a
                    # deliberate quarter-turn toward the side with room.
                    if (sl_num and sr_num
                            and abs(self.sl_f - self.sr_f)
                            > WEDGE_CLEARANCE_TIE_DB):
                        tdir = 1.0 if self.sl_f > self.sr_f else -1.0
                    else:
                        tdir = self.prev_turn_dir
                    self.wedge_cycles.clear()
                    self._start_turn(tdir, math.pi / 2.0, "wedge")
                    self._set_state("TURN", "wedge_" +
                                    ("left" if tdir > 0 else "right"))
                    return self._finalize(0.0, 0.0, e_lat, e_front,
                                          steer, yaw_rate, dt)
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

        # --- gateway probe (maze2/3 spawn): symmetric close fronts with
        #     open sides = frame to squeeze through, NOT a wall. The
        #     splayed rays hit the posts; the gap runs between them.
        #     Latch the probe once entered. One front ray can clear before
        #     the other; bounded front alignment plus side centering and
        #     gyro damping guides the crossing.
        in_maneuver_now = (self.reverse_ticks > 0 or self.backup_ticks > 0
                           or self.wedge_active or self.spin_dir != 0.0
                           or self.recover_active)
        gateway_sig = (
            fl_num and fr_num
            and fl_forward < GATEWAY_FRONT_MAX
            and fr_forward < GATEWAY_FRONT_MAX
            and abs(fl_forward - fr_forward) < GATEWAY_SYM_DB
            and sl_num and sr_num
            and self.sl_f >= GATEWAY_SIDE_OPEN
            and self.sr_f >= GATEWAY_SIDE_OPEN
        )
        # A successful handoff must not immediately retrigger on the same
        # persistent spawn signature. Rearm only after a front ray clears
        # the sensor-defined passage and the original signature is gone.
        # The entrance is a one-time passage, never a repeated route state.
        if not in_maneuver_now and (
                self.gateway_active
                or (gateway_sig and not self.gateway_rearm_latched)):
            if not self.gateway_active:
                self.gateway_active = True
                self.gateway_t = 0.0
                self.gateway_distance = 0.0
                self.gateway_clear_t = 0.0
                self.gateway_side_wall_t = 0.0
                self.gateway_start_sl = self.sl_f if sl_num else None
                self.gateway_start_sr = self.sr_f if sr_num else None
                self.gateway_tries += 1
            self.gateway_t += dt
            front_clear_l = (fl_s == "valid" and
                             fl_forward > self.RESUME_DIST)
            front_clear_r = (fr_s == "valid" and
                             fr_forward > self.RESUME_DIST)
            if front_clear_l and front_clear_r:
                self.gateway_clear_t += dt
            else:
                self.gateway_clear_t = 0.0
            if gateway_corridor and gateway_path_open:
                self.gateway_corridor_t += dt
            else:
                self.gateway_corridor_t = 0.0

            side_wall_l = (
                sl_num and self.gateway_start_sl is not None
                and self.gateway_start_sl - self.sl_f
                >= GATEWAY_SIDE_WALL_DELTA_M
            )
            side_wall_r = (
                sr_num and self.gateway_start_sr is not None
                and self.gateway_start_sr - self.sr_f
                >= GATEWAY_SIDE_WALL_DELTA_M
            )
            if side_wall_l or side_wall_r:
                self.gateway_side_wall_t += dt
            else:
                self.gateway_side_wall_t = 0.0

            front_exit = self.gateway_clear_t >= GATEWAY_CLEAR_S
            corridor_exit = self.gateway_corridor_t >= GATEWAY_CORRIDOR_S
            side_wall_exit = (
                self.gateway_side_wall_t >= GATEWAY_SIDE_WALL_DWELL_S
            )
            approach_limit = self.gateway_distance >= GATEWAY_APPROACH_M
            # Distance alone is not evidence that the entrance is clear.
            # Require at least one front ray to open before handing control
            # to normal obstacle escape; otherwise stop safely at the limit.
            approach_exit = approach_limit and gateway_path_open
            if front_exit or corridor_exit or side_wall_exit or approach_exit:
                # Keep the entrance interpretation active through FOLLOW
                # while front rays remain blocked; side-wall acquisition
                # is the evidence that this is a passage, not a dead end.
                self.gateway_wall_follow_active = (
                    side_wall_exit and not gateway_path_open
                )
                self.gateway_rearm_latched = True
                if side_wall_exit:
                    if side_wall_l and side_wall_r:
                        self.follow_side = (
                            1.0 if self.sl_f <= self.sr_f else -1.0
                        )
                    elif side_wall_l:
                        self.follow_side = 1.0
                    else:
                        self.follow_side = -1.0
                    self.follow_t = 0.0
                    self.gap_t = 0.0
                self.gateway_active = False
                self.gateway_t = 0.0
                self.gateway_distance = 0.0
                self.gateway_clear_t = 0.0
                self.gateway_corridor_t = 0.0
                self.gateway_side_wall_t = 0.0
                reason = ("gateway_clear" if front_exit else
                          "gateway_corridor" if corridor_exit else
                          "gateway_side_wall" if side_wall_exit else
                          "gateway_approach_done")
                self._set_state("FOLLOW", reason)
                blocked = (self.observation.front == "blocked"
                           and not self.gateway_wall_follow_active)
                # Continue into FOLLOW or normal blocked/turn handling.
            elif approach_limit:
                self.gateway_active = False
                self.gateway_failed = True
                self._set_state("GATEWAY_HOLD", "gateway_no_clearance")
                return self._finalize(0.0, 0.0, e_lat, e_front,
                                      0.0, yaw_rate, dt)
            elif (self.gateway_t > GATEWAY_TIMEOUT
                  or any(v is not None and s in ("valid", "held")
                         and v < GATEWAY_ABORT_DIST
                         for v, s in ((fl_forward, fl_s),
                                      (fr_forward, fr_s)))):
                # Don't hand a failed entrance probe to the generic blocked
                # handler: that immediately reverses and turns out of spawn.
                self.gateway_active = False
                self.gateway_failed = True
                self._set_state("GATEWAY_HOLD", "gateway_timeout")
                return self._finalize(0.0, 0.0, e_lat, e_front,
                                      0.0, yaw_rate, dt)
            else:
                # Creep forward with all feedback bounded for the narrow
                # frame; front rays can be asymmetric while clearing posts.
                v_gw = CRUISE_LINEAR_MPS * GATEWAY_V_FRAC
                base_gw = v_gw / K_LIN
                steer_front_gw = max(
                    -GATEWAY_FRONT_STEER_MAX,
                    min(steer_front, GATEWAY_FRONT_STEER_MAX),
                )
                steer_gw = steer_lat + steer_front_gw - KD_YAW * yaw_rate
                steer_gw = max(-MAX_STEER, min(steer_gw, MAX_STEER))
                left_gw = base_gw - steer_gw
                right_gw = base_gw + steer_gw
                left_gw = max(-MAX_SPEED, min(left_gw, MAX_SPEED))
                right_gw = max(-MAX_SPEED, min(right_gw, MAX_SPEED))
                self.gateway_distance += max(
                    0.0, K_LIN * (left_gw + right_gw) * 0.5 * dt)
                self._set_state("GATEWAY", "probe")
                return self._finalize(left_gw, right_gw, e_lat, e_front,
                                      steer_gw, yaw_rate, dt)
        elif not self.gateway_active:
            self.gateway_active = False
            self.gateway_t = 0.0
            self.gateway_clear_t = 0.0

        # A lone near-contact ray is ambiguous for choosing a turn, but
        # not safe to drive toward. Back straight until that same ray has
        # a clear margin; route choice is made from openings or a blocked
        # front, never from emergency recurrence alone.
        if self.front_backout_active:
            distance, status = (
                (fl_forward, fl_s) if self.front_backout_side > 0
                else (fr_forward, fr_s)
            )
            if (_num(distance, status)
                    and distance >= FRONT_EMERGENCY_RELEASE):
                self.front_backout_active = False
                self.front_backout_side = 0.0
            else:
                self._set_state("FRONT_BACKOUT", "front_emergency")
                return self._finalize(
                    -FRONT_BACKOUT_SPEED, -FRONT_BACKOUT_SPEED,
                    e_lat, e_front, steer, yaw_rate, dt)
        emergency_l = fl_num and fl_forward <= FRONT_EMERGENCY_DIST
        emergency_r = fr_num and fr_forward <= FRONT_EMERGENCY_DIST
        if emergency_l or emergency_r:
            if self.turn_active or self.spin_dir != 0.0:
                self.abort_reason = "front_emergency"
                self.turn_outcome = "aborted"
                self.turn_active = False
                self.spin_dir = 0.0
                self.reverse_ticks = 0
                self.brake_until = None
            self.front_backout_active = True
            self.front_backout_side = 1.0 if emergency_l else -1.0
            self._record_backout_signature(
                (fl_forward, fr_forward, self.sl_f, self.sr_f))
            self._set_state("FRONT_BACKOUT", "front_emergency")
            return self._finalize(
                -FRONT_BACKOUT_SPEED, -FRONT_BACKOUT_SPEED,
                e_lat, e_front, steer, yaw_rate, dt)

        if (self.state == "FOLLOW" and self.spin_dir == 0.0
                and not self.gateway_wall_follow_active
                and self.follow_t >= FOLLOW_ESTABLISH_S
                and self.observation.hazard == "single_ray"):
            self.front_check_t += dt
            # A grazing ray gets a bounded cautious approach. If the same
            # near ray never clears, back straight out and decide afresh.
            if self.front_check_t >= 1.5 and front_clear is not None \
                    and front_clear < 0.12:
                self.front_check_t = 0.0
                self.front_backout_active = True
                self.front_backout_side = (
                    1.0 if fl_forward is not None and fl_forward < 0.12
                    else -1.0)
                self._record_backout_signature(
                    (fl_forward, fr_forward, self.sl_f, self.sr_f))
                self._set_state("FRONT_BACKOUT", "single_ray_check")
                return self._finalize(
                    -FRONT_BACKOUT_SPEED, -FRONT_BACKOUT_SPEED,
                    e_lat, e_front, steer, yaw_rate, dt)
        else:
            self.front_check_t = 0.0

        # Recenter with the gyro turn servo rather than asking wall PD to
        # fight a large heading error. A progress watchdog uses the same
        # correction; if already aligned, it falls back to the sensor-based
        # junction choice rather than issuing a meaningless zero-angle turn.
        # Unchanged ranges provide no evidence of a stall in a uniform
        # corridor. Route turns require observed openings or a blocked front.

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
                side_left_open = self.observation.left == "open"
                side_right_open = self.observation.right == "open"
                deadend = blocked and not (side_left_open or side_right_open)
                mag = math.pi if deadend else math.pi / 2.0
                turn_cause = "deadend" if deadend else "blocked"
                if self.retry_dir is not None:
                    tdir = self.retry_dir  # hard-timeout retry: same dir
                    self.retry_dir = None
                elif blocked and (side_left_open or side_right_open):
                    # Route choice follows observed traversability, not
                    # whichever side merely has a few centimetres more
                    # range. At a two-way choice, keep the latched hand.
                    if side_left_open and side_right_open:
                        tdir = (self.follow_side or
                                FOLLOW_SIDE_PREFERENCE)
                    else:
                        tdir = 1.0 if side_left_open else -1.0
                    turn_cause = "junction"
                else:
                    tdir = 1.0  # deterministic U-turn direction
                self.spin_dir = tdir
                if self.flip_next:
                    self.spin_dir = -self.spin_dir
                    tdir = self.spin_dir
                    self.flip_next = False
                self._start_turn(tdir, mag, turn_cause)
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
                self.turn_outcome = "aborted"
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
                        self.turn_outcome = "aborted"
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
                if abs(err) <= RETRIM_TOL:
                    # Angle is the turn-completion criterion. A close
                    # front reading is a new obstacle decision, not a
                    # reason to repeat the entire turn from this heading.
                    self.prev_turn_dir = 1.0 if self.turn_orig > 0 else -1.0
                    self.turn_outcome = "complete"
                    self.corridor_heading = self.gyro_th
                    if self.turn_updates_heading_ref:
                        self.heading_ref += self.turn_orig
                    self.turn_updates_heading_ref = True
                    self.spin_dir = 0.0
                    self.turn_active = False
                    self.retry_used = False
                    self.retry_dir = None
                    self._set_state(
                        "FOLLOW", "turn_done" if front_clear is None
                        or front_clear > FRONT_STOP_DIST
                        else "turn_done_front_blocked")
                    # fall through to FOLLOW; a still-blocked front gets
                    # its own fresh reverse-and-direction decision.
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
                        self.turn_outcome = "aborted"
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
                        self.turn_outcome = "aborted"
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
        # from emergency distance to CAP-0.02; a saturated front reads the cap, i.e.
        # full cruise. Unknown front -> cautious half speed.
        if front_clear is not None and not blocked:
            span = (SENSOR_CAP_M - 0.02) - FRONT_SPEED_TAPER_END
            scale = ((front_clear - FRONT_SPEED_TAPER_END) / span
                     if span > 0 else 1.0)
            scale = max(0.0, min(1.0, scale))
            v_cmd = max(FRONT_MIN_SPEED_MPS,
                        CRUISE_LINEAR_MPS * scale)
        elif front_clear is None:
            v_cmd = CRUISE_LINEAR_MPS * 0.5
        else:
            v_cmd = CRUISE_LINEAR_MPS * 0.25
        v_cmd = min(v_cmd, MAX_LINEAR_MPS)
        base = v_cmd / K_LIN

        # --- Wall follow + junction seek: latch one wall, then treat a
        #     sustained opening on THAT side as a branch even if the
        #     opposite wall remains present. This catches T-junctions as
        #     well as full-width gaps; unknown/dropout readings do not
        #     count as an opening.
        L_close = self.observation.left == "wall"
        R_close = self.observation.right == "wall"
        if self.follow_side == 0.0:
            if L_close != R_close:
                self.follow_side = 1.0 if L_close else -1.0
                self.follow_t = 0.0
                self.gap_t = 0.0
                self.corridor_heading = self.gyro_th
                self._last_follow_range = None
                self.range_trend = 0.0
            elif L_close and R_close:
                # A narrow corridor has two useful walls. Latch a stable
                # hand rather than leaving junction detection unarmed.
                self.follow_side = FOLLOW_SIDE_PREFERENCE
                self.follow_t = 0.0
                self.gap_t = 0.0
                self.corridor_heading = self.gyro_th
                self._last_follow_range = None
                self.range_trend = 0.0
        else:
            followed_close = L_close if self.follow_side > 0 else R_close
            followed_status = sl_s if self.follow_side > 0 else sr_s
            followed_range = self.sl_f if self.follow_side > 0 else self.sr_f
            followed_open = self.observation.opening(self.follow_side)
            if followed_close:
                self.follow_t += dt
                self.gap_t = 0.0
            elif followed_open:
                self.gap_t += dt
                if (self.follow_side > 0 and self.follow_t >= FOLLOW_ESTABLISH_S
                        and self.gap_t >= GAP_OPEN_S and not blocked
                        and self.junction_stage == "none"):
                    self.junction_stage = "advance"
                    self.junction_distance_est = 0.0
            else:
                # Sensor unknown: pause the opening timer, don't infer void.
                self.gap_t = 0.0

        if self.junction_stage == "advance":
            if blocked or self.observation.hazard == "emergency":
                self.junction_stage = "none"
            else:
                self.junction_distance_est += max(0.0, v_cmd) * dt
                if self.junction_distance_est >= 0.055:
                    # The splayed front rays must leave room to rotate.
                    if (front_clear is not None and front_clear >= 0.12
                            and self.observation.left == "open"
                            and sr_s == "valid" and self.sr_f >= 0.035):
                        self._start_turn(1.0, math.pi / 2.0, "junction")
                        self._set_state("TURN", "junction_left")
                        return self._finalize(0.0, 0.0, e_lat, e_front,
                                              steer, yaw_rate, dt)
                    self.junction_stage = "none"
                else:
                    v_cmd = min(v_cmd, 0.025)
                    base = v_cmd / K_LIN

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
        # their wheels, maneuvers ignore it). Fade it out when local ToF
        # steering is large so a maze turn is not pulled back toward the
        # previous corridor heading.
        self.e_heading = self._wrap(self.corridor_heading - self.gyro_th)
        local_steer = max(abs(steer_lat), abs(steer_front))
        if local_steer <= HEADING_STEER_FADE_START:
            heading_scale = 1.0
        elif local_steer >= HEADING_STEER_FADE_FULL:
            heading_scale = 0.0
        else:
            heading_scale = (
                (HEADING_STEER_FADE_FULL - local_steer)
                / (HEADING_STEER_FADE_FULL - HEADING_STEER_FADE_START)
            )
        self.steer_heading = max(-STEER_HEADING_MAX,
                                 min(KP_HEADING * self.e_heading * heading_scale,
                                     STEER_HEADING_MAX))
        steer = max(-MAX_STEER, min(steer + self.steer_heading, MAX_STEER))
        # FOLLOW is for forward corridor tracking, not pivoting. When front
        # slowdown makes base small, unrestricted wall PD can overpower it
        # and command one wheel backward (the maze15 in-place-spin failure).
        # Reserve at least 25% of base on the slower wheel; explicit escape
        # and TURN states above retain their independent pivot commands.
        follow_steer_limit = FOLLOW_STEER_RATIO * max(0.0, base)
        steer = max(-follow_steer_limit,
                    min(steer, follow_steer_limit))
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


def on_connect(client, userdata, flags, reason_code, properties=None):
    """Subscribe after every successful connection, including reconnects."""
    failed = getattr(reason_code, "is_failure", None)
    if failed is None:
        try:
            failed = int(reason_code) != 0
        except (TypeError, ValueError):
            failed = False
    if not failed:
        client.subscribe(TOPIC_SENSORS)
        client.subscribe(TOPIC_RESULT)


def on_message(client, userdata, msg):
    global _PREV_STATE, _FIRST_MSG
    if msg.topic == TOPIC_RESULT:
        try:
            result_payload = json.loads(msg.payload.decode())
        except (UnicodeDecodeError, json.JSONDecodeError):
            result_payload = msg.payload.decode(errors="replace")
        if LOGGER is not None:
            LOGGER.result(result_payload, topic=msg.topic)
        else:
            print(f"[result] {result_payload}", flush=True)
        return
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


    # PD lateral centering + front alignment + gyro damping.
    left_vel, right_vel, e_lat, e_front, steer = CONTROLLER.update(
        fl, fr, sl, sr, yaw_rate, dt)
    nan = float("nan")
    e_lat = nan if e_lat is None else e_lat
    e_front = nan if e_front is None else e_front
    if int(CONTROLLER.t * 2) != int((CONTROLLER.t - dt) * 2):
        print(f"t={CONTROLLER.t:.1f} {CONTROLLER.state} "
              f"hazard={CONTROLLER.observation.hazard} "
              f"L={left_vel:+.2f} R={right_vel:+.2f}")

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
                     "heading": CONTROLLER.steer_heading},
            navigation={
                "wall_left": CONTROLLER.observation.left,
                "wall_right": CONTROLLER.observation.right,
                "front_path": CONTROLLER.observation.front,
                "front_hazard": CONTROLLER.observation.hazard,
                "follow_side": CONTROLLER.follow_side,
                "wall_target": CONTROLLER.wall_target,
                "junction_stage": CONTROLLER.junction_stage,
                "turn_outcome": CONTROLLER.turn_outcome,
            })
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
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT)
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
