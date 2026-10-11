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
from types import SimpleNamespace

import paho.mqtt.client as mqtt

from runlog import RunLogger
from sensing import StuckDetector, Tof, WallPerception
from flood_fill import FloodFillPlanner

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
REVERSE_WHEEL_W = BASE_SPEED * 0.5  # wheel speed for every reverse/backout

KP_LAT = 3.5           # lateral centering: e_lat = sl - sr (meters)
LAT_DEADBAND = 0.0015   # ignore sub-noise lateral error (damping)
LAT_DERIV_ALPHA = 0.4  # low-pass on the D term (damping)
KI_LAT = 0.45          # corridor-centering integral gain
KD_LAT = 0.35          # corridor-centering derivative damping
LAT_INTEGRAL_MAX = 0.08
LAT_DERIV_MAX = 0.50
KP_FRONT = 1.5         # heading trim: e_front = fl - fr (meters)
KD_YAW = 0.8           # gyro damping: -KD_YAW * yaw_rate

FRONT_STOP_DIST = 0.10 # below this, route around the front wall
TIGHT_CORRIDOR_ROUTE_DIST = 0.13
FRONT_JUNCTION_DIST = 0.16  # near front + open side: commit route turn
FRONT_SINGLE_OPEN_SIDE_M = 0.18  # hysteresis for single-ray corner safety
SINGLE_RAY_ROUTE_DIST = 0.12  # single splayed ray needs deeper evidence
SINGLE_RAY_CONFIRM_SIDE_M = 0.10  # single ray is valid only deep in the slot
SIDE_CENTER_MAX_M = 0.30  # use both side rays for centering up to sensor cap
FRONT_SPEED_TAPER_END = 0.06  # speed remains positive above this range
FRONT_EMERGENCY_DIST = 0.08  # back out before the taper can stall the bot
FRONT_EMERGENCY_RELEASE = 0.12  # hysteresis: keep backing until clear
FRONT_CRUISE_RESUME_DIST = 0.15  # creep after backout until this clear
FORWARD_WATCHDOG_S = 3.0         # suspected no-progress dwell
FORWARD_WATCHDOG_FRONT_M = 0.45  # only arm near geometry
FORWARD_WATCHDOG_DELTA_M = 0.004 # sensor movement expected while advancing
POST_GATEWAY_SUPPRESS_S = 2.0  # no junction turns right after gateway exit
GATEWAY_BLOCKED_PAIR_DWELL_S = 0.20  # real wall after spawn grace -> route
NO_PROGRESS_NEAR_TOL = 0.17  # ~10 deg: never abort a stall this close to target
FRONT_BACKOUT_SPEED = 1.5  # wheel rad/s; no turn while contact is imminent
FRONT_BACKOUT_MAX_S = 1.5  # never reverse indefinitely on a persistent wall
TURN_ENTRY_BACKOFF_M = 0.020  # short measured clearance before spinning
TURN_ENTRY_BACKOFF_MAX_S = 0.65
TURN_ENTRY_STOP_S = 0.10
FOLLOW_STEER_RATIO = 0.75  # FOLLOW keeps both wheels driving forward
SPIN_SPEED = 3.0       # spin-in-place wheel speed
FRONT_RAY_ANGLE = math.radians(20.0)
FRONT_RAY_COS = math.cos(FRONT_RAY_ANGLE)
FRONT_ALIGN_MAX_DIST = 0.5  # only align to a nearby wall seen by both rays
FRONT_ALIGN_SYM_DB = 0.05   # both splayed rays must see nearly same range
FRONT_MIN_SPEED_MPS = 0.02  # keep a useful crawl above emergency threshold
BACKOUT_REPEAT_TOL_M = 0.01
BACKOUT_REPEAT_LIMIT = 3

MAX_RANGE = 2.0        # clip ToF readings to this (meters)
FILTER_TAU = 0.05      # low-pass time constant for ToF (seconds)
TOF_HOLD_S = 0.1       # validity hold: coast through dropouts this long

# Initial side-sensor clearance from the observed two-wall maze19 segment.
WALL_TARGET = 0.051
SINGLE_WALL_TARGET_M = 0.095  # keep clearance when only one wall is visible

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
TURN_KI = 0.10         # controlled integral gain experiment
TURN_KD_RATE = 0.25    # yaw-rate damping for turn PID
TURN_INTEGRAL_MAX = 0.20
TURN_MIN_W = 1.2       # minimum wheel speed in a turn (rad/s)
TURN_ACCEL_W = 15.0    # wheel-speed ramp, rad/s^2, for smooth turn entry/exit
TURN_PD_ASSIST_GAIN = 0.25  # bounded side-wall PD contribution during spin
TURN_PD_ASSIST_MAX_W = 0.35
TURN_TOF_JUMP_M = 0.18       # side wall opening must be this large
TURN_TOF_OPEN_M = 0.20       # and reach clear/open range
TURN_TOF_MIN_ANGLE = math.radians(85.0)
TURN_TOF_DWELL_S = 0.04      # reject one noisy jump sample
TURN_EXIT_ERR = 0.052  # 3 deg; coast adds ~1-2 deg -> final inside +/-5
TURN_MAX_RETRIES = 2
TURN_TIMEOUT_BASE = 0.7    # hard timeout = BASE + PER_RAD*|target|
TURN_TIMEOUT_PER_RAD = 3.5  # allow the smooth low-speed approach to target
# NOTE: step_test t90@SPIN3 capped at the 2 s window, so the honest
# recompute (1.5*t90+0.5 = 3.5 s) waits on real s1_turns data. The
# no-progress watchdog is the real stall protection; this stays loose.
NO_PROGRESS_START = 0.5   # begin no-progress watch this far into a turn
NO_PROGRESS_WIN = 1.0     # abort if integrated angle gains < MIN in any window
NO_PROGRESS_MIN = 0.0873  # 5 deg
# Coast braking (step_test: 4-30 deg passive coast kills the +/-5 deg
# budget, so turns end with a closed-loop rotation stop, not a timer).
BRAKE_W = 0.5            # opposing wheel speed to stop rotation
BRAKE_EXIT_GYRO = 0.12   # stopped threshold (rad/s)
BRAKE_TIMEOUT = 0.35     # short bounded braking window
TURN_STOP_DWELL_S = 0.04 # low yaw must persist before turn completion
RETRIM_TOL = 0.0873      # 5 deg: accept final error within this
OVERSHOOT_MAX = 0.14     # 8 deg past target: give up, don't re-trim
POST_TURN_SETTLE_S = 0.15   # stop and collect fresh ranges after rotation
POST_TURN_VERIFY_MAX_S = 0.50  # bounded wait when front samples are stale
POST_TURN_ADVANCE_M = 0.06  # acquire the outgoing corridor before rearming
POST_TURN_SPEED_MPS = 0.025
POST_TURN_CENTER_GAIN = 0.3
POST_TURN_CENTER_MAX_W = 0.28
POST_TURN_HEADING_GAIN = 0.16  # weak hold; avoid long acquisition arcs
POST_TURN_SEEK_BIAS_W = 0.07  # gentle blind curve toward the route side
POST_TURN_FRONT_AVOID_GAIN = 1.0  # steer away from close splayed front ray
POST_TURN_FRONT_AVOID_MAX_W = 0.16
POST_TURN_OPEN_SIDE_GAIN = 0.4  # balance wide-mouth side readings pre-PID
POST_TURN_OPEN_SIDE_MAX_W = 0.09
ACQUIRE_SINGLE_WALL_GAIN = 0.55
ACQUIRE_CENTER_DWELL_S = 0.12
ACQUIRE_ONE_WALL_DWELL_S = 0.35
# The gyro/brake handoff can leave roughly 0.17 rad of residual heading error
# even after a turn passes the normal +/-5 degree turn criterion.  Acquisition
# must not drive forever waiting for a tighter heading than the turn can yield.
POST_TURN_HEADING_TOL = 0.20
POST_TURN_LATERAL_TOL = 0.035
POST_TURN_CLEAR_DWELL_S = 0.08
POST_TURN_BLOCK_DWELL_S = 0.12
CORRIDOR_ENTRY_DISTANCE_M = 0.16
CORRIDOR_ENTRY_CLEAR_DWELL_S = 0.10
# During acquisition the nearest splayed ray may still see the wall the bot
# has just rotated away from.  Abort only when the estimated centreline is
# genuinely occupied; do not let that old-wall ray cancel the latched route.
POST_TURN_CENTER_BLOCK_DIST = 0.10
POST_TURN_SAFE_FRONT_M = 0.14
POST_TURN_HOLD_S = 0.15
POST_TURN_SENSOR_WAIT_MAX_S = 0.50
POST_TURN_WIDE_MOUTH_M = 0.22
POST_TURN_BLOCK_RECOVERY_S = 3.0
POST_TURN_EMERGENCY_RECOVERY_S = 1.0
POST_TURN_MAX_HOLD_S = 6.0  # never wait indefinitely for a blocked exit
POST_TURN_NO_MOTION_MAX_S = 2.0  # zero-command acquisition must resolve
TURN_ENTRY_LATERAL_TOL = 0.025
TURN_ENTRY_CENTER_DWELL_S = 0.10
TURN_ENTRY_MAX_FRONT_ASYM_M = 0.60

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
CRUISE_LINEAR_MPS = 0.07   # faster long-corridor cruise
LEAN_FOLLOW_STEER_SCALE = 0.55  # broad damping for the lean controller
LEAN_SINGLE_WALL_SCALE = 0.35
LEAN_WEAK_EVIDENCE_SPEED_MPS = 0.05
LEAN_LARGE_LAT_ERR_M = 0.025
MAX_LINEAR_MPS = 0.12      # s2c ceiling (only if scoring rewards speed)
GATEWAY_LINEAR_MPS = 0.04  # keep entrance probing at the validated speed

# Saturation model (Stage 2). SAT_MODE="ceiling": a reading at the cap
# is a lower bound ("at least 0.3"), never a distance. The current
# distance interpretation accepts every finite reading through MAX_RANGE.
SENSOR_CAP_M = 0.300
SAT_MODE = "distance"   # 0.300 m is a measured distance

# Exploration behaviors (maze1 post-mortem: the controller turned
# away from blockages but never INTO openings, so it walked past the
# entrance and cruised 20 m into the void).
FOLLOW_ESTABLISH_S = 1.0  # sustained single-wall follow before gaps count
GAP_OPEN_S = 0.5         # follow wall lost this long -> seek it (90 deg)
FOLLOW_SIDE_PREFERENCE = 1.0  # stable tie-break when both corridor walls are present
SIDE_ROUTE_MIN_M = 0.20       # side opening threshold for routing
SIDE_ROUTE_TIE_DB = 0.03
SIDE_OPEN_DWELL_S = 0.50
SIDE_OPEN_CONFIRM_S = 0.30
ROUTE_BUFFER_M = 0.08         # creep into a junction before route choice
JUNCTION_ADVANCE_M = 0.08     # enter a detected side branch before turning
FRONT_BRANCH_SIDE_MAX = 0.20  # side-wall context for asymmetric front branch
ROUTE_BUFFER_SPEED_MPS = 0.018
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
GATEWAY_SIDE_WALL_DELTA_M = 0.04  # side range fell from spawn baseline
GATEWAY_SIDE_WALL_DWELL_S = 0.25  # sensor persistence debounce, not mode time
GATEWAY_FRONT_STEER_MAX = 0.50  # bound front alignment while in the frame
GATEWAY_APPROACH_M = 0.65  # move into the maze before route handoff
GATEWAY_TIMEOUT = 12.0     # enough time to cover the approach at cruise speed

# Heading-aware centering (Stage 3). The reference accumulates each
# COMPLETED turn's intended target (never the measured exit angle --
# that would bake the ~3 deg residual in). Illustration used KP 3.0 /
# 2.0; tuning starts low.
KP_HEADING = 0.5       # reduced; local ToF steering takes priority
STEER_HEADING_MAX = 1.0  # bound on the heading contribution
HEADING_STEER_FADE_START = 0.15  # begin fading when local ToF correction grows
HEADING_STEER_FADE_FULL = 0.60   # no heading hold above this ToF correction
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
        "KI_LAT": KI_LAT, "KD_LAT": KD_LAT,
        "LAT_INTEGRAL_MAX": LAT_INTEGRAL_MAX,
        "LAT_DERIV_MAX": LAT_DERIV_MAX,
        "LAT_DEADBAND": LAT_DEADBAND,
        "LAT_DERIV_ALPHA": LAT_DERIV_ALPHA,
        "KP_FRONT": KP_FRONT, "KD_YAW": KD_YAW,
        "FRONT_STOP_DIST": FRONT_STOP_DIST,
        "TIGHT_CORRIDOR_ROUTE_DIST": TIGHT_CORRIDOR_ROUTE_DIST,
        "FRONT_JUNCTION_DIST": FRONT_JUNCTION_DIST,
        "FRONT_SINGLE_OPEN_SIDE_M": FRONT_SINGLE_OPEN_SIDE_M,
        "SINGLE_RAY_ROUTE_DIST": SINGLE_RAY_ROUTE_DIST,
        "SINGLE_RAY_CONFIRM_SIDE_M": SINGLE_RAY_CONFIRM_SIDE_M,
        "SIDE_CENTER_MAX_M": SIDE_CENTER_MAX_M,
        "FRONT_SPEED_TAPER_END": FRONT_SPEED_TAPER_END,
        "FRONT_EMERGENCY_DIST": FRONT_EMERGENCY_DIST,
        "FRONT_EMERGENCY_RELEASE": FRONT_EMERGENCY_RELEASE,
        "FRONT_CRUISE_RESUME_DIST": FRONT_CRUISE_RESUME_DIST,
        "FORWARD_WATCHDOG_S": FORWARD_WATCHDOG_S,
        "FORWARD_WATCHDOG_FRONT_M": FORWARD_WATCHDOG_FRONT_M,
        "FORWARD_WATCHDOG_DELTA_M": FORWARD_WATCHDOG_DELTA_M,
        "POST_GATEWAY_SUPPRESS_S": POST_GATEWAY_SUPPRESS_S,
        "GATEWAY_BLOCKED_PAIR_DWELL_S": GATEWAY_BLOCKED_PAIR_DWELL_S,
        "NO_PROGRESS_NEAR_TOL": NO_PROGRESS_NEAR_TOL,
        "FRONT_BACKOUT_SPEED": FRONT_BACKOUT_SPEED,
        "FRONT_BACKOUT_MAX_S": FRONT_BACKOUT_MAX_S,
        "FRONT_RAY_ANGLE_DEG": math.degrees(FRONT_RAY_ANGLE),
        "FRONT_ALIGN_MAX_DIST": FRONT_ALIGN_MAX_DIST,
        "FRONT_ALIGN_SYM_DB": FRONT_ALIGN_SYM_DB,
        "FRONT_MIN_SPEED_MPS": FRONT_MIN_SPEED_MPS,
        "BACKOUT_REPEAT_TOL_M": BACKOUT_REPEAT_TOL_M,
        "BACKOUT_REPEAT_LIMIT": BACKOUT_REPEAT_LIMIT,
        "TURN_ENTRY_BACKOFF_M": TURN_ENTRY_BACKOFF_M,
        "TURN_ENTRY_BACKOFF_MAX_S": TURN_ENTRY_BACKOFF_MAX_S,
        "TURN_ENTRY_STOP_S": TURN_ENTRY_STOP_S,
        "FOLLOW_STEER_RATIO": FOLLOW_STEER_RATIO,
        "SPIN_SPEED": SPIN_SPEED,
        "K_LIN": K_LIN, "CRUISE_LINEAR_MPS": CRUISE_LINEAR_MPS,
        "LEAN_FOLLOW_STEER_SCALE": LEAN_FOLLOW_STEER_SCALE,
        "LEAN_SINGLE_WALL_SCALE": LEAN_SINGLE_WALL_SCALE,
        "LEAN_WEAK_EVIDENCE_SPEED_MPS": LEAN_WEAK_EVIDENCE_SPEED_MPS,
        "LEAN_LARGE_LAT_ERR_M": LEAN_LARGE_LAT_ERR_M,
        "MAX_LINEAR_MPS": MAX_LINEAR_MPS,
        "GATEWAY_LINEAR_MPS": GATEWAY_LINEAR_MPS,
        "SENSOR_CAP_M": SENSOR_CAP_M, "SAT_MODE": SAT_MODE,
        "FOLLOW_ESTABLISH_S": FOLLOW_ESTABLISH_S,
        "GAP_OPEN_S": GAP_OPEN_S,
        "FOLLOW_SIDE_PREFERENCE": FOLLOW_SIDE_PREFERENCE,
        "SIDE_ROUTE_MIN_M": SIDE_ROUTE_MIN_M,
        "SIDE_ROUTE_TIE_DB": SIDE_ROUTE_TIE_DB,
        "SIDE_OPEN_DWELL_S": SIDE_OPEN_DWELL_S,
        "SIDE_OPEN_CONFIRM_S": SIDE_OPEN_CONFIRM_S,
        "ROUTE_BUFFER_M": ROUTE_BUFFER_M,
        "JUNCTION_ADVANCE_M": JUNCTION_ADVANCE_M,
        "FRONT_BRANCH_SIDE_MAX": FRONT_BRANCH_SIDE_MAX,
        "ROUTE_BUFFER_SPEED_MPS": ROUTE_BUFFER_SPEED_MPS,
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
        "GATEWAY_FRONT_STEER_MAX": GATEWAY_FRONT_STEER_MAX,
        "GATEWAY_APPROACH_M": GATEWAY_APPROACH_M,
        "GATEWAY_TIMEOUT": GATEWAY_TIMEOUT,
        "KP_HEADING": KP_HEADING, "STEER_HEADING_MAX": STEER_HEADING_MAX,
        "HEADING_STEER_FADE_START": HEADING_STEER_FADE_START,
        "HEADING_STEER_FADE_FULL": HEADING_STEER_FADE_FULL,
        "MAX_RANGE": MAX_RANGE, "FILTER_TAU": FILTER_TAU,
        "TOF_HOLD_S": TOF_HOLD_S, "WALL_TARGET": WALL_TARGET,
        "SINGLE_WALL_TARGET_M": SINGLE_WALL_TARGET_M,
        "WEDGE_ENTER": WEDGE_ENTER, "WEDGE_EXIT": WEDGE_EXIT,
        "WEDGE_ENTER_T": WEDGE_ENTER_T, "WEDGE_MIN_DWELL": WEDGE_MIN_DWELL,
        "WEDGE_MAX_T": WEDGE_MAX_T,
        "WEDGE_REPEAT_WINDOW_S": WEDGE_REPEAT_WINDOW_S,
        "WEDGE_TURN_AFTER": WEDGE_TURN_AFTER,
        "WEDGE_CLEARANCE_TIE_DB": WEDGE_CLEARANCE_TIE_DB,
        "TURN_KP": TURN_KP, "TURN_KI": TURN_KI,
        "TURN_KD_RATE": TURN_KD_RATE,
        "TURN_INTEGRAL_MAX": TURN_INTEGRAL_MAX,
        "TURN_MIN_W": TURN_MIN_W,
        "TURN_PD_ASSIST_GAIN": TURN_PD_ASSIST_GAIN,
        "TURN_PD_ASSIST_MAX_W": TURN_PD_ASSIST_MAX_W,
        "TURN_TOF_JUMP_M": TURN_TOF_JUMP_M,
        "TURN_TOF_OPEN_M": TURN_TOF_OPEN_M,
        "TURN_TOF_MIN_ANGLE_DEG": math.degrees(TURN_TOF_MIN_ANGLE),
        "TURN_TOF_DWELL_S": TURN_TOF_DWELL_S,
        "TURN_EXIT_ERR": TURN_EXIT_ERR,
        "TURN_MAX_RETRIES": TURN_MAX_RETRIES,
        "POST_TURN_SETTLE_S": POST_TURN_SETTLE_S,
        "POST_TURN_VERIFY_MAX_S": POST_TURN_VERIFY_MAX_S,
        "POST_TURN_ADVANCE_M": POST_TURN_ADVANCE_M,
        "POST_TURN_SPEED_MPS": POST_TURN_SPEED_MPS,
        "POST_TURN_CENTER_GAIN": POST_TURN_CENTER_GAIN,
        "POST_TURN_CENTER_MAX_W": POST_TURN_CENTER_MAX_W,
        "POST_TURN_HEADING_GAIN": POST_TURN_HEADING_GAIN,
        "POST_TURN_SEEK_BIAS_W": POST_TURN_SEEK_BIAS_W,
        "POST_TURN_FRONT_AVOID_GAIN": POST_TURN_FRONT_AVOID_GAIN,
        "POST_TURN_FRONT_AVOID_MAX_W": POST_TURN_FRONT_AVOID_MAX_W,
        "POST_TURN_OPEN_SIDE_GAIN": POST_TURN_OPEN_SIDE_GAIN,
        "POST_TURN_OPEN_SIDE_MAX_W": POST_TURN_OPEN_SIDE_MAX_W,
        "ACQUIRE_SINGLE_WALL_GAIN": ACQUIRE_SINGLE_WALL_GAIN,
        "ACQUIRE_CENTER_DWELL_S": ACQUIRE_CENTER_DWELL_S,
        "ACQUIRE_ONE_WALL_DWELL_S": ACQUIRE_ONE_WALL_DWELL_S,
        "POST_TURN_HEADING_TOL": POST_TURN_HEADING_TOL,
        "POST_TURN_LATERAL_TOL": POST_TURN_LATERAL_TOL,
        "POST_TURN_CLEAR_DWELL_S": POST_TURN_CLEAR_DWELL_S,
        "POST_TURN_BLOCK_DWELL_S": POST_TURN_BLOCK_DWELL_S,
        "CORRIDOR_ENTRY_DISTANCE_M": CORRIDOR_ENTRY_DISTANCE_M,
        "CORRIDOR_ENTRY_CLEAR_DWELL_S": CORRIDOR_ENTRY_CLEAR_DWELL_S,
        "POST_TURN_CENTER_BLOCK_DIST": POST_TURN_CENTER_BLOCK_DIST,
        "POST_TURN_SAFE_FRONT_M": POST_TURN_SAFE_FRONT_M,
        "POST_TURN_HOLD_S": POST_TURN_HOLD_S,
        "POST_TURN_SENSOR_WAIT_MAX_S": POST_TURN_SENSOR_WAIT_MAX_S,
        "POST_TURN_WIDE_MOUTH_M": POST_TURN_WIDE_MOUTH_M,
        "POST_TURN_BLOCK_RECOVERY_S": POST_TURN_BLOCK_RECOVERY_S,
        "POST_TURN_EMERGENCY_RECOVERY_S": POST_TURN_EMERGENCY_RECOVERY_S,
        "POST_TURN_MAX_HOLD_S": POST_TURN_MAX_HOLD_S,
        "POST_TURN_NO_MOTION_MAX_S": POST_TURN_NO_MOTION_MAX_S,
        "TURN_ENTRY_LATERAL_TOL": TURN_ENTRY_LATERAL_TOL,
        "TURN_ENTRY_CENTER_DWELL_S": TURN_ENTRY_CENTER_DWELL_S,
        "TURN_ENTRY_MAX_FRONT_ASYM_M": TURN_ENTRY_MAX_FRONT_ASYM_M,
        "TURN_TIMEOUT_BASE": TURN_TIMEOUT_BASE,
        "TURN_TIMEOUT_PER_RAD": TURN_TIMEOUT_PER_RAD,
        "NO_PROGRESS_START": NO_PROGRESS_START,
        "NO_PROGRESS_WIN": NO_PROGRESS_WIN,
        "NO_PROGRESS_MIN": NO_PROGRESS_MIN,
        "BRAKE_W": BRAKE_W, "BRAKE_EXIT_GYRO": BRAKE_EXIT_GYRO,
        "BRAKE_TIMEOUT": BRAKE_TIMEOUT,
        "TURN_STOP_DWELL_S": TURN_STOP_DWELL_S,
        "RETRIM_TOL": RETRIM_TOL,
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
    """Lean maze controller.

    Intentional state machine:
      GATEWAY -> FOLLOW -> ROUTE_BUFFER -> REVERSE -> TURN_ENTRY_STOP
              -> TURN -> BRAKE -> POST_TURN_ADVANCE -> FOLLOW

    Rules are deliberately simple:
      * follow/correct from side ToF when available;
      * route from front blockage + side openings;
      * use gyro targets for turns; ToF never rewrites turn targets;
      * back away by measured front-clearance before spinning;
      * hold after rotation and never creep into a blocked outgoing corridor.
    """

    BACKUP_S = 0.35
    REVERSE_S = 0.35
    RESUME_DIST = 0.18
    MIN_SPIN_S = 0.20

    def __init__(self):
        cap = SENSOR_CAP_M if SAT_MODE == "ceiling" else None
        self.tof_fl = Tof(MAX_RANGE, TOF_HOLD_S, FILTER_TAU, cap)
        self.tof_fr = Tof(MAX_RANGE, TOF_HOLD_S, FILTER_TAU, cap)
        self.tof_sl = Tof(MAX_RANGE, TOF_HOLD_S, FILTER_TAU, cap)
        self.tof_sr = Tof(MAX_RANGE, TOF_HOLD_S, FILTER_TAU, cap)
        self.perception = WallPerception(side_dwell=0.08)

        self.t = 0.0
        self.state = "GATEWAY"
        self.state_reason = "init"
        self.observation = None
        self.last_status = ("none",) * 4
        self.fl_f = self.fr_f = self.sl_f = self.sr_f = None
        self.fl_forward = self.fr_forward = None
        self.front_clear = None
        self.front_center_clear = None
        self.exit_set = "{}"

        self.wall_target = WALL_TARGET
        self.single_wall_target = SINGLE_WALL_TARGET_M
        self.lat_mode = "blind"
        self.lat_prev_error = None
        self.lat_deriv_filt = 0.0
        self.lat_integral = 0.0
        self.steer_lat = 0.0
        self.steer_front = 0.0
        self.steer_heading = 0.0
        self.e_heading = 0.0
        self.yaw_cmd = 0.0
        self.stuck_flag = False
        self.stuck = StuckDetector(STUCK_WIN_S, STUCK_CMD_MIN, STUCK_RATIO)

        self.follow_side = 0.0
        self.junction_stage = "none"
        self.junction_decision = "none"
        self.turn_outcome = "none"
        # Discrete maze heading: 0/2 are the original/opposite travel axes,
        # 1/3 are the perpendicular axes.  This is updated only after a
        # completed gyro turn, never while the turn PID is active.
        self.heading_quadrant = 0
        self.opposite_direction = False
        self.abort_reason = ""

        self.gateway_distance = 0.0
        self.route_dir = 0.0
        self.route_hint = 0.0
        self.route_buffer_distance = 0.0
        self.reverse_t = 0.0
        self.side_wall_seen = {1.0: False, -1.0: False}
        self.side_open_t = {1.0: 0.0, -1.0: 0.0}
        self.side_route_dir = 0.0
        self.turn_entry_front_ref = None
        self.turn_entry_stop_t = 0.0
        self.post_turn_distance = 0.0
        self.post_turn_t = 0.0
        self.post_turn_hold_t = 0.0
        self.post_turn_no_motion_t = 0.0
        self.post_turn_block_t = 0.0
        self.front_single_t = 0.0
        self.post_turn_grace_t = 0.0
        self.acquire_center_t = 0.0
        self.acquire_wall_t = 0.0
        self.turn_entry_center_t = 0.0
        self.forward_watch_t = 0.0
        self.forward_watch_ref = None
        self.forward_recovery_only = False
        self.forward_recovery_escape = False
        self.forward_recovery_dir = 0.0
        self.forward_recovery_count = 0
        self.forward_recovery_signature = None
        self.forward_recovery_escape = False
        self.forward_recovery_dir = 0.0
        self.forward_recovery_count = 0
        self.forward_recovery_signature = None
        self.forward_recovery_t = -999.0
        self.front_recovery_signature = None
        self.front_recovery_count = 0
        self.front_recovery_t = -999.0

        self.turn_active = False
        self.turn_entry = 0.0
        self.turn_target = 0.0
        self.turn_orig = 0.0
        self.turn_angle = 0.0
        self.turn_error = 0.0
        self.turn_t = 0.0
        self.turn_progress = 0.0
        self.turn_integral = 0.0
        self.turn_w = 0.0
        self.brake_t = 0.0
        self.brake_w = 0.0

        # Legacy/log compatibility only.
        self.gateway_active = False
        self.gateway_wall_follow_active = False
        self.route_latched = False
        self.corridor_entry_active = False
        self.flood = FloodFillPlanner()

    def _set_state(self, state, reason=""):
        if state != self.state:
            self.state = state
            self.state_reason = reason

    def _wrap(self, a):
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def _valid(self, v, status):
        return status == "valid" and v is not None

    def _clamp(self, x, lo, hi):
        return max(lo, min(hi, x))

    def _filter(self, fl, fr, sl, sr, dt):
        self.fl_f, fl_s = self.tof_fl.update(fl, dt)
        self.fr_f, fr_s = self.tof_fr.update(fr, dt)
        self.sl_f, sl_s = self.tof_sl.update(sl, dt)
        self.sr_f, sr_s = self.tof_sr.update(sr, dt)
        self.last_status = (fl_s, fr_s, sl_s, sr_s)
        self.fl_forward = (self.fl_f * FRONT_RAY_COS
                           if self._valid(self.fl_f, fl_s) else None)
        self.fr_forward = (self.fr_f * FRONT_RAY_COS
                           if self._valid(self.fr_f, fr_s) else None)
        fronts = [v for v in (self.fl_forward, self.fr_forward)
                  if v is not None]
        self.front_clear = min(fronts) if fronts else None
        self.front_center_clear = (sum(fronts) / 2.0
                                   if len(fronts) == 2 else None)
        self.observation = self.perception.update(
            (self.fl_f, self.fr_f, self.sl_f, self.sr_f),
            self.last_status, dt, FRONT_RAY_COS)

    def _side_open(self, side):
        v = self.sl_f if side > 0 else self.sr_f
        st = self.last_status[2] if side > 0 else self.last_status[3]
        label = self.observation.left if side > 0 else self.observation.right
        return label == "open" or (self._valid(v, st) and v >= SIDE_ROUTE_MIN_M)

    def _side_wall(self, side):
        v = self.sl_f if side > 0 else self.sr_f
        st = self.last_status[2] if side > 0 else self.last_status[3]
        return self._valid(v, st) and v <= 0.16

    def _side_open_candidate(self, dt):
        """Return a latched side opening after wall-loss confirmation."""
        candidates = []
        for side in (1.0, -1.0):
            if self._side_wall(side):
                self.side_wall_seen[side] = True
                self.side_open_t[side] = 0.0
                continue
            if self.side_wall_seen[side] and self._side_open(side):
                self.side_open_t[side] += dt
            else:
                self.side_open_t[side] = 0.0
            if self.side_open_t[side] >= SIDE_OPEN_CONFIRM_S:
                candidates.append(side)
        if not candidates:
            return 0.0
        if len(candidates) == 1:
            return candidates[0]
        sl = self.sl_f if self.sl_f is not None else 0.0
        sr = self.sr_f if self.sr_f is not None else 0.0
        return 1.0 if sl >= sr else -1.0

    def _front_blocked(self):
        if self.fl_forward is None or self.fr_forward is None:
            return False
        return (self.fl_forward <= FRONT_STOP_DIST
                and self.fr_forward <= FRONT_STOP_DIST)

    def _front_emergency(self):
        # WallPerception is the single authority for hazard classification.
        # Do not reinterpret the same rays here with a second threshold set.
        return self.observation.hazard == "emergency"

    def _repeated_front_emergency(self, dt):
        """Track unchanged emergency geometry without changing first response."""
        if not self._front_emergency():
            # Preserve the last emergency signature through the short reverse
            # interval; the recovery tick commonly clears the hazard label
            # even though it has not changed the local geometry.
            return False
        values = (self.fl_forward, self.fr_forward, self.sl_f, self.sr_f)
        if any(v is None for v in values):
            self.front_recovery_signature = None
            self.front_recovery_count = 0
            return False
        signature = tuple(round(v, 2) for v in values)
        repeated = (
            self.front_recovery_signature is not None
            and self.t - self.front_recovery_t <= 12.0
            and all(abs(a - b) <= 0.02 for a, b in
                    zip(signature, self.front_recovery_signature)))
        self.front_recovery_count = (
            self.front_recovery_count + 1 if repeated else 1)
        self.front_recovery_signature = signature
        self.front_recovery_t = self.t
        return self.front_recovery_count >= 2

    def _front_single_route(self, dt):
        if self.fl_forward is None or self.fr_forward is None:
            self.front_single_t = 0.0
            self.route_hint = 0.0
            return False
        left_near = self.fl_forward <= SINGLE_RAY_ROUTE_DIST
        right_near = self.fr_forward <= SINGLE_RAY_ROUTE_DIST
        one_near = left_near != right_near
        corridor = self._side_wall(1.0) and self._side_wall(-1.0)
        if one_near and corridor:
            self.front_single_t += dt
        else:
            self.front_single_t = 0.0
            self.route_hint = 0.0
        if self.front_single_t >= 0.25:
            # A close splayed ray means the opposite side is the available
            # branch/turn direction: close right ray -> turn left.
            self.route_hint = 1.0 if right_near else -1.0
            return True
        return False

    def _choose_route(self):
        if self.side_route_dir != 0.0:
            return self.side_route_dir, math.pi / 2.0
        left_open = self._side_open(1.0)
        right_open = self._side_open(-1.0)
        if left_open and not right_open:
            return 1.0, math.pi / 2.0
        if right_open and not left_open:
            return -1.0, math.pi / 2.0
        if left_open and right_open:
            sl = self.sl_f if self.sl_f is not None else 0.0
            sr = self.sr_f if self.sr_f is not None else 0.0
            return (1.0 if sl >= sr else -1.0), math.pi / 2.0
        if self.route_hint != 0.0:
            return self.route_hint, math.pi / 2.0
        # Dead end: direct U-turn, no blind left probe.
        return 1.0, math.pi

    def _forward_progress_stalled(self, dt):
        """Heuristic translation watchdog for a chassis wedged while driving."""
        values = (self.fl_forward, self.fr_forward, self.sl_f, self.sr_f)
        if (self.front_clear is None
                or self.front_clear > FORWARD_WATCHDOG_FRONT_M
                or any(v is None for v in values)):
            self.forward_watch_t = 0.0
            self.forward_watch_ref = None
            return False
        if self.forward_watch_ref is None:
            self.forward_watch_ref = values
            self.forward_watch_t = 0.0
            return False
        movement = max(abs(a - b)
                       for a, b in zip(values, self.forward_watch_ref))
        if movement <= FORWARD_WATCHDOG_DELTA_M:
            self.forward_watch_t += dt
        else:
            self.forward_watch_ref = values
            self.forward_watch_t = 0.0
        return self.forward_watch_t >= FORWARD_WATCHDOG_S

    def _escape_direction(self):
        left_open = self._side_open(1.0)
        right_open = self._side_open(-1.0)
        if left_open and not right_open:
            return 1.0
        if right_open and not left_open:
            return -1.0
        if self.route_hint != 0.0:
            return self.route_hint
        return FOLLOW_SIDE_PREFERENCE

    def _start_turn(self, direction, magnitude, reason="junction"):
        self.route_dir = direction
        self.turn_target = direction * magnitude
        self.turn_orig = self.turn_target
        self.turn_entry = 0.0
        self.turn_angle = 0.0
        self.turn_error = self.turn_target
        self.turn_t = 0.0
        self.turn_integral = 0.0
        self.turn_w = 0.0
        self.turn_active = False
        self.turn_outcome = "active"
        self.abort_reason = ""
        self.junction_decision = (
            "uturn" if abs(magnitude - math.pi) < 0.01
            else "left" if direction > 0 else "right")
        self._set_state("TURN", reason)

    def _record_completed_heading(self):
        """Commit the new travel heading after gyro control has finished."""
        quarter_turns = max(1, int(round(abs(self.turn_orig) /
                                         (math.pi / 2.0))))
        signed_quarters = quarter_turns if self.turn_orig > 0 else -quarter_turns
        self.heading_quadrant = (self.heading_quadrant + signed_quarters) % 4
        self.opposite_direction = self.heading_quadrant == 2
        # Wall-PID memory belongs to the old corridor frame.
        self.follow_side = 0.0
        self.lat_prev_error = None
        self.lat_deriv_filt = 0.0
        self.side_wall_seen = {1.0: False, -1.0: False}
        self.side_open_t = {1.0: 0.0, -1.0: 0.0}
        self.side_route_dir = 0.0

    def _compose_follow_steer(self, dt):
        self.steer_lat = 0.0
        self.steer_front = 0.0
        self.steer_heading = 0.0
        self.lat_mode = "blind"
        e_lat = None
        e_front = None

        sl_ok = self._valid(self.sl_f, self.last_status[2])
        sr_ok = self._valid(self.sr_f, self.last_status[3])
        if (sl_ok and sr_ok and self.sl_f <= SIDE_CENTER_MAX_M
                and self.sr_f <= SIDE_CENTER_MAX_M):
            e_lat = self.sl_f - self.sr_f
            e_eff = 0.0 if abs(e_lat) < LAT_DEADBAND else (
                e_lat - math.copysign(LAT_DEADBAND, e_lat))
            if self.lat_prev_error is None or dt <= 0:
                raw_d = 0.0
            else:
                raw_d = self._clamp((e_lat - self.lat_prev_error) / dt,
                                    -LAT_DERIV_MAX, LAT_DERIV_MAX)
            self.lat_deriv_filt += LAT_DERIV_ALPHA * (raw_d - self.lat_deriv_filt)
            self.lat_prev_error = e_lat
            self.steer_lat = LEAN_FOLLOW_STEER_SCALE * (
                KP_LAT * e_eff + KD_LAT * self.lat_deriv_filt)
            self.lat_mode = "two_wall"
        elif sl_ok and self.sl_f <= 0.18:
            self.steer_lat = (LEAN_SINGLE_WALL_SCALE * KP_LAT
                              * (self.sl_f - self.single_wall_target))
            self.lat_prev_error = None
            self.lat_deriv_filt = 0.0
            self.follow_side = 1.0
            self.lat_mode = "left_wall"
        elif sr_ok and self.sr_f <= 0.18:
            self.steer_lat = (-LEAN_SINGLE_WALL_SCALE * KP_LAT
                              * (self.sr_f - self.single_wall_target))
            self.lat_prev_error = None
            self.lat_deriv_filt = 0.0
            self.follow_side = -1.0
            self.lat_mode = "right_wall"
        else:
            self.lat_prev_error = None
            self.lat_deriv_filt = 0.0

        if self._valid(self.fl_forward, "valid") and self._valid(self.fr_forward, "valid"):
            e_front = self.fl_forward - self.fr_forward
            single_ray_geometry = (
                min(self.fl_forward, self.fr_forward) <= FRONT_JUNCTION_DIST
                and max(self.fl_forward, self.fr_forward) > FRONT_JUNCTION_DIST)
            if (not single_ray_geometry
                    and self.front_center_clear is not None
                    and self.front_center_clear < FRONT_ALIGN_MAX_DIST
                    and abs(e_front) < FRONT_ALIGN_SYM_DB):
                self.steer_front = KP_FRONT * e_front

        steer = self._clamp(self.steer_lat + self.steer_front,
                            -MAX_STEER, MAX_STEER)
        return steer, e_lat, e_front

    def _finish(self, left, right, steer, e_lat=None, e_front=None, dt=0.0,
                yaw_rate=0.0):
        left = self._clamp(left, -MAX_SPEED, MAX_SPEED)
        right = self._clamp(right, -MAX_SPEED, MAX_SPEED)
        self.yaw_cmd = YAW_GAIN_K * (right - left)
        self.stuck_flag = self.stuck.update(self.yaw_cmd, yaw_rate, dt)
        return left, right, e_lat, e_front, steer

    def update(self, fl, fr, sl, sr, yaw_rate, dt):
        dt = dt if dt and dt > 0 else 0.0
        self.t += dt
        self._filter(fl, fr, sl, sr, dt)
        # Corridor PID owns only forward corridor motion.  In particular, do
        # not advance its derivative/state while the gyro turn or brake owns
        # the wheels; stale wall correction can otherwise fight turn exit.
        if self.state in ("GATEWAY", "FOLLOW"):
            steer, e_lat, e_front = self._compose_follow_steer(dt)
        else:
            steer, e_lat, e_front = 0.0, None, None

        if self.state == "GATEWAY":
            v = min(GATEWAY_LINEAR_MPS, MAX_LINEAR_MPS)
            base = v / K_LIN
            self.gateway_distance += v * dt
            # Move through the spawn posts into the actual first corridor.
            # The spawn signature is symmetric front blockage + wide sides;
            # routing immediately from that looks like a hardcoded turn.  Hand
            # off only once at least one side wall is visible, or after a
            # bounded approach distance as a fail-safe.
            # Do not hand the spawn's symmetric front posts to FOLLOW merely
            # because a distance timer expired.  Normal route logic must wait
            # for an actual side-wall/corridor signature.
            if (self.observation.left == "wall"
                    or self.observation.right == "wall"):
                self._set_state("FOLLOW", "gateway_clear")
            return self._finish(base - steer, base + steer, steer,
                                e_lat, e_front, dt, yaw_rate)

        if self.state == "ROUTE_BUFFER":
            v = min(ROUTE_BUFFER_SPEED_MPS, MAX_LINEAR_MPS)
            base = v / K_LIN
            self.route_buffer_distance += v * dt
            if self.route_buffer_distance >= ROUTE_BUFFER_M or self._front_blocked():
                self.route_buffer_distance = 0.0
                self.reverse_t = 0.0
                self.turn_entry_front_ref = self.front_center_clear
                self._set_state("REVERSE", "blocked")
            return self._finish(base, base, 0.0, e_lat, e_front, dt, yaw_rate)

        if self.state in ("REVERSE", "BACKUP"):
            self.reverse_t += dt
            ref = self.turn_entry_front_ref
            gained_clearance = (
                ref is not None and self.front_center_clear is not None
                and self.front_center_clear >= ref + TURN_ENTRY_BACKOFF_M)
            # For a watchdog/backout without a usable front reference, retain
            # the old bounded timer fallback.  Normal junction turns use the
            # measured clearance target instead of a fixed reverse duration.
            backoff_done = gained_clearance or (
                ref is None and self.reverse_t >= self.REVERSE_S)
            if backoff_done or self.reverse_t >= TURN_ENTRY_BACKOFF_MAX_S:
                if self.forward_recovery_escape:
                    # A second matching stall means the first reverse did
                    # not change the local geometry.  Stop treating this as
                    # ordinary recovery: use the remembered escape side and
                    # enter the normal turn funnel.
                    self.route_hint = self.forward_recovery_dir
                    self.forward_recovery_escape = False
                    self.forward_recovery_only = False
                    self.forward_recovery_dir = 0.0
                    self.turn_entry_stop_t = 0.0
                    self._set_state("TURN_ENTRY_STOP", "forward_stalled_escape")
                    return self._finish(0.0, 0.0, 0.0,
                                        e_lat, e_front, dt, yaw_rate)
                if self.forward_recovery_only:
                    self.forward_recovery_only = False
                    self.forward_recovery_dir = 0.0
                    self.post_turn_grace_t = 0.5
                    self._set_state("FOLLOW", "forward_recovered")
                    return self._finish(0.0, 0.0, 0.0,
                                        e_lat, e_front, dt, yaw_rate)
                self.turn_entry_stop_t = 0.0
                self._set_state("TURN_ENTRY_STOP", "backoff_complete")
                return self._finish(0.0, 0.0, 0.0, e_lat, e_front, dt, yaw_rate)
            return self._finish(-REVERSE_WHEEL_W, -REVERSE_WHEEL_W, 0.0,
                                e_lat, e_front, dt, yaw_rate)

        if self.state == "TURN_ENTRY_STOP":
            self.turn_entry_stop_t += dt
            if self.turn_entry_stop_t >= TURN_ENTRY_STOP_S:
                direction, magnitude = self._choose_route()
                self._start_turn(direction, magnitude,
                                 "junction_" + ("left" if direction > 0 else "right"))
            return self._finish(0.0, 0.0, 0.0, e_lat, e_front, dt, yaw_rate)

        if self.state == "TURN":
            return self._run_turn(yaw_rate, dt, e_lat, e_front)

        if self.state == "BRAKE":
            return self._run_brake(yaw_rate, dt, e_lat, e_front)

        if self.state == "POST_TURN_ADVANCE":
            return self._run_post_turn(dt, yaw_rate, e_lat, e_front)

        # FOLLOW: one place decides route.  Near front + any side opening means
        # approach the corner, then reverse slightly and turn.  If no side is
        # open, it is a dead end and the same path chooses a U-turn.
        if self.post_turn_grace_t > 0.0:
            self.post_turn_grace_t = max(0.0, self.post_turn_grace_t - dt)
        blocked_route = self._front_blocked()
        single_ray_route = self._front_single_route(dt)
        side_candidate = self._side_open_candidate(dt)
        if side_candidate != 0.0:
            self.side_route_dir = side_candidate
        left_open = self._side_open(1.0)
        right_open = self._side_open(-1.0)
        emergency_front = self._front_emergency()
        # A persistent single-ray obstruction is actionable only when the
        # robot is confirmed inside a two-wall corridor.  The helper applies
        # its dwell and rejects transient splayed-ray readings before this
        # route candidate is accepted.
        route_candidate = (
            blocked_route
            or side_candidate != 0.0
            or single_ray_route)
        centered_for_entry = (e_lat is not None
                              and abs(e_lat) <= TURN_ENTRY_LATERAL_TOL)
        self.turn_entry_center_t = (
            self.turn_entry_center_t + dt
            if route_candidate and centered_for_entry else 0.0)
        entry_ready = (
            (blocked_route and self.front_center_clear is not None)
            or side_candidate != 0.0
            or self.turn_entry_center_t >= TURN_ENTRY_CENTER_DWELL_S)
        route_front = (
            self.post_turn_grace_t <= 0.0
            and route_candidate
            and entry_ready
        )
        # Emergency contact is a bounded reverse-only safety recovery, never
        # a direction or U-turn decision.
        if emergency_front and not route_front:
            if self._repeated_front_emergency(dt):
                # The first emergency keeps its existing safety response.  If
                # the same geometry survives that reverse, recovery has not
                # changed the situation; give route selection ownership
                # instead of repeating FOLLOW <-> REVERSE forever.
                self.route_hint = self._escape_direction()
                self.route_dir = self.route_hint
                self.route_buffer_distance = 0.0
                self.front_recovery_signature = None
                self.front_recovery_count = 0
                self.forward_recovery_only = False
                self.forward_recovery_escape = False
                self._set_state("ROUTE_BUFFER", "front_emergency_repeat")
                return self._finish(0.0, 0.0, 0.0, e_lat, e_front,
                                    dt, yaw_rate)
            self.forward_recovery_only = True
            self.forward_recovery_escape = False
            self.reverse_t = 0.0
            self.turn_entry_front_ref = self.front_center_clear
            self._set_state("REVERSE", "front_emergency")
            return self._finish(0.0, 0.0, 0.0, e_lat, e_front,
                                dt, yaw_rate)
        # Protect against a physical forward wedge even when the classifier
        # says the front is clear.  The watchdog itself still requires a
        # near-front geometry and frozen sensor values, and the longer dwell
        # prevents ordinary corridor motion from creating a recovery turn.
        if self._forward_progress_stalled(dt):
            signature = tuple(round(v, 2) for v in
                             (self.fl_forward, self.fr_forward,
                              self.sl_f, self.sr_f))
            repeated = (
                self.forward_recovery_signature is not None
                and self.t - self.forward_recovery_t <= 12.0
                and all(abs(a - b) <= 0.02 for a, b in
                        zip(signature, self.forward_recovery_signature)))
            self.forward_recovery_count = (
                self.forward_recovery_count + 1 if repeated else 1)
            self.forward_recovery_signature = signature
            self.forward_recovery_t = self.t
            self.forward_watch_t = 0.0
            self.forward_watch_ref = None
            self.route_buffer_distance = 0.0
            self.reverse_t = 0.0
            self.forward_recovery_escape = self.forward_recovery_count >= 2
            self.forward_recovery_dir = (
                self._escape_direction()
                if self.forward_recovery_escape else 0.0)
            self.forward_recovery_only = not self.forward_recovery_escape
            self.turn_entry_front_ref = self.front_center_clear
            self._set_state("REVERSE", "forward_stalled")
            return self._finish(0.0, 0.0, 0.0, e_lat, e_front,
                                dt, yaw_rate)
        if route_front:
            self.turn_entry_center_t = 0.0
            self.forward_recovery_only = False
            self.forward_recovery_escape = False
            self.forward_recovery_count = 0
            self.forward_recovery_signature = None
            self.route_dir, _ = self._choose_route()
            self.route_buffer_distance = 0.0
            self._set_state("ROUTE_BUFFER", "front_blocked")
            return self._finish(0.0, 0.0, 0.0, e_lat, e_front, dt, yaw_rate)

        # Slow down as front gets close; otherwise cruise.
        v = CRUISE_LINEAR_MPS
        two_wall = self._side_wall(1.0) and self._side_wall(-1.0)
        if not two_wall or (e_lat is not None
                            and abs(e_lat) >= LEAN_LARGE_LAT_ERR_M):
            v = min(v, LEAN_WEAK_EVIDENCE_SPEED_MPS)
        if self.front_clear is not None and self.front_clear < 0.20:
            scale = self._clamp((self.front_clear - FRONT_EMERGENCY_DIST) / 0.12,
                                0.35, 1.0)
            v *= scale
        base = min(v, MAX_LINEAR_MPS) / K_LIN
        limit = FOLLOW_STEER_RATIO * base
        steer = self._clamp(steer, -limit, limit)
        self._set_state("FOLLOW", "approach")
        return self._finish(base - steer, base + steer, steer,
                            e_lat, e_front, dt, yaw_rate)

    def _run_turn(self, yaw_rate, dt, e_lat, e_front):
        if not self.turn_active:
            self.turn_active = True
            self.turn_entry = 0.0
            self.turn_angle = 0.0
            self.turn_t = 0.0
            self.turn_integral = 0.0
            self.stuck.reset()
        self.turn_t += dt
        self.turn_angle += yaw_rate * dt
        self.turn_error = self.turn_target - self.turn_angle
        if self.turn_t >= 1.4 and abs(self.turn_angle) < NO_PROGRESS_MIN:
            self.abort_reason = "no_progress"
            self.turn_outcome = "aborted"
            self.reverse_t = 0.0
            self._set_state("BACKUP", "no_progress")
            return self._finish(0.0, 0.0, 0.0, e_lat, e_front, dt, yaw_rate)
        self.turn_integral = self._clamp(
            self.turn_integral + self.turn_error * dt,
            -TURN_INTEGRAL_MAX, TURN_INTEGRAL_MAX)
        if abs(self.turn_error) <= TURN_EXIT_ERR and self.turn_t >= self.MIN_SPIN_S:
            self.brake_t = 0.0
            # Lean controller: stop commanding spin and let residual trim finish
            # if passive coast does not settle close enough.  The old active
            # counter-brake made the local plant unwind 5-12 deg.
            self.brake_w = 0.0
            self._set_state("BRAKE", "target")
            return self._finish(0.0, 0.0, 0.0, e_lat, e_front, dt, yaw_rate)
        timeout = TURN_TIMEOUT_BASE + TURN_TIMEOUT_PER_RAD * abs(self.turn_target)
        if self.turn_t > timeout:
            self.abort_reason = "no_progress"
            self.turn_outcome = "aborted"
            self.reverse_t = 0.0
            self._set_state("BACKUP", "no_progress")
            return self._finish(0.0, 0.0, 0.0, e_lat, e_front, dt, yaw_rate)
        w = TURN_KP * self.turn_error + TURN_KI * self.turn_integral - TURN_KD_RATE * yaw_rate
        if abs(w) < TURN_MIN_W:
            w = math.copysign(TURN_MIN_W, w if w != 0 else self.turn_error)
        w = self._clamp(w, -SPIN_SPEED, SPIN_SPEED)
        # Smooth wheel command changes.
        max_step = TURN_ACCEL_W * dt
        self.turn_w += self._clamp(w - self.turn_w, -max_step, max_step)
        return self._finish(-self.turn_w, self.turn_w, 0.0,
                            e_lat, e_front, dt, yaw_rate)

    def _run_brake(self, yaw_rate, dt, e_lat, e_front):
        self.brake_t += dt
        self.turn_angle += yaw_rate * dt
        self.turn_error = self.turn_target - self.turn_angle
        if self.brake_t >= 0.10 and abs(self.turn_error) <= RETRIM_TOL:
            self.turn_outcome = "complete"
            self.turn_active = False
            self._record_completed_heading()
            self.post_turn_distance = 0.0
            self.post_turn_t = 0.0
            self.post_turn_hold_t = 0.0
            self.post_turn_no_motion_t = 0.0
            self.post_turn_block_t = 0.0
            self.acquire_center_t = 0.0
            self.acquire_wall_t = 0.0
            self._set_state("POST_TURN_ADVANCE", "turn_complete")
            return self._finish(0.0, 0.0, 0.0, e_lat, e_front, dt, yaw_rate)
        if self.brake_t >= BRAKE_TIMEOUT:
            if abs(self.turn_error) <= RETRIM_TOL:
                self.turn_outcome = "complete"
                self.turn_active = False
                self._record_completed_heading()
                self.post_turn_distance = 0.0
                self.post_turn_t = 0.0
                self.post_turn_hold_t = 0.0
                self.post_turn_no_motion_t = 0.0
                self.post_turn_block_t = 0.0
                self.acquire_center_t = 0.0
                self.acquire_wall_t = 0.0
                self._set_state("POST_TURN_ADVANCE", "turn_complete")
                return self._finish(0.0, 0.0, 0.0, e_lat, e_front, dt, yaw_rate)
            # Still short: resume the same gyro target for a residual trim.
            self.brake_t = 0.0
            self._set_state("TURN", "retrim")
            return self._finish(0.0, 0.0, 0.0, e_lat, e_front, dt, yaw_rate)
        return self._finish(0.0, 0.0, 0.0, e_lat, e_front, dt, yaw_rate)

    def _run_post_turn(self, dt, yaw_rate, e_lat, e_front):
        # The first part of the exit is deliberately stationary.  The front
        # rays are unreliable while rotating, so collect a fresh pair before
        # allowing translation.  In particular, never push forward through an
        # emergency/blocked centre reading just because the turn angle was
        # correct.
        self.post_turn_hold_t += dt
        if self.observation.hazard in ("blocked_pair", "emergency"):
            self.post_turn_block_t += dt
        else:
            self.post_turn_block_t = 0.0
        wide_mouth = (
            self.sl_f is not None and self.sr_f is not None
            and self.sl_f >= POST_TURN_WIDE_MOUTH_M
            and self.sr_f >= POST_TURN_WIDE_MOUTH_M)
        front_safe = (
            self.front_center_clear is None
            or self.front_center_clear >= POST_TURN_SAFE_FRONT_M
            or (wide_mouth and self.front_center_clear >= FRONT_EMERGENCY_DIST))
        if self.post_turn_hold_t < POST_TURN_HOLD_S or not front_safe:
            # This branch commands zero wheels.  It is allowed to wait for
            # fresh post-turn rays briefly, but it must not become an
            # unbounded state just because the hazard label is ambiguous.
            self.post_turn_no_motion_t += dt
            if self.post_turn_no_motion_t >= POST_TURN_NO_MOTION_MAX_S:
                self.reverse_t = 0.0
                self.turn_entry_front_ref = self.front_center_clear
                self._set_state("REVERSE", "post_turn_no_motion")
                return self._finish(0.0, 0.0, 0.0, e_lat, e_front,
                                    dt, yaw_rate)
            # A turn can leave the splayed rays looking at the old corner
            # while the side mouth is still wide.  Creep through that mouth,
            # but once it closes and the front remains blocked, recover via
            # the normal bounded reverse/turn path instead of waiting forever.
            if (self.post_turn_hold_t >= POST_TURN_MAX_HOLD_S
                    and self.observation.hazard in ("blocked_pair", "emergency")):
                self.reverse_t = 0.0
                self.turn_entry_front_ref = self.front_center_clear
                self._set_state("REVERSE", "post_turn_timeout")
                return self._finish(0.0, 0.0, 0.0, e_lat, e_front,
                                    dt, yaw_rate)
            recovery_limit = (
                POST_TURN_EMERGENCY_RECOVERY_S
                if self.observation.hazard == "emergency"
                else POST_TURN_BLOCK_RECOVERY_S)
            if (self.post_turn_block_t >= recovery_limit
                    and self.observation.hazard in ("blocked_pair", "emergency")
                    and not wide_mouth):
                self.reverse_t = 0.0
                self.turn_entry_front_ref = self.front_center_clear
                self._set_state(
                    "REVERSE",
                    "post_turn_emergency" if self.observation.hazard == "emergency"
                    else "post_turn_blocked")
            return self._finish(0.0, 0.0, 0.0, e_lat, e_front, dt, yaw_rate)

        self.post_turn_no_motion_t = 0.0
        v = min(POST_TURN_SPEED_MPS, MAX_LINEAR_MPS)
        base = v / K_LIN
        self.post_turn_distance += v * dt
        self.post_turn_t += dt

        # Acquisition steering: gentle, bounded, no special-case loops.
        steer = 0.0
        left_wall = self._side_wall(1.0)
        right_wall = self._side_wall(-1.0)
        left_center = (self._valid(self.sl_f, self.last_status[2])
                       and self.sl_f <= SIDE_CENTER_MAX_M)
        right_center = (self._valid(self.sr_f, self.last_status[3])
                        and self.sr_f <= SIDE_CENTER_MAX_M)
        # Fresh post-turn centering only.  Do not reuse FOLLOW's PID output
        # from before the turn: the robot may now be travelling on the
        # perpendicular or opposite heading.
        if left_center and right_center:
            e_lat = self.sl_f - self.sr_f
            e_eff = 0.0 if abs(e_lat) < LAT_DEADBAND else (
                e_lat - math.copysign(LAT_DEADBAND, e_lat))
            post_lat = LEAN_FOLLOW_STEER_SCALE * KP_LAT * e_eff
            steer += POST_TURN_CENTER_GAIN * post_lat
        elif left_wall:
            # One-wall acquisition owns steering until two-wall PID is ready.
            # This pulls away from/gently toward the visible wall instead of
            # letting a splayed front ray dominate the corridor entry.
            steer += ACQUIRE_SINGLE_WALL_GAIN * KP_LAT * (
                self.sl_f - self.single_wall_target)
        elif right_wall:
            steer += -ACQUIRE_SINGLE_WALL_GAIN * KP_LAT * (
                self.sr_f - self.single_wall_target)
        if (self.fl_forward is not None and self.fr_forward is not None
                and min(self.fl_forward, self.fr_forward) < FRONT_JUNCTION_DIST):
            steer += self._clamp(-(self.fr_forward - self.fl_forward),
                                 -POST_TURN_FRONT_AVOID_MAX_W,
                                 POST_TURN_FRONT_AVOID_MAX_W)
        steer = self._clamp(steer, -POST_TURN_CENTER_MAX_W, POST_TURN_CENTER_MAX_W)

        centered = (e_lat is not None and abs(e_lat) <= POST_TURN_LATERAL_TOL)
        one_wall = left_center != right_center
        two_wall = left_center and right_center
        # Splayed front rays often see the corner/side wall right after a
        # correct turn.  Treat them as unsafe only at true contact range; do
        # not spend tens of seconds waiting for a perfect clear pair before
        # following through into the selected corridor.
        # The nearest front ray may still see the wall just rotated away
        # from.  Use the forward centre estimate for acquisition safety;
        # otherwise one grazing ray can reset the dwell forever even when the
        # outgoing corridor centre is clear.
        front_ok = (
            self.observation.hazard != "emergency"
            and (self.observation.hazard != "blocked_pair" or wide_mouth)
            and (self.front_center_clear is None
                 or self.front_center_clear >= POST_TURN_CENTER_BLOCK_DIST
                 or (wide_mouth and self.front_center_clear >= FRONT_EMERGENCY_DIST)))
        self.acquire_center_t = (
            self.acquire_center_t + dt if (two_wall and centered and front_ok)
            else 0.0)
        self.acquire_wall_t = (
            self.acquire_wall_t + dt
            if (one_wall and front_ok
                and self.observation.hazard != "emergency") else 0.0)
        if (self.post_turn_distance >= POST_TURN_ADVANCE_M
                and self.observation.hazard not in ("blocked_pair", "emergency")
                and (self.acquire_center_t >= ACQUIRE_CENTER_DWELL_S
                     or self.acquire_wall_t >= ACQUIRE_ONE_WALL_DWELL_S)):
            self.post_turn_grace_t = 1.0
            self._set_state("FOLLOW", "post_turn_acquired")
        return self._finish(base - steer, base + steer, steer,
                            e_lat, e_front, dt, yaw_rate)


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
                  "single_wall_target": CONTROLLER.single_wall_target,
                  "heading_quadrant": CONTROLLER.heading_quadrant,
                  "opposite_direction": CONTROLLER.opposite_direction,
                  "junction_stage": CONTROLLER.junction_stage,
                 "exit_set": CONTROLLER.exit_set,
                 "junction_decision": CONTROLLER.junction_decision,
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
