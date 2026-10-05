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

# Gyro-terminated turns (fix 3). TURN_RATE_REF derives from SPIN_SPEED
# and the wheel-size estimates -- recheck after Phase-1 calibration.
TURN_KP = 2.0          # P gain on angle error (rad/s per rad)
TURN_MIN_W = 1.5       # minimum wheel speed in a turn (rad/s)
TURN_EXIT_ERR = 0.0873  # 5 deg
TURN_EXIT_GYRO = 0.2   # rad/s
TURN_TIE_DB = 0.05     # side-openness deadband: tie -> prev turn dir
DEADEND_DIST = 0.15    # both sides below this + blocked front = 180 deg
TURN_VERIFY_DIST = 0.20  # front clearance required after a turn
TURN_MAX_RETRIES = 2

# Stuck detection + recovery (fix 2).
STUCK_WIN_S = 1.0
STUCK_CMD_MIN = 0.5    # |commanded yaw| must exceed this (rad/s)
STUCK_RATIO = 0.25     # |measured| below this fraction of commanded
STUCK_ESCALATE_S = 3.0  # re-flag within this -> escalate
RECOVER_DIST = 0.10    # back-out distance per recovery (m, estimated)

# Wheel-size ESTIMATES for runlog dead-reckoning (mesh guess -- Phase 1
# step test replaces these with measured values).
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
        "MAX_RANGE": MAX_RANGE, "FILTER_TAU": FILTER_TAU,
        "TOF_HOLD_S": TOF_HOLD_S, "WALL_TARGET": WALL_TARGET,
        "WEDGE_ENTER": WEDGE_ENTER, "WEDGE_EXIT": WEDGE_EXIT,
        "WEDGE_ENTER_T": WEDGE_ENTER_T, "WEDGE_MIN_DWELL": WEDGE_MIN_DWELL,
        "WEDGE_MAX_T": WEDGE_MAX_T,
        "TURN_KP": TURN_KP, "TURN_MIN_W": TURN_MIN_W,
        "TURN_EXIT_ERR": TURN_EXIT_ERR, "TURN_EXIT_GYRO": TURN_EXIT_GYRO,
        "TURN_TIE_DB": TURN_TIE_DB, "DEADEND_DIST": DEADEND_DIST,
        "TURN_VERIFY_DIST": TURN_VERIFY_DIST,
        "TURN_MAX_RETRIES": TURN_MAX_RETRIES,
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
        # Validity-filtered sensors (fix 1). Values are None = unknown.
        self.tof_fl = Tof(MAX_RANGE, TOF_HOLD_S, FILTER_TAU)
        self.tof_fr = Tof(MAX_RANGE, TOF_HOLD_S, FILTER_TAU)
        self.tof_sl = Tof(MAX_RANGE, TOF_HOLD_S, FILTER_TAU)
        self.tof_sr = Tof(MAX_RANGE, TOF_HOLD_S, FILTER_TAU)
        self.fl_f = None
        self.fr_f = None
        self.sl_f = None
        self.sr_f = None
        self.last_status = ("none",) * 4
        self.i_lat = 0.0
        self.t = 0.0              # controller clock (sim-time, from dt)
        self.gyro_th = 0.0        # integrated heading (turn termination)
        # Escape state
        self.spin_dir = 0.0
        self.backup_ticks = 0
        self.reverse_ticks = 0
        self.flip_next = False   # after a give-up, try the other way first
        self.spin_done_s = 0.0   # committed turn executed this escape
        # Gyro turn state (fix 3)
        self.turn_active = False
        self.turn_entry = 0.0
        self.turn_target = 0.0   # signed radians
        self.turn_t = 0.0
        self.turn_retries = 0
        self.prev_turn_dir = 1.0  # tie-break default: left (fixed rule)
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

    @property
    def TURN_RATE_REF(self):
        return WHEEL_R_EST * 2.0 * SPIN_SPEED / WHEEL_TRACK_EST


    def _set_state(self, state, reason):
        self.state = state
        self.state_reason = reason

    def _finalize(self, L, R, e_lat, e_front, steer, yaw_rate, dt):
        """Single exit path: runs stuck detection (fix 2) on the final
        command. A rising stuck edge during any maneuver switches this
        tick's output to stuck recovery (distance-driven reverse)."""
        yaw_cmd = WHEEL_R_EST * (R - L) / WHEEL_TRACK_EST
        self.yaw_cmd = yaw_cmd
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

        # 2. Errors, validity-gated. e_front needs both front sensors;
        #    e_lat needs both sides. One side only -> hold WALL_TARGET
        #    from the good wall. Nothing -> steer 0, drive straight.
        e_lat = None
        e_front = None
        single = None  # (+1 side sign, reading) for single-wall hold
        if self.sl_f is not None and self.sr_f is not None:
            e_lat = self.sl_f - self.sr_f
        elif self.sl_f is not None:
            single = (1.0, self.sl_f)
        elif self.sr_f is not None:
            single = (-1.0, self.sr_f)
        if self.fl_f is not None and self.fr_f is not None:
            e_front = self.fl_f - self.fr_f

        # 3. Leaky integral (OFF by default). Same cautions as before;
        #    additionally gated on e_lat being known.
        if KI_LAT > 0.0 and e_lat is not None and abs(e_lat) > I_DEADBAND:
            self.i_lat = self.i_lat * I_LEAK + e_lat * dt
            self.i_lat = max(-I_MAX, min(self.i_lat, I_MAX))
        else:
            self.i_lat *= I_LEAK

        # 4. Steer composition. Single-wall hold: too far from the left
        #    wall (sl > target) steers left (+), and symmetrically right.
        steer = 0.0
        if e_lat is not None:
            steer += KP_LAT * e_lat
        elif single is not None:
            side, reading = single
            steer += side * KP_LAT * (reading - WALL_TARGET)
        if e_front is not None:
            steer += KP_FRONT * e_front
        steer += KI_LAT * self.i_lat - KD_YAW * yaw_rate
        steer = max(-MAX_STEER, min(steer, MAX_STEER))

        # 5. Longitudinal + escape. States: FOLLOW / REVERSE / TURN /
        #    BACKUP / WEDGE / RECOVER. Every return goes through
        #    _finalize() for stuck detection.
        self.t += dt
        self.gyro_th += yaw_rate * dt

        front_known = self.fl_f is not None and self.fr_f is not None
        front_clear = min(self.fl_f, self.fr_f) if front_known else None
        blocked = front_known and front_clear < FRONT_STOP_DIST

        # --- stuck recovery finishes first (distance-driven backout) ---
        if self.recover_active:
            back = BASE_SPEED * 0.5
            self.recover_done += abs(WHEEL_R_EST * back) * dt
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
        if (front_known and front_clear > self.RESUME_DIST
                and not in_maneuver and not spin_committed
                and not self.turn_active):
            self.spin_dir = 0.0
            self.turn_active = False

        if self.backup_ticks > 0:
            self.backup_ticks -= 1
            self._set_state("BACKUP", "giveup")
            back = BASE_SPEED * 0.5
            return self._finalize(-back, -back, e_lat, e_front,
                                  steer, yaw_rate, dt)

        if blocked or self.spin_dir != 0.0:
            if self.spin_dir == 0.0 and self.reverse_ticks == 0:
                # fresh block: pick the turn, reverse first for room
                deadend = (front_known and self.sl_f is not None
                           and self.sr_f is not None
                           and self.sl_f < DEADEND_DIST
                           and self.sr_f < DEADEND_DIST)
                mag = math.pi if deadend else math.pi / 2.0
                if (self.sl_f is not None and self.sr_f is not None
                        and abs(self.sl_f - self.sr_f) > TURN_TIE_DB):
                    tdir = 1.0 if self.sl_f > self.sr_f else -1.0
                else:
                    tdir = self.prev_turn_dir  # tie/unknown: fixed rule
                self.spin_dir = tdir
                if self.flip_next:
                    self.spin_dir = -self.spin_dir
                    tdir = self.spin_dir
                    self.flip_next = False
                self.turn_target = tdir * mag
                self.turn_active = False
                self.turn_retries = 0
                self.turn_t = 0.0
                self.reverse_ticks = max(1, int(self.REVERSE_S / dt)) if dt > 0 else 200
                self.spin_done_s = 0.0  # new escape -> new committed turn
            if self.reverse_ticks > 0:
                self.reverse_ticks -= 1
                self._set_state("REVERSE", "blocked")
                back = BASE_SPEED * 0.5
                return self._finalize(-back, -back, e_lat, e_front,
                                      steer, yaw_rate, dt)
            # --- gyro-terminated turn (fix 3): P servo on integrated
            #     angle, NOT on time. Time-based turns inherit the ~15%
            #     wheel-size error measured in the log. ---
            if not self.turn_active:
                self.turn_active = True
                self.turn_entry = self.gyro_th
                self.turn_t = 0.0
            self.turn_t += dt
            turned = self.gyro_th - self.turn_entry
            err = self.turn_target - turned
            timeout = 2.0 * abs(self.turn_target) / self.TURN_RATE_REF
            if self.turn_t > timeout:
                self.backup_ticks = max(1, int(self.BACKUP_S / dt)) if dt > 0 else 250
                self.spin_dir = 0.0
                self.turn_active = False
                self.flip_next = True
                self._set_state("BACKUP", "turn_timeout")
                back = BASE_SPEED * 0.5
                return self._finalize(-back, -back, e_lat, e_front,
                                      steer, yaw_rate, dt)
            if abs(err) < TURN_EXIT_ERR and abs(yaw_rate) < TURN_EXIT_GYRO:
                verified = (not front_known) or (front_clear > TURN_VERIFY_DIST)
                if verified:
                    self.prev_turn_dir = 1.0 if self.turn_target > 0 else -1.0
                    self.spin_dir = 0.0
                    self.turn_active = False
                    self._set_state("FOLLOW", "turn_done" if front_known
                                    else "turn_done_unverified")
                    # fall through to FOLLOW tail below
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
                    err = self.turn_target
            if self.turn_active:
                w = TURN_KP * err
                wmag = min(SPIN_SPEED, max(TURN_MIN_W, abs(w)))
                w = math.copysign(wmag, err) if err != 0.0 else 0.0
                self.spin_done_s += dt
                self._set_state("TURN",
                                "left" if self.turn_target > 0 else "right")
                return self._finalize(-w, w, e_lat, e_front,
                                      steer, yaw_rate, dt)
        # FOLLOW tail (also reached after a completed turn). Unknown
        # front -> cautious half speed since clearance is unverified.
        if front_known and not blocked:
            span = FRONT_SLOW_DIST - FRONT_STOP_DIST
            scale = (front_clear - FRONT_STOP_DIST) / span if span > 0 else 1.0
            scale = max(0.25, min(1.0, scale))
            base = BASE_SPEED * scale
        elif not front_known:
            base = BASE_SPEED * 0.5
        else:
            base = BASE_SPEED * 0.25

        left_vel = base - steer
        right_vel = base + steer
        left_vel = max(-MAX_SPEED, min(left_vel, MAX_SPEED))
        right_vel = max(-MAX_SPEED, min(right_vel, MAX_SPEED))
        self._set_state("FOLLOW",
                        "clear" if front_known and front_clear > FRONT_SLOW_DIST else "approach")
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
        LOGGER.tick(
            raw=(fl, fr, sl, sr),
            filt=(CONTROLLER.fl_f, CONTROLLER.fr_f,
                  CONTROLLER.sl_f, CONTROLLER.sr_f),
            gyro_z=yaw_rate, dt_rep=dt, e_lat=e_lat, e_front=e_front,
            steer=steer, L=left_vel, R=right_vel,
            state=CONTROLLER.state, true_pose=true_pose, extra=extra,
            statuses=CONTROLLER.last_status,
            yaw_cmd=CONTROLLER.yaw_cmd,
            stuck=CONTROLLER.stuck_flag)
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
                           wheel_track=WHEEL_TRACK_EST)

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
