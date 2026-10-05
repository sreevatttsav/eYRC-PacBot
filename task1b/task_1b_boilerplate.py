"""Boilerplate for PB Task 1B.

Subscribes to the simulator's sensor topic, logs each reading, and publishes
a wheel velocity command back. Fill in your control logic where marked.

Run (three terminals):
    mosquitto
    ./task_1b_launch
    python3 task_1b_boilerplate.py
"""
import json

import paho.mqtt.client as mqtt

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


class CenteringController:
    """PD lateral + front-alignment P + gyro D. No raw I by default."""

    def __init__(self):
        self.fl_f = None
        self.fr_f = None
        self.sl_f = None
        self.sr_f = None
        self.i_lat = 0.0

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
        # 1. Sanitize + low-pass (ToF is noisy; don't differentiate raw noise)
        fl = self._clip_range(fl)
        fr = self._clip_range(fr)
        sl = self._clip_range(sl)
        sr = self._clip_range(sr)
        self.fl_f = self._filter(self.fl_f, fl, dt)
        self.fr_f = self._filter(self.fr_f, fr, dt)
        self.sl_f = self._filter(self.sl_f, sl, dt)
        self.sr_f = self._filter(self.sr_f, sr, dt)

        # 2. Errors. e_lat > 0 means too close to RIGHT wall -> steer left.
        #    e_front > 0 means nose pointed RIGHT (fr shorter) -> steer left.
        e_lat = self.sl_f - self.sr_f
        e_front = self.fl_f - self.fr_f

        # 3. Leaky integral (OFF by default). Why off:
        #    - In a corridor, a persistent e_lat (e.g. one wall missing at a
        #      junction) charges a pure integrator, then it unloads late as a
        #      big spurious steer -> overshoot / wall hug / oscillation.
        #    - Gyro bias + asymmetric ToF also look like a DC error and wind
        #      the integrator into drift. Leaky + clamped + deadbanded I is
        #      the only safe form here.
        if KI_LAT > 0.0 and abs(e_lat) > I_DEADBAND:
            self.i_lat = self.i_lat * I_LEAK + e_lat * dt
            self.i_lat = max(-I_MAX, min(self.i_lat, I_MAX))
        else:
            self.i_lat *= I_LEAK

        # 4. PD + gyro damping. yaw_rate IS the D term: it is the derivative
        #    of heading, so -KD_YAW*yaw_rate damps oscillation without
        #    differentiating noisy (sl - sr).
        steer = (KP_LAT * e_lat
                 + KP_FRONT * e_front
                 + KI_LAT * self.i_lat
                 - KD_YAW * yaw_rate)
        steer = max(-MAX_STEER, min(steer, MAX_STEER))

        # 5. Longitudinal: slow near front walls, spin if blocked.
        front_clear = min(self.fl_f, self.fr_f)
        if front_clear < FRONT_STOP_DIST:
            # Turn toward the more open side, in place.
            side = 1.0 if self.sl_f > self.sr_f else -1.0  # +1 = left
            return -side * SPIN_SPEED, side * SPIN_SPEED, e_lat, e_front, steer
        span = FRONT_SLOW_DIST - FRONT_STOP_DIST
        scale = (front_clear - FRONT_STOP_DIST) / span if span > 0 else 1.0
        scale = max(0.25, min(1.0, scale))
        base = BASE_SPEED * scale

        left_vel = base - steer
        right_vel = base + steer
        left_vel = max(-MAX_SPEED, min(left_vel, MAX_SPEED))
        right_vel = max(-MAX_SPEED, min(right_vel, MAX_SPEED))
        return left_vel, right_vel, e_lat, e_front, steer


CONTROLLER = CenteringController()


def _mqtt_client():
    # paho-mqtt >= 2.0 requires picking a callback API version explicitly.
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    except AttributeError:
        return mqtt.Client()


def on_message(client, userdata, msg):
    data = json.loads(msg.payload.decode())

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
    print(f"  e_lat={e_lat:+.3f} e_front={e_front:+.3f} "
          f"steer={steer:+.3f} -> L={left_vel:+.2f} R={right_vel:+.2f}")

    client.publish(TOPIC_WHEEL_VEL, json.dumps({
        "left": float(left_vel), "right": float(right_vel),
    }))


def main():
    client = _mqtt_client()
    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT)
    client.subscribe(TOPIC_SENSORS)
    client.loop_forever()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
