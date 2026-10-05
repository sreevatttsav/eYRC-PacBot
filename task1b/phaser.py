"""Fresh-sample pump for the test scripts (step_test, speed_test).

Background: the test scripts used to free-run their control loop,
logging (and clocking phases by) every iteration. The sim bridge only
publishes fresh payloads at ~200-450 Hz, so ~97% of logged rows were
stale repeats and phase clocks ran ~15-36x hot. The controller never
had this bug (1 command per received sample).

Sampler blocks until the payload BYTES change, so every processed
sample is fresh and phase clocks advance in honest sim-durations.
Static phases (all-zero commands) may additionally fall back to a
wall-tick so a parked robot cannot hang a phase; active phases raise
StallTimeout instead (a non-responding sim is data, not a hang).
"""
import json
import time


class StallTimeout(Exception):
    pass


class Sampler:
    def __init__(self, stall_wall_s=5.0, static_tick_s=0.05):
        self.stall_wall_s = stall_wall_s
        self.static_tick_s = static_tick_s
        self.latest = {}
        self._pending_raw = None
        self._pending_parsed = None
        self._consumed_raw = None

    def on_message(self, client, userdata, msg):
        try:
            parsed = json.loads(msg.payload.decode())
        except ValueError:
            return
        self.latest = parsed
        self._pending_raw = msg.payload
        self._pending_parsed = parsed

    def next(self, fresh_only=True):
        """Return (payload dict copy, dt, is_fresh). fresh_only=True
        waits for changed bytes (active phases). False also accepts a
        wall tick (static phases); those rows reuse the last payload
        with the measured wall dt so clocks keep moving while parked."""
        t0 = time.monotonic()
        last_wall = t0
        while True:
            if (self._pending_raw is not None
                    and self._pending_raw != self._consumed_raw):
                self._consumed_raw = self._pending_raw
                out = dict(self._pending_parsed)
                return out, float(out.get("dt", 0.002) or 0.002), True
            now = time.monotonic()
            if not fresh_only and now - last_wall >= self.static_tick_s:
                return dict(self.latest), now - last_wall, False
            if now - t0 > self.stall_wall_s:
                raise StallTimeout(
                    f"no fresh sample for {self.stall_wall_s}s wall")
            time.sleep(0.001)
