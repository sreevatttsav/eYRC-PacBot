"""MQTT entry point for the scratch Task 1B lattice controller.

Run from this directory: python3 task_1b_boilerplate.py --label trial
"""

from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import signal

from controller import LatticeController, Settings
from lattice import Frame


SENSORS = "pacbot/sensors"
WHEELS = "pacbot/wheel_vel"
RESULT = "pacbot/result"


class RunLog:
    def __init__(self, parent: Path, label: str, settings: Settings) -> None:
        safe = "".join(c if c.isalnum() or c in "-_" else "_" for c in label)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        self.directory = parent / f"{stamp}_{safe}"
        self.directory.mkdir(parents=True, exist_ok=False)
        (self.directory / "meta.json").write_text(
            json.dumps({"controller": "lattice-from-scratch",
                        "created_utc": stamp, "settings": asdict(settings)}, indent=2) + "\n"
        )
        self.ticks = (self.directory / "ticks.jsonl").open("w")
        self.events = (self.directory / "events.jsonl").open("w")
        self.event_cursor = 0
        self.result_received = False

    def record(self, controller: LatticeController, payload: dict) -> None:
        sample = controller.snapshot()
        sample["raw"] = payload
        self.ticks.write(json.dumps(sample) + "\n")
        if controller.state == "DECIDE":
            controller.events.append({"t": round(controller.sim_time, 4),
                                      "event": "decision_votes",
                                      "votes": {f"{k[0]}:{k[1]}": v
                                                for k, v in controller.votes.items()}})
        for event in controller.events[self.event_cursor:]:
            self.events.write(json.dumps(event) + "\n")
        self.event_cursor = len(controller.events)
        if controller.tick % 50 == 0:
            self.ticks.flush()
            self.events.flush()

    def result(self, payload: object) -> None:
        (self.directory / "result.json").write_text(
            json.dumps(payload, indent=2) + "\n"
        )
        self.result_received = True

    def close(self) -> None:
        if not self.result_received:
            (self.directory / "result.json").write_text(json.dumps(
                {"status": "not_received", "reason": "run ended before simulator verdict"},
                indent=2) + "\n")
        self.ticks.close()
        self.events.close()


def run(label: str, log_parent: Path, settings: Settings,
        host: str = "localhost", port: int = 1883) -> None:
    # Import here so estimator/controller tests need no MQTT installation.
    import paho.mqtt.client as mqtt

    controller = LatticeController(settings)
    log = RunLog(log_parent, label, settings)
    client = mqtt.Client()

    def on_connect(mqtt_client, userdata, flags, rc, *extra):
        if rc != 0:
            raise RuntimeError(f"MQTT connection failed: {rc}")
        mqtt_client.subscribe([(SENSORS, 0), (RESULT, 0)])

    def on_message(mqtt_client, userdata, message):
        if message.topic == RESULT:
            try:
                payload = json.loads(message.payload)
            except (UnicodeDecodeError, json.JSONDecodeError):
                payload = message.payload.decode(errors="replace")
            log.result(payload)
            print(f"[result] {payload}", flush=True)
            return
        if message.topic != SENSORS:
            return
        payload = {}
        try:
            payload = json.loads(message.payload)
            frame = Frame.from_payload(payload, controller.range_hold)
            left, right = controller.step(frame)
        except (UnicodeDecodeError, json.JSONDecodeError, TypeError, ValueError) as exc:
            print(f"[sensor error] {exc}", flush=True)
            left, right = 0.0, 0.0
        mqtt_client.publish(WHEELS, json.dumps({"left": left, "right": right}))
        log.record(controller, payload)

    client.on_connect = on_connect
    client.on_message = on_message

    def stop_signal(signum, _frame):
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, stop_signal)
    signal.signal(signal.SIGINT, stop_signal)
    try:
        client.connect(host, port, keepalive=60)
        print(f"[lattice] logs: {log.directory}", flush=True)
        client.loop_forever()
    finally:
        try:
            client.publish(WHEELS, json.dumps({"left": 0.0, "right": 0.0}))
            client.disconnect()
        finally:
            log.close()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--label", default="lattice")
    parser.add_argument("--log-dir", type=Path, default=Path(__file__).parent / "logs")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--dock-range", type=float, default=0.06)
    args = parser.parse_args()
    if not 0.045 <= args.dock_range <= 0.08:
        parser.error("--dock-range must be between 0.045 and 0.08 m")
    run(args.label, args.log_dir, Settings(dock_range=args.dock_range),
        args.host, args.port)


if __name__ == "__main__":
    main()
