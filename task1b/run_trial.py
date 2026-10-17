"""Run one live trial: simulator, independent MQTT monitor, then controller.

The simulator executable must be supplied by the user; it is not in this repo.
An early sensor dropout is kept on disk but marked invalid, not scored as a run.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys
import threading
import time


def early_dropout(sim_seconds: float, last_frame_age: float,
                  wall_seconds: float) -> bool:
    return wall_seconds >= 8.0 and sim_seconds < 1.0 and last_frame_age >= 3.0


def stop(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)


def run(simulator: Path, label: str, timeout: float, host: str, port: int,
        output: Path) -> Path:
    import paho.mqtt.client as mqtt

    simulator = simulator.resolve(strict=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    trial = output / f"{stamp}_{label}"
    trial.mkdir(parents=True, exist_ok=False)
    state = {"frames": 0, "sim_seconds": 0.0, "last_frame": time.monotonic(),
             "verdict": None}
    lock = threading.Lock()
    ready = threading.Event()
    monitor = mqtt.Client()
    sim: subprocess.Popen | None = None
    controller: subprocess.Popen | None = None
    start = time.monotonic()
    status = "setup_failed"
    with (trial / "sim.stdout.log").open("w") as sim_log, \
            (trial / "controller.stdout.log").open("w") as ctl_log, \
            (trial / "mqtt.jsonl").open("w") as mqtt_log:
        def on_connect(client, _userdata, _flags, rc, *_extra):
            if rc == 0:
                client.subscribe([("pacbot/sensors", 0), ("pacbot/result", 0)])
                ready.set()

        def on_message(_client, _userdata, message):
            now = time.monotonic()
            raw = message.payload.decode(errors="replace")
            mqtt_log.write(json.dumps({"wall_offset_s": round(now - start, 4),
                                       "topic": message.topic, "payload": raw}) + "\n")
            if message.topic == "pacbot/sensors":
                try:
                    dt = float(json.loads(raw).get("dt", 0.0))
                except (ValueError, TypeError, json.JSONDecodeError):
                    dt = 0.0
                with lock:
                    state["frames"] += 1
                    state["sim_seconds"] += max(0.0, min(dt, 0.2))
                    state["last_frame"] = now
            elif message.topic == "pacbot/result":
                with lock:
                    state["verdict"] = raw

        monitor.on_connect = on_connect
        monitor.on_message = on_message
        try:
            # Preserve the launch order used by the successful sustained logs.
            sim = subprocess.Popen([str(simulator)], cwd=simulator.parent,
                                   stdout=sim_log, stderr=subprocess.STDOUT)
            monitor.connect(host, port, keepalive=60)
            monitor.loop_start()
            if not ready.wait(timeout=8):
                status = "monitor_not_ready"
            else:
                controller = subprocess.Popen(
                    [sys.executable, str(Path(__file__).with_name("task_1b_boilerplate.py")),
                     "--label", label, "--log-dir", str(trial / "controller_logs"),
                     "--host", host, "--port", str(port)],
                    cwd=Path(__file__).parent, stdout=ctl_log,
                    stderr=subprocess.STDOUT)
                while True:
                    now = time.monotonic()
                    with lock:
                        verdict = state["verdict"]
                        frames = state["frames"]
                        sim_seconds = state["sim_seconds"]
                        age = now - state["last_frame"]
                    if verdict is not None:
                        status = "verdict_received"
                        break
                    if early_dropout(sim_seconds, age, now - start):
                        status = "invalid_early_sensor_dropout"
                        break
                    if now - start >= timeout:
                        status = "time_limit"
                        break
                    if sim.poll() is not None or controller.poll() is not None:
                        status = "process_exited_before_verdict"
                        break
                    time.sleep(0.1)
        finally:
            stop(controller)
            stop(sim)
            monitor.loop_stop()
            monitor.disconnect()
            with lock:
                if (status in ("time_limit", "process_exited_before_verdict") and
                        state["sim_seconds"] < 1.0 and state["verdict"] is None):
                    status = "invalid_early_sensor_dropout"
                summary = {"status": status, "frames": state["frames"],
                           "sim_seconds": round(state["sim_seconds"], 3),
                           "wall_seconds": round(time.monotonic() - start, 3),
                           "accepted": status in ("verdict_received", "time_limit"),
                           "verdict_raw": state["verdict"]}
            (trial / "harness.json").write_text(json.dumps(summary, indent=2) + "\n")
    return trial


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", type=Path, required=True,
                        help="path to the official task_1b_launch executable")
    parser.add_argument("--label", default="lattice_validation")
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--output", type=Path, default=Path(__file__).parent / "trials")
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    trial = run(args.sim, args.label, args.timeout, args.host, args.port, args.output)
    print(trial)
    print((trial / "harness.json").read_text())


if __name__ == "__main__":
    main()
