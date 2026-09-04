#!/usr/bin/env python3
"""GUI-integrated receiving/defect shuttle using the latest LiDAR+A* stack."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from common.comm import StatusClient, TriggerServer
from common.hw import Hardware
from common.mapping import build_distance_field
from common.navigate import goto_zone

CONFIG_PATH = PROJECT_ROOT / "config" / "course_config.json"
DATA_DIR = PROJECT_ROOT / "data"


class EmergencyStop(RuntimeError):
    """Raised inside blocking navigation as soon as an E-stop arrives."""


class MissionCommands:
    """Latch handoff and safety commands while retaining their job identity."""

    def __init__(self, server: TriggerServer) -> None:
        self.server = server
        self.events: list[dict] = []
        self.estopped = False
        self.reset_requested = False

    def poll(self) -> None:
        for message in self.server.poll():
            event = str(message.get("event", ""))
            if event == "emergency_stop":
                self.estopped = True
                self.reset_requested = False
            elif event == "reset":
                self.reset_requested = True
            else:
                self.events.append(message)
        if self.estopped:
            raise EmergencyStop

    def take(self, event: str, job_id: str | None = None) -> dict | None:
        self.poll()
        aliases = {"box_picked", "operator_unloaded"} if event == "box_picked" else {event}
        for index, message in enumerate(self.events):
            incoming_id = message.get("job_id")
            if (message.get("event") in aliases and
                    (job_id is None or incoming_id in (None, job_id))):
                return self.events.pop(index)
        return None

    def consume_reset(self) -> bool:
        for message in self.server.poll():
            event = message.get("event")
            if event == "reset":
                self.reset_requested = True
            elif event == "emergency_stop":
                self.estopped = True
            else:
                self.events.append(message)
        if not self.reset_requested:
            return False
        self.reset_requested = False
        self.estopped = False
        self.events.clear()
        return True


def load_json(path: Path):
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


class ShuttleMission:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.cfg = load_json(Path(args.config))
        self.server = TriggerServer(port=args.trigger_port)
        self.commands = MissionCommands(self.server)
        self.status = StatusClient(args.status_host, args.status_port) if args.status_host else None
        self.hw: Hardware | None = None
        self.job_id: str | None = None
        self.state = "STARTING"
        self._last_heartbeat = 0.0
        self._log_stream = None
        self._log_writer = None

        points = [tuple(point) for point in load_json(DATA_DIR / "map_points.json")]
        boundary = self.cfg["boundary"]
        self.distance_field = build_distance_field(points, boundary["x_m"], boundary["y_m"])
        obstacle_path = DATA_DIR / "obstacle_map.json"
        obstacle_map = load_json(obstacle_path) if obstacle_path.exists() else {}
        self.obstacles = [tuple(rect) for rects in obstacle_map.values() for rect in rects]
        self.references = {
            name: load_json(DATA_DIR / f"{name}_reference_scan.json")
            for name in ("receiving", "defect")
        }

    def send(self, text: str, **extra) -> None:
        if self._log_writer is not None:
            self._log_writer.writerow({
                'timestamp': time.time(), 'state': self.state,
                'status': text, 'job_id': self.job_id or '',
                'detail': extra.get('detail', ''),
            })
            self._log_stream.flush()
        if self.status is not None:
            self.status.send_status(
                text, mission_state=self.state, job_id=self.job_id,
                hardware_connected=bool(self.hw and self.hw.is_connected()), **extra)

    def poll_control(self) -> None:
        """Safety/heartbeat hook called from each blocking navigation loop."""
        self.commands.poll()
        now = time.monotonic()
        if now - self._last_heartbeat < 1.0:
            return
        if self.state == 'GOTO_DEFECT':
            self.send('defect 존으로 이동중', to='defect_zone')
        elif self.state == 'GOTO_RECEIVING':
            self.send('대기 존으로 이동중', to='receiving_zone')
        else:
            self.send(self.state)
        self._last_heartbeat = now

    def heartbeat_wait(self, event: str, state: str, at: str) -> None:
        self.state = state
        print(f"[{state}] waiting for {event} on TCP :{self.args.trigger_port}")
        last = 0.0
        while True:
            now = time.monotonic()
            if now - last >= 1.0:
                self.send("대기" if state == "WAIT_SIGNAL" else "도착", at=at)
                self._last_heartbeat = now
                last = now
            message = self.commands.take(event, self.job_id if event == "box_picked" else None)
            if message is not None:
                if event == "box_placed":
                    self.job_id = message.get("job_id")
                return
            time.sleep(0.1)

    def goto(self, source: str, destination: str) -> bool:
        self.state = "GOTO_DEFECT" if destination == "defect" else "GOTO_RECEIVING"
        self.send("출발", to=f"{destination}_zone")
        return goto_zone(
            self.hw, self.cfg, self.distance_field, self.obstacles,
            source, destination, self.references[destination],
            dynamic_obstacles=self.args.dynamic_obstacles,
            align=self.args.align, align_map=not self.args.no_align_map,
            align_heading=self.args.align_heading)

    def wait_after_estop(self) -> str:
        assert self.hw is not None
        interrupted_state = self.state
        self.hw.stop()
        self.state = "EMERGENCY_STOP"
        last = 0.0
        while not self.commands.consume_reset():
            now = time.monotonic()
            if now - last >= 1.0:
                self.send("비상정지")
                last = now
            time.sleep(0.1)
        if interrupted_state == 'WAIT_SIGNAL':
            self.state = 'WAIT_SIGNAL'
            self.send('대기', at='receiving_zone')
            return 'receiving'
        if interrupted_state == 'WAIT_PICKED':
            return 'defect'
        # An interrupted A* leg no longer has a trustworthy configured start
        # pose. Stay stopped rather than guessing a recovery trajectory.
        self.state = "SENSOR_FAIL"
        self.send("SENSOR_FAIL", detail="reset received; relocalize Beagle before restart")
        return 'unknown'

    def run(self) -> int:
        log_path = Path(self.args.output)
        log_path.parent.mkdir(parents=True, exist_ok=True)
        self._log_stream = log_path.open('a', encoding='utf-8', newline='')
        self._log_writer = csv.DictWriter(
            self._log_stream,
            fieldnames=['timestamp', 'state', 'status', 'job_id', 'detail'])
        if self._log_stream.tell() == 0:
            self._log_writer.writeheader()
        self.server.start()
        if self.status is not None:
            self.status.start()
        cycles = 0
        try:
            self.hw = Hardware(port_name=self.args.port_name, control_poll=self.poll_control)
            self.hw.start_lidar()
            if not self.hw.is_connected():
                raise RuntimeError("physical Beagle receiver is not connected")
            while self.args.cycles <= 0 or cycles < self.args.cycles:
                try:
                    self.heartbeat_wait("box_placed", "WAIT_SIGNAL", "receiving_zone")
                    if not self.goto("receiving", "defect"):
                        raise RuntimeError("final alignment at defect did not converge")
                    self.heartbeat_wait("box_picked", "WAIT_PICKED", "defect_zone")
                    if not self.goto("defect", "receiving"):
                        raise RuntimeError("final alignment at receiving did not converge")
                except EmergencyStop:
                    recovery = self.wait_after_estop()
                    if recovery == 'receiving':
                        self.job_id = None
                        continue
                    if recovery == 'defect':
                        try:
                            if self.goto('defect', 'receiving'):
                                self.job_id = None
                                continue
                        except EmergencyStop:
                            pass
                    return 2
                cycles += 1
                self.state = "WAIT_SIGNAL"
                self.send("도착", at="receiving_zone")
                self.job_id = None
            return 0
        except Exception as error:
            if self.hw is not None:
                self.hw.stop()
            self.state = "SENSOR_FAIL"
            self.send("SENSOR_FAIL", detail=str(error))
            print(f"[failed] {error}", file=sys.stderr)
            return 1
        finally:
            if self.hw is not None:
                self.hw.stop()
            self.server.stop()
            if self.status is not None:
                self.status.stop()
            if self._log_stream is not None:
                self._log_stream.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(CONFIG_PATH))
    parser.add_argument("--cycles", type=int, default=0)
    parser.add_argument("--trigger-port", type=int, default=8765)
    parser.add_argument("--status-host", default=None)
    parser.add_argument("--status-port", type=int, default=9000)
    parser.add_argument("--port-name", default=None)
    parser.add_argument("--output", default="runtime/logs/beagle_shuttle.csv")
    parser.add_argument("--dynamic-obstacles", action="store_true")
    parser.add_argument("--align", action="store_true")
    parser.add_argument("--no-align-map", action="store_true")
    parser.add_argument("--align-heading", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    raise SystemExit(ShuttleMission(parse_args()).run())

