from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""Real hardware only. Signal-triggered receiving<->defect shuttle, built on
this session's proven navigation stack (common/navigate.py's goto_zone():
continuous odometry + periodic map localization while driving, then
find_pose() for the precise final alignment).

Robot must already be sitting at a LiDAR-verified pose at receiving (e.g. just
ran scripts/04_find_pose.py --zone receiving).

Cycle:
  WAIT_SIGNAL      -- idle at receiving until a {"event": "box_placed"} TCP
                       message arrives (common/comm.py's TriggerServer,
                       default port 8765 -- same message the OMX arm sends)
  GOTO_DEFECT      -- goto_zone(receiving -> defect); aligns to 9 o'clock
  DWELL_DEFECT     -- wait for the GUI's operator_unloaded event once aligned
  GOTO_RECEIVING   -- goto_zone(defect -> receiving); aligns to 3 o'clock
  -> back to WAIT_SIGNAL, repeat

If a leg's find_pose() doesn't converge, the mission stops (driving the next
leg from an unverified pose would just compound the error) -- see the printed
RESULT for which zone it was heading to when it stopped.
"""

import argparse
import json
import time

from common.comm import StatusClient, TriggerServer
from common.hw import Hardware, MissionInterrupted
from common.mapping import build_distance_field
from common.navigate import goto_zone

CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "course_config.json"
DATA_DIR = Path(__file__).resolve().parents[1] / "data"
DWELL_S = 5.0
TRIGGER_PORT = 8765
STATUS_PORT = 9000
HEARTBEAT_S = 1.0


def load_json(path: Path) -> dict:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def load_reference(zone: str) -> list[float] | None:
    path = DATA_DIR / f"{zone}_reference_scan.json"
    if not path.exists():
        print(f"[error] no reference scan for {zone} at {path} -- run "
              f"scripts/03_calibrate_and_realign.py --zone {zone} --calibrate first.")
        return None
    return load_json(path)


class MissionControl:
    """Bridge the blocking A* mission to the OMX GUI safety protocol."""

    def __init__(self, server: TriggerServer, status: StatusClient | None) -> None:
        self.server = server
        self.status = status
        self.hw: Hardware | None = None
        self.state = "CONNECTING"
        self.box_placed = False
        self.operator_unloaded = False
        self.emergency_stop = False
        self.reset_requested = False
        self.connection_lost = False
        self._last_heartbeat = 0.0

    def hardware_connected(self) -> bool:
        return self.hw is not None and self.hw.is_connected()

    def send(self, text: str, *, force: bool = False, **extra: object) -> None:
        now = time.monotonic()
        if not force and now - self._last_heartbeat < HEARTBEAT_S:
            return
        if self.status is not None:
            self.status.send_status(
                text,
                mission_state=self.state,
                hardware_connected=self.hardware_connected(),
                **extra,
            )
        self._last_heartbeat = now

    def heartbeat(self, *, force: bool = False) -> None:
        if not self.hardware_connected():
            self.send("연결 끊김", force=force)
        elif self.state == "WAIT_SIGNAL":
            self.send("대기", force=force, at="receiving_zone")
        elif self.state == "GOTO_DEFECT":
            self.send("defect 존으로 이동중", force=force)
        elif self.state == "DWELL_DEFECT":
            self.send("도착", force=force, at="defect_zone")
        elif self.state == "GOTO_RECEIVING":
            self.send("대기 존으로 이동중", force=force)
        elif self.state == "EMERGENCY_STOP":
            self.send("비상정지", force=force)
        elif self.state == "SENSOR_FAIL":
            self.send("SENSOR_FAIL", force=force)
        else:
            self.send("연결 중", force=force)

    def poll(self) -> None:
        for message in self.server.poll():
            event = message.get("event")
            if event == "box_placed":
                if self.state == "WAIT_SIGNAL":
                    self.box_placed = True
                    print("[WAIT_SIGNAL] box_placed received.")
            elif event == "operator_unloaded":
                if self.state == "DWELL_DEFECT":
                    self.operator_unloaded = True
                    print("[DWELL_DEFECT] operator_unloaded received.")
            elif event == "emergency_stop":
                self.emergency_stop = True
                self.reset_requested = False
                self.state = "EMERGENCY_STOP"
                if self.hw is not None:
                    self.hw.stop()
                self.heartbeat(force=True)
                print("[EMERGENCY_STOP] stop requested by GUI.")
            elif event == "reset":
                self.reset_requested = True
                print("[RESET] reset requested by GUI.")

        # A reset while already stationary is also the reconnect handshake.
        if self.reset_requested and not self.emergency_stop:
            self.reset_requested = False
            if self.state in ("CONNECTING", "WAIT_SIGNAL"):
                self.state = "WAIT_SIGNAL"
                self.heartbeat(force=True)
        self.heartbeat()

    def should_stop(self) -> bool:
        self.poll()
        if self.hw is not None and not self.hw.is_connected():
            self.connection_lost = True
            self.hw.stop()
            self.heartbeat(force=True)
            return True
        return self.emergency_stop

    def wait_for_box_placed(self) -> None:
        self.box_placed = False
        self.operator_unloaded = False
        self.state = "WAIT_SIGNAL"
        self.heartbeat(force=True)
        print("[WAIT_SIGNAL] idling at receiving, waiting for box_placed signal...")
        while not self.box_placed and not self.emergency_stop:
            self.poll()
            time.sleep(0.1)

    def wait_for_operator_unloaded(self) -> None:
        self.operator_unloaded = False
        self.state = "DWELL_DEFECT"
        self.heartbeat(force=True)
        print("[DWELL_DEFECT] waiting for the GUI 하역 완료 signal...")
        while not self.operator_unloaded and not self.emergency_stop:
            self.poll()
            time.sleep(0.1)

    def wait_for_reset(self) -> None:
        if self.hw is not None:
            self.hw.stop()
        while self.emergency_stop:
            self.poll()
            if self.reset_requested:
                self.reset_requested = False
                self.emergency_stop = False
                self.box_placed = False
                self.operator_unloaded = False
                self.state = "WAIT_SIGNAL"
                self.heartbeat(force=True)
                print("[RESET] emergency latch cleared; remaining stationary at WAIT_SIGNAL.")
                return
            time.sleep(0.1)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dwell", type=float, default=DWELL_S,
        help="Deprecated launcher compatibility option; GUI 하역 완료 controls return.")
    parser.add_argument("--cycles", type=int, default=0,
                         help="Stop after N receiving->defect->receiving round trips (0 = unlimited).")
    parser.add_argument("--trigger-port", type=int, default=TRIGGER_PORT)
    parser.add_argument("--status-host", default=None)
    parser.add_argument("--status-port", type=int, default=STATUS_PORT)
    parser.add_argument("--port-name", default=None)
    # Retained for compatibility with the existing local Beagle launcher.
    parser.add_argument("--output", default=None)
    parser.add_argument("--dynamic-obstacles", action="store_true",
                         help="EXPERIMENTAL, off by default (2026-09-01 regression -- see "
                              "common/navigate.py's goto_zone() docstring): live obstacle "
                              "detection/replanning while driving.")
    parser.add_argument("--align", action="store_true",
                         help="Off by default: run the OLDER find_pose() (linearized single-scan "
                              "estimate) final alignment instead of --align-map -- kept for "
                              "comparison only, observed to diverge at defect (see "
                              "common/navigate.py's goto_zone() docstring).")
    parser.add_argument("--no-align-map", action="store_true",
                         help="Disable the default final alignment (find_pose_via_map(), heading "
                              "AND position via the frozen map) -- NOT recommended, this is the "
                              "load-bearing default (see common/navigate.py's goto_zone() "
                              "docstring). Falls back to --align-heading if also passed, else no "
                              "alignment at all (unsafe -- corrupts the next leg's dead-reckoning).")
    parser.add_argument("--align-heading", action="store_true",
                         help="Off by default: run the older heading-ONLY realign_heading() "
                              "instead of --align-map (see common/navigate.py's goto_zone() "
                              "docstring).")
    args = parser.parse_args()

    cfg = load_json(CONFIG_PATH)
    boundary = cfg["boundary"]

    map_points_path = DATA_DIR / "map_points.json"
    if not map_points_path.exists():
        print(f"[error] no map at {map_points_path} -- run scripts/08_build_map.py first.")
        return
    map_points = [tuple(p) for p in load_json(map_points_path)]
    print(f"[map] {len(map_points)} points loaded -- building distance field...")
    distance_field = build_distance_field(map_points, boundary["x_m"], boundary["y_m"])

    obstacles = []
    obstacle_map_path = DATA_DIR / "obstacle_map.json"
    if obstacle_map_path.exists():
        obstacle_map = load_json(obstacle_map_path)
        obstacles = [tuple(r) for rects in obstacle_map.values() for r in rects]
    print(f"[plan] {len(obstacles)} known obstacle rect(s) loaded for A* avoidance")

    receiving_ref = load_reference("receiving")
    defect_ref = load_reference("defect")
    if receiving_ref is None or defect_ref is None:
        return

    server = TriggerServer(port=args.trigger_port)
    server.start()
    status = StatusClient(args.status_host, args.status_port) if args.status_host else None
    if status is not None:
        status.start()
    control = MissionControl(server, status)

    hw = Hardware(port_name=args.port_name, should_stop=control.should_stop)
    control.hw = hw
    cycles_done = 0
    try:
        control.heartbeat(force=True)
        hw.start_lidar()
        while args.cycles <= 0 or cycles_done < args.cycles:
            control.wait_for_box_placed()
            if control.emergency_stop:
                control.wait_for_reset()
                continue

            print("\n===== GOTO_DEFECT =====")
            control.state = "GOTO_DEFECT"
            control.send("출발", force=True, to="defect_zone")
            try:
                ok = goto_zone(hw, cfg, distance_field, obstacles, "receiving", "defect", defect_ref,
                               dynamic_obstacles=args.dynamic_obstacles, align=args.align,
                               align_map=not args.no_align_map, align_heading=args.align_heading)
            except MissionInterrupted:
                if control.connection_lost:
                    break
                control.wait_for_reset()
                continue
            if not ok:
                print("[stop] did not converge at defect -- stopping mission.")
                control.state = "SENSOR_FAIL"
                control.heartbeat(force=True)
                break

            print("\n===== DWELL_DEFECT =====")
            control.wait_for_operator_unloaded()
            if control.emergency_stop:
                control.wait_for_reset()
                continue

            print("\n===== GOTO_RECEIVING =====")
            control.state = "GOTO_RECEIVING"
            control.send("출발", force=True, to="receiving_zone")
            try:
                ok = goto_zone(hw, cfg, distance_field, obstacles, "defect", "receiving", receiving_ref,
                               dynamic_obstacles=args.dynamic_obstacles, align=args.align,
                               align_map=not args.no_align_map, align_heading=args.align_heading)
            except MissionInterrupted:
                if control.connection_lost:
                    break
                control.wait_for_reset()
                continue
            if not ok:
                print("[stop] did not converge at receiving -- stopping mission.")
                control.state = "SENSOR_FAIL"
                control.heartbeat(force=True)
                break

            cycles_done += 1
            print(f"\n[cycle] {cycles_done} complete.")
            control.state = "WAIT_SIGNAL"
            control.heartbeat(force=True)
    except MissionInterrupted:
        if control.emergency_stop:
            control.wait_for_reset()
        else:
            control.heartbeat(force=True)
    except TimeoutError as error:
        print(f"[stop] {error}")
        control.state = "SENSOR_FAIL"
        control.heartbeat(force=True)
    except KeyboardInterrupt:
        print("\n[stop] interrupted by user.")
    finally:
        hw.stop()
        try:
            hw.robot.stop_lidar()
        except Exception:
            pass
        server.stop()
        if status is not None:
            status.stop()

    print(f"\nRESULT: {cycles_done} cycle(s) completed.")


if __name__ == "__main__":
    main()
