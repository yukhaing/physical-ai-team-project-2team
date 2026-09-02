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
  WAIT_PICKED      -- idle at defect until a {"event": "box_picked"} TCP
                       message arrives on the same TriggerServer/port --
                       mirrors WAIT_SIGNAL, so the OMX arm decides when it's
                       actually done picking up the box (no fixed dwell --
                       removed 2026-09-02 as redundant now that this signal
                       exists)
  GOTO_RECEIVING   -- goto_zone(defect -> receiving); aligns to 3 o'clock
  -> back to WAIT_SIGNAL, repeat

If a leg's find_pose() doesn't converge, the mission stops (driving the next
leg from an unverified pose would just compound the error) -- see the printed
RESULT for which zone it was heading to when it stopped.
"""

import argparse
import json
import time

from common.comm import TriggerServer
from common.hw import Hardware
from common.mapping import build_distance_field
from common.navigate import goto_zone

CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "course_config.json"
DATA_DIR = Path(__file__).resolve().parents[1] / "data"
TRIGGER_PORT = 8765


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


def wait_for_event(server: TriggerServer, event: str, label: str) -> None:
    """Block until a {"event": <event>} TCP message arrives (see
    common/comm.py's TriggerServer) -- shared by WAIT_SIGNAL (box_placed, at
    receiving) and WAIT_PICKED (box_picked, at defect) below, same server/
    port, just a different event name and idle-print label."""
    print(f"[{label}] idling, waiting for {event} signal (TCP :{TRIGGER_PORT}, or Ctrl+C to stop)...")
    while True:
        for message in server.poll():
            if message.get("event") == event:
                print(f"[{label}] {event} received.")
                return
        time.sleep(0.1)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cycles", type=int, default=0,
                         help="Stop after N receiving->defect->receiving round trips (0 = unlimited).")
    parser.add_argument("--trigger-port", type=int, default=TRIGGER_PORT)
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

    hw = Hardware()
    cycles_done = 0
    try:
        hw.start_lidar()
        while args.cycles <= 0 or cycles_done < args.cycles:
            wait_for_event(server, "box_placed", "WAIT_SIGNAL")

            print("\n===== GOTO_DEFECT =====")
            ok = goto_zone(hw, cfg, distance_field, obstacles, "receiving", "defect", defect_ref,
                           dynamic_obstacles=args.dynamic_obstacles, align=args.align,
                           align_map=not args.no_align_map, align_heading=args.align_heading)
            if not ok:
                print("[stop] did not converge at defect -- stopping mission.")
                break

            print("\n===== WAIT_PICKED =====")
            wait_for_event(server, "box_picked", "WAIT_PICKED")

            print("\n===== GOTO_RECEIVING =====")
            ok = goto_zone(hw, cfg, distance_field, obstacles, "defect", "receiving", receiving_ref,
                           dynamic_obstacles=args.dynamic_obstacles, align=args.align,
                           align_map=not args.no_align_map, align_heading=args.align_heading)
            if not ok:
                print("[stop] did not converge at receiving -- stopping mission.")
                break

            cycles_done += 1
            print(f"\n[cycle] {cycles_done} complete.")
    except KeyboardInterrupt:
        print("\n[stop] interrupted by user.")
    finally:
        hw.stop()
        server.stop()

    print(f"\nRESULT: {cycles_done} cycle(s) completed.")


if __name__ == "__main__":
    main()
