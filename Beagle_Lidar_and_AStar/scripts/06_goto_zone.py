from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""Real hardware only. Robot must already be sitting at a LiDAR-verified pose
at --from (just ran scripts/04_find_pose.py --zone <from> successfully).

Plans an A* path (avoiding obstacles recorded in data/obstacle_map.json, see
scripts/05_map_obstacles.py) from --from to --to, drives it with real
encoder+gyro dead-reckoning pure pursuit, periodically correcting against that
SAME static obstacle map via LiDAR checkpoints (common/navigate.py's
drive_path_real(checkpoint_segments=...) -> common/localize.py's
checkpoint_correct() -- the map is never updated during the drive, so this
carries none of continuous SLAM's self-referential feedback-loop risk).
Confirmed 2026-08-31: dead reckoning with no checkpoints drifted ~15cm+ over a
single ~40cm continuous curve, well past find_pose()'s 100mm sanity
threshold -- checkpoints exist to bound that. Finishes with find_pose() at
--to for the precise final correction. --to's reference scan must already
exist (scripts/03_calibrate_and_realign.py --zone <to> --calibrate).
"""

import argparse
import json
import math

from common.dock import find_pose
from common.geometry import Pose2D
from common.hw import Hardware
from common.lidar import rectangle_segments
from common.localize import obstacle_segments
from common.navigate import drive_path_real
from common.planning import astar_path

CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "course_config.json"
DATA_DIR = Path(__file__).resolve().parents[1] / "data"
OBSTACLE_MAP_PATH = DATA_DIR / "obstacle_map.json"


def load_json(path: Path) -> dict:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--from", dest="from_zone", default="receiving", choices=["receiving", "defect"])
    parser.add_argument("--to", dest="to_zone", default="defect", choices=["receiving", "defect"])
    args = parser.parse_args()

    if args.from_zone == args.to_zone:
        print("[error] --from and --to are the same zone, nothing to do.")
        return

    cfg = load_json(CONFIG_PATH)
    boundary = cfg["boundary"]
    from_zone = cfg["zones"][args.from_zone]
    to_zone = cfg["zones"][args.to_zone]

    to_reference_path = DATA_DIR / f"{args.to_zone}_reference_scan.json"
    if not to_reference_path.exists():
        print(f"[error] no reference scan for {args.to_zone} at {to_reference_path} -- run "
              f"scripts/03_calibrate_and_realign.py --zone {args.to_zone} --calibrate first.")
        return
    to_reference_scan = load_json(to_reference_path)

    obstacles = []
    if OBSTACLE_MAP_PATH.exists():
        obstacle_map = load_json(OBSTACLE_MAP_PATH)
        obstacles = [tuple(r) for rects in obstacle_map.values() for r in rects]
    print(f"[plan] {len(obstacles)} known obstacle rect(s) loaded from {OBSTACLE_MAP_PATH}")

    path = astar_path(
        (from_zone["x_m"], from_zone["y_m"]), (to_zone["x_m"], to_zone["y_m"]),
        boundary["x_m"], boundary["y_m"], obstacles,
    )
    print(f"[plan] {args.from_zone} -> {args.to_zone}: {len(path)} waypoints")

    checkpoint_segments = rectangle_segments(0.0, 0.0, boundary["x_m"], boundary["y_m"]) + obstacle_segments(obstacles)

    start_pose = Pose2D(from_zone["x_m"], from_zone["y_m"], math.radians(from_zone["heading_deg"]))

    hw = Hardware()
    try:
        hw.start_lidar()
        print("[drive] starting pure-pursuit drive with periodic LiDAR checkpoints (expect this "
              "to end up only APPROXIMATELY at the target -- find_pose() below does the precise "
              "final correction)...")
        dr_pose = drive_path_real(hw, path, start_pose, checkpoint_segments=checkpoint_segments)
        print(f"[drive] dead-reckoned arrival pose = ({dr_pose.x:.3f}, {dr_pose.y:.3f}) "
              f"heading={math.degrees(dr_pose.theta):.1f}deg")

        print(f"[dock] fine-correcting at {args.to_zone} via find_pose()...")
        converged = find_pose(hw, to_reference_scan)
    finally:
        hw.stop()

    print()
    print("RESULT:", f"arrived and converged at {args.to_zone}" if converged
          else f"drove to {args.to_zone} but did NOT fully converge there -- see log above")


if __name__ == "__main__":
    main()
