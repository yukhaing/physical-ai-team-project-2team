from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""Real hardware only. Robot must already be sitting at a LiDAR-verified pose
(just ran scripts/04_find_pose.py successfully at the given --zone) -- this
does NOT estimate or correct pose itself, it only detects obstacles relative
to the pose you tell it it's at.

Takes one scan, compares it to the known empty-room wall model, and any point
that reads meaningfully closer than a bare wall would (e.g. an OMX arm base)
gets recorded as an obstacle rectangle. Rects are merged into
data/obstacle_map.json (one static file, built once per zone -- not updated
during actual navigation, see the design note in common/obstacles.py's
docstring and this session's SLAM discussion).
"""

import argparse
import json
import math
import time

from common.geometry import Pose2D
from common.hw import Hardware
from common.lidar import rectangle_segments
from common.obstacles import detect_obstacle_points, obstacle_rects_from_points

CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "course_config.json"
OBSTACLE_MAP_PATH = Path(__file__).resolve().parents[1] / "data" / "obstacle_map.json"
SETTLE_S = 0.15
REFERENCE_HEADING_DEG = 0.0  # matches scripts/03_calibrate_and_realign.py's convention


def load_config() -> dict:
    with open(CONFIG_PATH, encoding="utf-8") as f:
        return json.load(f)


def load_obstacle_map() -> dict:
    if OBSTACLE_MAP_PATH.exists():
        with open(OBSTACLE_MAP_PATH, encoding="utf-8") as f:
            return json.load(f)
    return {}


def save_obstacle_map(data: dict) -> None:
    OBSTACLE_MAP_PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(OBSTACLE_MAP_PATH, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--zone", required=True, choices=["receiving", "defect"],
                         help="Which zone the robot is currently sitting at (must already be "
                              "LiDAR-verified via scripts/04_find_pose.py for that zone).")
    args = parser.parse_args()

    cfg = load_config()
    boundary = cfg["boundary"]
    zone = cfg["zones"][args.zone]
    wall_segments = rectangle_segments(0.0, 0.0, boundary["x_m"], boundary["y_m"])
    # find_pose() aligns the robot to this zone's own reference heading (see
    # course_config.json's "heading_deg" -- receiving=0/3 o'clock, defect=180/9
    # o'clock), not always 0.
    pose = Pose2D(zone["x_m"], zone["y_m"], math.radians(zone["heading_deg"]))

    hw = Hardware()
    try:
        hw.start_lidar()
        time.sleep(SETTLE_S)
        scan = hw.scan()
    finally:
        hw.stop()

    points = detect_obstacle_points(pose, scan, wall_segments)
    rects = obstacle_rects_from_points(points)
    print(f"[{args.zone}] {len(points)} obstacle points -> {len(rects)} rect(s):")
    for r in rects:
        print(f"  ({r[0]:.3f}, {r[1]:.3f}) - ({r[2]:.3f}, {r[3]:.3f})")

    obstacle_map = load_obstacle_map()
    obstacle_map[args.zone] = rects
    save_obstacle_map(obstacle_map)
    print(f"saved -> {OBSTACLE_MAP_PATH}")


if __name__ == "__main__":
    main()
