from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""Real hardware only. Alternative to scripts/04_find_pose.py -- finds and
fixes heading AND position together via common/dock.py's find_pose_via_map()
(wide-radius grid search against the frozen point-cloud map, same tool
drive_with_localization() uses while driving) instead of find_pose()'s
linearized single-scan comparison, which was observed to diverge at the
defect zone for larger offsets (see find_pose_via_map()'s docstring).

Robot should be sitting near --zone (position and/or heading can both be
off). Needs data/map_points.json (scripts/08_build_map.py) -- does NOT need
--zone's reference scan.
"""

import argparse
import json
import math

from common.dock import find_pose_via_map
from common.hw import Hardware
from common.mapping import build_distance_field

CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "course_config.json"
DATA_DIR = Path(__file__).resolve().parents[1] / "data"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--zone", default="receiving", choices=["receiving", "defect"])
    args = parser.parse_args()

    with open(CONFIG_PATH, encoding="utf-8") as f:
        cfg = json.load(f)
    boundary = cfg["boundary"]
    zone = cfg["zones"][args.zone]

    map_points_path = DATA_DIR / "map_points.json"
    if not map_points_path.exists():
        print(f"[error] no map at {map_points_path} -- run scripts/08_build_map.py first.")
        return
    with open(map_points_path, encoding="utf-8") as f:
        map_points = [tuple(p) for p in json.load(f)]
    print(f"[map] {len(map_points)} points loaded -- building distance field...")
    distance_field = build_distance_field(map_points, boundary["x_m"], boundary["y_m"])

    hw = Hardware()
    try:
        hw.start_lidar()
        converged = find_pose_via_map(
            hw, distance_field, zone["x_m"], zone["y_m"], math.radians(zone["heading_deg"]),
        )
    finally:
        hw.stop()

    print()
    print("RESULT:", "converged (heading + position within tolerance)" if converged
          else "did NOT fully converge -- see log above")


if __name__ == "__main__":
    main()
