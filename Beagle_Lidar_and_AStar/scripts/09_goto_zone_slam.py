from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""Real hardware only. This session's chosen architecture:

    Map: built ONCE, saved, fixed (scripts/08_build_map.py) --------- done ahead of time
    Localization: CONTINUOUS while driving (odometry every tick +
                  periodic LiDAR check against the fixed map)
    Final alignment: LiDAR-only, tight, once near the target zone

Robot must already be sitting at a LiDAR-verified pose at --from (e.g. just
ran scripts/04_find_pose.py --zone <from> successfully). Plans an A* path
(avoiding obstacles in data/obstacle_map.json) from --from to --to, drives it
with common/navigate.py's goto_zone() (drive_with_localization() for
continuous encoder+gyro odometry corrected periodically against
data/map_points.json, then find_pose() for the final precise alignment).

--round-trip: after reaching --to, turns right back around and drives --to ->
--from too, so a full receiving<->defect cycle can be checked in one run.
"""

import argparse
import json

from common.hw import Hardware
from common.mapping import build_distance_field
from common.navigate import goto_zone

CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "course_config.json"
DATA_DIR = Path(__file__).resolve().parents[1] / "data"


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


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--from", dest="from_zone", default="receiving", choices=["receiving", "defect"])
    parser.add_argument("--to", dest="to_zone", default="defect", choices=["receiving", "defect"])
    parser.add_argument("--round-trip", action="store_true",
                         help="After reaching --to, drive right back --to -> --from too "
                              "(check a full receiving<->defect cycle in one run).")
    parser.add_argument("--dynamic-obstacles", action="store_true",
                         help="EXPERIMENTAL, off by default (2026-09-01 regression -- see "
                              "common/navigate.py's goto_zone() docstring): live obstacle "
                              "detection/replanning while driving.")
    parser.add_argument("--align", action="store_true",
                         help="EXPERIMENTAL, off by default: run find_pose() final alignment "
                              "after arriving (see common/navigate.py's goto_zone() docstring "
                              "for its unresolved accuracy issue at defect).")
    parser.add_argument("--no-align-heading", action="store_true",
                         help="Disable even the lightweight heading-only realign_heading() that "
                              "runs by default when --align is off -- NOT recommended, see "
                              "common/navigate.py's goto_zone() docstring (skipping it left the "
                              "next leg's dead-reckoning ~205deg off and drove toward a wall).")
    args = parser.parse_args()

    if args.from_zone == args.to_zone:
        print("[error] --from and --to are the same zone, nothing to do.")
        return

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

    legs = [(args.from_zone, args.to_zone)]
    if args.round_trip:
        legs.append((args.to_zone, args.from_zone))

    reference_scans = {}
    for _, to_name in legs:
        if to_name not in reference_scans:
            scan = load_reference(to_name)
            if scan is None:
                return
            reference_scans[to_name] = scan

    hw = Hardware()
    results = []
    try:
        hw.start_lidar()
        for i, (from_name, to_name) in enumerate(legs):
            print()
            print(f"===== leg {i + 1}/{len(legs)}: {from_name} -> {to_name} =====")
            ok = goto_zone(hw, cfg, distance_field, obstacles, from_name, to_name, reference_scans[to_name],
                           dynamic_obstacles=args.dynamic_obstacles, align=args.align,
                           align_heading=not args.no_align_heading)
            results.append((from_name, to_name, ok))
            if not ok:
                print(f"[stop] leg {from_name} -> {to_name} did not converge -- not attempting "
                      "further legs from an unverified pose.")
                break
    finally:
        hw.stop()

    print()
    for from_name, to_name, ok in results:
        print(f"  {from_name} -> {to_name}: {'OK' if ok else 'FAILED'}")


if __name__ == "__main__":
    main()
