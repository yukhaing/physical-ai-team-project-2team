from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""ONE-TIME SETUP -- build data/map_points.json from the reference scans
already captured at receiving/defect (each was captured at a known, verified
pose: course_config.json's zone x_m/y_m/heading_deg -- the same pose
scripts/03_calibrate_and_realign.py --calibrate used). No new hardware access
needed; no SLAM-style simultaneous localization happens here -- every pose
used is already trusted, this just converts each scan to world-frame points
(common/mapping.py's scan_to_world_points()) and merges them.

Run this once whenever any zone's reference scan is recaptured, or a new
zone's reference scan is added. Re-run scripts that consume the map (e.g. a
future point-cloud-based checkpoint_correct()) after updating it.
"""

import json
import math

from common.geometry import Pose2D
from common.mapping import build_map_from_references

CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "course_config.json"
DATA_DIR = Path(__file__).resolve().parents[1] / "data"
MAP_PATH = DATA_DIR / "map_points.json"
ZONES = ["receiving", "defect"]


def load_json(path: Path) -> dict:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def main() -> None:
    cfg = load_json(CONFIG_PATH)
    scans_with_poses = []
    used_zones = []
    for zone in ZONES:
        ref_path = DATA_DIR / f"{zone}_reference_scan.json"
        if not ref_path.exists():
            print(f"[skip] no reference scan for {zone} at {ref_path}")
            continue
        scan = load_json(ref_path)
        z = cfg["zones"][zone]
        pose = Pose2D(z["x_m"], z["y_m"], math.radians(z["heading_deg"]))
        scans_with_poses.append((scan, pose))
        used_zones.append(zone)
        print(f"[load] {zone}: pose=({pose.x:.3f},{pose.y:.3f}) heading={z['heading_deg']:.0f}deg, "
              f"{len(scan)} rays")

    if not scans_with_poses:
        print("[error] no reference scans found -- run scripts/03_calibrate_and_realign.py "
              "--zone <zone> --calibrate for at least one zone first.")
        return

    map_points = build_map_from_references(scans_with_poses)
    print(f"[build] merged {len(scans_with_poses)} scan(s) ({', '.join(used_zones)}) -> "
          f"{len(map_points)} world-frame points")

    MAP_PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(MAP_PATH, "w", encoding="utf-8") as f:
        json.dump(map_points, f)
    print(f"[save] -> {MAP_PATH}")


if __name__ == "__main__":
    main()

