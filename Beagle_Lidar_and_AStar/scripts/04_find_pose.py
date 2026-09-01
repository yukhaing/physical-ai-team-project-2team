from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""Real hardware only. Finds and fixes heading AND position together: every
iteration takes one LiDAR scan, compares it to the saved --zone reference
scan, and moves accordingly (see common/dock.py's find_pose()). Nothing is
assumed from a previous run -- 03_calibrate_and_realign.py does not need to be
run first, this handles heading on its own too (falling back to a coarse
robust turn first if it's far off, since the fine joint estimate only holds
for small rotation).

Robot should be sitting near --zone (position and/or heading can both be off).
Uses the SAME reference scan as script 03 -- no separate capture needed.
"""

import argparse
import json

from common.dock import SANITY_MATCH_ERR_M, find_pose
from common.hw import Hardware
from common.scan_align import mask_from_angle_range

CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "course_config.json"
DATA_DIR = Path(__file__).resolve().parents[1] / "data"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--zone", default="receiving", choices=["receiving", "defect"])
    args = parser.parse_args()

    reference_path = DATA_DIR / f"{args.zone}_reference_scan.json"
    if not reference_path.exists():
        print(f"[error] no reference scan saved yet at {reference_path} -- run "
              f"scripts/03_calibrate_and_realign.py --zone {args.zone} --calibrate first "
              f"(robot placed exactly at the {args.zone} zone, facing 3 o'clock).")
        return
    with open(reference_path, encoding="utf-8") as f:
        reference_scan = json.load(f)

    with open(CONFIG_PATH, encoding="utf-8") as f:
        cfg = json.load(f)
    exclude_deg_range = cfg["zones"][args.zone].get("exclude_deg_range")
    mask = mask_from_angle_range(len(reference_scan), *exclude_deg_range) if exclude_deg_range else None
    if mask:
        print(f"[mask] excluding {len(mask)}/{len(reference_scan)} rays "
              f"({exclude_deg_range[0]:+.0f}deg to {exclude_deg_range[1]:+.0f}deg) -- non-static scene element")

    sanity_match_err_m = cfg["zones"][args.zone].get("match_sanity_mm", SANITY_MATCH_ERR_M * 1000.0) / 1000.0
    if sanity_match_err_m != SANITY_MATCH_ERR_M:
        print(f"[sanity] using {sanity_match_err_m * 1000:.0f}mm match-quality threshold for {args.zone} "
              f"(default {SANITY_MATCH_ERR_M * 1000:.0f}mm)")

    hw = Hardware()
    try:
        hw.start_lidar()
        converged = find_pose(hw, reference_scan, mask=mask, sanity_match_err_m=sanity_match_err_m)
    finally:
        hw.stop()

    print()
    print("RESULT:", "converged (heading + position within tolerance)" if converged
          else "did NOT fully converge -- see log above")


if __name__ == "__main__":
    main()
