from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""Real hardware only. Diagnostic -- NEVER moves the robot (no turn_by_angle,
no drive_forward, no wheels() call). Takes ONE scan and reports what
best_rotation_offset()/estimate_pose_offset() read from it against the saved
reference scan, so heading/position estimation can be checked in isolation
from any movement/execution error.

Place the robot as close as you can to EXACTLY how it was when you ran
--calibrate (same spot, same heading, 3 o'clock) and run this. If the
estimator itself is sound, it should report close to 0deg / 0cm. If it
already reports several cm/deg here with no movement involved at all, the
problem is in the measurement, not in turning or driving.
"""

import argparse
import json
import math
import time

from common.dock import SETTLE_S
from common.hw import Hardware
from common.scan_align import best_rotation_offset, estimate_pose_offset, mask_from_angle_range

CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "course_config.json"
DATA_DIR = Path(__file__).resolve().parents[1] / "data"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--zone", default="receiving", choices=["receiving", "defect"])
    args = parser.parse_args()

    reference_path = DATA_DIR / f"{args.zone}_reference_scan.json"
    if not reference_path.exists():
        print(f"[error] no reference scan saved yet at {reference_path}")
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

    hw = Hardware()
    try:
        hw.start_lidar()
        for i in range(5):
            time.sleep(SETTLE_S)
            scan = hw.scan()
            rotation_rad, rot_match_err = best_rotation_offset(scan, reference_scan, mask=mask)
            dtheta, dx, dy, residual = estimate_pose_offset(scan, reference_scan, mask=mask)
            pos_err_m = math.hypot(dx, dy)
            print(f"[measure {i}] rotation_only={math.degrees(rotation_rad):+.1f}deg "
                  f"rot_match_err={rot_match_err * 1000:.1f}mm | joint_dtheta={math.degrees(dtheta):+.1f}deg "
                  f"pos=({dx * 100:+.1f},{dy * 100:+.1f})cm={pos_err_m * 100:.1f}cm residual={residual * 1000:.1f}mm")
    finally:
        hw.stop()


if __name__ == "__main__":
    main()

