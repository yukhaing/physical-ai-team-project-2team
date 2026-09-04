from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import argparse
import json
import math
import time

from common.geometry import Pose2D, wrap_angle
from common.lidar import rectangle_segments, simulate_scan
from common.motion import align_to_heading_command, integrate_dead_reckoning
from common.scan_align import best_rotation_offset, mask_from_angle_range

CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "course_config.json"
DATA_DIR = Path(__file__).resolve().parents[1] / "data"
WHEEL_BASE_M = 0.0956


def reference_paths(zone: str) -> tuple[Path, Path]:
    """(real_path, sim_path) for `zone`. Separate files -- a real LiDAR scan
    (with the OMX arms etc. in it) and a --dry-run simulated scan (walls only,
    no obstacles) are not the same shape, so comparing one against the other
    would just produce nonsense offsets."""
    return DATA_DIR / f"{zone}_reference_scan.json", DATA_DIR / f"{zone}_reference_scan_dry_run.json"

# Clock-position labels for each zone's reference heading (0deg=3 o'clock/+x,
# 90deg=12 o'clock/+y, 180deg=9 o'clock/-x, 270deg=6 o'clock/-y), just for
# human-readable messages -- the actual target degree comes from each zone's
# "heading_deg" in course_config.json (receiving=0/3 o'clock, defect=180/9
# o'clock, matching the OMX arm orientation at that zone).
CLOCK_LABELS = {0.0: "3 o'clock", 90.0: "12 o'clock", 180.0: "9 o'clock", 270.0: "6 o'clock"}


def clock_label(heading_deg: float) -> str:
    return CLOCK_LABELS.get(heading_deg % 360.0, f"{heading_deg:.0f}deg")


# --dry-run only: pretend the robot got placed back down facing the wrong way,
# to prove realign() can recover the zone's reference heading from the saved
# scan alone (no wall coordinates involved in the realign step).
TEST_MISPLACED_HEADING_DEG = 130.0

REALIGN_TOL_DEG = 3.0
REALIGN_TURN_MAX_STEP_S = 0.5
REALIGN_MAX_ITERS = 30
REALIGN_TURN_MPS_SIM = 0.05  # --dry-run: ideal kinematic wheel speed
# Matches common/dock.py's SETTLE_S (raised from 0.15 the same day, 2026-09-01)
# -- a reference scan captured here becomes the permanent baseline everything
# else gets judged against, so it deserves at least as much settle time as any
# other scan, not less.
SETTLE_S = 1.0  # real hardware only: pause after stopping before trusting the scan


def load_config() -> dict:
    with open(CONFIG_PATH, encoding="utf-8") as f:
        return json.load(f)


def capture_reference_sim(zone_coords: dict, wall_segments) -> list[float]:
    heading_deg = zone_coords["heading_deg"]
    ref_pose = Pose2D(zone_coords["x_m"], zone_coords["y_m"], math.radians(heading_deg))
    scan = simulate_scan(ref_pose, wall_segments)
    print(f"[calibrate/dry-run] simulated at ({ref_pose.x:.3f}, {ref_pose.y:.3f}) "
          f"heading={heading_deg:.1f}deg ({clock_label(heading_deg)})")
    return scan


def capture_reference_real(heading_deg: float) -> list[float]:
    from common.hw import Hardware

    hw = Hardware()
    try:
        hw.start_lidar()
        time.sleep(SETTLE_S)
        scan = hw.scan()
    finally:
        hw.stop()
    print(f"[calibrate/real] captured {len(scan)}-ray scan from the real LiDAR "
          f"(robot must have been sitting exactly at the zone, facing {clock_label(heading_deg)})")
    return scan


def save_reference(scan: list[float], path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(scan, f)
    print(f"[calibrate] saved {len(scan)}-ray reference scan -> {path}")


def load_reference(path: Path) -> list[float]:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def realign_sim(pose: Pose2D, wall_segments, reference_scan: list[float]) -> Pose2D:
    """--dry-run: rotate a simulated pose in place until its scan matches
    `reference_scan`, re-scanning and re-checking after every turn."""
    tol_rad = math.radians(REALIGN_TOL_DEG)
    turn_rate_rad_s = 2.0 * REALIGN_TURN_MPS_SIM / WHEEL_BASE_M
    for i in range(REALIGN_MAX_ITERS):
        scan = simulate_scan(pose, wall_segments)
        rotation_rad, match_err_m = best_rotation_offset(scan, reference_scan)
        print(f"[realign {i}] heading={math.degrees(pose.theta):.1f}deg "
              f"needed_turn={math.degrees(rotation_rad):+.1f}deg match_err={match_err_m * 1000:.1f}mm")
        if abs(rotation_rad) <= tol_rad:
            break
        duration_s = min(REALIGN_TURN_MAX_STEP_S, abs(rotation_rad) / turn_rate_rad_s)
        target_theta = wrap_angle(pose.theta + rotation_rad)
        left, right, _ = align_to_heading_command(pose, target_theta, REALIGN_TURN_MPS_SIM, tol_rad)
        pose = integrate_dead_reckoning(pose, left, right, WHEEL_BASE_M, duration_s)
    return pose


def realign_real(reference_scan: list[float], mask: set[int] | None = None,
                  tol_deg: float | None = None) -> None:
    """Real hardware: rotate in place until the live LiDAR scan matches
    `reference_scan`. Delegates to common/dock.py's realign_heading() so this
    and scripts/04_dock_position_and_heading.py share one implementation."""
    from common.dock import REALIGN_TOL_DEG, realign_heading
    from common.hw import Hardware

    hw = Hardware()
    try:
        hw.start_lidar()
        converged, _ = realign_heading(hw, reference_scan, mask=mask,
                                        tol_deg=tol_deg if tol_deg is not None else REALIGN_TOL_DEG)
        if converged:
            print("[realign] within tolerance, done.")
        else:
            print("[realign] max iterations reached without converging -- check match_err "
                  "above; if it stayed large, the reference scan may not match this spot.")
    finally:
        hw.stop()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--zone", default="receiving", choices=["receiving", "defect"],
                         help="Which zone's reference scan to use/capture (default: receiving).")
    parser.add_argument("--dry-run", action="store_true",
                         help="Simulate instead of driving the real robot.")
    parser.add_argument("--calibrate", action="store_true",
                         help="Capture and save the reference scan (robot must be placed "
                              "exactly at --zone's heading_deg, right now).")
    args = parser.parse_args()

    cfg = load_config()
    boundary = cfg["boundary"]
    zone_coords = cfg["zones"][args.zone]
    heading_deg = zone_coords["heading_deg"]
    wall_segments = rectangle_segments(0.0, 0.0, boundary["x_m"], boundary["y_m"])

    reference_path_real, reference_path_sim = reference_paths(args.zone)
    reference_path = reference_path_sim if args.dry_run else reference_path_real

    if args.calibrate:
        scan = capture_reference_sim(zone_coords, wall_segments) if args.dry_run else capture_reference_real(heading_deg)
        save_reference(scan, reference_path)
        return

    if not reference_path.exists():
        print(f"[error] no reference scan saved yet at {reference_path} -- run with --calibrate first "
              f"(robot placed exactly at the {args.zone} zone, facing {clock_label(heading_deg)}).")
        return
    reference_scan = load_reference(reference_path)

    if args.dry_run:
        test_pose = Pose2D(zone_coords["x_m"], zone_coords["y_m"], math.radians(TEST_MISPLACED_HEADING_DEG))
        print(f"[start/dry-run] placed at {args.zone} zone ({test_pose.x:.3f}, {test_pose.y:.3f}) "
              f"heading={TEST_MISPLACED_HEADING_DEG:.1f}deg (wrong on purpose, for this sim test)")
        final_pose = realign_sim(test_pose, wall_segments, reference_scan)
        heading_error_deg = math.degrees(wrap_angle(math.radians(heading_deg) - final_pose.theta))
        print()
        print(f"reference heading = {heading_deg:.1f}deg ({clock_label(heading_deg)})")
        print(f"final heading     = {math.degrees(final_pose.theta):.1f}deg")
        print(f"heading error     = {heading_error_deg:+.1f} deg")
    else:
        print(f"[start/real] robot should already be sitting at the {args.zone} zone "
              "(position only -- heading can be anything).")
        exclude_deg_range = zone_coords.get("exclude_deg_range")
        mask = mask_from_angle_range(len(reference_scan), *exclude_deg_range) if exclude_deg_range else None
        if mask:
            print(f"[mask] excluding {len(mask)}/{len(reference_scan)} rays "
                  f"({exclude_deg_range[0]:+.0f}deg to {exclude_deg_range[1]:+.0f}deg) -- non-static scene element")
        heading_tol_deg = zone_coords.get("heading_tol_deg")
        realign_real(reference_scan, mask=mask, tol_deg=heading_tol_deg)


if __name__ == "__main__":
    main()

