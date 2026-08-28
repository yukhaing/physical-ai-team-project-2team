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
from common.scan_align import best_rotation_offset

CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "course_config.json"
REFERENCE_PATH = Path(__file__).resolve().parents[1] / "data" / "receiving_reference_scan.json"
WHEEL_BASE_M = 0.0956

# "3 o'clock" == facing +x == theta=0. This is the heading the robot is placed
# at, by hand, the one time we capture the reference scan.
REFERENCE_HEADING_DEG = 0.0

# --dry-run only: pretend the robot got placed back down facing the wrong way,
# to prove realign() can recover REFERENCE_HEADING_DEG from the saved scan
# alone (no wall coordinates involved in the realign step).
TEST_MISPLACED_HEADING_DEG = 130.0

REALIGN_TOL_DEG = 3.0
REALIGN_TURN_MAX_STEP_S = 0.3
REALIGN_MAX_ITERS = 15
REALIGN_TURN_MPS_SIM = 0.05  # --dry-run: ideal kinematic wheel speed
REALIGN_TURN_PERCENT_REAL = 10.0  # real hardware: conservative wheel percent for in-place turns
# Rough estimate only (wheel_percent -> deg/s) -- exact rate does not matter much
# because every step re-measures the true heading with LiDAR afterward instead
# of trusting this number; it only affects how many iterations convergence takes.
REALIGN_TURN_RATE_DEG_PER_S_REAL = 35.0
SETTLE_S = 0.15  # real hardware only: pause after stopping before trusting the scan


def load_config() -> dict:
    with open(CONFIG_PATH, encoding="utf-8") as f:
        return json.load(f)


def capture_reference_sim(receiving: dict, wall_segments) -> list[float]:
    ref_pose = Pose2D(receiving["x_m"], receiving["y_m"], math.radians(REFERENCE_HEADING_DEG))
    scan = simulate_scan(ref_pose, wall_segments)
    print(f"[calibrate/dry-run] simulated at ({ref_pose.x:.3f}, {ref_pose.y:.3f}) "
          f"heading={REFERENCE_HEADING_DEG:.1f}deg (3 o'clock)")
    return scan


def capture_reference_real() -> list[float]:
    from common.hw import Hardware

    hw = Hardware()
    try:
        hw.start_lidar()
        time.sleep(SETTLE_S)
        scan = hw.scan()
    finally:
        hw.stop()
    print(f"[calibrate/real] captured {len(scan)}-ray scan from the real LiDAR "
          f"(robot must have been sitting exactly at the receiving zone, facing 3 o'clock)")
    return scan


def save_reference(scan: list[float]) -> None:
    REFERENCE_PATH.parent.mkdir(parents=True, exist_ok=True)
    with open(REFERENCE_PATH, "w", encoding="utf-8") as f:
        json.dump(scan, f)
    print(f"[calibrate] saved {len(scan)}-ray reference scan -> {REFERENCE_PATH}")


def load_reference() -> list[float]:
    with open(REFERENCE_PATH, encoding="utf-8") as f:
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


def realign_real(reference_scan: list[float]) -> None:
    """Real hardware: rotate in place until the live LiDAR scan matches
    `reference_scan`. Always stops the wheels and re-scans before deciding the
    next move -- never trusts an open-loop turn's estimated angle by itself."""
    from common.hw import Hardware

    tol_rad = math.radians(REALIGN_TOL_DEG)
    turn_rate_rad_s = math.radians(REALIGN_TURN_RATE_DEG_PER_S_REAL)

    hw = Hardware()
    try:
        hw.start_lidar()
        for i in range(REALIGN_MAX_ITERS):
            time.sleep(SETTLE_S)
            scan = hw.scan()
            rotation_rad, match_err_m = best_rotation_offset(scan, reference_scan)
            print(f"[realign {i}] needed_turn={math.degrees(rotation_rad):+.1f}deg "
                  f"match_err={match_err_m * 1000:.1f}mm")
            if abs(rotation_rad) <= tol_rad:
                print("[realign] within tolerance, done.")
                break
            duration_s = min(REALIGN_TURN_MAX_STEP_S, abs(rotation_rad) / turn_rate_rad_s)
            direction = 1.0 if rotation_rad > 0 else -1.0
            hw.wheels(-direction * REALIGN_TURN_PERCENT_REAL, direction * REALIGN_TURN_PERCENT_REAL)
            time.sleep(duration_s)
            hw.stop()
        else:
            print("[realign] max iterations reached without converging -- check match_err "
                  "above; if it stayed large, the reference scan may not match this spot.")
    finally:
        hw.stop()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry-run", action="store_true",
                         help="Simulate instead of driving the real robot.")
    parser.add_argument("--calibrate", action="store_true",
                         help="Capture and save the reference scan (robot must be placed "
                              "exactly at the receiving zone, facing 3 o'clock, right now).")
    args = parser.parse_args()

    cfg = load_config()
    boundary = cfg["boundary"]
    receiving = cfg["zones"]["receiving"]
    wall_segments = rectangle_segments(0.0, 0.0, boundary["x_m"], boundary["y_m"])

    if args.calibrate:
        scan = capture_reference_sim(receiving, wall_segments) if args.dry_run else capture_reference_real()
        save_reference(scan)
        return

    if not REFERENCE_PATH.exists():
        print("[error] no reference scan saved yet -- run with --calibrate first "
              "(robot placed exactly at the receiving zone, facing 3 o'clock).")
        return
    reference_scan = load_reference()

    if args.dry_run:
        test_pose = Pose2D(receiving["x_m"], receiving["y_m"], math.radians(TEST_MISPLACED_HEADING_DEG))
        print(f"[start/dry-run] placed at receiving zone ({test_pose.x:.3f}, {test_pose.y:.3f}) "
              f"heading={TEST_MISPLACED_HEADING_DEG:.1f}deg (wrong on purpose, for this sim test)")
        final_pose = realign_sim(test_pose, wall_segments, reference_scan)
        heading_error_deg = math.degrees(wrap_angle(math.radians(REFERENCE_HEADING_DEG) - final_pose.theta))
        print()
        print(f"reference heading = {REFERENCE_HEADING_DEG:.1f}deg (3 o'clock)")
        print(f"final heading     = {math.degrees(final_pose.theta):.1f}deg")
        print(f"heading error     = {heading_error_deg:+.1f} deg")
    else:
        print("[start/real] robot should already be sitting at the receiving zone "
              "(position only -- heading can be anything).")
        realign_real(reference_scan)


if __name__ == "__main__":
    main()
