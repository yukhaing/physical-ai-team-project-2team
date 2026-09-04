from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import json
import math

from common.geometry import Pose2D, wrap_angle
from common.lidar import rectangle_segments, simulate_scan
from common.localize import localize
from common.motion import align_to_heading_command, integrate_dead_reckoning

CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "course_config.json"
WHEEL_BASE_M = 0.0956

# Robot is hand-placed exactly at the receiving zone position -- only heading is
# off. STARTING_HEADING_DEG is the (wrong, on purpose) heading to test correcting
# from; on the real robot this would come from wherever it happened to be set down.
STARTING_HEADING_DEG = 130.0

# The heading we *want* the robot facing once it's docked in the receiving zone
# (0deg = facing +x / "3 o'clock", matching the room's x-axis).
DESIRED_HEADING_DEG = 0.0

DOCK_HEADING_TOL_DEG = 3.0
DOCK_TURN_MPS = 0.05
DOCK_TURN_MAX_STEP_S = 0.3
DOCK_MAX_ITERS = 15
LOCALIZE_THETA_WINDOW_DEG = 40.0  # search this wide around the current dead-reckoning heading guess
LOCALIZE_THETA_STEPS = 80  # -> 0.5deg resolution within that window


def load_config() -> dict:
    with open(CONFIG_PATH, encoding="utf-8") as f:
        return json.load(f)


def dock_heading(pose: Pose2D, wall_segments, desired_theta: float) -> Pose2D:
    """Rotate in place until a LiDAR scan-match confirms the robot is facing
    `desired_theta`. Re-scans and re-localizes after every turn instead of
    trusting the open-loop dead-reckoning turn, and scales each turn's duration
    to the remaining error so it converges instead of overshooting back and
    forth (which a fixed-size turn step does once the error gets small)."""
    tol_rad = math.radians(DOCK_HEADING_TOL_DEG)
    turn_rate_rad_s = 2.0 * DOCK_TURN_MPS / WHEEL_BASE_M  # from wheel geometry, in-place turn
    est_pose = pose
    for i in range(DOCK_MAX_ITERS):
        scan = simulate_scan(pose, wall_segments)  # what the real sensor would see
        est_pose, match_err_m = localize(
            scan, wall_segments, pose.x, pose.y, pose.theta,
            theta_search_range_rad=math.radians(LOCALIZE_THETA_WINDOW_DEG),
            theta_steps=LOCALIZE_THETA_STEPS,
        )
        heading_err_rad = wrap_angle(desired_theta - est_pose.theta)
        heading_err_deg = math.degrees(heading_err_rad)
        print(f"[dock {i}] lidar pose=({est_pose.x:.3f}, {est_pose.y:.3f}) "
              f"heading={math.degrees(est_pose.theta):.1f}deg heading_err={heading_err_deg:+.1f}deg "
              f"match_err={match_err_m * 1000:.1f}mm")
        if abs(heading_err_deg) <= DOCK_HEADING_TOL_DEG:
            break
        duration_s = min(DOCK_TURN_MAX_STEP_S, abs(heading_err_rad) / turn_rate_rad_s)
        left, right, _ = align_to_heading_command(est_pose, desired_theta, DOCK_TURN_MPS, tol_rad)
        pose = integrate_dead_reckoning(est_pose, left, right, WHEEL_BASE_M, duration_s)
    return est_pose


def main() -> None:
    cfg = load_config()
    boundary = cfg["boundary"]
    receiving = cfg["zones"]["receiving"]
    wall_segments = rectangle_segments(0.0, 0.0, boundary["x_m"], boundary["y_m"])
    desired_theta = math.radians(DESIRED_HEADING_DEG)

    start_pose = Pose2D(receiving["x_m"], receiving["y_m"], math.radians(STARTING_HEADING_DEG))
    print(f"[start] placed at receiving zone ({start_pose.x:.3f}, {start_pose.y:.3f}) "
          f"heading={STARTING_HEADING_DEG:.1f}deg (wrong on purpose)")

    final_pose = dock_heading(start_pose, wall_segments, desired_theta)

    pos_error_cm = final_pose.distance_to(receiving["x_m"], receiving["y_m"]) * 100.0
    heading_error_deg = math.degrees(wrap_angle(desired_theta - final_pose.theta))

    print()
    print(f"target      = ({receiving['x_m']:.3f}, {receiving['y_m']:.3f}) desired heading={DESIRED_HEADING_DEG:.1f}deg")
    print(f"lidar pose  = ({final_pose.x:.3f}, {final_pose.y:.3f}) heading={math.degrees(final_pose.theta):.1f}deg")
    print(f"position error = {pos_error_cm:.1f} cm, heading error = {heading_error_deg:+.1f} deg")


if __name__ == "__main__":
    main()

