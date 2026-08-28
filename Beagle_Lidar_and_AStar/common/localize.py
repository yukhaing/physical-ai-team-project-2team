from __future__ import annotations

import math

from common.geometry import Pose2D, wrap_angle
from common.lidar import Segment, simulate_scan


def _match_error(scan: list[float], pose: Pose2D, segments: list[Segment], max_range_m: float) -> float:
    """Mean absolute difference (meters) between an observed scan and the scan
    predicted at `pose` -- lower means `pose` better explains what the LiDAR saw."""
    predicted = simulate_scan(pose, segments, num_rays=len(scan), max_range_m=max_range_m)
    usable = [(a, b) for a, b in zip(scan, predicted) if a < max_range_m - 0.05 and b < max_range_m - 0.05]
    if not usable:
        return math.inf
    return sum(abs(a - b) for a, b in usable) / len(usable)


def localize(
    scan: list[float],
    segments: list[Segment],
    guess_x: float,
    guess_y: float,
    guess_theta: float,
    pos_search_radius_m: float = 0.03,
    pos_step_m: float = 0.01,
    theta_steps: int = 72,
    theta_search_range_rad: float = 2.0 * math.pi,
    max_range_m: float = 5.0,
) -> tuple[Pose2D, float]:
    """Grid search over (x, y, theta) near (guess_x, guess_y, guess_theta) for the
    pose whose predicted LiDAR scan best matches the observed `scan`. Returns
    (best_pose, match_error_m). Assumes the guess is already close (e.g. from
    dead-reckoning) -- this refines/verifies it, it does not search the whole room.

    theta is searched over `theta_steps` samples spanning guess_theta +/-
    theta_search_range_rad/2 (default: the full circle). Pass a narrow range with
    a high step count for a fast, fine-grained heading check once position is
    already known (e.g. during in-place docking).
    """
    steps = max(1, round(pos_search_radius_m / pos_step_m))
    best_pose = Pose2D(guess_x, guess_y, guess_theta)
    best_err = math.inf
    theta_start = guess_theta - theta_search_range_rad / 2.0
    for ix in range(-steps, steps + 1):
        for iy in range(-steps, steps + 1):
            x = guess_x + ix * pos_step_m
            y = guess_y + iy * pos_step_m
            for it in range(theta_steps):
                theta = wrap_angle(theta_start + it * theta_search_range_rad / max(1, theta_steps - 1))
                pose = Pose2D(x, y, theta)
                err = _match_error(scan, pose, segments, max_range_m)
                if err < best_err:
                    best_err = err
                    best_pose = pose
    return best_pose, best_err
