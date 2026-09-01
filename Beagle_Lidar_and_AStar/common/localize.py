from __future__ import annotations

import math

from common.geometry import Pose2D, wrap_angle
from common.lidar import Segment, rectangle_segments, simulate_scan
from common.scan_align import best_rotation_offset, estimate_pose_offset


def obstacle_segments(obstacle_rects: list[tuple[float, float, float, float]]) -> list[Segment]:
    """Convert obstacle (xmin, ymin, xmax, ymax) rects (e.g. from
    data/obstacle_map.json) into wall-style Segments for simulate_scan()."""
    segments: list[Segment] = []
    for xmin, ymin, xmax, ymax in obstacle_rects:
        segments.extend(rectangle_segments(xmin, ymin, xmax, ymax))
    return segments


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


# On a scan taken close to a place segments actually models well (e.g. a
# checkpoint during navigate.py's short drive legs), match_err is normally
# well under 100mm -- see common/dock.py's SANITY_MATCH_ERR_M, same threshold
# used for the same reason there.
SANITY_MATCH_ERR_M = 0.10
COARSE_ROTATION_THRESHOLD_RAD = math.radians(15.0)


def checkpoint_correct(
    scan: list[float], guess_pose: Pose2D, segments: list[Segment], max_range_m: float = 5.0,
) -> tuple[Pose2D, float, bool]:
    """Correct a dead-reckoned `guess_pose` against the known STATIC map
    (`segments` = walls + obstacle_segments(), built once beforehand -- never
    updated here, so this has none of continuous SLAM's self-referential
    feedback-loop risk) using ONE live scan. No grid search: reuses
    common/scan_align.py's rotation/translation estimators (validated against
    real hardware in common/dock.py's find_pose()) by treating "what the model
    predicts guess_pose would see" as the reference scan.

    Assumes guess_pose is already reasonably close (small heading/position
    error, e.g. from dead reckoning over a short interval since the last
    checkpoint) -- NOT for localizing from a totally unknown starting
    position (that needs localize() above, or a much wider search).

    Returns (corrected_pose, match_err_m, trustworthy). trustworthy is False
    if the rotation-only match_err came back too high (predicted scan doesn't
    look like what was actually seen -- guess_pose is probably too far off
    for this fast check to fix); caller should keep trusting dead reckoning
    alone for this checkpoint rather than applying a bad correction.
    """
    predicted = simulate_scan(guess_pose, segments, num_rays=len(scan), max_range_m=max_range_m)
    rotation_rad, rot_match_err = best_rotation_offset(scan, predicted)
    if rot_match_err > SANITY_MATCH_ERR_M:
        return guess_pose, rot_match_err, False

    if abs(rotation_rad) > COARSE_ROTATION_THRESHOLD_RAD:
        # Heading is off by more than estimate_pose_offset()'s linearization
        # can trust -- apply just the robust rotation-only correction this
        # round (same convention as best_rotation_offset: guess_pose.theta -
        # rotation_rad gives the true heading), leave position for next
        # checkpoint once heading is back in the small-angle range.
        corrected = Pose2D(guess_pose.x, guess_pose.y, wrap_angle(guess_pose.theta - rotation_rad))
        return corrected, rot_match_err, True

    dtheta, dx, dy, residual = estimate_pose_offset(scan, predicted)
    corrected = Pose2D(guess_pose.x + dx, guess_pose.y + dy, wrap_angle(guess_pose.theta - dtheta))
    return corrected, residual, True
