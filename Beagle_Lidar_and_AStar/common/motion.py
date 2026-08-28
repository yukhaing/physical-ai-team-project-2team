from __future__ import annotations

import math

from common.geometry import Pose2D, wrap_angle


def integrate_dead_reckoning(
    pose: Pose2D, left_mps: float, right_mps: float, wheel_base_m: float, dt: float
) -> Pose2D:
    """Advance `pose` by dt from differential-drive wheel speeds (encoder+gyro style)."""
    v = (left_mps + right_mps) / 2.0
    omega = (right_mps - left_mps) / wheel_base_m
    mid_theta = pose.theta + omega * dt / 2.0
    x = pose.x + v * math.cos(mid_theta) * dt
    y = pose.y + v * math.sin(mid_theta) * dt
    theta = wrap_angle(pose.theta + omega * dt)
    return Pose2D(x, y, theta)


def align_to_heading_command(
    pose: Pose2D, target_theta: float, turn_mps: float, tol_rad: float = math.radians(3.0)
) -> tuple[float, float, bool]:
    """In-place turn command (left_mps, right_mps, aligned) to rotate toward target_theta."""
    err = wrap_angle(target_theta - pose.theta)
    if abs(err) <= tol_rad:
        return 0.0, 0.0, True
    direction = 1.0 if err > 0 else -1.0
    return -direction * turn_mps, direction * turn_mps, False
