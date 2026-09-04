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


def integrate_real_dead_reckoning(
    pose: Pose2D, d_left_m: float, d_right_m: float, gyro_delta_rad: float,
    wheel_base_m: float = 0.0956, gyro_weight: float = 0.65,
) -> Pose2D:
    """Advance `pose` using MEASURED wheel distances (real encoder deltas, not
    commanded speeds) and MEASURED gyro heading change. Blends the rotation
    estimate from the two (gyro_weight=0.65 toward gyro) instead of trusting
    either alone -- gyro tends to be more reliable than a short differential
    wheelbase baseline for rotation, but pure gyro drifts on its own over
    time, so encoder still contributes. gyro_weight=0.65 matches a real
    physical robot's prior calibration (see Beagle/common/motion.py's
    integrate_wheel_distances) -- reused as a measured fact about this class
    of hardware, not re-derived, same as this session's other calibration
    constants (encoder counts/meter, gyro bias)."""
    distance_m = (d_left_m + d_right_m) / 2.0
    encoder_delta_theta = (d_right_m - d_left_m) / wheel_base_m
    delta_theta = gyro_weight * gyro_delta_rad + (1.0 - gyro_weight) * encoder_delta_theta
    mid_theta = pose.theta + delta_theta / 2.0
    x = pose.x + distance_m * math.cos(mid_theta)
    y = pose.y + distance_m * math.sin(mid_theta)
    theta = wrap_angle(pose.theta + delta_theta)
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

