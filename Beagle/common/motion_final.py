from __future__ import annotations

import math
import time

from .geometry import Pose2D, clamp, euclidean, wrap_angle
from .robot import SafeBeagle


def calibrate_gyro_bias(robot: SafeBeagle, seconds: float = 0.8, sample_s: float = 0.02) -> float:
    """정지 상태에서 자이로 z축 평균을 재서 영점(bias)을 구합니다."""
    values: list[float] = []
    robot.stop()
    deadline = time.monotonic() + max(0.1, seconds)
    while time.monotonic() < deadline:
        values.append(float(robot.gyroscope_z()))
        time.sleep(sample_s)
    return sum(values) / max(1, len(values))


def align_to_heading_command(
    pose: Pose2D,
    target_heading_rad: float,
    *,
    turn_speed: float = 10.0,
    tolerance_deg: float = 4.0,
    slow_zone_deg: float = 20.0,
    min_turn_speed: float = 7.0,
) -> tuple[float, float, bool]:
    """제자리에서 target_heading_rad를 향해 회전합니다.

    pure_pursuit는 로봇이 이미 경로 근처/비슷한 방향을 보고 있다고 가정하고
    짧은 lookahead 지점을 향한 원호를 계속 계산합니다. leg가 막 시작되어 로봇이
    완전히 다른 방향을 보고 있으면, 전진하면서 그 원호를 쫓다가 넓게 휘어 도는
    루프 모양이 나옵니다. 경로 추종을 시작하기 전에 이 함수로 먼저 제자리
    회전해서 대략 맞춰두면 그 문제를 피할 수 있습니다.

    Returns (left_pct, right_pct, aligned).
    """

    heading_error = wrap_angle(target_heading_rad - pose.theta)
    error_deg = abs(math.degrees(heading_error))
    if error_deg <= tolerance_deg:
        return 0.0, 0.0, True
    # Confirmed 2026-08-27: real hardware's slow (~0.3-0.4s) frame loop means a single
    # full-speed turn command can overshoot well past a small remaining error before the next
    # check, causing the direction to flip back and forth (seen oscillating CW/CCW several
    # times during one ALIGNING leg) instead of settling. Each extra oscillation adds more
    # cumulative rotation, which appears to be how residual rotation-tracking error compounds
    # into a badly wrong heading by the time TRACKING starts. Scaling speed down as the error
    # shrinks reduces overshoot without needing a faster control loop.
    speed = turn_speed
    if error_deg < slow_zone_deg:
        speed = max(min_turn_speed, turn_speed * (error_deg / slow_zone_deg))
    if heading_error > 0:
        return -speed, speed, False
    return speed, -speed, False


def line_follow_command(
    pose: Pose2D,
    line_start: tuple[float, float],
    line_target: tuple[float, float],
    *,
    speed: float = 12.0,
    max_turn: float = 14.0,
    cross_track_gain: float = 2.5,
    max_correction_deg: float = 60.0,
    distance_tolerance_m: float = 0.08,
) -> tuple[float, float, float, bool]:
    """line_start -> line_target 직선을 따라가는 line-following 제어(Stanley 조향 방식).

    A* + pure pursuit(경로 위 미리보기 점을 계속 쫓음)와 달리, 이 방법은 고정된
    직선을 정해두고 그 선에서 벗어난 수직 거리(cross-track error)를 직접 줄이는
    방식으로 조향합니다. 선에 가까우면 line_heading을 그대로 따르고, 멀어질수록
    선 쪽으로 더 강하게(단, max_correction_deg까지만) 꺾어 돌아옵니다.

    Returns (left_pct, right_pct, distance_to_target_m, reached).
    """

    distance = euclidean((pose.x, pose.y), line_target)
    if distance <= distance_tolerance_m:
        return 0.0, 0.0, distance, True

    dx = line_target[0] - line_start[0]
    dy = line_target[1] - line_start[1]
    line_length = math.hypot(dx, dy)
    if line_length < 1e-6:
        return 0.0, 0.0, distance, True
    line_heading = math.atan2(dy, dx)

    to_robot_x = pose.x - line_start[0]
    to_robot_y = pose.y - line_start[1]
    cross_track = -math.sin(line_heading) * to_robot_x + math.cos(line_heading) * to_robot_y

    correction_angle = clamp(
        math.atan(cross_track_gain * cross_track),
        -math.radians(max_correction_deg),
        math.radians(max_correction_deg),
    )
    desired_heading = line_heading - correction_angle
    heading_error = wrap_angle(desired_heading - pose.theta)

    correction = clamp(math.degrees(heading_error) * 0.7, -max_turn, max_turn)
    abs_error = abs(heading_error)
    if abs_error > 1.0:
        forward_scale = 0.15
    elif abs_error > 0.55:
        forward_scale = 0.4
    else:
        forward_scale = 1.0
    forward = speed * forward_scale
    return forward - correction, forward + correction, distance, False
