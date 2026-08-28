from __future__ import annotations

import math
import time

from .geometry import Pose2D, clamp, euclidean, wrap_angle
from .localization import localize, pose_match_error, resolve_180_ambiguity, scan_multiple
from .robot import ENCODER_M_PER_COUNT_LEFT, ENCODER_M_PER_COUNT_RIGHT, SafeBeagle


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


def dock_to_pose(
    robot: SafeBeagle,
    segments: list[tuple[float, float, float, float]],
    room_width_m: float,
    room_height_m: float,
    target_x: float,
    target_y: float,
    target_theta_rad: float,
    *,
    disambiguation_segments: list[tuple[float, float, float, float]] | None = None,
    pos_tol_m: float = 0.01,
    theta_tol_deg: float = 3.0,
    max_iters: int = 12,
    turn_speed: float = 10.0,
    drive_speed: float = 10.0,
    search_radius: float = 0.15,
    verbose: bool = True,
) -> tuple[Pose2D, bool]:
    """A*/pure pursuit로 미션을 시작하기 전에, 로봇을 (target_x, target_y,
    target_theta_rad)에 최대한 정확히 세워 놓는 도킹 루틴 (예: 매번 receiving zone에
    heading=0(3시 방향)으로 정확히 세우기).

    매 iteration: LiDAR 풀스캔으로 실제 pose 측정 -> 오차가 허용치 이내면 정지 -> 아니면
    잔여 위치 오차 방향으로 회전 후 그만큼 직진 -> target_theta_rad로 재정렬 -> 반복.
    회전/직진 사이의 소량 이동은 자이로/엔코더로만 추적하고(빠름), 각 iteration 시작
    시점의 "진짜" 오차만 LiDAR 풀스캔(localize)으로 다시 잰다 -- 그래야 누적 드리프트가
    매 iteration마다 스스로 정정된다.

    주의: localize()의 격자 탐색 해상도는 2cm/2deg라서, pos_tol_m을 그보다 좁게 요청해도
    아래에서 pose_match_error()로 격자에 얽매이지 않는 국소 재탐색을 한 번 더 해서 그
    아래까지 정밀도를 끌어올린다. 그래도 LiDAR 자체 노이즈의 실측 하한(~5-10mm)보다
    더 좁은 tolerance는 물리적으로 못 만족할 수 있다.

    segments: 좌표 측정/coarse localize에 쓰는 벽 전용 segment (OMX 등 노이즈 있는
    landmark를 넣으면 위치 정밀도가 오히려 나빠질 수 있음 -- 2026-08-27 확인).
    disambiguation_segments: 180도 대칭 후보 중 하나를 고를 때만 쓰는 segment. 사각형
    벽만으로는 room 중심 기준 180도 회전에 대해 완전히 대칭이라 (2026-08-28 확인: 두
    후보 매칭오차가 부동소수점까지 정확히 같아서, 그때그때 운으로 틀린 쪽이 뽑힘) 반드시
    비대칭 landmark(OMX 등)가 포함된 segment를 넘겨야 함. 생략 시 segments를 그대로 씀
    (즉 이 disambiguation 자체가 무력화됨 -- landmark가 있으면 항상 넘길 것).

    Returns (최종 측정된 pose, tolerance 이내로 수렴했는지 여부).
    """
    disambig_segments = disambiguation_segments if disambiguation_segments is not None else segments
    gyro_bias = calibrate_gyro_bias(robot)

    def _refine(scan_mm: list[float], pose: Pose2D) -> Pose2D:
        best_pose, best_err = pose, pose_match_error(scan_mm, segments, pose)
        for dx in (-0.015, -0.0075, 0.0, 0.0075, 0.015):
            for dy in (-0.015, -0.0075, 0.0, 0.0075, 0.015):
                for dtheta_deg in (-1.5, -0.75, 0.0, 0.75, 1.5):
                    cand = Pose2D(pose.x + dx, pose.y + dy, pose.theta + math.radians(dtheta_deg))
                    err = pose_match_error(scan_mm, segments, cand)
                    if err < best_err:
                        best_pose, best_err = cand, err
        return best_pose

    def _measure() -> Pose2D:
        scan = scan_multiple(robot)
        pose, _ = localize(scan, segments, target_x, target_y, search_radius, verbose=False)
        pose, _ = resolve_180_ambiguity(scan, disambig_segments, pose, room_width_m, room_height_m, verbose=False)
        return _refine(scan, pose)

    def _turn_by_gyro(delta_deg: float, speed: float) -> float:
        """delta_deg(부호 있음)만큼 자이로만으로 제자리 회전. 실제로 돈 각도(deg)를 반환."""
        if abs(delta_deg) < 1.0:
            return 0.0
        direction = 1.0 if delta_deg > 0 else -1.0
        robot.wheels(-direction * speed, direction * speed)
        turned_deg = 0.0
        last_t = time.monotonic()
        deadline = last_t + 4.0  # safety cutoff
        while abs(turned_deg) < abs(delta_deg) and time.monotonic() < deadline:
            time.sleep(0.02)
            now = time.monotonic()
            dt = now - last_t
            last_t = now
            gyro_dps = robot.gyroscope_z() - gyro_bias
            turned_deg += gyro_dps * dt
        robot.stop()
        return turned_deg

    def _drive_by_encoder(distance_m: float, speed: float) -> None:
        if abs(distance_m) < 0.002:
            return
        direction = 1.0 if distance_m > 0 else -1.0
        left0 = robot.left_encoder()
        right0 = robot.right_encoder()
        robot.wheels(direction * speed, direction * speed)
        traveled = 0.0
        deadline = time.monotonic() + 5.0  # safety cutoff
        while traveled < abs(distance_m) and time.monotonic() < deadline:
            time.sleep(0.02)
            dl = (robot.left_encoder() - left0) * ENCODER_M_PER_COUNT_LEFT
            dr = (robot.right_encoder() - right0) * ENCODER_M_PER_COUNT_RIGHT
            traveled = abs((dl + dr) / 2.0)
        robot.stop()

    pose = Pose2D(target_x, target_y, target_theta_rad)
    for i in range(max_iters):
        pose = _measure()
        pos_err_m = euclidean((pose.x, pose.y), (target_x, target_y))
        theta_err_deg = math.degrees(wrap_angle(target_theta_rad - pose.theta))
        if verbose:
            print(f"[dock] iter={i} pose=({pose.x:.3f},{pose.y:.3f}) heading={math.degrees(pose.theta):.1f}deg "
                  f"pos_err={pos_err_m * 100:.1f}cm theta_err={theta_err_deg:.1f}deg")
        if pos_err_m <= pos_tol_m and abs(theta_err_deg) <= theta_tol_deg:
            robot.stop()
            return pose, True

        current_theta = pose.theta
        if pos_err_m > pos_tol_m:
            bearing = math.atan2(target_y - pose.y, target_x - pose.x)
            turn1_deg = math.degrees(wrap_angle(bearing - current_theta))
            current_theta += math.radians(_turn_by_gyro(turn1_deg, turn_speed))
            _drive_by_encoder(pos_err_m, drive_speed)

        turn2_deg = math.degrees(wrap_angle(target_theta_rad - current_theta))
        _turn_by_gyro(turn2_deg, turn_speed)

    robot.stop()
    return pose, False
