from __future__ import annotations

import math

import numpy as np

from .geometry import Pose2D, polar_to_xy
from .lidar import sanitize_scan
from .robot import SafeBeagle

SAMPLES = 10  # 노이즈 대비 여러 번 읽어 각도별 중앙값 사용
THETA_STEP_DEG = 2.0
XY_STEP_M = 0.02
SAMPLE_EVERY = 4  # 탐색 속도를 위해 스캔 점을 4개마다 하나씩만 사용


def median(values: list[float]) -> float:
    s = sorted(values)
    n = len(s)
    mid = n // 2
    return s[mid] if n % 2 else (s[mid - 1] + s[mid]) / 2.0


def scan_multiple(robot: SafeBeagle, samples: int = SAMPLES) -> list[float]:
    """제자리에서 LiDAR를 여러 번 읽어 각도별 중앙값을 반환합니다 (회전 없음, 이동 없음)."""
    readings = [sanitize_scan(robot.lidar()) for _ in range(samples)]
    return [median(vals) for vals in zip(*readings)]


def point_to_segments_min_dist(wx: np.ndarray, wy: np.ndarray, segs: np.ndarray) -> np.ndarray:
    """wx, wy: 임의 shape의 world 좌표(m) 배열. segs: (S,4) x1,y1,x2,y2.
    반환: wx/wy와 같은 shape, 각 점에서 segs 중 가장 가까운 선분까지의 거리(m)."""
    x1, y1, x2, y2 = segs[:, 0], segs[:, 1], segs[:, 2], segs[:, 3]
    dx, dy = x2 - x1, y2 - y1
    seg_len_sq = dx * dx + dy * dy
    wx_ = wx[..., None]
    wy_ = wy[..., None]
    t = ((wx_ - x1) * dx + (wy_ - y1) * dy) / seg_len_sq
    t = np.clip(t, 0.0, 1.0)
    proj_x = x1 + t * dx
    proj_y = y1 + t * dy
    dist = np.hypot(wx_ - proj_x, wy_ - proj_y)
    return dist.min(axis=-1)


def pose_match_error(
    scan_mm: list[float],
    segments: list[tuple[float, float, float, float]],
    pose: Pose2D,
) -> float:
    """scan_mm을 pose에 놓았을 때 segments와 얼마나 잘 맞는지(중앙값 거리, m)를 계산합니다.
    localize()의 그리드 탐색과 달리 grid step에 얽매이지 않고 pose 하나를 정확히 채점합니다
    -- 그리드 해상도 때문에 놓칠 수 있는, 서로 가까운 두 후보(예: 180도 대칭 쌍)를 정밀하게
    비교할 때 씁니다."""
    local_pts = polar_to_xy(scan_mm)[::SAMPLE_EVERY]
    if not local_pts:
        return math.inf
    local = np.array(local_pts, dtype=np.float64) / 1000.0
    segs = np.array(segments, dtype=np.float64)
    c, s = math.cos(pose.theta), math.sin(pose.theta)
    wx = pose.x + c * local[:, 0] - s * local[:, 1]
    wy = pose.y + s * local[:, 0] + c * local[:, 1]
    return float(np.median(point_to_segments_min_dist(wx, wy, segs)))


def resolve_180_ambiguity(
    scan_mm: list[float],
    segments: list[tuple[float, float, float, float]],
    pose: Pose2D,
    room_width: float,
    room_height: float,
    *,
    verbose: bool = True,
) -> tuple[Pose2D, float]:
    """직사각형 방 벽만 봐서는 pose와 방 중심 기준 180도 회전한 pose가 매칭 점수가 완전히
    같아서(회전 대칭) localize()가 둘 중 아무거나 고를 수 있음. segments에 비대칭 장애물이
    있으면 보통 구분되지만, 격자 탐색 해상도 탓에 grid가 그 차이를 놓칠 수 있음 -- 그래서
    pose와 그 180도 대응쌍을 여기서 직접(그리드에 얽매이지 않고) 채점해서 더 잘 맞는 쪽을
    최종적으로 고릅니다. 반환: (최종 선택된 pose, 그 매칭오차 m)."""
    twin = Pose2D(room_width - pose.x, room_height - pose.y, pose.theta + math.pi)
    err_pose = pose_match_error(scan_mm, segments, pose)
    err_twin = pose_match_error(scan_mm, segments, twin)
    if err_twin < err_pose:
        if verbose:
            print(
                f"[180도 대칭 보정] 원래 후보 매칭오차 {err_pose * 1000:.1f}mm보다 반대쪽이 "
                f"{err_twin * 1000:.1f}mm로 더 잘 맞아 반대쪽으로 바꿉니다."
            )
        return twin, err_twin
    return pose, err_pose


def localize(
    scan_mm: list[float],
    segments: list[tuple[float, float, float, float]],
    cx: float,
    cy: float,
    search_radius: float,
    *,
    verbose: bool = True,
) -> tuple[Pose2D, float]:
    """(cx, cy) 주변 사각 영역 x 전체 360도를 격자 탐색해서, LiDAR 스캔이 방 벽과
    가장 잘 겹치는 (x, y, heading)을 찾습니다. 반환: (추정 Pose2D, 매칭 오차 m)."""
    local_pts = polar_to_xy(scan_mm)[::SAMPLE_EVERY]
    if not local_pts:
        raise ValueError("유효한 LiDAR 포인트가 없습니다.")
    local = np.array(local_pts, dtype=np.float64) / 1000.0  # (N,2) meters

    segs = np.array(segments, dtype=np.float64)  # (S,4)
    xs = np.arange(cx - search_radius, cx + search_radius + 1e-9, XY_STEP_M)
    ys = np.arange(cy - search_radius, cy + search_radius + 1e-9, XY_STEP_M)
    XX, YY = np.meshgrid(xs, ys, indexing="ij")  # (X,Y)
    thetas_deg = np.arange(0.0, 360.0, THETA_STEP_DEG)

    best_score = np.inf
    best_x = best_y = best_theta = 0.0
    for theta_deg in thetas_deg:
        theta = math.radians(theta_deg)
        c, s = math.cos(theta), math.sin(theta)
        rx = c * local[:, 0] - s * local[:, 1]  # (N,)
        ry = s * local[:, 0] + c * local[:, 1]
        wx = XX[:, :, None] + rx[None, None, :]  # (X,Y,N)
        wy = YY[:, :, None] + ry[None, None, :]
        d = point_to_segments_min_dist(wx, wy, segs)  # (X,Y,N)
        # 평균 대신 중앙값 사용: 방 밖 물체를 찍은 점(어느 pose를 넣어도 벽과 안 맞아
        # 거리가 항상 큼)이 전체 중 소수만 되면, 중앙값은 그 점들에 거의 영향받지
        # 않지만 평균은 크게 끌려가서 실제로 잘 맞는 pose를 밀어낼 수 있음.
        score = np.median(d, axis=2)  # (X,Y)
        idx = np.unravel_index(np.argmin(score), score.shape)
        if score[idx] < best_score:
            best_score = float(score[idx])
            best_x, best_y = float(XX[idx]), float(YY[idx])
            best_theta = theta

    edge = XY_STEP_M * 1.5
    on_edge = (
        best_x <= xs[0] + edge or best_x >= xs[-1] - edge
        or best_y <= ys[0] + edge or best_y >= ys[-1] - edge
    )
    if on_edge and verbose:
        print(
            "[WARN] 추정 위치가 탐색 범위 경계에 걸려 있습니다 -- 실제 위치가 "
            "search_radius 밖에 있을 수 있습니다. search_radius를 늘려서 다시 시도하세요."
        )

    return Pose2D(best_x, best_y, best_theta), best_score


def localize_robot(
    robot: SafeBeagle,
    segments: list[tuple[float, float, float, float]],
    cx: float,
    cy: float,
    search_radius: float = 0.25,
    *,
    verbose: bool = True,
) -> tuple[Pose2D, float]:
    """로봇을 제자리에서 여러 번 스캔하고 위치/heading을 추정합니다 (이동 없음)."""
    scan = scan_multiple(robot)
    return localize(scan, segments, cx, cy, search_radius, verbose=verbose)
