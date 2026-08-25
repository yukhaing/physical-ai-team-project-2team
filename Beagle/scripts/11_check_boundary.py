from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""LiDAR가 실제 방 경계(벽)를 얼마나 정확하게 감지하는지 확인합니다.

로봇을 start zone 안 아무 위치/방향으로나 놓고 실행하면 됩니다 -- 정확한 위치나
heading을 몰라도 됩니다. common/localization.py의 localize_robot()이 LiDAR를 여러 번
읽고(제자리, 회전 없음) start zone 주변 사각 영역(--cx/--cy 중심, --search-radius 반경)
x 전체 360도 heading을 격자 탐색해서 실측 방 치수(0.90m x 0.70m) 벽 4개와 가장 잘 맞는
위치/heading을 찾습니다.

그 다음 그 추정 위치/heading 기준으로 각도별 예상 거리 vs 실제 거리를 비교합니다.

로봇은 전혀 움직이지 않습니다 (스캔만 함) -- 안전합니다.

11_check_boundary.py는 진단 전용입니다. 04_mission_astar_slam.py는 실물 모드에서
시작할 때 같은 localize_robot()을 자동으로 써서 실제 시작 위치를 파악하고, start
zone과 떨어져 있으면 먼저 그쪽으로 이동합니다.

사용법:
  python scripts\\11_check_boundary.py --dry-run    # 시뮬레이션으로 스크립트 자체 점검
  python scripts\\11_check_boundary.py              # 실물 로봇, start zone 근처라고만 가정
  python scripts\\11_check_boundary.py --cx 0.795 --cy 0.105 --search-radius 0.2   # 탐색 범위 조정
"""

import argparse
import math
import time

import matplotlib.pyplot as plt

from common.geometry import Pose2D, transform_points_mm
from common.localization import localize, scan_multiple
from common.robot import SafeBeagle, ray_segment_distance, rectangle_segments

ROOM_WIDTH_M = 0.90
ROOM_HEIGHT_M = 0.70
ROOM_BOUNDARY = rectangle_segments(0.0, 0.0, ROOM_WIDTH_M, ROOM_HEIGHT_M)
TOLERANCE_MM = 80.0  # 이 이상 차이나면 "불일치"로 표시


def expected_distance_mm(pose: Pose2D, degree: int, segments: list) -> float:
    theta = pose.theta + math.radians(degree)
    dx, dy = math.cos(theta), math.sin(theta)
    distance = min(
        (ray_segment_distance(pose.x, pose.y, dx, dy, seg) for seg in segments),
        default=math.inf,
    )
    return distance * 1000.0 if math.isfinite(distance) else math.inf


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--cx", type=float, default=0.12, help="탐색 영역 중심 x (m) -- start zone 대략 위치")
    parser.add_argument("--cy", type=float, default=0.12, help="탐색 영역 중심 y (m)")
    parser.add_argument("--search-radius", type=float, default=0.25, help="중심에서 탐색할 반경 (m)")
    args = parser.parse_args()

    with SafeBeagle(dry_run=args.dry_run, scene="open", max_speed=25) as robot:
        if args.dry_run:
            # dry-run 검증용: 로봇을 탐색 중심에서 살짝 벗어난 위치/임의 heading에 둬서
            # 정말로 위치/heading을 "찾아내는지" 확인합니다.
            robot.robot.segments = ROOM_BOUNDARY
            robot.robot.pose = Pose2D(args.cx + 0.06, args.cy - 0.04, math.radians(137.0))
        robot.start_lidar()
        robot.wait_until_lidar_ready()

        print("제자리에서 LiDAR 읽는 중...")
        scan = scan_multiple(robot)
        print("위치/heading 탐색 중...")
        t0 = time.monotonic()
        pose, match_err_m = localize(scan, ROOM_BOUNDARY, args.cx, args.cy, args.search_radius)
    print(f"탐색 완료 ({time.monotonic() - t0:.1f}초)")

    heading_deg = math.degrees(pose.theta) % 360.0
    print(f"\n추정 위치: ({pose.x:.3f}, {pose.y:.3f})  heading={heading_deg:.0f}도")
    print(f"매칭 중앙값 오차: {match_err_m * 1000:.1f}mm")
    if match_err_m > 0.05:
        print(
            "[WARN] 매칭 오차가 큽니다 -- 탐색 범위(--search-radius) 밖에 있거나, "
            "벽이 없는/뚫린 구간이 많을 수 있습니다."
        )

    expected = [expected_distance_mm(pose, deg, ROOM_BOUNDARY) for deg in range(360)]

    errors: list[float] = []
    bad_angles: list[tuple[int, float, float, float]] = []
    for deg in range(360):
        exp = expected[deg]
        act = scan[deg]
        if not (math.isfinite(exp) and math.isfinite(act)):
            continue
        err = abs(act - exp)
        errors.append(err)
        if err > TOLERANCE_MM:
            bad_angles.append((deg, exp, act, err))

    print(f"\n비교 가능한 각도 수: {len(errors)}/360")
    if errors:
        print(f"각도별 평균 오차: {sum(errors) / len(errors):.1f}mm  최대 오차: {max(errors):.1f}mm")
    mismatch_pct = len(bad_angles) / len(errors) * 100 if errors else 100.0
    print(f"허용오차({TOLERANCE_MM:.0f}mm) 초과 각도: {len(bad_angles)}개 ({mismatch_pct:.1f}%)")
    if bad_angles:
        print("불일치 각도 (최대 20개, 로봇 기준 상대 각도):")
        for deg, exp, act, err in bad_angles[:20]:
            print(f"  {deg:3d}도: 예상={exp:6.0f}mm 실제={act:6.0f}mm 차이={err:6.0f}mm")

    if mismatch_pct < 10:
        print("\n판정: LiDAR가 방 경계를 잘 감지하고 있습니다.")
    elif mismatch_pct < 30:
        print("\n판정: 일부 각도 불일치 -- 벽 틈/위치를 확인하세요.")
    else:
        print("\n판정: 경계 감지 불량 -- 특정 각도 구간에 벽이 없거나 뚫려 있을 가능성이 높습니다.")

    # polar_to_xy()는 배열 길이로 각도 간격(360/len)을 계산하므로, 걸러낸 부분집합을 그대로
    # 넘기면 각도가 전부 잘못 계산됩니다 -- 그래서 원래 deg 인덱스를 유지한 채 직접 좌표를
    # 계산합니다. scan[deg]/expected[deg] 색상과 world 점 개수를 같은 필터로 맞춰야
    # 어긋나지 않습니다 (따로 만들면 개수 불일치로 scatter가 터짐).
    valid_degs = [
        deg for deg in range(360)
        if math.isfinite(scan[deg]) and 50.0 <= scan[deg] <= 5000.0
    ]
    points_mm = [
        (scan[deg] * math.cos(math.radians(deg)), scan[deg] * math.sin(math.radians(deg)))
        for deg in valid_degs
    ]
    world = transform_points_mm(points_mm, pose)

    fig, ax = plt.subplots(figsize=(7, 7))
    for sx, sy, ex, ey in ROOM_BOUNDARY:
        ax.plot([sx, ex], [sy, ey], color="#33415F", linewidth=3)
    if world:
        wx, wy = zip(*world)
        colors = [
            "#D9534F" if (math.isfinite(expected[deg]) and abs(scan[deg] - expected[deg]) > TOLERANCE_MM)
            else "#3B9C4C"
            for deg in valid_degs
        ]
        ax.scatter(wx, wy, s=10, c=colors)
    ax.plot(pose.x, pose.y, "^", color="#20242C", markersize=14)
    ax.arrow(pose.x, pose.y, 0.1 * math.cos(pose.theta), 0.1 * math.sin(pose.theta),
              head_width=0.03, color="#20242C")
    ax.set_xlim(-0.3, ROOM_WIDTH_M + 0.3)
    ax.set_ylim(-0.3, ROOM_HEIGHT_M + 0.3)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_title(
        f"Boundary Check | pos=({pose.x:.2f},{pose.y:.2f}) heading={heading_deg:.0f}deg | "
        f"mismatch={mismatch_pct:.1f}% | green=OK red=off (>{TOLERANCE_MM:.0f}mm)"
    )
    plt.show()


if __name__ == "__main__":
    main()
