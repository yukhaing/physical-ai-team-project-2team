from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""LiDAR raw 배열이 실제로 어떤 방향/오프셋으로 인덱싱되는지 진단합니다.

11_check_boundary.py로 확인한 결과, 방 전체를 다 뒤져도(search_radius로 방 전체를
덮어도) 실제 로봇 위치(사진으로 확인된 오른쪽)가 아니라 계속 왼쪽에서 그럴듯한(하지만
완벽하지 않은) 매칭을 찾습니다 -- 이는 raw 스캔이 거울 대칭으로 뒤집혀 있어서
회전만으로는(로컬 좌표 재정렬만으로는) 절대 진짜 자세를 찾을 수 없다는 뜻입니다.

이 스크립트는 로봇을 한 번 스캔한 뒤, 있을 법한 배열 변환 후보 여러 개(원본, 반전,
180도 회전 등)에 대해 각각 localize()를 돌려서 실제 방 벽과 가장 잘 맞는(오차가 가장
작은) 변환이 무엇인지 찾습니다. 가장 오차가 작은 후보가 실제 정답 변환입니다.

로봇은 움직이지 않습니다 -- 안전합니다.

사용법: python scripts\\14_diagnose_lidar_orientation.py
        (--cx/--cy/--search-radius로 탐색 범위 조정 가능 -- 기본은 방 전체)
"""

import argparse
import math
import time

from common.geometry import Pose2D
from common.localization import localize, scan_multiple
from common.robot import SafeBeagle, rectangle_segments

ROOM_WIDTH_M = 0.90
ROOM_HEIGHT_M = 0.70
ROOM_BOUNDARY = rectangle_segments(0.0, 0.0, ROOM_WIDTH_M, ROOM_HEIGHT_M)


def build_candidates(scan: list[float]) -> dict[str, list[float]]:
    n = len(scan)
    half = n // 2
    reverse_keep0 = [scan[0]] + scan[1:][::-1]
    reverse_all = scan[::-1]
    shift180 = scan[half:] + scan[:half]
    shift180_reverse_keep0 = [shift180[0]] + shift180[1:][::-1]
    shift180_reverse_all = shift180[::-1]
    return {
        "identity (현재 원본, 변환 없음)": scan,
        "reverse_keep0 (0도 고정, 나머지 반전 -- 지금 적용된 수정)": reverse_keep0,
        "reverse_all (전체 반전)": reverse_all,
        "shift180 (앞/뒤 뒤바꿈, 반전 없음)": shift180,
        "shift180_reverse_keep0 (앞/뒤 뒤바꿈 + 0도 고정 반전)": shift180_reverse_keep0,
        "shift180_reverse_all (앞/뒤 뒤바꿈 + 전체 반전)": shift180_reverse_all,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cx", type=float, default=0.45, help="탐색 영역 중심 x (m) -- 기본: 방 중앙")
    parser.add_argument("--cy", type=float, default=0.35, help="탐색 영역 중심 y (m)")
    parser.add_argument("--search-radius", type=float, default=0.5, help="탐색 반경 (m) -- 기본: 방 전체")
    args = parser.parse_args()

    with SafeBeagle(dry_run=False, scene="open", max_speed=25) as robot:
        robot.start_lidar()
        robot.wait_until_lidar_ready()
        print("제자리에서 LiDAR 읽는 중...")
        scan = scan_multiple(robot)

    candidates = build_candidates(scan)
    print(f"\n{len(candidates)}가지 변환 후보를 각각 탐색합니다 (후보당 몇 초씩 걸립니다)...\n")

    results = []
    for name, candidate_scan in candidates.items():
        t0 = time.monotonic()
        pose, err_m = localize(candidate_scan, ROOM_BOUNDARY, args.cx, args.cy, args.search_radius, verbose=False)
        elapsed = time.monotonic() - t0
        heading_deg = math.degrees(pose.theta) % 360.0
        results.append((name, pose, err_m, heading_deg))
        print(f"[{elapsed:4.1f}s] {name}")
        print(f"         위치=({pose.x:.3f},{pose.y:.3f}) heading={heading_deg:.0f}도  매칭오차={err_m * 1000:.1f}mm")

    results.sort(key=lambda r: r[2])
    best_name, best_pose, best_err, best_heading = results[0]
    print("\n" + "=" * 60)
    print(f"가장 잘 맞는 변환: {best_name}")
    print(f"  위치=({best_pose.x:.3f},{best_pose.y:.3f}) heading={best_heading:.0f}도  매칭오차={best_err * 1000:.1f}mm")
    print("=" * 60)
    print("\n이 위치가 로봇의 실제 물리적 위치와 (대략이라도) 맞는지 확인하고 알려주세요.")
    print("맞다면 이 변환을 common/robot.py의 SafeBeagle.lidar()에 영구 반영하겠습니다.")


if __name__ == "__main__":
    main()
