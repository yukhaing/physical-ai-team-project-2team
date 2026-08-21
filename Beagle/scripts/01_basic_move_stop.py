from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""1단계: 최소 이동/정지 테스트.
저속으로 짧게 직진하다가 시간이 다 되거나 전방이 가까워지면 반드시 정지합니다.
회피/회전 로직은 없습니다 -- 오직 "움직이고 멈춘다"만 확인하는 스크립트입니다.
00_connection_check.py로 연결을 먼저 확인한 뒤에 실행하세요.
"""

import argparse
import time

from common.lidar import cardinal_distances
from common.robot import SafeBeagle


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry-run", action="store_true", help="실물 없이 이 스크립트 자체만 점검")
    parser.add_argument("--scene", default="obstacles", help="--dry-run일 때만 사용되는 가상 장면")
    parser.add_argument("--speed", type=float, default=20.0, help="바퀴 속도(%%), 10~15 권장")
    parser.add_argument("--duration", type=float, default=3.0, help="최대 주행 시간(초)")
    parser.add_argument("--stop-mm", type=float, default=300.0, help="전방이 이 거리보다 가까워지면 즉시 정지")
    args = parser.parse_args()

    speed = max(0.0, min(15.0, args.speed))  # SafeBeagle의 25% 상한과 별개로 이 스크립트 자체에서 15%로 재차 제한

    with SafeBeagle(dry_run=args.dry_run, scene=args.scene, max_speed=25) as robot:
        robot.start_lidar()
        robot.wait_until_lidar_ready()

        print(f"{speed:.0f}% 속도로 최대 {args.duration:.1f}초간 직진합니다. "
              f"전방 {args.stop_mm:.0f}mm 이내면 즉시 정지. Ctrl+C로 언제든 정지 가능.")
        for remaining in (3, 2, 1):
            print(f"  {remaining}...")
            time.sleep(1.0)

        try:
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                front = cardinal_distances(robot.lidar())["front"]
                if front < args.stop_mm:
                    print(f"전방 {front:.0f}mm -> 정지")
                    break
                robot.wheels(speed, speed)
                time.sleep(0.05)
            else:
                print("시간 종료 -> 정지")
        finally:
            robot.stop()
            print("정지 완료")


if __name__ == "__main__":
    main()
