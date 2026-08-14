from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""실물 연결 확인: 바퀴를 전혀 움직이지 않고 연결/배터리/LiDAR 값만 확인합니다.
실물 테스트를 시작하기 전 항상 이 스크립트를 가장 먼저 실행하세요.
이 파일에는 robot.wheels() 호출이 어디에도 없습니다 -- 로봇은 절대 움직이지 않습니다.
"""

import argparse
import time

from common.lidar import cardinal_distances, valid_fraction
from common.robot import SafeBeagle


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry-run", action="store_true", help="실물 없이 이 스크립트 자체만 점검")
    parser.add_argument("--duration", type=float, default=5.0)
    args = parser.parse_args()

    with SafeBeagle(dry_run=args.dry_run, max_speed=25) as robot:
        print("연결 성공. battery:", robot.battery_state(), "signal:", robot.signal_strength())
        robot.start_lidar()
        robot.wait_until_lidar_ready()
        print("LiDAR 준비 완료 (바퀴는 움직이지 않습니다)")

        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            scan = robot.lidar()
            features = cardinal_distances(scan)
            rounded = {k: round(v, 0) for k, v in features.items()}
            print(rounded, "valid_ratio=%.2f" % valid_fraction(scan))
            time.sleep(0.2)

    print("연결 확인 종료 (바퀴는 전혀 움직이지 않았습니다)")


if __name__ == "__main__":
    main()
