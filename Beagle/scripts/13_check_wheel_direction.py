from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""왼쪽/오른쪽 바퀴를 하나씩, 짧게, 낮은 속도로 돌려서 실제로 어느 쪽으로 도는지 확인합니다.
matplotlib이 보여주는 방향과 실제 로봇이 가는 방향이 다를 때, 모터/엔코더 부호가
서로 안 맞는지 확인하기 위한 최소 진단 스크립트입니다.

로봇을 바닥에 놓고 주변에 충분한 공간을 확보한 뒤 실행하세요.
각 단계마다 Enter를 눌러야 다음으로 넘어갑니다 -- 직접 보고 판단할 시간을 줍니다.
"""

import time

from common.robot import SafeBeagle

SPEED = 12.0  # 안전 규칙: 10~15%
DURATION_S = 1.0


def spin(robot: SafeBeagle, left: float, right: float, label: str) -> None:
    input(f"\n[{label}] Enter를 누르면 {DURATION_S}초간 돌립니다...")
    left0 = robot.left_encoder()
    right0 = robot.right_encoder()
    robot.wheels(left, right)
    time.sleep(DURATION_S)
    robot.stop()
    left1 = robot.left_encoder()
    right1 = robot.right_encoder()
    print(f"  명령: left={left}% right={right}%")
    print(f"  left_encoder 변화량:  {left1 - left0:+.0f}")
    print(f"  right_encoder 변화량: {right1 - right0:+.0f}")
    print("  -> 실제로 로봇이 어느 방향으로 움직였는지 눈으로 확인하고 기록하세요.")


def main() -> None:
    with SafeBeagle(dry_run=False, max_speed=20) as robot:
        print("바퀴 방향 확인 -- 4단계. 각 단계 후 실제 움직임과 위 encoder 변화량 부호를 비교하세요.")
        spin(robot, SPEED, 0.0, "왼쪽 바퀴만 전진(+) 명령")
        spin(robot, 0.0, SPEED, "오른쪽 바퀴만 전진(+) 명령")
        spin(robot, -SPEED, 0.0, "왼쪽 바퀴만 후진(-) 명령")
        spin(robot, 0.0, -SPEED, "오른쪽 바퀴만 후진(-) 명령")

    print("\n각 단계에서:")
    print("  1) 로봇이 실제로 어느 방향(앞/뒤)으로 움직였는지")
    print("  2) encoder 변화량 부호가 그 방향과 맞는지(전진인데 음수 등)")
    print("결과를 알려주세요.")


if __name__ == "__main__":
    main()
