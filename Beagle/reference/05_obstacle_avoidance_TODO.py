
from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""실습 3-05: LiDAR 장애물 회피의 상태 결정 함수를 완성하세요."""

import argparse
import time

from common.lidar import cardinal_distances, valid_fraction
from common.robot import SafeBeagle

EMERGENCY_MM = 100.0
TURN_MM = 130.0
BACKUP_CLEAR_MM = 200.0  # rear clearance required before reversing out of a jam
TURN_MARGIN_MM = 20.0  # left/right must differ by more than this to switch turn direction

_last_turn_state = "TURN_LEFT"


# def decide_state(front: float, left: float, right: float, valid_ratio: float) -> str:
#     # TODO 1: valid_ratio < 0.5면 SENSOR_FAIL
#     # TODO 2: front < EMERGENCY_MM면 EMERGENCY_STOP
#     # TODO 3: front < TURN_MM이면 더 열린 쪽으로 회전
#     # TODO 4: 그 외 FORWARD
#     return "TODO"


# def wheel_command(state: str) -> tuple[float, float]:
#     # TODO: 상태별 바퀴 명령을 작성하세요.
#     return 0.0, 0.0

def decide_state(front: float, left: float, right: float, rear: float, valid_ratio: float) -> str:
    global _last_turn_state
    if valid_ratio < 0.5:
        return "SENSOR_FAIL"
    if front < EMERGENCY_MM:
        if rear > BACKUP_CLEAR_MM:
            return "BACKUP"
        return "EMERGENCY_STOP"
    if front < TURN_MM:
        if left - right > TURN_MARGIN_MM:
            _last_turn_state = "TURN_LEFT"
        elif right - left > TURN_MARGIN_MM:
            _last_turn_state = "TURN_RIGHT"
        return _last_turn_state
    return "FORWARD"


def wheel_command(state: str) -> tuple[float, float]:
    if state == "FORWARD":
        return 6.0, 6.0
    if state == "TURN_LEFT":
        return -10.0, 10.0  # curve left while still moving forward
    if state == "TURN_RIGHT":
        return 10.0, -10.0  # curve right while still moving forward
    if state == "BACKUP":
        return -10.0, -10.0

    return 0.0, 0.0  # EMERGENCY_STOP / SENSOR_FAIL



def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--scene", default="obstacles")
    parser.add_argument("--duration", type=float, default=60.0)
    args = parser.parse_args()

    with SafeBeagle(dry_run=args.dry_run, scene=args.scene, max_speed=20) as robot:
        robot.start_lidar(); robot.wait_until_lidar_ready()
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            scan = robot.lidar()
            features = cardinal_distances(scan)
            state = decide_state(features["front"], features["left"], features["right"], features["rear"], valid_fraction(scan))
            left_cmd, right_cmd = wheel_command(state)
            robot.wheels(left_cmd, right_cmd)
            print(state, features, (left_cmd, right_cmd))
            time.sleep(0.08)


if __name__ == "__main__":
    main()
