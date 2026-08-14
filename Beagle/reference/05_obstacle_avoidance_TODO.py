
from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""실습 3-05: LiDAR 장애물 회피의 상태 결정 함수를 완성하세요."""

import argparse
import time

from common.lidar import cardinal_distances, valid_fraction
from common.robot import SafeBeagle


EMERGENCY_MM = 130.0 #အရေးပေါ် ရပ်တန့်/နောက်ဆုတ်ရမည့် အန္တရာယ်ရှိ အကွာအဝေး
TURN_MM = 150.0  #အတားအဆီးကို စတင် ရှောင်ကွင်း ကွေ့ရမည့် အကွာအဝေး
CLEAR_MM = 250.0      # အတားအဆီး လုံးဝ လွတ်ကင်းသွားကြောင်း အတည်ပြုသည့် အကွာအဝေး
BACKUP_CLEAR_MM = 70.0 #အနောက်သို့ စိတ်ချစွာ နောက်ဆုတ်နိုင်ရန် လိုအပ်သော အနောက်ဘက် နေရာလွတ်
TURN_MARGIN_MM = 150.0 #ဘယ် သို့မဟုတ် ညာ လားရာ ရွေးချယ်ရန် လိုအပ်သည့် အနည်းဆုံး နေရာလွတ် ကွာခြားချက်

_last_turn_state = "TURN_LEFT"
_turning = False


def decide_state(front, left, right, rear, valid_ratio) -> str:
    global _last_turn_state, _turning
    if valid_ratio < 0.5:
        _turning = False
        return "SENSOR_FAIL"
    if front < EMERGENCY_MM:
        _turning = False
        if rear > BACKUP_CLEAR_MM:
            return "BACKUP"
        return "EMERGENCY_STOP"
    if _turning:
        if front >= CLEAR_MM:
            _turning = False
        else:
            return _last_turn_state
    if front < TURN_MM:
        _turning = True
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
