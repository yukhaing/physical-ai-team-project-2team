from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""Phase 1: 장애물 없는 열린 바닥에서 A* + pure pursuit로 목표점까지 이동을 검증합니다.
common/planning.py의 astar()/pure_pursuit_wheels()는 beagle_sim.py의 BeagleSimulator가
이미 통합해뒀으므로(set_goal() -> 자동 A* 계획 + 추종 + 막히면 재계획/회피), 여기서는
그 흐름을 그대로 재사용합니다 -- 별도의 이동 제어 로직을 새로 만들지 않습니다.

경로 위에 장애물은 없지만, 실물 로봇도 항상 방 벽 안에서 움직이므로 SLAM이
맞춰볼 수 있게 방 경계 벽을 하나 두고 SLAM을 켭니다 (벽이 전혀 없으면
scan_match()가 보정할 대상이 없어 SLAM을 켜도 아무 효과가 없습니다).
"""

import math
import time

from common.geometry import Pose2D
from common.robot import rectangle_segments
from simulator.beagle_sim import BeagleSimulator

ZONES = {
    "receiving": (0.26, 0.105),
    "normal": (0.105, 0.595),
    "defect": (0.795, 0.595),
}

START = Pose2D(0.795, 0.105, math.pi)
DT = 0.05
TIMEOUT_S = 90.0
ROOM_BOUNDARY = rectangle_segments(0.0, 0.0, 0.90, 0.70)  # 실측 방 치수 (90cm x 70cm)


def run_to_target(name: str, target_x: float, target_y: float) -> None:
    sim = BeagleSimulator(ROOM_BOUNDARY, START, odom_noise=0.06, use_slam=True)
    sim.set_goal(target_x, target_y)

    deadline = time.monotonic() + TIMEOUT_S
    reached = False
    while time.monotonic() < deadline:
        sim.step(DT)
        # MockBeagle._update() integrates the *true* pose using real wall-clock time
        # (time.monotonic()), not the dt passed to step() -- so this loop must actually
        # take DT seconds per iteration, or true and estimated pose desync badly.
        time.sleep(DT)
        if sim.status == "GOAL!":
            reached = True
            break
    sim.cmd = (0.0, 0.0)
    sim.step(DT)

    est = sim.est_pose
    true = sim.robot.pose
    est_error = math.hypot(est.x - target_x, est.y - target_y)
    true_error = math.hypot(true.x - target_x, true.y - target_y)

    print(f"[{name}] target=({target_x:.2f}, {target_y:.2f})"
          f"{' TIMEOUT' if not reached else ''}")
    print(f"  est_pose  = ({est.x:.3f}, {est.y:.3f})  error={est_error * 100:.1f}cm")
    print(f"  true_pose = ({true.x:.3f}, {true.y:.3f})  error={true_error * 100:.1f}cm")


def main() -> None:
    for name, (x, y) in ZONES.items():
        run_to_target(name, x, y)


if __name__ == "__main__":
    main()
