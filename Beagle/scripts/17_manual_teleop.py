from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""화살표 키로 직접 로봇을 몰아보면서, 명령 방향(전진/후진/좌회전/우회전)과 실제 로봇의
움직임 + 화면에 그려지는 위치/heading이 서로 맞는지 눈으로 확인하는 진단용 스크립트.
미션 로직(TriggerServer, DeliveryMission, 상태 전송) 전혀 없음 -- 순수 수동 조작 +
시각화만.

simulator/beagle_sim.py의 BeagleSimulator.run()은 key_press_event만 처리해서 한 번
누르면 다른 키를 누르기 전까지 계속 움직입니다 (release를 안 봄). 여기서는 누르고 있는
동안만 움직이고 떼면 바로 멈추도록 key_press_event + key_release_event를 직접 처리하는
자체 루프를 씁니다 (run()을 그대로 쓰지 않음 -- 다른 스크립트가 쓰는 공유 코드는 그대로 둠).

조작: 방향키/WASD를 누르고 있는 동안만 이동 (떼면 즉시 정지), SPACE 즉시 정지,
+/- 속도, Q 종료 (Ctrl+C는 matplotlib/Tkinter와 얽혀서 안 먹을 수 있으니 반드시 Q로 종료).

확인할 것: 예) Up(전진) 눌렀을 때 화면의 로봇 마커가 heading 화살표가 가리키는 방향으로
가는지, 그리고 그게 실제 로봇이 움직이는 방향과도 일치하는지.

사용법:
  python scripts\\17_manual_teleop.py --dry-run   # 시뮬레이션
  python scripts\\17_manual_teleop.py             # 실물 로봇
"""

import argparse
import math
import time

import matplotlib.pyplot as plt
import numpy as np

from common.geometry import Pose2D
from common.lidar import cardinal_distances, valid_fraction
from common.localization import localize, resolve_180_ambiguity, scan_multiple
from common.robot import rectangle_segments
from simulator.beagle_sim import BeagleSimulator

ROOM_WIDTH_M = 0.90
ROOM_HEIGHT_M = 0.70
ROOM_BOUNDARY = rectangle_segments(0.0, 0.0, ROOM_WIDTH_M, ROOM_HEIGHT_M)
OMX_CENTER = (0.17, 0.37)
OMX_RADIUS = 0.065
OBSTACLE = rectangle_segments(
    OMX_CENTER[0] - OMX_RADIUS, OMX_CENTER[1] - OMX_RADIUS,
    OMX_CENTER[0] + OMX_RADIUS, OMX_CENTER[1] + OMX_RADIUS,
)
SEGMENTS = ROOM_BOUNDARY + OBSTACLE
RECEIVING = (0.41, 0.38)
DEFECT = (0.75, 0.13)
QUICK_SEARCH_RADIUS = 0.3  # receiving zone 근처라는 흔한 경우를 빠르게 처리
QUICK_SEARCH_OK_ERROR_M = 0.03  # 이 이하 매칭오차면 방 전체 탐색으로 확대할 필요 없음


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    start = Pose2D(*RECEIVING, 0.0)
    sim = BeagleSimulator(SEGMENTS, start, odom_noise=0.06, use_slam=True, dry_run=args.dry_run)

    if not args.dry_run:
        print("시작 위치 파악 중 (제자리에서 LiDAR 스캔)...")
        scan = scan_multiple(sim.robot)

        print(f"빠른 탐색 중 (receiving zone 주변 {QUICK_SEARCH_RADIUS * 100:.0f}cm)...")
        # ROOM_BOUNDARY(사각형)만 보고 매칭하면 방 중심 기준 180도 회전 대칭이라 실제
        # pose와 그 180도 뒤집힌 pose가 매칭 점수가 완전히 같아서 heading이 반대로 나올 수
        # 있음 -- OMX 장애물(SEGMENTS)까지 포함해서 매칭해야 그 대칭이 깨져 정확히 구분됨.
        detected_pose, match_err_m = localize(scan, SEGMENTS, *RECEIVING, QUICK_SEARCH_RADIUS, verbose=False)
        if match_err_m > QUICK_SEARCH_OK_ERROR_M:
            print(
                f"빠른 탐색 매칭오차 {match_err_m * 1000:.0f}mm -- receiving zone 근처가 아닌 듯합니다. "
                "방 전체로 확대 탐색합니다 (시간이 더 걸립니다)..."
            )
            full_center = (ROOM_WIDTH_M / 2.0, ROOM_HEIGHT_M / 2.0)
            full_radius = math.hypot(ROOM_WIDTH_M, ROOM_HEIGHT_M) / 2.0 + 0.05
            detected_pose, match_err_m = localize(scan, SEGMENTS, *full_center, full_radius)

        # 그리드 탐색 해상도 때문에 180도 대칭 쌍 중 더 잘 맞는 쪽을 놓쳤을 수 있으니,
        # 그 두 후보만 정밀하게(그리드에 얽매이지 않고) 다시 채점해서 최종 확정합니다.
        detected_pose, match_err_m = resolve_180_ambiguity(
            scan, SEGMENTS, detected_pose, ROOM_WIDTH_M, ROOM_HEIGHT_M
        )

        heading_deg = math.degrees(detected_pose.theta) % 360.0
        print(
            f"추정 위치: ({detected_pose.x:.3f}, {detected_pose.y:.3f}) heading={heading_deg:.0f}deg "
            f"(매칭오차 {match_err_m * 1000:.0f}mm)"
        )
        sim.est_pose = detected_pose
        sim.robot.pose = detected_pose

    print(f"receiving {RECEIVING} -> defect {DEFECT} (참고용, 이 스크립트는 자동주행 안 함)")
    print("조작: 방향키/WASD를 누르고 있는 동안만 이동, SPACE 정지, +/- 속도, Q 종료")

    plt.ion()
    fig, (ax_world, ax_map) = plt.subplots(1, 2, figsize=(13, 6.5))
    fig.canvas.manager.set_window_title("Manual Teleop")

    MOVE_KEYS = {"up", "down", "left", "right", "w", "a", "s", "d"}
    held: set[str] = set()
    running = {"value": True}

    def recompute_cmd() -> None:
        speed = sim.speed
        turn = min(14.0, speed * 0.75)
        if "up" in held or "w" in held:
            sim.cmd = (speed, speed)
            sim.status = "FORWARD"
        elif "down" in held or "s" in held:
            sim.cmd = (-speed, -speed)
            sim.status = "BACKWARD"
        elif "left" in held or "a" in held:
            sim.cmd = (-turn, turn)
            sim.status = "TURN_LEFT"
        elif "right" in held or "d" in held:
            sim.cmd = (turn, -turn)
            sim.status = "TURN_RIGHT"
        else:
            sim.cmd = (0.0, 0.0)
            sim.status = "STOP"

    def on_key_press(event) -> None:
        key = (event.key or "").lower()
        if key == "q":
            running["value"] = False
        elif key == " ":
            held.clear()
            sim.cmd = (0.0, 0.0)
            sim.status = "STOP"
        elif key in ("+", "="):
            sim.speed = min(25.0, sim.speed + 2.0)
            print(f"Speed: {sim.speed:.0f}")
            recompute_cmd()
        elif key in ("-", "_"):
            sim.speed = max(6.0, sim.speed - 2.0)
            print(f"Speed: {sim.speed:.0f}")
            recompute_cmd()
        elif key in MOVE_KEYS:
            held.add(key)
            recompute_cmd()

    def on_key_release(event) -> None:
        key = (event.key or "").lower()
        if key in MOVE_KEYS:
            held.discard(key)
            recompute_cmd()

    fig.canvas.mpl_connect("key_press_event", on_key_press)
    fig.canvas.mpl_connect("key_release_event", on_key_release)

    previous = time.monotonic()
    last_print_time = 0.0

    try:
        while running["value"] and plt.fignum_exists(fig.number):
            now = time.monotonic()
            dt = min(0.15, max(0.0, now - previous))
            previous = now

            sim.step(dt)
            scan = sim.robot.lidar()

            if now - last_print_time >= 0.5:
                cd = cardinal_distances(scan)
                vf = valid_fraction(scan)
                print(
                    f"pos=({sim.est_pose.x:.3f},{sim.est_pose.y:.3f}) "
                    f"heading={math.degrees(sim.est_pose.theta):.1f}deg cmd={sim.cmd} status={sim.status} | "
                    f"lidar front={cd['front']:.0f} rear={cd['rear']:.0f} left={cd['left']:.0f} "
                    f"right={cd['right']:.0f}mm valid={vf * 100:.0f}%"
                )
                last_print_time = now

            ax_world.clear()
            for sx, sy, ex, ey in SEGMENTS:
                ax_world.plot([sx, ex], [sy, ey], color="#33415F", linewidth=3)
            if sim.show_trails and len(sim.true_trail) > 1:
                tx, ty = zip(*sim.true_trail)
                ax_world.plot(tx, ty, color="#D9534F", linewidth=1.4, alpha=0.7, label="True Trajectory")
                ex_, ey_ = zip(*sim.est_trail)
                ax_world.plot(ex_, ey_, color="#2C74F5", linewidth=1.4, alpha=0.7,
                              linestyle="--", label="Estimated Trajectory")
                ax_world.legend(loc="upper right", fontsize=8)
            ax_world.plot(sim.robot.pose.x, sim.robot.pose.y, "o", color="#D9534F", markersize=10)
            ax_world.arrow(sim.robot.pose.x, sim.robot.pose.y,
                           0.12 * math.cos(sim.robot.pose.theta), 0.12 * math.sin(sim.robot.pose.theta),
                           head_width=0.045, color="#D9534F")
            ax_world.plot(sim.est_pose.x, sim.est_pose.y, "x", color="#2C74F5", markersize=9)
            x0, y0, x1, y1 = sim.bounds
            ax_world.set_xlim(x0, x1)
            ax_world.set_ylim(y0, y1)
            ax_world.set_aspect("equal")
            ax_world.grid(True, alpha=0.3)
            ax_world.set_title(f"World | {sim.status} | speed={sim.speed:.0f} | Q=quit")

            ax_map.clear()
            occ = sim.grid.occupancy()
            image = np.full(occ.shape, 0.65)
            image[occ == 0] = 1.0
            image[occ >= 65] = 0.0
            ax_map.imshow(image, cmap="gray", origin="lower", vmin=0, vmax=1, extent=(x0, x1, y0, y1))
            ax_map.plot(sim.est_pose.x, sim.est_pose.y, "x", color="#2C74F5", markersize=9)
            ax_map.set_title("Occupancy Grid")
            ax_map.set_aspect("equal")

            fig.canvas.draw_idle()
            plt.pause(0.03)
    finally:
        sim.robot.stop()

    plt.ioff()
    if plt.fignum_exists(fig.number):
        plt.show()


if __name__ == "__main__":
    main()
