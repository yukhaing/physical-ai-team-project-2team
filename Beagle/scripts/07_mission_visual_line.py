from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""YOLO class 신호 + OMX box-placed 신호로 구동되는 전체 미션을 matplotlib 창에서 확인합니다.
신호는 TriggerServer(TCP+JSON)로 받습니다 -- scripts\\05_send_trigger.py --port 8766로 테스트.

이동 방법: line_follow_command() -- 각 leg의 시작점(그 시점의 est_pose) -> 목표점을 잇는
직선을 따라가는 line-following(Stanley 조향) 방식입니다. A*/경로 계획은 사용하지 않습니다.
같은 미션을 A*+pure pursuit으로 돌려보려면 scripts\\04_mission_astar_slam.py를 실행하세요.

방 배치는 실제 코스 그림(정사각형 방, 좌상단 Normal, 우상단 Defect, 하단 중앙
Receiving zone, 우하단 Start)에 맞췄습니다. 각 zone은 점이 아니라 실제 크기의
사각형(ZONE_SIZE)으로 그려서 로봇이 그 안에 실제로 들어갔는지 눈으로 바로
확인할 수 있습니다("앞에 멈춤"과 "안에 들어감"을 구분하기 위함).
목적지(정상/불량 zone)에 도착하면 5초간 대기 후 자동으로 복귀합니다.

포트를 8765(astar 버전)와 다르게 8766으로 써서 두 창을 동시에 띄워 비교할 수 있습니다.
"""

import math
import time

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from common.comm import TriggerServer
from common.geometry import Pose2D
from common.mission import Mission
from common.motion import line_follow_command
from common.robot import rectangle_segments
from simulator.beagle_sim import BeagleSimulator, draw_planned_path, draw_pursuit_target


def closest_point_on_segment(
    point: tuple[float, float], seg_start: tuple[float, float], seg_end: tuple[float, float]
) -> tuple[float, float]:
    """The point on segment seg_start->seg_end closest to `point` -- i.e. what
    line_follow_command's cross-track error is measured against right now."""
    dx = seg_end[0] - seg_start[0]
    dy = seg_end[1] - seg_start[1]
    length_sq = dx * dx + dy * dy
    if length_sq < 1e-9:
        return seg_start
    t = ((point[0] - seg_start[0]) * dx + (point[1] - seg_start[1]) * dy) / length_sq
    t = max(0.0, min(1.0, t))
    return (seg_start[0] + t * dx, seg_start[1] + t * dy)

ZONE_SIZE = 0.21  # 각 zone 사각형의 한 변 길이(m) -- 실측값
ZONES = {
    "start": (0.12, 0.12),
    "receiving": (0.43, 0.38),
    "normal": (0.78, 0.58),
    "defect": (0.78, 0.12),
}
ZONE_COLORS = {
    "start": "#16C3B2",
    "receiving": "#F5A623",
    "normal": "#3B4CCA",
    "defect": "#D0021B",
}
ROOM_BOUNDARY = rectangle_segments(0.0, 0.0, 0.90, 0.70)  # 실측 방 치수 (90cm x 70cm)
PORT = 8766


def draw_zones(ax) -> None:
    half = ZONE_SIZE / 2.0
    for zone_name, (zx, zy) in ZONES.items():
        rect = Rectangle(
            (zx - half, zy - half), ZONE_SIZE, ZONE_SIZE,
            facecolor=ZONE_COLORS[zone_name], edgecolor="#20242C", linewidth=1.5, alpha=0.85,
        )
        ax.add_patch(rect)
        ax.text(zx, zy, zone_name, ha="center", va="center", fontsize=9, color="white", weight="bold")


def main() -> None:
    start_heading = math.atan2(
        ZONES["receiving"][1] - ZONES["start"][1], ZONES["receiving"][0] - ZONES["start"][0]
    )
    sim = BeagleSimulator(ROOM_BOUNDARY, Pose2D(*ZONES["start"], start_heading), odom_noise=0.06, use_slam=True)
    mission = Mission(zones=ZONES)
    server = TriggerServer(host="0.0.0.0", port=PORT)
    server.start()

    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7))
    fig.canvas.manager.set_window_title("Mission Test (line)")

    running = {"value": True}

    def on_key(event) -> None:
        if (event.key or "").lower() == "q":
            running["value"] = False

    fig.canvas.mpl_connect("key_press_event", on_key)

    true_trail: list[tuple[float, float]] = []
    est_trail: list[tuple[float, float]] = []
    last_mission_state = None
    line_start: tuple[float, float] | None = None
    line_target: tuple[float, float] | None = None
    all_paths: list[tuple[tuple[float, float], tuple[float, float]]] = []  # every leg's line, kept for the whole run
    previous = time.monotonic()

    try:
        while running["value"] and plt.fignum_exists(fig.number):
            now = time.monotonic()
            dt = min(0.15, max(0.0, now - previous))
            previous = now

            for message in server.poll():
                if "class" in message:
                    mission.on_yolo_class(message["class"])
                elif message.get("event") == "box_placed":
                    mission.on_omx_box_placed()

            mission.tick()

            if mission.state != last_mission_state:
                target = mission.target_zone()
                if target is not None:
                    sim.auto_on = False
                    line_start = (sim.est_pose.x, sim.est_pose.y)
                    line_target = target
                    all_paths.append((line_start, line_target))
                else:
                    sim.auto_on = False
                    sim.cmd = (0.0, 0.0)
                    line_start = line_target = None
                last_mission_state = mission.state

            reached = False
            pp_target_point: tuple[float, float] | None = None
            if line_target is not None:
                left, right, _, reached = line_follow_command(sim.est_pose, line_start, line_target)
                sim.cmd = (left, right)
                if not reached:
                    pp_target_point = closest_point_on_segment(
                        (sim.est_pose.x, sim.est_pose.y), line_start, line_target
                    )

            sim.step(dt)

            if reached and mission.state in {
                "MOVE_TO_RECEIVING", "MOVE_TO_ZONE", "RETURN_TO_START",
            }:
                mission.notify_arrived()

            true_trail.append((sim.robot.pose.x, sim.robot.pose.y))
            est_trail.append((sim.est_pose.x, sim.est_pose.y))

            ax.clear()
            for sx, sy, ex, ey in ROOM_BOUNDARY:
                ax.plot([sx, ex], [sy, ey], color="#33415F", linewidth=3)
            draw_zones(ax)
            for i, (leg_start, leg_target) in enumerate(all_paths):
                draw_planned_path(ax, [leg_start, leg_target], label="Line Path" if i == 0 else "_nolegend_")
            draw_pursuit_target(ax, (sim.est_pose.x, sim.est_pose.y), pp_target_point)
            if true_trail:
                xs, ys = zip(*true_trail)
                ax.plot(xs, ys, color="#D9534F", linewidth=1.4, label="True Trajectory")
            if est_trail:
                xs, ys = zip(*est_trail)
                ax.plot(xs, ys, "--", color="#2C74F5", linewidth=1.4, label="Estimated Trajectory")
            ax.plot(sim.robot.pose.x, sim.robot.pose.y, "o", color="#D9534F", markersize=10)
            ax.plot(sim.est_pose.x, sim.est_pose.y, "x", color="#2C74F5", markersize=9)
            ax.set_aspect("equal")
            ax.grid(True, alpha=0.3)
            box_tag = f" ({mission.box_class})" if mission.box_class else ""
            dwell_tag = f" | waiting {mission.dwell_remaining():.1f}s" if mission.state == "AT_DESTINATION" else ""
            ax.set_title(f"Mission: {mission.state}{box_tag}{dwell_tag} | listening on :{PORT} | Q=quit")
            ax.legend(loc="lower left", fontsize=8)
            fig.canvas.draw_idle()
            plt.pause(0.03)
    finally:
        server.stop()

    plt.ioff()
    if plt.fignum_exists(fig.number):
        plt.show()


if __name__ == "__main__":
    main()
