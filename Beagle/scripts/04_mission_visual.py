from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""YOLO class 신호 + OMX box-placed 신호로 구동되는 전체 미션을 matplotlib 창에서 확인합니다.
신호는 TriggerServer(TCP+JSON)로 받습니다 -- scripts\\05_send_trigger.py로 테스트하거나,
나중에는 실제 YOLO/OMX 코드가 같은 형식으로 메시지를 보내면 됩니다.

이동 방법: A* + pure pursuit (beagle_sim.py의 set_goal() 그대로 사용).
같은 미션을 line-following으로 돌려보려면 scripts\\07_mission_visual_line.py를 실행하세요.

방 배치는 실제 코스 그림(정사각형 방, 좌상단 Normal, 우상단 Defect, 하단 중앙
Receiving zone, 우하단 Start)에 맞췄습니다. 각 zone은 점이 아니라 실제 크기의
사각형(ZONE_SIZE)으로 그려서 로봇이 그 안에 실제로 들어갔는지 눈으로 바로
확인할 수 있습니다("앞에 멈춤"과 "안에 들어감"을 구분하기 위함).
목적지(정상/불량 zone)에 도착하면 5초간 대기 후 자동으로 복귀합니다.
"""

import math
import time

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from common.comm import TriggerServer
from common.geometry import Pose2D
from common.mission import Mission
from common.motion import align_to_heading_command
from common.robot import rectangle_segments
from simulator.beagle_sim import BeagleSimulator, draw_planned_path, draw_pursuit_target

ZONE_SIZE = 0.6  # 각 zone 사각형의 한 변 길이(m) -- 0.4는 실측 위치오차(26~31cm)보다 작아서 0.6으로 여유를 둠
ZONES = {
    "start": (2.1, 0.3),
    "receiving": (0.9, 0.3),
    "normal": (0.3, 2.1),
    "defect": (2.1, 2.1),
}
ZONE_COLORS = {
    "start": "#16C3B2",
    "receiving": "#F5A623",
    "normal": "#3B4CCA",
    "defect": "#D0021B",
}
ROOM_BOUNDARY = rectangle_segments(0.0, 0.0, 2.4, 2.4)  # 실제 방 치수로 나중에 교체


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
    sim = BeagleSimulator(ROOM_BOUNDARY, Pose2D(*ZONES["start"], math.pi), odom_noise=0.06, use_slam=True)
    mission = Mission(zones=ZONES)
    server = TriggerServer(host="0.0.0.0", port=8765)
    server.start()

    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7))
    fig.canvas.manager.set_window_title("Mission Test (astar)")

    running = {"value": True}

    def on_key(event) -> None:
        if (event.key or "").lower() == "q":
            running["value"] = False

    fig.canvas.mpl_connect("key_press_event", on_key)

    true_trail: list[tuple[float, float]] = []
    est_trail: list[tuple[float, float]] = []
    last_mission_state = None
    all_paths: list[list[tuple[float, float]]] = []  # every leg's planned path, kept for the whole run
    leg_phase = "TRACKING"  # "ALIGNING" (turn in place first) or "TRACKING" (pure pursuit)
    align_heading = 0.0
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
                    sim.set_goal(*target)  # computes/stores the path; auto_on flipped back off below
                    all_paths.append(list(sim.auto_path))
                    align_heading = math.atan2(target[1] - sim.est_pose.y, target[0] - sim.est_pose.x)
                    leg_phase = "ALIGNING"
                    sim.auto_on = False  # hold off pure pursuit until roughly facing the target
                else:
                    sim.auto_on = False
                    sim.cmd = (0.0, 0.0)
                last_mission_state = mission.state

            if leg_phase == "ALIGNING":
                left, right, aligned = align_to_heading_command(sim.est_pose, align_heading)
                sim.cmd = (left, right)
                if aligned:
                    leg_phase = "TRACKING"
                    sim.auto_on = True  # hand off to set_goal()'s pure pursuit on the path already computed above

            sim.step(dt)

            if sim.status == "GOAL!" and mission.state in {
                "MOVE_TO_RECEIVING", "MOVE_TO_ZONE", "RETURN_TO_START",
            }:
                mission.notify_arrived()

            true_trail.append((sim.robot.pose.x, sim.robot.pose.y))
            est_trail.append((sim.est_pose.x, sim.est_pose.y))

            ax.clear()
            for sx, sy, ex, ey in ROOM_BOUNDARY:
                ax.plot([sx, ex], [sy, ey], color="#33415F", linewidth=3)
            draw_zones(ax)
            for i, leg_path in enumerate(all_paths):
                draw_planned_path(ax, leg_path, label="A* Path" if i == 0 else "_nolegend_")
            draw_pursuit_target(ax, (sim.est_pose.x, sim.est_pose.y), sim.pp_target_point)
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
            phase_tag = f" [{leg_phase}]" if mission.target_zone() is not None else ""
            ax.set_title(f"Mission: {mission.state}{box_tag}{dwell_tag}{phase_tag} | listening on :8765 | Q=quit")
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
