from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""scripts\\04_mission_astar_slam.py와 완전히 동일하지만, 방 중앙에 벽(장애물)을 하나 두고
A*가 그 주변으로 실제로 돌아가는지 확인합니다. 04는 그대로 두고(장애물 없는 기준선),
이 파일만 장애물 버전입니다.

장애물: 아래쪽 벽에 붙은 중앙 기둥 (x=0.45, y=0~0.30). start<->receiving 직선 경로(y=0.105)와
receiving<->defect 대각선 경로를 막아서 위쪽(y>0.30)으로 돌아가게 합니다. 90cm x 70cm 실측
방 치수에 맞춘 크기입니다.
막히면(전방 LiDAR 근접이 일정 시간 지속) beagle_sim.py의 step()에 이미 있는
backup-turn-replan 복구 로직이 sim.auto_on을 통해 그대로 적용됩니다(이 파일은
sim.set_goal()에 실제 주행을 맡기므로 별도 복구 코드가 필요 없습니다).

포트 8768 사용 (8765=장애물 없는 astar, 8766=line, 8767=장애물 없는 dstar).
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

ZONE_SIZE = 0.21
ZONES = {
    "start": (0.795, 0.105),
    "receiving": (0.26, 0.105),
    "normal": (0.105, 0.595),
    "defect": (0.795,0.595),
}
ZONE_COLORS = {
    "start": "#F5A623",
    "receiving": "#F5A623",
    "normal": "#18DF39",
    "defect": "#D0021B",
}
ROOM_BOUNDARY = rectangle_segments(0.0, 0.0, 0.90, 0.70)  # 실측 방 치수 (90cm x 70cm)
OBSTACLE = [
    # 아래쪽 벽에 붙은 기둥. start(0.795,0.105)<->receiving(0.26,0.105) 직선 경로와
    # receiving<->defect 대각선을 막아서 y>0.30 쪽으로 돌아가게 만듭니다. 위로 0.40m
    # 여유가 남아서(방 높이 0.70m 기준) 팽창 후에도 충분히 넓은 통로가 유지됩니다.
    (0.45, 0.0, 0.45, 0.30),
]
SEGMENTS = ROOM_BOUNDARY + OBSTACLE
PORT = 8768


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
    sim = BeagleSimulator(SEGMENTS, Pose2D(*ZONES["start"], math.pi), odom_noise=0.06, use_slam=True)
    mission = Mission(zones=ZONES)
    server = TriggerServer(host="0.0.0.0", port=PORT)
    server.start()

    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7))
    fig.canvas.manager.set_window_title("Mission Test (astar+obstacle)")

    running = {"value": True}

    def on_key(event) -> None:
        if (event.key or "").lower() == "q":
            running["value"] = False

    fig.canvas.mpl_connect("key_press_event", on_key)

    true_trail: list[tuple[float, float]] = []
    est_trail: list[tuple[float, float]] = []
    last_mission_state = None
    all_paths: list[list[tuple[float, float]]] = []
    leg_phase = "TRACKING"
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
                    sim.set_goal(*target)
                    all_paths.append(list(sim.auto_path))
                    align_heading = math.atan2(target[1] - sim.est_pose.y, target[0] - sim.est_pose.x)
                    leg_phase = "ALIGNING"
                    sim.auto_on = False
                else:
                    sim.auto_on = False
                    sim.cmd = (0.0, 0.0)
                last_mission_state = mission.state

            if leg_phase == "ALIGNING":
                left, right, aligned = align_to_heading_command(sim.est_pose, align_heading)
                sim.cmd = (left, right)
                if aligned:
                    leg_phase = "TRACKING"
                    sim.auto_on = True

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
            for i, (sx, sy, ex, ey) in enumerate(OBSTACLE):
                ax.plot([sx, ex], [sy, ey], color="#E8590C", linewidth=5,
                        label="Obstacle" if i == 0 else "_nolegend_")
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
            ax.set_title(f"Mission: {mission.state}{box_tag}{dwell_tag}{phase_tag} | listening on :{PORT} | Q=quit")
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
