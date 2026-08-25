from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""YOLO class 신호 + OMX box-placed 신호로 구동되는 전체 미션을 matplotlib 창에서 확인합니다.
신호는 TriggerServer(TCP+JSON)로 받습니다 -- scripts\\05_send_trigger.py --port 8767로 테스트.

이동 방법: D* Lite (common/dstar_lite.py) + pure_pursuit_wheels(). beagle_sim.py의
set_goal()(A* 사용)을 거치지 않고, sim.plan_grid를 직접 재사용해 D* Lite로 경로를
계획하고 수동으로 추종합니다. 같은 미션을 A*/line-following으로 돌려보려면
scripts\\04_mission_astar_slam.py / scripts\\07_mission_visual_line.py를 실행하세요.

방 배치는 실제 코스 그림(정사각형 방, 좌상단 Normal, 우상단 Defect, 하단 중앙
Receiving zone, 우하단 Start)에 맞췄습니다. 각 zone은 점이 아니라 실제 크기의
사각형(ZONE_SIZE)으로 그려서 로봇이 그 안에 실제로 들어갔는지 눈으로 바로
확인할 수 있습니다. 목적지(정상/불량 zone)에 도착하면 5초간 대기 후 자동으로
복귀합니다.

포트를 8765(astar)/8766(line)과 다르게 8767로 써서 세 창을 동시에 띄워 비교할 수 있습니다.
"""

import math
import time

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from common.comm import TriggerServer
from common.dstar_lite import DStarLite
from common.geometry import Pose2D
from common.mission import Mission
from common.motion import align_to_heading_command
from common.planning import grid_path_to_world, pure_pursuit_wheels, reduce_waypoints
from common.robot import rectangle_segments
from simulator.beagle_sim import BeagleSimulator, draw_planned_path, draw_pursuit_target

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
PORT = 8767


def draw_zones(ax) -> None:
    half = ZONE_SIZE / 2.0
    for zone_name, (zx, zy) in ZONES.items():
        rect = Rectangle(
            (zx - half, zy - half), ZONE_SIZE, ZONE_SIZE,
            facecolor=ZONE_COLORS[zone_name], edgecolor="#20242C", linewidth=1.5, alpha=0.85,
        )
        ax.add_patch(rect)
        ax.text(zx, zy, zone_name, ha="center", va="center", fontsize=9, color="white", weight="bold")


def plan_leg(sim: BeagleSimulator, target_xy: tuple[float, float]) -> list[tuple[float, float]]:
    """sim.plan_grid를 그대로 재사용해 D* Lite로 현재 위치 -> target_xy 경로를 계획합니다.
    beagle_sim.py의 set_goal()과 같은 방식(가장 가까운 빈 칸으로 스냅 -> 경로 -> waypoint 축소
    -> 촘촘하게 보간)으로 world 좌표 경로를 만듭니다.
    """
    meta = sim.grid.meta

    def to_cell(x: float, y: float) -> tuple[int, int]:
        return int((x - meta.origin_x_m) / meta.resolution_m), int((y - meta.origin_y_m) / meta.resolution_m)

    start_cell = sim._nearest_free(*to_cell(sim.est_pose.x, sim.est_pose.y))
    goal_cell = sim._nearest_free(*to_cell(*target_xy))
    if start_cell is None or goal_cell is None:
        return []

    planner = DStarLite(sim.plan_grid, start_cell, goal_cell)
    planner.compute_shortest_path()
    path = planner.extract_path()
    if not path:
        return []

    waypoints = reduce_waypoints(sim.plan_grid, path)
    world_pts = grid_path_to_world(
        waypoints, resolution_m=meta.resolution_m, origin_x_m=meta.origin_x_m, origin_y_m=meta.origin_y_m
    )
    dense: list[tuple[float, float]] = [world_pts[0]]
    for a, b in zip(world_pts, world_pts[1:]):
        seg_len = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(1, int(seg_len / 0.06))
        for k in range(1, n + 1):
            dense.append((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n))
    return dense


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
    fig.canvas.manager.set_window_title("Mission Test (dstar)")

    running = {"value": True}

    def on_key(event) -> None:
        if (event.key or "").lower() == "q":
            running["value"] = False

    fig.canvas.mpl_connect("key_press_event", on_key)

    true_trail: list[tuple[float, float]] = []
    est_trail: list[tuple[float, float]] = []
    last_mission_state = None
    path: list[tuple[float, float]] = []
    all_paths: list[list[tuple[float, float]]] = []  # every leg's planned path, kept for the whole run
    path_index = 0
    pp_target_point: tuple[float, float] | None = None
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
                sim.auto_on = False
                target = mission.target_zone()
                if target is not None:
                    path = plan_leg(sim, target)
                    all_paths.append(path)
                    path_index = 0
                    pp_target_point = None
                    align_heading = math.atan2(target[1] - sim.est_pose.y, target[0] - sim.est_pose.x)
                    leg_phase = "ALIGNING"
                else:
                    sim.cmd = (0.0, 0.0)
                    path = []
                    pp_target_point = None
                last_mission_state = mission.state

            reached = False
            if leg_phase == "ALIGNING":
                left, right, aligned = align_to_heading_command(sim.est_pose, align_heading)
                sim.cmd = (left, right)
                if aligned:
                    leg_phase = "TRACKING"
            elif path:
                left, right, path_index = pure_pursuit_wheels(
                    sim.est_pose, path, lookahead_m=0.30, speed_mps=0.08, start_index=max(0, path_index - 1)
                )
                if 0 <= path_index < len(path):
                    pp_target_point = path[path_index]
                sim.cmd = (max(-20.0, min(20.0, left)), max(-20.0, min(20.0, right)))
                gx, gy = path[-1]
                if math.hypot(sim.est_pose.x - gx, sim.est_pose.y - gy) < 0.08:
                    reached = True
                    sim.cmd = (0.0, 0.0)
                    pp_target_point = None

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
            for i, leg_path in enumerate(all_paths):
                draw_planned_path(ax, leg_path, label="D* Path" if i == 0 else "_nolegend_")
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
            phase_tag = f" [{leg_phase}]" if path else ""
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
