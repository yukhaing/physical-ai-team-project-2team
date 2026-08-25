from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

"""Phase 1: A* + pure pursuit 이동을 matplotlib 창으로 실시간 확인합니다.
BeagleSimulator.set_goal()이 이미 A* 계획 + 추종 + 막히면 재계획/회피를 통합해
두고 있으므로 그대로 재사용합니다 (별도 이동 제어 로직 없음).
경로 위에 장애물은 없지만, 방 벽을 두고 SLAM을 켜서 relocalization이 다리(leg)
사이에 누적되는 오차를 얼마나 줄여주는지 눈으로 비교합니다. Q로 언제든 종료.
"""

import math
import time

import matplotlib.pyplot as plt

from common.geometry import Pose2D
from common.robot import rectangle_segments
from simulator.beagle_sim import BeagleSimulator, draw_planned_path, draw_pursuit_target

ZONES = {
    "receiving": (0.43, 0.38),
    "normal": (0.78, 0.58),
    "defect": (0.78, 0.12),
}
START = Pose2D(0.12, 0.12, math.atan2(ZONES["receiving"][1] - 0.12, ZONES["receiving"][0] - 0.12))
TIMEOUT_S = 90.0
ROOM_BOUNDARY = rectangle_segments(0.0, 0.0, 0.90, 0.70)  # 실측 방 치수 (90cm x 70cm)


def main() -> None:
    sim = BeagleSimulator(ROOM_BOUNDARY, START, odom_noise=0.06, use_slam=True)

    plt.ion()
    fig, ax = plt.subplots(figsize=(7, 7))
    fig.canvas.manager.set_window_title("Point-to-Point Test")

    running = {"value": True}

    def on_key(event) -> None:
        if (event.key or "").lower() == "q":
            running["value"] = False

    fig.canvas.mpl_connect("key_press_event", on_key)

    true_trail: list[tuple[float, float]] = []
    est_trail: list[tuple[float, float]] = []
    results: list[tuple[str, bool, float, float]] = []

    zone_items = list(ZONES.items())
    zone_index = 0
    name, (tx, ty) = zone_items[zone_index]
    sim.set_goal(tx, ty)
    leg_deadline = time.monotonic() + TIMEOUT_S
    previous = time.monotonic()

    while running["value"] and plt.fignum_exists(fig.number) and zone_index < len(zone_items):
        now = time.monotonic()
        dt = min(0.15, max(0.0, now - previous))
        previous = now

        sim.step(dt)

        true_trail.append((sim.robot.pose.x, sim.robot.pose.y))
        est_trail.append((sim.est_pose.x, sim.est_pose.y))

        reached = sim.status == "GOAL!"
        timed_out = time.monotonic() > leg_deadline
        if reached or timed_out:
            true_err = math.hypot(sim.robot.pose.x - tx, sim.robot.pose.y - ty)
            est_err = math.hypot(sim.est_pose.x - tx, sim.est_pose.y - ty)
            results.append((name, reached, true_err, est_err))
            sim.cmd = (0.0, 0.0)
            sim.auto_on = False
            sim.step(dt)
            zone_index += 1
            if zone_index < len(zone_items):
                name, (tx, ty) = zone_items[zone_index]
                sim.set_goal(tx, ty)
                leg_deadline = time.monotonic() + TIMEOUT_S

        ax.clear()
        for sx, sy, ex, ey in ROOM_BOUNDARY:
            ax.plot([sx, ex], [sy, ey], color="#33415F", linewidth=3)
        for zone_name, (zx, zy) in ZONES.items():
            ax.plot(zx, zy, "*", markersize=16, color="#7A62F6")
            ax.annotate(zone_name, (zx, zy), textcoords="offset points", xytext=(8, 8))
        draw_planned_path(ax, sim.auto_path, label="A* Path")
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
        status = "DONE" if zone_index >= len(zone_items) else f"-> {name}"
        ax.set_title(f"Point-to-Point | {status} | Q to quit")
        ax.legend(loc="upper right", fontsize=8)
        fig.canvas.draw_idle()
        plt.pause(0.03)

    print("--- results ---")
    for zone_name, reached, true_err, est_err in results:
        tag = "" if reached else " (TIMEOUT)"
        print(f"{zone_name}{tag}: true_err={true_err * 100:.1f}cm  est_err={est_err * 100:.1f}cm")

    plt.ioff()
    if plt.fignum_exists(fig.number):
        ax.set_title(ax.get_title() + "  (finished -- close window to exit)")
        plt.show()


if __name__ == "__main__":
    main()
