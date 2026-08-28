from __future__ import annotations

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import json
import math

from common.geometry import Pose2D
from simulator.sim import SimRobot

CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "course_config.json"
DT = 0.05
TIMEOUT_S = 60.0
START_X, START_Y = 0.12, 0.12  # arbitrary corner, far from the receiving zone


def load_config() -> dict:
    with open(CONFIG_PATH, encoding="utf-8") as f:
        return json.load(f)


def main() -> None:
    cfg = load_config()
    boundary = cfg["boundary"]
    receiving = cfg["zones"]["receiving"]

    start_theta = math.atan2(receiving["y_m"] - START_Y, receiving["x_m"] - START_X)
    start = Pose2D(START_X, START_Y, start_theta)

    robot = SimRobot(start, boundary["x_m"], boundary["y_m"])
    robot.set_goal(receiving["x_m"], receiving["y_m"])

    t = 0.0
    status = "MOVING"
    while t < TIMEOUT_S and status != "GOAL":
        status = robot.step(DT)
        t += DT

    pos_error_cm = robot.pose.distance_to(receiving["x_m"], receiving["y_m"]) * 100.0
    heading_deg = math.degrees(robot.pose.theta) % 360.0

    print(f"status={status} time={t:.1f}s")
    print(f"start pose  = ({START_X:.3f}, {START_Y:.3f})")
    print(f"target      = ({receiving['x_m']:.3f}, {receiving['y_m']:.3f})")
    print(f"final pose  = ({robot.pose.x:.3f}, {robot.pose.y:.3f}) heading={heading_deg:.1f}deg")
    print(f"position error = {pos_error_cm:.1f} cm")


if __name__ == "__main__":
    main()
