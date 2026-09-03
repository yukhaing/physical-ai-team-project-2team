from __future__ import annotations

from common.geometry import Pose2D
from common.motion import integrate_dead_reckoning
from common.planning import Rect, astar_path, pure_pursuit_command, pure_pursuit_target


class SimRobot:
    """Minimal 2D kinematic simulator: true pose == estimated pose, no odometry
    noise yet. Step 2 validates the A*/pure-pursuit math itself before adding
    sensor/odometry error in a later step.
    """

    def __init__(
        self,
        start: Pose2D,
        boundary_w_m: float,
        boundary_h_m: float,
        wheel_base_m: float = 0.0956,
        max_wheel_mps: float = 0.15,
    ):
        self.pose = start
        self.boundary_w_m = boundary_w_m
        self.boundary_h_m = boundary_h_m
        self.wheel_base_m = wheel_base_m
        self.max_wheel_mps = max_wheel_mps
        self.path: list[tuple[float, float]] = []
        self.goal: tuple[float, float] | None = None

    def set_goal(self, x: float, y: float, obstacles: list[Rect] | None = None) -> None:
        self.goal = (x, y)
        self.path = astar_path(
            (self.pose.x, self.pose.y), (x, y), self.boundary_w_m, self.boundary_h_m, obstacles
        )

    def step(
        self, dt: float, speed_mps: float = 0.06, lookahead_m: float = 0.20, goal_tolerance_m: float = 0.01
    ) -> str:
        if self.goal is None:
            return "IDLE"
        if self.pose.distance_to(*self.goal) <= goal_tolerance_m:
            return "GOAL"
        target = pure_pursuit_target(self.pose, self.path, lookahead_m)
        left, right = pure_pursuit_command(self.pose, target, speed_mps, self.max_wheel_mps, self.wheel_base_m)
        self.pose = integrate_dead_reckoning(self.pose, left, right, self.wheel_base_m, dt)
        return "MOVING"
