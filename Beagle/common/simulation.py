from __future__ import annotations

from .geometry import Pose2D, euclidean, integrate_velocity, wheel_percent_to_mps
from .planning import pure_pursuit_wheels


def demo_path() -> list[tuple[float, float]]:
    return [(0.0, 0.0), (0.7, 0.0), (1.2, 0.4), (1.6, 1.0), (2.2, 1.0), (2.8, 0.55)]


def simulate_pure_pursuit(
    lookahead: float = 0.25,
    max_steps: int = 1200,
    *,
    dt_s: float = 0.04,
) -> tuple[list[Pose2D], bool]:
    path = demo_path()
    pose = Pose2D(-0.05, -0.10, 0.0)
    history = [Pose2D(pose.x, pose.y, pose.theta)]
    index = 0
    for _ in range(max_steps):
        left, right, index = pure_pursuit_wheels(
            pose,
            path,
            lookahead_m=lookahead,
            speed_mps=0.11,
            start_index=max(0, index - 1),
        )
        pose = integrate_velocity(
            pose,
            wheel_percent_to_mps(left),
            wheel_percent_to_mps(right),
            dt_s,
        )
        history.append(Pose2D(pose.x, pose.y, pose.theta))
        if euclidean((pose.x, pose.y), path[-1]) < 0.08:
            return history, True
    return history, False
