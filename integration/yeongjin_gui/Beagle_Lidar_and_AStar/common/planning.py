from __future__ import annotations

import heapq
import math

from common.geometry import Pose2D, wrap_angle

Point = tuple[float, float]
Rect = tuple[float, float, float, float]  # xmin, ymin, xmax, ymax


def astar_path(
    start: Point,
    goal: Point,
    boundary_w_m: float,
    boundary_h_m: float,
    obstacles: list[Rect] | None = None,
    cell_m: float = 0.02,
) -> list[Point]:
    """Grid A* over the rectangular room [0, boundary_w_m] x [0, boundary_h_m].

    obstacles: rectangles (xmin, ymin, xmax, ymax) in meters to avoid. Returns a
    list of (x, y) waypoints in meters from start to goal (inclusive).
    """
    obstacles = obstacles or []
    cols = max(1, round(boundary_w_m / cell_m))
    rows = max(1, round(boundary_h_m / cell_m))

    def to_cell(p: Point) -> tuple[int, int]:
        return (
            min(cols - 1, max(0, round(p[0] / cell_m))),
            min(rows - 1, max(0, round(p[1] / cell_m))),
        )

    def to_point(c: tuple[int, int]) -> Point:
        return (c[0] * cell_m, c[1] * cell_m)

    def blocked(c: tuple[int, int]) -> bool:
        x, y = to_point(c)
        return any(xmin <= x <= xmax and ymin <= y <= ymax for xmin, ymin, xmax, ymax in obstacles)

    start_c, goal_c = to_cell(start), to_cell(goal)
    neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

    open_set: list[tuple[float, tuple[int, int]]] = [(0.0, start_c)]
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    g_score = {start_c: 0.0}

    def heuristic(c: tuple[int, int]) -> float:
        return math.hypot(c[0] - goal_c[0], c[1] - goal_c[1])

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal_c:
            break
        for dx, dy in neighbors:
            nxt = (current[0] + dx, current[1] + dy)
            if not (0 <= nxt[0] < cols and 0 <= nxt[1] < rows) or blocked(nxt):
                continue
            tentative = g_score[current] + math.hypot(dx, dy)
            if tentative < g_score.get(nxt, math.inf):
                g_score[nxt] = tentative
                came_from[nxt] = current
                heapq.heappush(open_set, (tentative + heuristic(nxt), nxt))

    if goal_c not in came_from and goal_c != start_c:
        return [start, goal]  # fully blocked -- fall back to a direct line

    path_cells = [goal_c]
    while path_cells[-1] != start_c:
        path_cells.append(came_from[path_cells[-1]])
    path_cells.reverse()
    return [to_point(c) for c in path_cells]


def pure_pursuit_target(pose: Pose2D, path: list[Point], lookahead_m: float) -> Point:
    """Point on `path` roughly lookahead_m ahead of the robot's current position."""
    if not path:
        return (pose.x, pose.y)
    nearest_i = min(range(len(path)), key=lambda i: pose.distance_to(*path[i]))
    for i in range(nearest_i, len(path)):
        if pose.distance_to(*path[i]) >= lookahead_m:
            return path[i]
    return path[-1]


def pure_pursuit_command(
    pose: Pose2D, target: Point, speed_mps: float, max_wheel_mps: float, wheel_base_m: float
) -> tuple[float, float]:
    """Differential-drive wheel speeds (left_mps, right_mps) to steer toward `target`."""
    dist = pose.distance_to(*target)
    if dist < 1e-6:
        return 0.0, 0.0
    heading_err = wrap_angle(pose.heading_to(*target) - pose.theta)
    curvature = 2.0 * math.sin(heading_err) / dist
    omega = curvature * speed_mps
    left = speed_mps - omega * wheel_base_m / 2.0
    right = speed_mps + omega * wheel_base_m / 2.0
    scale = max(1.0, abs(left) / max_wheel_mps, abs(right) / max_wheel_mps)
    return left / scale, right / scale

