from __future__ import annotations

from dataclasses import dataclass
import heapq
import math
from typing import Iterable, Sequence

import numpy as np

from .geometry import Pose2D, clamp, euclidean, twist_to_wheel_percent, wrap_angle
from .mapping import bresenham

GridPoint = tuple[int, int]


@dataclass(slots=True)
class AStarResult:
    path: list[GridPoint]
    expanded: list[GridPoint]
    cost: float


def astar(
    grid: np.ndarray,
    start: GridPoint,
    goal: GridPoint,
    *,
    allow_unknown: bool = False,
    allow_diagonal: bool = True,
    prevent_corner_cutting: bool = True,
    cost_grid: np.ndarray | None = None,
) -> AStarResult:
    """Occupancy Grid에서 A* 경로를 계산합니다.

    cost_grid를 주면 각 셀에 진입할 때 move_cost에 cost_grid[y, x]를
    더해서 계산한다. 예를 들어 장애물에서 멀수록 값이 작아지는 격자를
    주면, 그냥 최단 경로가 아니라 "여유 있는 곳은 가운데로, 좁은 곳만
    어쩔 수 없이 스치는" 경로를 고를 수 있다. 단위는 move_cost(1.0/sqrt2)
    와 같은 스케일이어야 한다.
    """

    height, width = grid.shape

    def valid(point: GridPoint) -> bool:
        x, y = point
        if not (0 <= x < width and 0 <= y < height):
            return False
        value = int(grid[y, x])
        if value >= 65:
            return False
        return allow_unknown or value >= 0

    if not valid(start):
        raise ValueError(f"start is blocked or out of bounds: {start}")
    if not valid(goal):
        raise ValueError(f"goal is blocked or out of bounds: {goal}")

    moves = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0)]
    if allow_diagonal:
        root2 = math.sqrt(2.0)
        moves += [(1, 1, root2), (1, -1, root2), (-1, 1, root2), (-1, -1, root2)]

    frontier: list[tuple[float, float, GridPoint]] = [(0.0, 0.0, start)]
    came_from: dict[GridPoint, GridPoint] = {}
    g_score: dict[GridPoint, float] = {start: 0.0}
    expanded: list[GridPoint] = []
    closed: set[GridPoint] = set()

    while frontier:
        _, current_g, current = heapq.heappop(frontier)
        if current in closed:
            continue
        closed.add(current)
        expanded.append(current)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return AStarResult(path, expanded, current_g)

        cx, cy = current
        for dx, dy, move_cost in moves:
            nxt = (cx + dx, cy + dy)
            if not valid(nxt) or nxt in closed:
                continue
            if dx != 0 and dy != 0 and prevent_corner_cutting:
                if not valid((cx + dx, cy)) or not valid((cx, cy + dy)):
                    continue
            step_cost = move_cost
            if cost_grid is not None:
                step_cost += float(cost_grid[nxt[1], nxt[0]])
            tentative = current_g + step_cost
            if tentative >= g_score.get(nxt, math.inf):
                continue
            came_from[nxt] = current
            g_score[nxt] = tentative
            heuristic = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
            heapq.heappush(frontier, (tentative + heuristic, tentative, nxt))
    return AStarResult([], expanded, math.inf)


def line_is_free(grid: np.ndarray, a: GridPoint, b: GridPoint, *, allow_unknown: bool = False) -> bool:
    height, width = grid.shape
    for x, y in bresenham(a[0], a[1], b[0], b[1]):
        if not (0 <= x < width and 0 <= y < height):
            return False
        value = int(grid[y, x])
        if value >= 65 or (value < 0 and not allow_unknown):
            return False
    return True


def reduce_waypoints(grid: np.ndarray, path: Sequence[GridPoint], *, allow_unknown: bool = False) -> list[GridPoint]:
    if len(path) <= 2:
        return list(path)
    waypoints = [path[0]]
    anchor = 0
    probe = 2
    while probe < len(path):
        if not line_is_free(grid, path[anchor], path[probe], allow_unknown=allow_unknown):
            waypoints.append(path[probe - 1])
            anchor = probe - 1
        probe += 1
    waypoints.append(path[-1])
    return waypoints


def grid_path_to_world(
    path: Iterable[GridPoint],
    *,
    resolution_m: float,
    origin_x_m: float = 0.0,
    origin_y_m: float = 0.0,
) -> list[tuple[float, float]]:
    return [
        (origin_x_m + (x + 0.5) * resolution_m, origin_y_m + (y + 0.5) * resolution_m)
        for x, y in path
    ]


def nearest_path_index(pose: Pose2D, path: Sequence[tuple[float, float]], start_index: int = 0) -> int:
    if not path:
        return -1
    start_index = max(0, min(start_index, len(path) - 1))
    return min(
        range(start_index, len(path)),
        key=lambda index: euclidean((pose.x, pose.y), path[index]),
    )


def lookahead_target_index(
    pose: Pose2D,
    path: Sequence[tuple[float, float]],
    lookahead_m: float,
    *,
    start_index: int = 0,
) -> int:
    if not path:
        return -1
    nearest = nearest_path_index(pose, path, start_index)
    accumulated = 0.0
    target = nearest
    for index in range(nearest, len(path) - 1):
        accumulated += euclidean(path[index], path[index + 1])
        target = index + 1
        if accumulated >= lookahead_m:
            break
    return target


def pure_pursuit_command(
    pose: Pose2D,
    path: Sequence[tuple[float, float]],
    *,
    lookahead_m: float = 0.25,
    speed_mps: float = 0.10,
    max_omega_rps: float = 1.5,
    start_index: int = 0,
) -> tuple[float, float, int]:
    if not path:
        return 0.0, 0.0, -1
    target_index = lookahead_target_index(pose, path, lookahead_m, start_index=start_index)
    target_x, target_y = path[target_index]
    # Near the end of the path there may be less than lookahead_m of path left, so
    # lookahead_target_index saturates at the final waypoint -- the *actual* distance
    # to that point can be much smaller than lookahead_m. The curvature law below is
    # only correct if L is the true distance to the target, so use that instead of the
    # nominal lookahead_m; using lookahead_m unconditionally makes the commanded turn
    # too gentle once the target is close, and the robot orbits the goal instead of
    # converging onto it.
    actual_lookahead_m = max(euclidean((pose.x, pose.y), (target_x, target_y)), 1e-3)
    alpha = wrap_angle(math.atan2(target_y - pose.y, target_x - pose.x) - pose.theta)
    omega = 2.0 * speed_mps * math.sin(alpha) / actual_lookahead_m
    omega = clamp(omega, -max_omega_rps, max_omega_rps)
    adjusted_speed = speed_mps
    if abs(alpha) > 1.0:
        adjusted_speed *= 0.25
    elif abs(alpha) > 0.55:
        adjusted_speed *= 0.55
    return adjusted_speed, omega, target_index


def pure_pursuit_wheels(
    pose: Pose2D,
    path: Sequence[tuple[float, float]],
    *,
    lookahead_m: float = 0.25,
    speed_mps: float = 0.10,
    max_omega_rps: float = 1.5,
    max_wheel_percent: float = 25.0,
    start_index: int = 0,
) -> tuple[float, float, int]:
    linear, omega, target_index = pure_pursuit_command(
        pose,
        path,
        lookahead_m=lookahead_m,
        speed_mps=speed_mps,
        max_omega_rps=max_omega_rps,
        start_index=start_index,
    )
    left, right = twist_to_wheel_percent(linear, omega)
    scale = max(1.0, max(abs(left), abs(right)) / max_wheel_percent)
    return left / scale, right / scale, target_index
