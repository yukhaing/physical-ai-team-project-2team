"""Beagle Integrated Simulator (Windows, no real robot required)

Features
  1. Map setup : preset scenes (default/room/room_exit/corridor) or JSON map file
  2. Teleop    : drive with arrow keys (or WASD), SPACE to stop
  3. SLAM      : odometry (with noise) + LiDAR accumulated Occupancy Grid
                 Turn on --slam to correct pose using scan-to-map matching.

Run examples
  python simulator\\beagle_sim.py                        # default scene
  python simulator\\beagle_sim.py --scene room_exit      # preset scene
  python simulator\\beagle_sim.py --map simulator\\maps\\sample_room.json
  python simulator\\beagle_sim.py --slam                 # scan matching SLAM
  python simulator\\beagle_sim.py --odom-noise 0.15      # larger odometry error

Controls (with the graph window focused)
  Click on the left World view : plan an A* path to that point + Pure Pursuit autonomous driving
                        (autonomous driving is recommended with SLAM on — if it's off, drift
                         causing the robot to get stuck on walls is itself a good learning point)
  G : cancel autonomous driving      Up/Down/Left/Right or W/S/A/D : manual driving   SPACE : stop
  +/- : speed             E : toggle SLAM on/off     N : change noise level
  T : toggle trajectory display     C : clear trajectory        M : save map   R : reset map   Q : quit

GUI launcher (no command needed):  python simulator\\sim_launcher.py
"""
from __future__ import annotations

import argparse
import json
import math
import random
import time
from pathlib import Path
import sys

import matplotlib
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.beagle_safe import MockBeagle, build_scene, Segment  # noqa: E402
from common.geometry import Pose2D, integrate_differential_drive, polar_to_xy, transform_points_mm, twist_to_wheel_percent, wheel_percent_to_mps  # noqa: E402
from common.mapping import GridMeta, OccupancyGridMap, bresenham, inflate_obstacles  # noqa: E402
from common.planning import astar, grid_path_to_world, pure_pursuit_command, reduce_waypoints  # noqa: E402


# ---------------------------------------------------------------- Map loading

def load_map_json(path: str | Path) -> tuple[list[Segment], Pose2D]:
    """JSON map: {"start": [x, y, theta_deg], "walls": [[x1,y1,x2,y2], ...]} (units: m)"""
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    walls = [tuple(map(float, seg)) for seg in data['walls']]
    sx, sy, sdeg = data.get('start', [0.3, 0.3, 0.0])
    return walls, Pose2D(float(sx), float(sy), math.radians(float(sdeg)))


def scene_bounds(segments: list[Segment]) -> tuple[float, float, float, float]:
    if not segments:
        return -1.0, -1.0, 3.0, 3.0
    xs = [v for s in segments for v in (s[0], s[2])]
    ys = [v for s in segments for v in (s[1], s[3])]
    pad = 0.3
    return min(xs) - pad, min(ys) - pad, max(xs) + pad, max(ys) + pad


# ---------------------------------------------------------------- SLAM correction

def scan_match(grid: OccupancyGridMap, pose: Pose2D, points_mm: list[tuple[float, float]],
               search_xy: float = 0.06, search_deg: float = 4.0, step: int = 3) -> Pose2D:
    """A very simple correlative matching: picks the (dx, dy, dtheta) candidate whose
    scan endpoints overlap the most with occupied cells on the map, and uses it to
    correct the pose."""
    if len(points_mm) < 20:
        return pose
    occ = grid.log_odds
    sample = points_mm[::6]
    best = (0.0, 0.0, 0.0)
    best_score = -1e18

    for ddeg in np.linspace(-search_deg, search_deg, 2 * step + 1):
        dth = math.radians(float(ddeg))
        c, s = math.cos(pose.theta + dth), math.sin(pose.theta + dth)
        local = [(x / 1000.0, y / 1000.0) for x, y in sample]
        for dx in np.linspace(-search_xy, search_xy, 2 * step + 1):
            for dy in np.linspace(-search_xy, search_xy, 2 * step + 1):
                score = 0.0
                for lx, ly in local:
                    wx = pose.x + dx + c * lx - s * ly
                    wy = pose.y + dy + s * lx + c * ly
                    gx, gy = grid.world_to_grid(wx, wy)
                    if grid.in_bounds(gx, gy):
                        score += float(occ[gy, gx])
                if score > best_score:
                    best_score = score
                    best = (float(dx), float(dy), dth)
    return Pose2D(pose.x + best[0], pose.y + best[1], pose.theta + best[2])


# ---------------------------------------------------------------- Simulator

class BeagleSimulator:
    def __init__(self, segments: list[Segment], start: Pose2D,
                 odom_noise: float = 0.06, use_slam: bool = False) -> None:
        self.robot = MockBeagle(scene='open')
        self.robot.segments = segments
        self.robot.pose = Pose2D(start.x, start.y, start.theta)
        self.segments = segments
        self.odom_noise = odom_noise
        self.use_slam = use_slam

        x0, y0, x1, y1 = scene_bounds(segments)
        meta = GridMeta(width_m=x1 - x0, height_m=y1 - y0, resolution_m=0.03,
                        origin_x_m=x0, origin_y_m=y0)
        self.grid = OccupancyGridMap(meta)
        self.bounds = (x0, y0, x1, y1)

        # Odometry-estimated pose (separate from the true pose — this is what SLAM corrects)
        self.est_pose = Pose2D(start.x, start.y, start.theta)
        # Real-world-like "systematic" error: left/right wheel effective diameter
        # difference + gyro zero-point bias
        self.bias_left = 1.0 + random.gauss(0.0, odom_noise)
        self.bias_right = 1.0 - random.gauss(0.0, odom_noise * 0.7)
        self.gyro_bias_dps = random.gauss(0.0, 25.0 * odom_noise)
        self.cmd = (0.0, 0.0)
        self.speed = 16.0
        self.true_trail: list[tuple[float, float]] = []
        self.est_trail: list[tuple[float, float]] = []
        self.show_trails = True
        self.running = True
        self.frame = 0
        self.status = 'STOP'

        # Autonomous driving (A* + Pure Pursuit): move to the clicked goal
        self.plan_grid = self._build_plan_grid()
        self.auto_path: list[tuple[float, float]] = []
        self.auto_on = False
        self.goal: tuple[float, float] | None = None
        self._ax_world = None

    # ---------------- Planning grid (rasterize walls + inflate by robot size)
    def _build_plan_grid(self):
        meta = self.grid.meta
        grid = np.zeros((self.grid.height, self.grid.width), dtype=np.int16)

        def to_cell(x, y):
            return (int((x - meta.origin_x_m) / meta.resolution_m),
                    int((y - meta.origin_y_m) / meta.resolution_m))

        for sx, sy, ex, ey in self.segments:
            for gx, gy in bresenham(*to_cell(sx, sy), *to_cell(ex, ey)):
                if 0 <= gx < self.grid.width and 0 <= gy < self.grid.height:
                    grid[gy, gx] = 100
        return inflate_obstacles(grid, radius_cells=4)

    def _nearest_free(self, gx: int, gy: int):
        h, w = self.plan_grid.shape
        for r in range(0, 8):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    nx, ny = gx + dx, gy + dy
                    if 0 <= nx < w and 0 <= ny < h and self.plan_grid[ny, nx] == 0:
                        return nx, ny
        return None

    def set_goal(self, x: float, y: float) -> None:
        """Given a goal in world coordinates, build an A* path and enable autonomous driving."""
        meta = self.grid.meta
        res = meta.resolution_m

        def to_cell(px, py):
            return (int((px - meta.origin_x_m) / res), int((py - meta.origin_y_m) / res))

        start = self._nearest_free(*to_cell(self.est_pose.x, self.est_pose.y))
        goal = self._nearest_free(*to_cell(x, y))
        if start is None or goal is None:
            print('Cannot plan path: start/goal is inside an obstacle'); return
        try:
            result = astar(self.plan_grid, start, goal)
        except ValueError:
            result = None
        path = result.path if result else []
        if not path:
            print('No path found'); self.status = 'NO_PATH'; return
        waypoints = reduce_waypoints(self.plan_grid, path)
        world_pts = grid_path_to_world(
            waypoints, resolution_m=res, origin_x_m=meta.origin_x_m, origin_y_m=meta.origin_y_m
        )
        # Densely interpolate the path so pure pursuit doesn't cut corners
        dense: list[tuple[float, float]] = [world_pts[0]]
        for a, b in zip(world_pts, world_pts[1:]):
            seg_len = math.hypot(b[0] - a[0], b[1] - a[1])
            n = max(1, int(seg_len / 0.06))
            for k in range(1, n + 1):
                dense.append((a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n))
        self.auto_path = dense
        self.goal = (x, y)
        self.auto_on = True
        self._replan_cooldown = 0
        self.status = 'AUTO'
        print(f'A* path generated: {len(path)} cells -> {len(waypoints)} waypoints')

    def on_click(self, event) -> None:
        if event.inaxes is not self._ax_world or event.xdata is None:
            return
        self.set_goal(float(event.xdata), float(event.ydata))

    def _reroll_bias(self) -> None:
        self.bias_left = 1.0 + random.gauss(0.0, self.odom_noise)
        self.bias_right = 1.0 - random.gauss(0.0, self.odom_noise * 0.7)
        self.gyro_bias_dps = random.gauss(0.0, 25.0 * self.odom_noise)

    # ---------------- Controls
    def on_key(self, event) -> None:
        key = (event.key or '').lower()
        speed = self.speed
        turn = min(14.0, speed * 0.75)
        if key in ('up', 'w'):
            self.auto_on = False
            self.cmd = (speed, speed); self.status = 'FORWARD'
        elif key in ('down', 's'):
            self.auto_on = False
            self.cmd = (-speed, -speed); self.status = 'BACKWARD'
        elif key in ('left', 'a'):
            self.auto_on = False
            self.cmd = (-turn, turn); self.status = 'TURN_LEFT'
        elif key in ('right', 'd'):
            self.auto_on = False
            self.cmd = (turn, -turn); self.status = 'TURN_RIGHT'
        elif key == ' ':
            self.auto_on = False
            self.cmd = (0.0, 0.0); self.status = 'STOP'
        elif key == 'g':
            self.auto_on = False
            self.auto_path = []; self.goal = None
            self.cmd = (0.0, 0.0); self.status = 'STOP'
            print('Autonomous driving cancelled')
        elif key in ('+', '='):
            self.speed = min(25.0, self.speed + 2.0); print(f'Speed: {self.speed:.0f}')
        elif key in ('-', '_'):
            self.speed = max(6.0, self.speed - 2.0); print(f'Speed: {self.speed:.0f}')
        elif key == 'e':
            self.use_slam = not self.use_slam
            print('SLAM:', 'ON' if self.use_slam else 'OFF')
        elif key == 'n':
            levels = [0.03, 0.06, 0.10, 0.15]
            idx = min(range(len(levels)), key=lambda i: abs(levels[i] - self.odom_noise))
            self.odom_noise = levels[(idx + 1) % len(levels)]
            self._reroll_bias()
            print(f'Odometry noise: {self.odom_noise:.2f} (bias re-rolled)')
        elif key == 't':
            self.show_trails = not self.show_trails
        elif key == 'c':
            self.true_trail.clear(); self.est_trail.clear()
            print('Trajectory reset')
        elif key == 'm':
            paths = self.grid.save('map_sim')
            print('Map saved:', *paths)
        elif key == 'r':
            self.grid = OccupancyGridMap(self.grid.meta)
            print('Map reset')
        elif key == 'q':
            self.running = False

    # ---------------- Single step
    def step(self, dt: float) -> None:
        # Autonomous driving: Pure Pursuit generates wheel commands based on the estimated pose
        if self.auto_on and self.auto_path:
            self._replan_cooldown = max(0, getattr(self, '_replan_cooldown', 0) - 1)
            if self.robot.front_lidar() < 110 and self._replan_cooldown == 0 and self.goal:
                # Path is blocked -> replan from the current position
                print('Front blocked -> replanning path')
                self.set_goal(*self.goal)
                self._replan_cooldown = 40
            v, omega, _ = pure_pursuit_command(
                self.est_pose, self.auto_path, lookahead_m=0.15, speed_mps=0.08)
            left_pct, right_pct = twist_to_wheel_percent(v, omega)
            self.cmd = (max(-20, min(20, left_pct)), max(-20, min(20, right_pct)))
            gx, gy = self.auto_path[-1]
            if math.hypot(self.est_pose.x - gx, self.est_pose.y - gy) < 0.08:
                self.auto_on = False
                self.cmd = (0.0, 0.0)
                self.status = 'GOAL!'
                print('Goal reached')
            else:
                self.status = 'AUTO'

        # Front collision protection (simulator-level safety)
        front = self.robot.front_lidar()
        left_cmd, right_cmd = self.cmd
        if front < 90 and left_cmd > 0 and right_cmd > 0:
            left_cmd = right_cmd = 0.0
            self.status = 'BLOCKED'
        self.robot.wheels(left_cmd, right_cmd)

        # Noisy odometry (estimated pose drifts, just like on the real robot)
        # Applies both bias (systematic error) and instantaneous jitter together.
        jitter = self.odom_noise * 0.3
        left_mps = wheel_percent_to_mps(left_cmd) * (self.bias_left + random.gauss(0.0, jitter))
        right_mps = wheel_percent_to_mps(right_cmd) * (self.bias_right + random.gauss(0.0, jitter))
        gyro = self.robot.gyroscope_z() + self.gyro_bias_dps + random.gauss(0.0, 1.0)
        self.est_pose = integrate_differential_drive(
            self.est_pose, left_mps, right_mps, dt, gyro_z_dps=gyro)

        # LiDAR + (optional) scan matching correction + map accumulation
        scan = self.robot.lidar()
        points = polar_to_xy(scan)
        if self.use_slam and self.frame > 20 and self.frame % 4 == 0:
            self.est_pose = scan_match(self.grid, self.est_pose, points)
        world = transform_points_mm(points, self.est_pose)
        self.grid.update_endpoints(self.est_pose, world[::3])
        if self.frame % 2 == 0:
            self.true_trail.append((self.robot.pose.x, self.robot.pose.y))
            self.est_trail.append((self.est_pose.x, self.est_pose.y))
            if len(self.true_trail) > 1500:
                del self.true_trail[:200]; del self.est_trail[:200]
        self.frame += 1

    # ---------------- Main loop
    def run(self) -> None:
        plt.ion()
        fig, (ax_world, ax_map) = plt.subplots(1, 2, figsize=(13, 6.5))
        fig.canvas.manager.set_window_title('Beagle Simulator')
        fig.canvas.mpl_connect('key_press_event', self.on_key)
        fig.canvas.mpl_connect('button_press_event', self.on_click)
        self._ax_world = ax_world
        fig.subplots_adjust(bottom=0.13)
        fig.text(0.5, 0.045,
                 'Click on left map = set goal (A* autonomous driving)   G cancel autonomous driving   '
                 'Move: Arrows/WASD   SPACE stop   +/- speed',
                 ha='center', fontsize=9, color='#1A2233')
        fig.text(0.5, 0.015,
                 'E: Toggle SLAM   N: Change noise   T: Show trajectory   C: Clear trajectory   M: Save map   R: Reset map   Q: Quit',
                 ha='center', fontsize=9, color='#4A5568')
        x0, y0, x1, y1 = self.bounds

        previous = time.monotonic()
        while self.running and plt.fignum_exists(fig.number):
            now = time.monotonic(); dt = min(0.15, now - previous); previous = now
            self.step(dt)

            true_pose = self.robot.pose
            scan = self.robot.lidar()
            pts = polar_to_xy(scan)
            world_true = transform_points_mm(pts, true_pose)

            # Left: real world
            ax_world.clear()
            for sx, sy, ex, ey in self.segments:
                ax_world.plot([sx, ex], [sy, ey], color='#33415F', linewidth=3)
            if world_true:
                wx, wy = zip(*world_true)
                ax_world.scatter(wx, wy, s=4, color='#16C3B2', alpha=0.6)
            if self.auto_path:
                px, py = zip(*self.auto_path)
                ax_world.plot(px, py, '--', color='#7A62F6', linewidth=1.6, alpha=0.85, label='A* Path')
                if self.goal:
                    ax_world.plot(self.goal[0], self.goal[1], '*', color='#7A62F6', markersize=15)
            if self.show_trails and len(self.true_trail) > 1:
                tx, ty = zip(*self.true_trail)
                ax_world.plot(tx, ty, color='#D9534F', linewidth=1.4, alpha=0.7, label='True Trajectory')
                ex_, ey_ = zip(*self.est_trail)
                ax_world.plot(ex_, ey_, color='#2C74F5', linewidth=1.4, alpha=0.7,
                              linestyle='--', label='Estimated Trajectory')
            if self.auto_path or (self.show_trails and len(self.true_trail) > 1):
                ax_world.legend(loc='upper right', fontsize=8)
            ax_world.plot(true_pose.x, true_pose.y, 'o', color='#D9534F', markersize=10)
            ax_world.arrow(true_pose.x, true_pose.y,
                           0.12 * math.cos(true_pose.theta), 0.12 * math.sin(true_pose.theta),
                           head_width=0.045, color='#D9534F')
            ax_world.plot(self.est_pose.x, self.est_pose.y, 'x', color='#2C74F5', markersize=9)
            ax_world.set_xlim(x0, x1); ax_world.set_ylim(y0, y1)
            ax_world.set_aspect('equal'); ax_world.grid(True, alpha=0.3)
            err = math.hypot(true_pose.x - self.est_pose.x, true_pose.y - self.est_pose.y)
            ax_world.set_title(f'World | {self.status} | err={err * 100:.1f} cm | '
                               f'noise={self.odom_noise:.2f} | speed={self.speed:.0f} | '
                               f'{"SLAM ON" if self.use_slam else "SLAM OFF"}')

            # Right: map built by the robot
            ax_map.clear()
            occ = self.grid.occupancy()
            image = np.full(occ.shape, 0.65)
            image[occ == 0] = 1.0
            image[occ >= 65] = 0.0
            ax_map.imshow(image, cmap='gray', origin='lower', vmin=0, vmax=1,
                          extent=(x0, x1, y0, y1))
            if self.auto_path:
                px, py = zip(*self.auto_path)
                ax_map.plot(px, py, '--', color='#7A62F6', linewidth=1.4, alpha=0.85)
            ax_map.plot(self.est_pose.x, self.est_pose.y, 'x', color='#2C74F5', markersize=9)
            ax_map.set_title('Occupancy Grid (M=save, R=reset)')
            ax_map.set_aspect('equal')

            fig.canvas.draw_idle()
            plt.pause(0.03)
        plt.close(fig)
        print('Simulator exit')


# ---------------------------------------------------------------- Map editor

def run_editor(save_path: str) -> None:
    """Click twice = one wall. U=undo last wall, S=save, Q=quit.
    Right-click before drawing the first wall = set robot start position."""
    walls: list[list[float]] = []
    start = [0.3, 0.3, 0.0]
    pending: list[tuple[float, float]] = []

    plt.ion()
    fig, ax = plt.subplots(figsize=(8, 8))
    fig.canvas.manager.set_window_title('Beagle Map Editor')
    state = {'running': True}

    def redraw() -> None:
        ax.clear()
        for x1, y1, x2, y2 in walls:
            ax.plot([x1, x2], [y1, y2], color='#33415F', linewidth=3)
        for px, py in pending:
            ax.plot(px, py, '+', color='#D9534F', markersize=12)
        ax.plot(start[0], start[1], 'o', color='#16C3B2', markersize=12)
        ax.set_xlim(-0.2, 3.2); ax.set_ylim(-0.2, 3.2)
        ax.set_aspect('equal'); ax.grid(True, alpha=0.4)
        ax.set_title(f'Click x2=wall | Right-click=start | U=undo S=save Q=quit  (walls: {len(walls)})')
        fig.canvas.draw_idle()

    def on_click(event) -> None:
        if event.inaxes != ax or event.xdata is None:
            return
        if event.button == 3:
            start[0], start[1] = float(event.xdata), float(event.ydata)
        else:
            pending.append((float(event.xdata), float(event.ydata)))
            if len(pending) == 2:
                (x1, y1), (x2, y2) = pending
                walls.append([round(x1, 3), round(y1, 3), round(x2, 3), round(y2, 3)])
                pending.clear()
        redraw()

    def on_key(event) -> None:
        key = (event.key or '').lower()
        if key == 'u' and walls:
            walls.pop()
        elif key == 's':
            Path(save_path).write_text(
                json.dumps({'start': start, 'walls': walls}, indent=2), encoding='utf-8')
            print('Saved:', save_path)
        elif key == 'q':
            state['running'] = False
        redraw()

    fig.canvas.mpl_connect('button_press_event', on_click)
    fig.canvas.mpl_connect('key_press_event', on_key)
    redraw()
    while state['running'] and plt.fignum_exists(fig.number):
        plt.pause(0.05)
    plt.close(fig)


# ---------------------------------------------------------------- main

def main() -> None:
    parser = argparse.ArgumentParser(description='Beagle Simulator: map + teleop + click autonomous driving + SLAM')
    parser.add_argument('--scene', default='default',
                        help='default | room | room_exit | corridor | open')
    parser.add_argument('--map', dest='map_file', default=None, help='Path to JSON map file')
    parser.add_argument('--slam', action='store_true', help='Use scan matching pose correction')
    parser.add_argument('--odom-noise', type=float, default=0.06,
                        help='Odometry noise standard deviation (0=perfect, 0.1=large)')
    parser.add_argument('--edit', metavar='SAVE_JSON', default=None,
                        help='Run the map editor and save to the given JSON file')
    args = parser.parse_args()

    if args.edit:
        run_editor(args.edit)
        return

    if args.map_file:
        segments, start = load_map_json(args.map_file)
    else:
        segments, start = build_scene(args.scene)

    sim = BeagleSimulator(segments, start, odom_noise=args.odom_noise, use_slam=args.slam)
    sim.run()


if __name__ == '__main__':
    main()