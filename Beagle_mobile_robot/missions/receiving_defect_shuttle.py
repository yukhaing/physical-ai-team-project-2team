from __future__ import annotations

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

"""Receiving-zone <-> defect-zone box shuttle mission.

The robot idles at the receiving zone until it gets a "box placed" signal
from the OMX arm, drives to the defect zone, waits there 5 seconds, then
drives back and waits again. All physical/tuning numbers (zone positions,
work-area boundary, lidar thresholds, speeds) come from config/course_config.json
so the team can retune the mission without touching this file.

A reactive obstacle-avoidance layer runs underneath the goal-seeking
controller: whenever something gets close it takes over the wheel command,
then hands control back once clear -- so an obstacle in the way gets routed
around instead of stopping the mission.
"""

import argparse
import math
import time
from dataclasses import dataclass

from common.config import load_config
from common.geometry import Pose2D, euclidean, integrate_velocity, twist_to_wheel_percent, wheel_percent_to_mps, wrap_angle
from common.lidar import cardinal_distances, valid_fraction
from common.logging_utils import CsvLogger
from common.motion import calibrate_gyro_bias
from common.robot import SafeBeagle, rectangle_segments
from common.comm import StatusClient, TriggerServer


@dataclass(slots=True)
class MissionSettings:
    boundary_x_m: float
    boundary_y_m: float
    receiving_m: tuple[float, float]
    defect_m: tuple[float, float]
    dwell_s: float
    goal_tolerance_m: float
    goal_stable_samples: int
    wheel_radius_m: float
    wheel_base_m: float
    max_rpm: float
    emergency_mm: float
    turn_mm: float
    side_clear_mm: float
    backup_clear_mm: float
    turn_margin_mm: float
    turn_percent: float
    turn_forward_bias_percent: float
    straight_after_clear_s: float
    straight_percent: float
    nav_speed_mps: float
    nav_lookahead_m: float
    nav_slowdown_alpha: float
    nav_max_percent: float
    box_flag_path: str
    simulate_signal_interval_s: float


def load_settings(config_path: str | None = None) -> MissionSettings:
    cfg = load_config(config_path)
    robot_cfg, lidar_cfg = cfg["robot"], cfg["lidar"]
    avoid_cfg, nav_cfg = cfg["avoidance"], cfg["navigation"]
    boundary_cfg, zones_cfg, mission_cfg = cfg["boundary"], cfg["zones"], cfg["mission"]

    settings = MissionSettings(
        boundary_x_m=boundary_cfg["x_m"],
        boundary_y_m=boundary_cfg["y_m"],
        receiving_m=(zones_cfg["receiving"]["x_m"], zones_cfg["receiving"]["y_m"]),
        defect_m=(zones_cfg["defect"]["x_m"], zones_cfg["defect"]["y_m"]),
        dwell_s=mission_cfg["dwell_seconds"],
        goal_tolerance_m=nav_cfg["goal_tolerance_m"],
        goal_stable_samples=int(nav_cfg["goal_stable_samples"]),
        wheel_radius_m=robot_cfg["wheel_radius_m"],
        wheel_base_m=robot_cfg["wheel_base_m"],
        max_rpm=robot_cfg["max_rpm"],
        emergency_mm=lidar_cfg["emergency_mm"],
        turn_mm=lidar_cfg["turn_mm"],
        side_clear_mm=lidar_cfg["side_clear_mm"],
        backup_clear_mm=lidar_cfg["backup_clear_mm"],
        turn_margin_mm=lidar_cfg["turn_margin_mm"],
        turn_percent=avoid_cfg["turn_percent"],
        turn_forward_bias_percent=avoid_cfg["turn_forward_bias_percent"],
        straight_after_clear_s=avoid_cfg["straight_after_clear_s"],
        straight_percent=avoid_cfg["straight_percent"],
        nav_speed_mps=nav_cfg["speed_mps"],
        nav_lookahead_m=nav_cfg["lookahead_m"],
        nav_slowdown_alpha=nav_cfg["slowdown_alpha_rad"],
        nav_max_percent=nav_cfg["max_wheel_percent"],
        box_flag_path=mission_cfg["box_flag_path"],
        simulate_signal_interval_s=mission_cfg["simulate_signal_interval_s"],
    )
    for label, zone in (("receiving", settings.receiving_m), ("defect", settings.defect_m)):
        if not (0.0 <= zone[0] <= settings.boundary_x_m and 0.0 <= zone[1] <= settings.boundary_y_m):
            raise ValueError(f"'{label}' zone {zone} falls outside the {settings.boundary_x_m}x{settings.boundary_y_m}m boundary in config")
    return settings


# ---- reactive obstacle-avoidance safety layer ------------------------------
class ReactiveAvoider:
    """Turns/backs away from close obstacles; CLEAR means goal-seeking may drive.

    A pure "turn away, then immediately re-aim at the goal" reflex fails
    whenever the goal sits behind the obstacle: the goal controller swings
    heading straight back at it within a tick or two, so the robot
    re-triggers avoidance at the same spot forever. `post_clear_state()`
    below adds a short commitment to drive straight on the turned-away
    heading before goal-seeking is allowed to recompute -- that is what
    actually buys the lateral clearance needed to get around something.
    """

    def __init__(self, settings: MissionSettings) -> None:
        self.settings = settings
        self._last_turn = "TURN_LEFT"
        self._straight_until = 0.0

    def raw_state(self, front: float, front_left: float, front_right: float, left: float, right: float, rear: float, valid_ratio: float) -> str:
        s = self.settings
        if valid_ratio < 0.5:
            return "SENSOR_FAIL"

        can_turn_left = front_left > s.side_clear_mm and left > s.side_clear_mm
        can_turn_right = front_right > s.side_clear_mm and right > s.side_clear_mm
        can_backup = rear > s.backup_clear_mm

        def choose_turn() -> str:
            if can_turn_left and can_turn_right:
                if left - right > s.turn_margin_mm:
                    return "TURN_LEFT"
                if right - left > s.turn_margin_mm:
                    return "TURN_RIGHT"
                return self._last_turn
            return "TURN_LEFT" if can_turn_left else "TURN_RIGHT"

        if front < s.emergency_mm:
            if can_backup:
                return "BACKUP"
            if can_turn_left or can_turn_right:
                self._last_turn = choose_turn()
                return self._last_turn
            return "STUCK"

        if front < s.turn_mm:
            if can_turn_left or can_turn_right:
                self._last_turn = choose_turn()
                return self._last_turn
            return "BACKUP" if can_backup else "STUCK"

        return "CLEAR"

    def state(self, front: float, front_left: float, front_right: float, left: float, right: float, rear: float, valid_ratio: float, *, now: float) -> str:
        raw = self.raw_state(front, front_left, front_right, left, right, rear, valid_ratio)
        if raw != "CLEAR":
            self._straight_until = now + self.settings.straight_after_clear_s
            return raw
        if now < self._straight_until:
            return "STRAIGHT"
        return "CLEAR"

    def wheel_command(self, state: str) -> tuple[float, float]:
        s = self.settings
        if state == "TURN_LEFT":
            return s.turn_forward_bias_percent - s.turn_percent, s.turn_forward_bias_percent + s.turn_percent
        if state == "TURN_RIGHT":
            return s.turn_forward_bias_percent + s.turn_percent, s.turn_forward_bias_percent - s.turn_percent
        if state == "BACKUP":
            return -s.turn_percent, -s.turn_percent
        if state == "STRAIGHT":
            return s.straight_percent, s.straight_percent
        return 0.0, 0.0  # STUCK / SENSOR_FAIL


# ---- goal-seeking (single-point pursuit) -----------------------------------
def goal_command(pose: Pose2D, goal: tuple[float, float], settings: MissionSettings) -> tuple[float, float, float]:
    gx, gy = goal
    dist = euclidean((pose.x, pose.y), goal)
    alpha = wrap_angle(math.atan2(gy - pose.y, gx - pose.x) - pose.theta)
    omega = 0.5 * math.atan(2.0 * math.sin(alpha) / settings.nav_lookahead_m)
    speed = settings.nav_speed_mps
    if abs(alpha) > settings.nav_slowdown_alpha:
        speed = max(0.0, settings.nav_speed_mps - abs(alpha) / 20.0)
    speed = min(speed, max(0.01, dist) * 0.6)  # ease in on final approach, avoid overshoot
    left, right = twist_to_wheel_percent(speed, omega, wheel_base_m=settings.wheel_base_m, wheel_radius_m=settings.wheel_radius_m, max_rpm=settings.max_rpm)
    biggest = max(abs(left), abs(right))
    if biggest > settings.nav_max_percent:
        scale = settings.nav_max_percent / biggest
        left *= scale
        right *= scale
    return left, right, dist


def update_pose(pose: Pose2D, commanded_left: float, commanded_right: float, dt: float, gyro_dps: float, gyro_bias: float, settings: MissionSettings) -> Pose2D:
    gyro = gyro_dps - gyro_bias
    if commanded_left == 0.0 and commanded_right == 0.0 and abs(gyro) <= 1.0:
        gyro = 0.0
    left_mps = wheel_percent_to_mps(commanded_left, wheel_radius_m=settings.wheel_radius_m, max_rpm=settings.max_rpm)
    right_mps = wheel_percent_to_mps(commanded_right, wheel_radius_m=settings.wheel_radius_m, max_rpm=settings.max_rpm)
    return integrate_velocity(pose, left_mps, right_mps, dt, wheel_base_m=settings.wheel_base_m, gyro_z_dps=gyro, gyro_weight=1.0)


# ---- box-placed signal from the OMX arm ------------------------------------
class BoxSignal:
    """Box-placed handoff signal from the OMX robot.

    Wraps a TriggerServer that OMX connects to and sends
    {"event": "box_placed"}\\n on. Once seen, the signal latches True and
    stays True until reset() clears it -- reset()/is_set() is the only
    interface the mission loop depends on, so nothing else changes.
    """

    def __init__(self, server: TriggerServer) -> None:
        self.server = server
        self._got_signal = False

    def reset(self) -> None:
        self._got_signal = False

    def is_set(self) -> bool:
        for message in self.server.poll():
            if message.get("event") == "box_placed":
                self._got_signal = True
        return self._got_signal



# ---- mission state machine -------------------------------------------------
@dataclass(slots=True)
class ShuttleData:
    avoidance: str
    goal_reached: bool
    box_signal: bool
    dwell_elapsed: float


def next_state(state: str, data: ShuttleData, dwell_s: float) -> str:
    if data.avoidance == "SENSOR_FAIL":
        return "SENSOR_FAIL"
    if state == "WAIT_SIGNAL":
        return "GOTO_DEFECT" if data.box_signal else "WAIT_SIGNAL"
    if state == "GOTO_DEFECT":
        return "DWELL_DEFECT" if data.goal_reached else "GOTO_DEFECT"
    if state == "DWELL_DEFECT":
        return "GOTO_RECEIVING" if data.dwell_elapsed >= dwell_s else "DWELL_DEFECT"
    if state == "GOTO_RECEIVING":
        return "WAIT_SIGNAL" if data.goal_reached else "GOTO_RECEIVING"
    return state  # SENSOR_FAIL is terminal


class ShuttleMission:
    def __init__(self, robot: SafeBeagle, box_signal: BoxSignal, settings: MissionSettings, *, status_client: StatusClient | None = None) -> None:
        self.robot = robot
        self.box_signal = box_signal
        self.settings = settings
        self.status_client = status_client
        self.avoider = ReactiveAvoider(settings)
        self.state = "WAIT_SIGNAL"
        self.pose = Pose2D(settings.receiving_m[0], settings.receiving_m[1], 0.0)
        self.commanded = (0.0, 0.0)
        self.dwell_started: float | None = None
        self.goal_stable = 0
        self.cycles_done = 0
        self._gyro_bias = 0.0
        self._previous_time = 0.0
        self._started_before = False

    def _send(self, text: str, **extra: object) -> None:
        if self.status_client is not None:
            self.status_client.send_status(text, **extra)

    def start(self) -> None:
        self._gyro_bias = calibrate_gyro_bias(self.robot)
        self.box_signal.reset()
        self._previous_time = time.monotonic()

    def _goal(self) -> tuple[float, float] | None:
        if self.state == "GOTO_DEFECT":
            return self.settings.defect_m
        if self.state == "GOTO_RECEIVING":
            return self.settings.receiving_m
        return None

    def step(self) -> dict[str, object]:
        s = self.settings
        now = time.monotonic()
        dt = max(0.001, now - self._previous_time)
        self._previous_time = now

        gyro_dps = float(self.robot.gyroscope_z())
        self.pose = update_pose(self.pose, *self.commanded, dt, gyro_dps, self._gyro_bias, s)

        scan = self.robot.lidar()
        ratio = valid_fraction(scan)
        f = cardinal_distances(scan)
        avoidance = self.avoider.state(f["front"], f["front_left"], f["front_right"], f["left"], f["right"], f["rear"], ratio, now=now)

        goal = self._goal()
        if goal is not None:
            nav_left, nav_right, dist = goal_command(self.pose, goal, s)
            self.goal_stable = self.goal_stable + 1 if dist <= s.goal_tolerance_m else 0
        else:
            nav_left = nav_right = dist = 0.0
        goal_reached = self.goal_stable >= s.goal_stable_samples

        dwell_elapsed = (now - self.dwell_started) if self.dwell_started is not None else 0.0
        data = ShuttleData(avoidance, goal_reached, self.box_signal.is_set(), dwell_elapsed)
        new_state = next_state(self.state, data, s.dwell_s)
        if new_state != self.state:
            self.goal_stable = 0
            if self.state == "WAIT_SIGNAL" and new_state == "GOTO_DEFECT":
                self._send("출발", to="defect_zone")
                self._send("defect 존으로 이동중")
            elif self.state == "GOTO_DEFECT" and new_state == "DWELL_DEFECT":
                self._send("도착", at="defect_zone")
            elif self.state == "DWELL_DEFECT" and new_state == "GOTO_RECEIVING":
                self._send("출발", to="receiving_zone")
                self._send("대기 존으로 이동중")
            elif self.state == "GOTO_RECEIVING" and new_state == "WAIT_SIGNAL":
                if self._started_before:
                    self._send("도착", at="receiving_zone")
                self._started_before = True

            if new_state == "DWELL_DEFECT":
                self.dwell_started = now
            if new_state == "WAIT_SIGNAL":
                if self.state == "GOTO_RECEIVING":
                    self.cycles_done += 1
                self.box_signal.reset()
            self.state = new_state

        if self.state == "SENSOR_FAIL":
            left = right = 0.0
        elif self.state in ("WAIT_SIGNAL", "DWELL_DEFECT"):
            left = right = 0.0
        elif avoidance != "CLEAR":
            left, right = self.avoider.wheel_command(avoidance)
        else:
            left, right = nav_left, nav_right

        self.robot.wheels(left, right)
        self.commanded = (left, right)
        return {
            "t": now,
            "x_cm": self.pose.x * 100.0,
            "y_cm": self.pose.y * 100.0,
            "theta_deg": math.degrees(self.pose.theta),
            "state": self.state,
            "avoid": avoidance,
            "signal": data.box_signal,
            "dist_cm": dist * 100.0,
            "cmd_l": left,
            "cmd_r": right,
        }


def run_mission(robot: SafeBeagle, box_signal: BoxSignal, settings: MissionSettings, *, duration_s: float, cycles: int, output_path: str, visualizer=None) -> str:
    mission = ShuttleMission(robot, box_signal, settings)
    mission.start()
    start = time.monotonic()
    deadline = start + duration_s if duration_s > 0 else math.inf

    with CsvLogger(output_path, ["t", "x_cm", "y_cm", "theta_deg", "state", "avoid", "signal", "dist_cm", "cmd_l", "cmd_r"]) as log:
        while time.monotonic() < deadline:
            row = mission.step()
            row["t"] = row["t"] - start
            log.write(row)
            print(
                f"t={row['t']:6.1f} state={row['state']:14s} avoid={row['avoid']:11s} "
                f"pose=({row['x_cm']:5.1f},{row['y_cm']:5.1f})cm dist={row['dist_cm']:5.1f}cm "
                f"cmd=({row['cmd_l']:5.1f},{row['cmd_r']:5.1f})"
            )
            if visualizer is not None:
                visualizer.update(row)
            if mission.state == "SENSOR_FAIL":
                robot.stop()
                print("SENSOR_FAIL: lidar data unreliable, stopping mission")
                return "SENSOR_FAIL"
            if cycles > 0 and mission.cycles_done >= cycles and mission.state == "WAIT_SIGNAL":
                robot.stop()
                return "CYCLES_DONE"
            time.sleep(0.08)
    robot.stop()
    return mission.state


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--scene", default="shuttle")
    parser.add_argument("--config", default=None, help="path to course_config.json (default: config/course_config.json)")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds; 0 = run until Ctrl+C or --cycles")
    parser.add_argument("--cycles", type=int, default=0, help="stop after N receiving->defect->receiving round trips (0 = unlimited)")
    parser.add_argument("--box-flag", default=None, help="override the box-placed flag file path from config")
    parser.add_argument("--simulate-signal-interval", type=float, default=None, help="dry-run only: override the auto-trigger interval from config")
    parser.add_argument("--output", default="logs/shuttle_mission.csv")
    parser.add_argument("--visualize", action="store_true", help="show a live top-down plot of the mission")
    args = parser.parse_args()

    settings = load_settings(args.config)
    box_flag_path = args.box_flag or settings.box_flag_path
    simulate_after_s = args.simulate_signal_interval if args.simulate_signal_interval is not None else settings.simulate_signal_interval_s
    
    # TEMP for step 2 testing — step 4 replaces this with a --trigger-port CLI flag
    trigger_server = TriggerServer(port=8765)
    trigger_server.start()
    box_signal = BoxSignal(trigger_server)

    with SafeBeagle(dry_run=args.dry_run, scene=args.scene, max_speed=settings.nav_max_percent) as robot:
        robot.start_lidar()
        robot.wait_until_lidar_ready()

        visualizer = None
        if args.visualize:
            from common.visualize import ShuttleVisualizer

            segments = robot.robot.segments if args.dry_run else rectangle_segments(0.0, 0.0, settings.boundary_x_m, settings.boundary_y_m)
            visualizer = ShuttleVisualizer(settings, segments)

        result = run_mission(robot, box_signal, settings, duration_s=args.duration, cycles=args.cycles, output_path=args.output, visualizer=visualizer)
        print("mission result:", result)
        if visualizer is not None:
            visualizer.finish(result)


if __name__ == "__main__":
    main()
