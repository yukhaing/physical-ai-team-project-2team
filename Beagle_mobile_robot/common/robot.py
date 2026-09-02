from __future__ import annotations

import atexit
import math
import random
import signal
import time
from typing import Any

from .geometry import Pose2D, integrate_velocity, wheel_percent_to_mps
from .lidar import cardinal_distances, sanitize_scan

Segment = tuple[float, float, float, float]


def rectangle_segments(x0: float, y0: float, x1: float, y1: float) -> list[Segment]:
    return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]


def build_scene(name: str) -> tuple[list[Segment], Pose2D]:
    name = name.lower().strip()
    if name == "open":
        return [], Pose2D(0.0, 0.0, 0.0)
    if name == "shuttle":
        # 90cm x 70cm work area. A small obstacle sits on the direct line between
        # the receiving zone (36,37) and the defect zone (75,12) so --dry-run can
        # exercise the reactive avoidance path, not just straight-line pursuit.
        walls = rectangle_segments(0.0, 0.0, 0.90, 0.70)
        walls += rectangle_segments(0.50, 0.20, 0.60, 0.30)
        return walls, Pose2D(0.36, 0.37, 0.0)
    raise ValueError(f"unknown mock scene: {name}")


def ray_segment_distance(x: float, y: float, dx: float, dy: float, segment: Segment) -> float:
    x1, y1, x2, y2 = segment
    sx, sy = x2 - x1, y2 - y1
    denominator = dx * sy - dy * sx
    if abs(denominator) < 1e-12:
        return math.inf
    qx, qy = x1 - x, y1 - y
    t = (qx * sy - qy * sx) / denominator
    u = (qx * dy - qy * dx) / denominator
    if t >= 0.0 and 0.0 <= u <= 1.0:
        return t
    return math.inf


class MockBeagle:
    """A minimal 2D Beagle stand-in for checking code flow and branching without hardware."""

    LEFT_WHEEL = 0
    RIGHT_WHEEL = 1

    def __init__(self, scene: str = "shuttle", *, seed: int = 7) -> None:
        self.scene = scene
        self.segments, self.pose = build_scene(scene)
        self.random = random.Random(seed)
        self.left_percent = 0.0
        self.right_percent = 0.0
        self.last_update = time.monotonic()
        self.left_distance_m = 0.0
        self.right_distance_m = 0.0
        self.lidar_started = False
        self.gyro_bias_dps = 0.35

    def _update(self) -> None:
        now = time.monotonic()
        dt = min(0.15, max(0.0, now - self.last_update))
        self.last_update = now
        left_mps = wheel_percent_to_mps(self.left_percent)
        right_mps = wheel_percent_to_mps(self.right_percent)
        self.pose = integrate_velocity(self.pose, left_mps, right_mps, dt)
        self.left_distance_m += left_mps * dt
        self.right_distance_m += right_mps * dt

    def wheels(self, left: float, right: float) -> None:
        self._update()
        self.left_percent = float(left)
        self.right_percent = float(right)

    def write(self, channel: int, value: float) -> None:
        if channel == self.LEFT_WHEEL:
            self.wheels(value, self.right_percent)
        elif channel == self.RIGHT_WHEEL:
            self.wheels(self.left_percent, value)

    def stop(self) -> None:
        self.wheels(0.0, 0.0)

    def start_lidar(self) -> None:
        self.lidar_started = True

    def wait_until_lidar_ready(self) -> None:
        self.lidar_started = True
        time.sleep(0.02)

    def is_lidar_ready(self) -> bool:
        return self.lidar_started

    def lidar(self) -> list[int]:
        self._update()
        result: list[int] = []
        for degree in range(360):
            theta = self.pose.theta + math.radians(degree)
            dx, dy = math.cos(theta), math.sin(theta)
            distance = min(
                (ray_segment_distance(self.pose.x, self.pose.y, dx, dy, segment) for segment in self.segments),
                default=5.0,
            )
            if not math.isfinite(distance):
                distance = 5.0
            distance += self.random.gauss(0.0, 0.004)
            result.append(int(max(50.0, min(5000.0, distance * 1000.0))))
        return result

    def _cardinal(self) -> dict[str, float]:
        return cardinal_distances(sanitize_scan(self.lidar()))

    def front_lidar(self) -> float:
        return self._cardinal()["front"]

    def rear_lidar(self) -> float:
        return self._cardinal()["rear"]

    def left_lidar(self) -> float:
        return self._cardinal()["left"]

    def right_lidar(self) -> float:
        return self._cardinal()["right"]

    def left_front_lidar(self) -> float:
        return self._cardinal()["front_left"]

    def right_front_lidar(self) -> float:
        return self._cardinal()["front_right"]

    def left_rear_lidar(self) -> float:
        scan = sanitize_scan(self.lidar())
        return cardinal_distances(scan)["rear"]

    def right_rear_lidar(self) -> float:
        scan = sanitize_scan(self.lidar())
        return cardinal_distances(scan)["rear"]

    def left_encoder(self) -> float:
        self._update()
        return self.left_distance_m * 1000.0

    def right_encoder(self) -> float:
        self._update()
        return self.right_distance_m * 1000.0

    def gyroscope_z(self) -> float:
        self._update()
        left_mps = wheel_percent_to_mps(self.left_percent)
        right_mps = wheel_percent_to_mps(self.right_percent)
        angular = (right_mps - left_mps) / 0.0956
        return math.degrees(angular) + self.gyro_bias_dps + self.random.gauss(0.0, 0.12)

    def gyroscope_x(self) -> float:
        return self.random.gauss(0.0, 0.04)

    def gyroscope_y(self) -> float:
        return self.random.gauss(0.0, 0.04)

    def accelerometer_x(self) -> float:
        return self.random.gauss(0.0, 0.02)

    def accelerometer_y(self) -> float:
        return self.random.gauss(0.0, 0.02)

    def accelerometer_z(self) -> float:
        return 1.0 + self.random.gauss(0.0, 0.015)

    def battery_state(self) -> int:
        return 3

    def signal_strength(self) -> int:
        return -42

    def temperature(self) -> float:
        return 27.0

    def charge_state(self) -> int:
        return 0

    def tilt(self) -> int:
        return -3

    def servo_input_a(self) -> float:
        return 0.0

    def sound(self, name: str, count: int | None = None) -> None:
        print(f"[MOCK] sound={name} count={count}")

    def sound_until_done(self, name: str) -> None:
        print(f"[MOCK] sound={name}")


class SafeBeagle:
    """Wraps the real/mock Beagle with speed limiting, guaranteed stop-on-exit, and dry-run support."""

    def __init__(
        self,
        *,
        dry_run: bool = False,
        max_speed: float = 25.0,
        scene: str = "shuttle",
        port_name: str | None = None,
    ) -> None:
        self.dry_run = dry_run
        self.max_speed = abs(float(max_speed))
        self._closed = False
        if dry_run:
            self.robot: Any = MockBeagle(scene=scene)
        else:
            try:
                from roboid import Beagle  # type: ignore
            except ImportError as exc:
                raise RuntimeError("Could not import roboid. Run with --dry-run first.") from exc
            self.robot = Beagle(0, port_name) if port_name else Beagle()
        atexit.register(self.stop)
        for sig in (getattr(signal, "SIGINT", None), getattr(signal, "SIGTERM", None)):
            if sig is not None:
                try:
                    signal.signal(sig, self._signal_handler)
                except (ValueError, OSError):
                    pass

    def _signal_handler(self, signum: int, frame: object) -> None:
        self.stop()
        raise KeyboardInterrupt

    def __enter__(self) -> "SafeBeagle":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def wheels(self, left: float, right: float) -> None:
        left = max(-self.max_speed, min(self.max_speed, float(left)))
        right = max(-self.max_speed, min(self.max_speed, float(right)))
        self.robot.wheels(left, right)

    def stop(self) -> None:
        try:
            self.robot.stop()
        except Exception:
            pass

    def close(self) -> None:
        self.stop()
        try:
            self.robot.stop_lidar()
        except Exception:
            pass
        self._closed = True

    def start_lidar(self) -> None:
        self.robot.start_lidar()

    def wait_until_lidar_ready(self) -> None:
        # The Roboid API can leave the mission process alive with cached sensor
        # values after its USB/BLE link disappears.  Do not treat those values
        # as proof that a physical Beagle is present.
        if not self.is_connected():
            return
        if hasattr(self.robot, "wait_until_lidar_ready"):
            self.robot.wait_until_lidar_ready()
        else:
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline:
                if getattr(self.robot, "is_lidar_ready", lambda: True)():
                    return
                time.sleep(0.05)
            raise TimeoutError("LiDAR did not become ready in time.")

    def is_connected(self) -> bool:
        if self.dry_run:
            return True
        roboid = getattr(self.robot, '_roboid', None)
        connector = getattr(roboid, '_connector', None)
        checker = getattr(connector, 'is_connected', None)
        if not callable(checker):
            return False
        try:
            return bool(checker())
        except Exception:
            return False

    def lidar(self) -> list[float]:
        return sanitize_scan(self.robot.lidar())

    def __getattr__(self, name: str) -> Any:
        return getattr(self.robot, name)
