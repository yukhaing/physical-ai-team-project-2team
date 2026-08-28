from __future__ import annotations

import math
import time
from typing import Any

# Measured facts about this specific physical robot (from Beagle/'s prior
# hardware work) -- not algorithm choices, so reused as-is rather than
# re-derived. See Beagle/common/robot.py and Beagle/scripts/12_calibrate_encoders.py.
INVALID_SENTINEL = 0xFFFF
LIDAR_MIN_MM = 50
LIDAR_MAX_MM = 5000
ENCODER_M_PER_COUNT_LEFT = 0.00012207
ENCODER_M_PER_COUNT_RIGHT = 0.00012264
GYRO_BIAS_DPS = 0.35
DEFAULT_MAX_WHEEL_PERCENT = 15.0  # conservative for in-place turning; raise once trusted


def _clean_distance_mm(value: int | float) -> float | None:
    """One raw LiDAR sample -> distance in mm, or None if invalid/out of range."""
    try:
        raw = int(value)
    except (TypeError, ValueError):
        return None
    if raw == INVALID_SENTINEL or raw <= 0:
        return None
    if (raw & 0xFF00) == 0xFF00 or raw < LIDAR_MIN_MM:
        decoded = raw & 0x00FF
        if LIDAR_MIN_MM <= decoded <= LIDAR_MAX_MM:
            raw = decoded
    return float(raw) if LIDAR_MIN_MM <= raw <= LIDAR_MAX_MM else None


class Hardware:
    """Thin wrapper over the vendor `roboid.Beagle` SDK -- this SDK object *is*
    the wire protocol for this physical robot (it owns the USB connection
    internally). Adds only what's needed to match common/lidar.py's simulated
    scan format: wheel-percent safety clamp, the raw-LiDAR-index-is-rear fix,
    invalid-sample cleanup, and encoder-count -> meters conversion.
    """

    def __init__(self, max_wheel_percent: float = DEFAULT_MAX_WHEEL_PERCENT) -> None:
        from roboid import Beagle  # type: ignore

        self.robot: Any = Beagle()
        self.max_wheel_percent = abs(max_wheel_percent)
        self._last_left_count: float | None = None
        self._last_right_count: float | None = None

    def start_lidar(self, timeout_s: float = 4.0) -> None:
        """Poll for LiDAR readiness with our own deadline instead of calling the
        SDK's wait_until_lidar_ready() directly -- that call blocks forever (no
        internal timeout) when nothing is actually connected, which would hang
        any script that runs it with no way to tell "disconnected" from "slow"."""
        self.robot.start_lidar()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.robot.is_lidar_ready():
                return
            time.sleep(0.05)
        raise TimeoutError(
            f"LiDAR not ready after {timeout_s:.1f}s -- is the robot actually connected/powered on?"
        )

    def wheels(self, left_percent: float, right_percent: float) -> None:
        left = max(-self.max_wheel_percent, min(self.max_wheel_percent, left_percent))
        right = max(-self.max_wheel_percent, min(self.max_wheel_percent, right_percent))
        self.robot.wheels(left, right)

    def stop(self) -> None:
        self.robot.stop()

    def scan(self, num_rays: int = 72) -> list[float]:
        """Front-relative distance scan in meters, index 0 = straight ahead,
        matching common/lidar.py's simulate_scan() format/convention exactly so
        common/scan_align.py and common/localize.py work unchanged on real data."""
        raw = self.robot.lidar()
        cleaned_mm = [_clean_distance_mm(v) for v in raw]
        cleaned_mm = [v if v is not None else float(LIDAR_MAX_MM) for v in cleaned_mm]
        half = len(cleaned_mm) // 2
        front_relative_mm = cleaned_mm[half:] + cleaned_mm[:half]  # raw index0 = rear (measured quirk)
        n = len(front_relative_mm)
        step = max(1, n // num_rays)
        sampled_mm = [front_relative_mm[i] for i in range(0, n, step)][:num_rays]
        return [v / 1000.0 for v in sampled_mm]

    def encoder_delta_m(self) -> tuple[float, float]:
        """Meters each wheel has moved since the previous call (0,0 the first time)."""
        left_count = self.robot.left_encoder()
        right_count = self.robot.right_encoder()
        if self._last_left_count is None:
            self._last_left_count, self._last_right_count = left_count, right_count
            return 0.0, 0.0
        d_left = (left_count - self._last_left_count) * ENCODER_M_PER_COUNT_LEFT
        d_right = (right_count - self._last_right_count) * ENCODER_M_PER_COUNT_RIGHT
        self._last_left_count, self._last_right_count = left_count, right_count
        return d_left, d_right

    def gyro_z_dps(self) -> float:
        return self.robot.gyroscope_z() - GYRO_BIAS_DPS

    def battery_state(self) -> int:
        return self.robot.battery_state()

    def signal_strength(self) -> int:
        return self.robot.signal_strength()
