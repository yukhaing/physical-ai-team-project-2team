from __future__ import annotations

import math
import time
from statistics import median
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


def _median_window(values: list[float], center: int, half_window: int) -> float:
    """Median of `values` in a circular window of +/-half_window around `center`
    -- smooths out a single-sample sensor glitch (e.g. a stray 2m reading among
    neighbors reading 0.15m) instead of letting it get picked up directly."""
    n = len(values)
    window = [values[(center + offset) % n] for offset in range(-half_window, half_window + 1)]
    return median(window)


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

    def start_lidar(self, timeout_s: float = 10.0, resend_interval_s: float = 3.0) -> None:
        """Poll for LiDAR readiness with our own deadline instead of calling the
        SDK's wait_until_lidar_ready() directly -- that call blocks forever (no
        internal timeout) when nothing is actually connected, which would hang
        any script that runs it with no way to tell "disconnected" from "slow".

        Raised from 4.0 to 10.0 (2026-09-01): reliably timed out on the very
        first run right after power-cycling the physical robot -- confirmed
        the robot WAS connected and became ready shortly after, just past the
        old deadline. That alone wasn't always enough though: seen timing out
        again even at 10.0s the same day, immediately after a successful
        serial connect ("Beagle[0] Connected") -- i.e. genuinely no reply to
        start_lidar() at all, not just a slow one, suggesting the initial
        command itself can be dropped (BLE/serial hiccup right after a fresh
        connection) rather than just needing more patience. Re-sends
        start_lidar() to the SDK every `resend_interval_s` while still
        waiting, in case a resend gets through where the first one didn't --
        cheap and harmless if the first one DID arrive (the SDK command is
        idempotent, already-spinning LiDAR just keeps spinning)."""
        self.robot.start_lidar()
        deadline = time.monotonic() + timeout_s
        next_resend = time.monotonic() + resend_interval_s
        while time.monotonic() < deadline:
            if self.robot.is_lidar_ready():
                return
            if time.monotonic() >= next_resend:
                self.robot.start_lidar()
                next_resend = time.monotonic() + resend_interval_s
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
        common/scan_align.py and common/localize.py work unchanged on real data.

        No index shift applied. An initial half-array shift (assuming raw index
        0 = rear) was tried first, based on a quirk documented for a different
        robot/LiDAR unit (see Beagle/common/robot.py) -- but a real-hardware
        test on THIS robot (2026-08-31: driving straight forward at the
        reference heading measurably DECREASED estimate_pose_offset()'s fitted
        dx instead of increasing it) showed index 0 was still ~180deg off from
        true front even with that shift applied. Since best_rotation_offset()'s
        heading correction is self-referential (compares live scans only
        against a reference captured the same way) it works regardless of this
        labeling, which is why heading alignment always converged correctly
        throughout debugging even while position estimates stayed wrong -- but
        estimate_translation_offset()/estimate_pose_offset() use cos/sin(angle)
        directly, which DOES need index 0 to be true physical front. Removing
        the shift entirely undoes that extra half-turn (two half-shifts cancel
        out). If position estimates are still off-axis after this, don't
        reintroduce the shift blindly -- re-run the diagnostic
        (scripts/04b_measure_only.py, move a known small distance at 3 o'clock
        heading, check which of dx/dy responds and with which sign) instead."""
        raw = self.robot.lidar()
        cleaned_mm = [_clean_distance_mm(v) for v in raw]
        front_relative_mm = [v if v is not None else float(LIDAR_MAX_MM) for v in cleaned_mm]
        n = len(front_relative_mm)
        step = max(1, n // num_rays)
        half_window = max(1, step // 2)
        sampled_mm = [
            _median_window(front_relative_mm, i, half_window) for i in range(0, n, step)
        ][:num_rays]
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

    def turn_by_angle(
        self, delta_rad: float, turn_percent: float = 10.0, poll_dt: float = 0.05, max_time_s: float = 5.0
    ) -> float:
        """In-place turn closed on the gyro (not open-loop timing): keeps turning
        until the gyro-integrated angle reaches `delta_rad` (signed, +theta
        direction) or `max_time_s` elapses. Intermediate/coarse -- callers that
        need a precise final heading should re-check with LiDAR afterward
        (see scripts/03_calibrate_and_realign.py's realign loop), this is just
        for repositioning moves in between LiDAR checks. Returns the actual
        gyro-integrated angle turned (radians)."""
        if abs(delta_rad) < 1e-6:
            return 0.0
        direction = 1.0 if delta_rad > 0 else -1.0
        turned = 0.0
        self.wheels(-direction * turn_percent, direction * turn_percent)
        start = time.monotonic()
        previous = start
        try:
            while abs(turned) < abs(delta_rad) and time.monotonic() - start < max_time_s:
                time.sleep(poll_dt)
                now = time.monotonic()
                turned += math.radians(self.gyro_z_dps()) * (now - previous)
                previous = now
        finally:
            self.stop()
        return turned

    def drive_forward(
        self, distance_m: float, forward_percent: float = 10.0, poll_dt: float = 0.05,
        max_time_s: float | None = None, min_expected_mps: float = 0.015, abort_backward_m: float = 0.02,
    ) -> float:
        """Drive straight closed on the encoders (not open-loop timing): keeps
        driving until the average of both wheels' measured distance reaches
        `distance_m` or `max_time_s` elapses. `distance_m` must be >= 0 --
        callers needing the opposite direction should turn_by_angle() first
        instead of driving backward, since a differential-drive robot can't
        strafe. Returns the actual encoder-measured distance driven (meters).

        Two safety nets: `max_time_s` defaults to a cap scaled to `distance_m`
        (via `min_expected_mps`) instead of one fixed generous timeout, so a
        wrong forward/encoder sign can't drive for a long fixed duration; and
        if the encoder-measured distance goes backward by more than
        `abort_backward_m`, it stops immediately instead of continuing toward
        max_time_s -- see scripts/00b_check_drive_direction.py (forward sign
        confirmed correct 2026-08-31, so that net is now a backstop, not the
        primary defense). The scaled cap itself was originally ceilinged at
        4.0s -- fine for find_pose()'s few-cm corrections, but it cut a 20.5cm
        leg off at ~14.7cm (confirmed 2026-08-31, scripts/07_goto_defect_checkpointed.py)
        well before reaching `distance_m`. Raised to 30.0s, generous enough for
        any single leg in this room at the conservative min_expected_mps."""
        if distance_m <= 0.0:
            return 0.0
        if max_time_s is None:
            max_time_s = min(30.0, max(0.5, distance_m / min_expected_mps))
        self.encoder_delta_m()  # reset the running baseline before starting to move
        traveled = 0.0
        self.wheels(forward_percent, forward_percent)
        start = time.monotonic()
        try:
            while traveled < distance_m and time.monotonic() - start < max_time_s:
                time.sleep(poll_dt)
                d_left, d_right = self.encoder_delta_m()
                traveled += (d_left + d_right) / 2.0
                if traveled <= -abort_backward_m:
                    print(f"  [drive_forward] WARNING: encoder says it moved {traveled * 100:.1f}cm "
                          f"backward instead of forward -- stopping early. forward_percent may need "
                          f"to be negative on this robot (check with scripts/00b_check_drive_direction.py).")
                    break
        finally:
            self.stop()
        return traveled

    def drive_backward(
        self, distance_m: float, backward_percent: float = 10.0, poll_dt: float = 0.05,
        max_time_s: float | None = None, min_expected_mps: float = 0.015, abort_forward_m: float = 0.02,
    ) -> float:
        """Mirror of drive_forward() for reverse -- same encoder-closed loop and
        safety nets, just negated. Letting find_pose() choose forward-or-backward
        (whichever needs less turning to face) instead of always turning to face
        forward keeps the total rotation per correction smaller -- see
        common/dock.py's find_pose(), since every turn on real hardware adds a
        bit of real positional drift (imperfect in-place pivot)."""
        if distance_m <= 0.0:
            return 0.0
        if max_time_s is None:
            max_time_s = min(4.0, max(0.5, distance_m / min_expected_mps))
        self.encoder_delta_m()
        traveled = 0.0
        self.wheels(-backward_percent, -backward_percent)
        start = time.monotonic()
        try:
            while traveled < distance_m and time.monotonic() - start < max_time_s:
                time.sleep(poll_dt)
                d_left, d_right = self.encoder_delta_m()
                traveled += -(d_left + d_right) / 2.0
                if traveled <= -abort_forward_m:
                    print(f"  [drive_backward] WARNING: encoder says it moved {traveled * 100:.1f}cm "
                          f"forward instead of backward -- stopping early.")
                    break
        finally:
            self.stop()
        return traveled

    def battery_state(self) -> int:
        return self.robot.battery_state()

    def signal_strength(self) -> int:
        return self.robot.signal_strength()
