from __future__ import annotations

import math

Scan = list[float]


def best_rotation_offset(scan: Scan, reference: Scan) -> tuple[float, float]:
    """How far the robot has rotated since `reference` was captured, found by
    comparing the two 360-degree LiDAR scans directly -- no wall/map geometry
    needed, just the two scans.

    Both scans are index-based (index 0 = straight ahead of the robot's heading
    at the moment it was captured, index increasing in the +theta direction --
    see common/lidar.py's simulate_scan()). Tries every circular shift between
    them and keeps the one with the lowest mean absolute difference: that shift
    is how many scan-steps the world has "rotated" in the new scan relative to
    the reference, which is exactly how much the robot itself rotated.

    Returns (rotation_rad, match_err_m):
      rotation_rad -- turn the robot by this much (in the +theta direction) to
      bring it back to the pose it was in when `reference` was captured.
      match_err_m -- residual mean-abs-difference at the best-fit shift (small
      = confident match, large = scans don't line up well at any rotation).
    """
    n = len(scan)
    if n == 0 or len(reference) != n:
        raise ValueError("scan and reference must be the same non-zero length")
    best_shift, best_err = 0, math.inf
    for k in range(n):
        err = sum(abs(scan[(i + k) % n] - reference[i]) for i in range(n)) / n
        if err < best_err:
            best_err = err
            best_shift = k
    rotation_rad = (best_shift / n) * 2.0 * math.pi
    if rotation_rad > math.pi:
        rotation_rad -= 2.0 * math.pi
    return rotation_rad, best_err
