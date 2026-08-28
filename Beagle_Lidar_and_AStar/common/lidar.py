from __future__ import annotations

import math

from common.geometry import Pose2D

Segment = tuple[float, float, float, float]  # x1, y1, x2, y2


def rectangle_segments(x0: float, y0: float, x1: float, y1: float) -> list[Segment]:
    """The four wall segments of the axis-aligned room rectangle [x0,x1] x [y0,y1]."""
    return [
        (x0, y0, x1, y0),
        (x1, y0, x1, y1),
        (x1, y1, x0, y1),
        (x0, y1, x0, y0),
    ]


def _ray_segment_range(px: float, py: float, dx: float, dy: float, seg: Segment) -> float | None:
    """Distance from (px,py) along ray direction (dx,dy) to its intersection with
    `seg`, or None if the ray misses the segment (behind it or off to the side)."""
    x1, y1, x2, y2 = seg
    sx, sy = x2 - x1, y2 - y1
    denom = dx * sy - dy * sx
    if abs(denom) < 1e-9:
        return None
    t = ((x1 - px) * sy - (y1 - py) * sx) / denom
    u = ((x1 - px) * dy - (y1 - py) * dx) / denom
    if t < 0 or u < 0 or u > 1:
        return None
    return t


def simulate_scan(
    pose: Pose2D, segments: list[Segment], num_rays: int = 72, max_range_m: float = 5.0
) -> list[float]:
    """Simulate a 360-degree LiDAR scan from `pose` by ray-casting against
    `segments`. Index 0 is straight ahead (robot heading), going counter-clockwise.
    """
    scan = []
    for i in range(num_rays):
        angle = pose.theta + (i * 2.0 * math.pi / num_rays)
        dx, dy = math.cos(angle), math.sin(angle)
        best = max_range_m
        for seg in segments:
            t = _ray_segment_range(pose.x, pose.y, dx, dy, seg)
            if t is not None and t < best:
                best = t
        scan.append(best)
    return scan
