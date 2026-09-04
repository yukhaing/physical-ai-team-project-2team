from __future__ import annotations

from dataclasses import dataclass
import math


def wrap_angle(theta: float) -> float:
    """Wrap an angle in radians into (-pi, pi]."""
    return (theta + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class Pose2D:
    x: float
    y: float
    theta: float  # radians, 0 = facing +x

    def distance_to(self, x: float, y: float) -> float:
        return math.hypot(x - self.x, y - self.y)

    def heading_to(self, x: float, y: float) -> float:
        return math.atan2(y - self.y, x - self.x)

