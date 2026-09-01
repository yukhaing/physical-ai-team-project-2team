from __future__ import annotations

import math

from common.geometry import Pose2D
from common.lidar import Segment, simulate_scan

Point = tuple[float, float]
Rect = tuple[float, float, float, float]  # xmin, ymin, xmax, ymax


def detect_obstacle_points(
    pose: Pose2D, scan: list[float], wall_segments: list[Segment],
    clearance_m: float = 0.03, max_range_m: float = 5.0,
) -> list[Point]:
    """Given a scan taken at a KNOWN (LiDAR-verified) pose, find world-frame
    points where the measured range is meaningfully shorter than what the
    known wall boundary alone would produce there -- those points are
    something solid that isn't a wall (e.g. an OMX arm base), not measurement
    noise. `pose` must already be trustworthy (e.g. just confirmed via
    common/dock.py's find_pose()) -- this does not estimate pose itself."""
    n = len(scan)
    expected = simulate_scan(pose, wall_segments, num_rays=n, max_range_m=max_range_m)
    step = 2.0 * math.pi / n
    points: list[Point] = []
    for i in range(n):
        measured = scan[i]
        if measured >= max_range_m - 0.05:
            continue  # no return -- can't be a near obstacle
        if measured < expected[i] - clearance_m:
            angle = pose.theta + i * step
            points.append((pose.x + measured * math.cos(angle), pose.y + measured * math.sin(angle)))
    return points


def cluster_points(points: list[Point], cluster_radius_m: float = 0.08) -> list[list[Point]]:
    """Greedy clustering: group points within cluster_radius_m of a cluster's
    running centroid. Good enough for a handful of compact obstacles (e.g. one
    or two OMX arm bases) in an otherwise empty room -- not a general-purpose
    clustering algorithm."""
    clusters: list[list[Point]] = []
    for p in points:
        placed = False
        for cluster in clusters:
            cx = sum(q[0] for q in cluster) / len(cluster)
            cy = sum(q[1] for q in cluster) / len(cluster)
            if math.hypot(p[0] - cx, p[1] - cy) <= cluster_radius_m:
                cluster.append(p)
                placed = True
                break
        if not placed:
            clusters.append([p])
    return clusters


def obstacle_rects_from_points(
    points: list[Point], cluster_radius_m: float = 0.08, padding_m: float = 0.05, min_points: int = 3,
) -> list[Rect]:
    """Cluster raw obstacle points and return one padded bounding-box Rect per
    cluster, ready to pass to common/planning.py's astar_path(obstacles=...).
    Clusters smaller than min_points are dropped as likely noise (a single
    stray short reading), not a real obstacle."""
    rects: list[Rect] = []
    for cluster in cluster_points(points, cluster_radius_m):
        if len(cluster) < min_points:
            continue
        xs = [p[0] for p in cluster]
        ys = [p[1] for p in cluster]
        rects.append((min(xs) - padding_m, min(ys) - padding_m, max(xs) + padding_m, max(ys) + padding_m))
    return rects
