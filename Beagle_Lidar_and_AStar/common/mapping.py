from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from common.geometry import Pose2D

Point = tuple[float, float]


def scan_to_world_points(
    scan: list[float], pose: Pose2D, max_range_m: float = 5.0, valid_margin_m: float = 0.05,
) -> list[Point]:
    """Convert one scan (taken at a KNOWN, verified `pose`) into world-frame
    (x, y) points -- every ray with a real return, not just rays that differ
    from a wall/obstacle model (unlike common/obstacles.py's
    detect_obstacle_points(), which only flags rays closer than a predicted
    model -- this keeps every point, so it can build the map itself rather
    than compare against one)."""
    n = len(scan)
    step = 2.0 * math.pi / n
    points: list[Point] = []
    for i, r in enumerate(scan):
        if r >= max_range_m - valid_margin_m:
            continue
        angle = pose.theta + i * step
        points.append((pose.x + r * math.cos(angle), pose.y + r * math.sin(angle)))
    return points


def build_map_from_references(scans_with_poses: list[tuple[list[float], Pose2D]]) -> list[Point]:
    """Merge several (scan, pose) captures -- each pose must already be known/
    verified (e.g. course_config.json's zone coordinates + heading_deg, the
    same pose scripts/03_calibrate_and_realign.py --calibrate captured that
    zone's reference scan at) -- into one combined world-frame point cloud.
    No SLAM-style simultaneous localization happens here: every pose is
    already trusted, this only does the coordinate conversion and merge."""
    points: list[Point] = []
    for scan, pose in scans_with_poses:
        points.extend(scan_to_world_points(scan, pose))
    return points


def predict_scan_from_points(
    pose: Pose2D, map_points: list[Point], num_rays: int = 72, max_range_m: float = 5.0,
) -> list[float]:
    """The point-cloud-map equivalent of common/lidar.py's simulate_scan(): for
    a candidate `pose`, predict what a scan from there would read, using the
    real merged map (build_map_from_references()) instead of an idealized
    wall-rectangle + crude obstacle-box model. For each ray direction, bins
    every map point by its angle from `pose` and keeps the nearest one in that
    bin -- this is what common/localize.py's checkpoint_correct() and
    common/obstacles.py's detect_obstacle_points() were previously missing:
    a model precise enough to match real LiDAR data closely (both were
    confirmed 2026-08-31 to leave match_err elevated, ~90-200mm+, likely
    because a few crude padded bounding boxes don't represent the OMX arms'
    true shape well enough for fine matching)."""
    step = 2.0 * math.pi / num_rays
    best = [max_range_m] * num_rays
    for px, py in map_points:
        dx, dy = px - pose.x, py - pose.y
        r = math.hypot(dx, dy)
        if r >= max_range_m:
            continue
        angle = math.atan2(dy, dx) - pose.theta
        bin_index = round(angle / step) % num_rays
        if r < best[bin_index]:
            best[bin_index] = r
    return best


@dataclass
class DistanceField:
    """Precomputed "distance to nearest map point" lookup table over a fine
    grid covering the room (+ a margin). Building this is the expensive part
    (brute-force nearest-neighbor for every grid cell) -- build it ONCE per
    map (build_distance_field()) and reuse it for every localize_from_map()
    call, instead of redoing a nearest-neighbor search from scratch for every
    candidate pose during a search (confirmed 2026-08-31: doing the latter,
    even vectorized, took ~80s per call -- the (positions x scan_rays x
    map_points) intermediate array is enormous.  Looking a point up in an
    already-built field is just array indexing, independent of how many map
    points went into building it)."""

    field: np.ndarray  # (W, H) distance in meters
    x0: float
    y0: float
    resolution_m: float

    def lookup(self, wx: np.ndarray, wy: np.ndarray) -> np.ndarray:
        ix = np.clip(np.round((wx - self.x0) / self.resolution_m).astype(np.int64), 0, self.field.shape[0] - 1)
        iy = np.clip(np.round((wy - self.y0) / self.resolution_m).astype(np.int64), 0, self.field.shape[1] - 1)
        return self.field[ix, iy]


def build_distance_field(
    map_points: list[Point], boundary_w_m: float, boundary_h_m: float,
    resolution_m: float = 0.01, margin_m: float = 0.15,
) -> DistanceField:
    """Build a DistanceField covering [-margin_m, boundary_w_m+margin_m] x
    [-margin_m, boundary_h_m+margin_m] at `resolution_m` spacing. The margin
    lets candidate poses/rays slightly outside the nominal room boundary
    during a search still get a meaningful (large, not clipped-to-edge)
    distance instead of being silently pinned to the room edge."""
    map_arr = np.asarray(map_points, dtype=np.float64)  # (M, 2)
    xs = np.arange(-margin_m, boundary_w_m + margin_m + 1e-9, resolution_m)
    ys = np.arange(-margin_m, boundary_h_m + margin_m + 1e-9, resolution_m)
    XX, YY = np.meshgrid(xs, ys, indexing="ij")  # (W, H)
    dx = XX[:, :, None] - map_arr[None, None, :, 0]
    dy = YY[:, :, None] - map_arr[None, None, :, 1]
    field = np.hypot(dx, dy).min(axis=2)
    return DistanceField(field, xs[0], ys[0], resolution_m)


def localize_from_map(
    scan: list[float], distance_field: DistanceField, guess_x: float, guess_y: float,
    pos_search_radius_m: float = 0.15, pos_step_m: float = 0.01, theta_steps: int = 180,
    max_range_m: float = 5.0,
) -> tuple[Pose2D, float]:
    """Wide-radius grid search over (x, y, theta) for the pose whose scan best
    matches the observed `scan` against `distance_field` (build_distance_field())
    -- the genuine "where am I" tool. Unlike common/dock.py's find_pose() (a
    fast linearized estimate that only trusts small offsets from an
    already-known pose), this searches a real position radius and the full
    360deg heading, so it doesn't carry that small-offset assumption.

    For each candidate heading, rotates the scan's own points into world frame
    at every candidate (x, y) at once, and scores each candidate pose by the
    MEDIAN of the distance field's value at each of those points (median, not
    mean, so that a handful of scan points landing outside the mapped area --
    e.g. through a gap -- don't drag down an otherwise-good match). Returns
    (best_pose, match_err_m)."""
    n = len(scan)
    step = 2.0 * math.pi / n
    local_pts = [
        (r * math.cos(i * step), r * math.sin(i * step))
        for i, r in enumerate(scan) if r < max_range_m - 0.05
    ]
    if not local_pts:
        return Pose2D(guess_x, guess_y, 0.0), math.inf
    local = np.asarray(local_pts, dtype=np.float64)  # (K, 2)

    steps = max(1, round(pos_search_radius_m / pos_step_m))
    xs = guess_x + np.arange(-steps, steps + 1) * pos_step_m
    ys = guess_y + np.arange(-steps, steps + 1) * pos_step_m
    XX, YY = np.meshgrid(xs, ys, indexing="ij")  # (X, Y)
    thetas = np.linspace(0.0, 2.0 * math.pi, theta_steps, endpoint=False)

    best_score = math.inf
    best_x = guess_x
    best_y = guess_y
    best_theta = 0.0
    for theta in thetas:
        c, s = math.cos(theta), math.sin(theta)
        rx = c * local[:, 0] - s * local[:, 1]  # (K,)
        ry = s * local[:, 0] + c * local[:, 1]
        wx = XX[:, :, None] + rx[None, None, :]  # (X, Y, K)
        wy = YY[:, :, None] + ry[None, None, :]
        d = distance_field.lookup(wx, wy)  # (X, Y, K)
        score = np.median(d, axis=2)  # (X, Y)
        idx = np.unravel_index(np.argmin(score), score.shape)
        if score[idx] < best_score:
            best_score = float(score[idx])
            best_x, best_y = float(XX[idx]), float(YY[idx])
            best_theta = theta
    return Pose2D(best_x, best_y, best_theta), best_score
