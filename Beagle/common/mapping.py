from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np

from .geometry import Pose2D, polar_to_xy, transform_points_mm


def bresenham(x0: int, y0: int, x1: int, y1: int) -> list[tuple[int, int]]:
    """두 격자점 사이의 정수 셀 목록."""

    points: list[tuple[int, int]] = []
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        doubled = 2 * error
        if doubled >= dy:
            error += dy
            x0 += sx
        if doubled <= dx:
            error += dx
            y0 += sy
    return points


@dataclass(slots=True)
class GridMeta:
    width_m: float = 6.0
    height_m: float = 5.0
    resolution_m: float = 0.05
    origin_x_m: float = -1.0
    origin_y_m: float = -1.0

    @property
    def width_cells(self) -> int:
        return int(math.ceil(self.width_m / self.resolution_m))

    @property
    def height_cells(self) -> int:
        return int(math.ceil(self.height_m / self.resolution_m))


class OccupancyGridMap:
    """간단한 log-odds Occupancy Grid."""

    def __init__(
        self,
        meta: GridMeta | None = None,
        *,
        free_delta: float = -0.35,
        occupied_delta: float = 0.85,
        min_score: float = -4.0,
        max_score: float = 4.0,
    ) -> None:
        self.meta = meta or GridMeta()
        if self.meta.resolution_m <= 0:
            raise ValueError("resolution_m must be positive")
        self.width = self.meta.width_cells
        self.height = self.meta.height_cells
        self.free_delta = float(free_delta)
        self.occupied_delta = float(occupied_delta)
        self.min_score = float(min_score)
        self.max_score = float(max_score)
        self.score = np.zeros((self.height, self.width), dtype=np.float32)
        self.observed = np.zeros((self.height, self.width), dtype=bool)

    @property
    def log_odds(self):
        """이전 저장소와의 호환 별칭. score 배열을 그대로 반환합니다."""
        return self.score

    def world_to_grid(self, x_m: float, y_m: float) -> tuple[int, int]:
        gx = int(math.floor((x_m - self.meta.origin_x_m) / self.meta.resolution_m))
        gy = int(math.floor((y_m - self.meta.origin_y_m) / self.meta.resolution_m))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> tuple[float, float]:
        return (
            self.meta.origin_x_m + (gx + 0.5) * self.meta.resolution_m,
            self.meta.origin_y_m + (gy + 0.5) * self.meta.resolution_m,
        )

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def update_ray(self, start_world_m: tuple[float, float], end_world_m: tuple[float, float]) -> None:
        sx, sy = self.world_to_grid(*start_world_m)
        ex, ey = self.world_to_grid(*end_world_m)
        cells = bresenham(sx, sy, ex, ey)
        if not cells:
            return
        for gx, gy in cells[:-1]:
            if self.in_bounds(gx, gy):
                self.score[gy, gx] = max(self.min_score, self.score[gy, gx] + self.free_delta)
                self.observed[gy, gx] = True
        ex, ey = cells[-1]
        if self.in_bounds(ex, ey):
            self.score[ey, ex] = min(self.max_score, self.score[ey, ex] + self.occupied_delta)
            self.observed[ey, ex] = True

    def update_endpoints(self, pose: Pose2D, endpoints_world_m: Iterable[tuple[float, float]]) -> None:
        for endpoint in endpoints_world_m:
            self.update_ray((pose.x, pose.y), endpoint)

    def update_scan(
        self,
        pose: Pose2D,
        scan_mm: Sequence[float],
        *,
        sample_step: int = 2,
        max_mm: float = 4500.0,
    ) -> int:
        local = polar_to_xy(scan_mm, max_mm=max_mm)
        world = transform_points_mm(local[:: max(1, sample_step)], pose)
        self.update_endpoints(pose, world)
        return len(world)

    def occupancy(
        self,
        *,
        occupied_threshold: float = 0.8,
        free_threshold: float = -0.5,
        uncertain_value: int = 50,
    ) -> np.ndarray:
        output = np.full((self.height, self.width), -1, dtype=np.int16)
        output[self.observed & (self.score <= free_threshold)] = 0
        output[self.observed & (self.score >= occupied_threshold)] = 100
        uncertain = self.observed & (output == -1)
        output[uncertain] = uncertain_value
        return output

    def save(
        self,
        stem: str | Path,
        *,
        occupied_threshold: float = 0.8,
        free_threshold: float = -0.5,
    ) -> dict[str, Path]:
        """CSV, PNG, PGM, YAML, metadata JSON을 저장합니다."""

        from PIL import Image
        import json

        stem = Path(stem)
        stem.parent.mkdir(parents=True, exist_ok=True)
        occ = self.occupancy(
            occupied_threshold=occupied_threshold,
            free_threshold=free_threshold,
        )
        csv_path = stem.with_suffix(".csv")
        png_path = stem.with_suffix(".png")
        pgm_path = stem.with_suffix(".pgm")
        yaml_path = stem.with_suffix(".yaml")
        json_path = stem.with_suffix(".json")
        np.savetxt(csv_path, occ, fmt="%d", delimiter=",")

        image = np.full(occ.shape, 205, dtype=np.uint8)
        image[occ == 0] = 254
        image[occ >= 65] = 0
        image[(occ > 0) & (occ < 65)] = 140
        flipped = np.flipud(image)
        Image.fromarray(flipped, mode="L").save(png_path)
        Image.fromarray(flipped, mode="L").save(pgm_path)

        yaml_path.write_text(
            "\n".join(
                [
                    f"image: {pgm_path.name}",
                    f"resolution: {self.meta.resolution_m}",
                    f"origin: [{self.meta.origin_x_m}, {self.meta.origin_y_m}, 0.0]",
                    "negate: 0",
                    "occupied_thresh: 0.65",
                    "free_thresh: 0.25",
                    "mode: trinary",
                    "",
                ]
            ),
            encoding="utf-8",
        )
        json_path.write_text(
            json.dumps(
                {
                    "width_cells": self.width,
                    "height_cells": self.height,
                    "width_m": self.meta.width_m,
                    "height_m": self.meta.height_m,
                    "resolution_m": self.meta.resolution_m,
                    "origin_x_m": self.meta.origin_x_m,
                    "origin_y_m": self.meta.origin_y_m,
                    "occupied_threshold_score": occupied_threshold,
                    "free_threshold_score": free_threshold,
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )
        return {"csv": csv_path, "png": png_path, "pgm": pgm_path, "yaml": yaml_path, "json": json_path}


def load_map_csv(path: str | Path) -> np.ndarray:
    grid = np.loadtxt(path, delimiter=",", dtype=np.int16)
    if grid.ndim != 2:
        raise ValueError("map CSV must be a 2-D array")
    return grid


def save_grid_png(grid: np.ndarray, path: str | Path) -> Path:
    from PIL import Image

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    image = np.full(grid.shape, 205, dtype=np.uint8)
    image[grid == 0] = 254
    image[grid >= 65] = 0
    image[(grid > 0) & (grid < 65)] = 140
    Image.fromarray(np.flipud(image), mode="L").save(path)
    return path


def inflate_obstacles(grid: np.ndarray, radius_cells: int = 2, *, unknown_is_blocked: bool = True) -> np.ndarray:
    """장애물을 로봇 반경만큼 팽창시킵니다."""

    blocked = grid >= 65
    if unknown_is_blocked:
        blocked |= grid < 0
    radius_cells = max(0, int(radius_cells))
    if radius_cells:
        try:
            from scipy.ndimage import binary_dilation

            y, x = np.ogrid[-radius_cells : radius_cells + 1, -radius_cells : radius_cells + 1]
            structure = x * x + y * y <= radius_cells * radius_cells
            blocked = binary_dilation(blocked, structure=structure)
        except ImportError:
            padded = np.pad(blocked, radius_cells, mode="constant", constant_values=True)
            result = np.zeros_like(blocked)
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    if dx * dx + dy * dy <= radius_cells * radius_cells:
                        result |= padded[
                            radius_cells + dy : radius_cells + dy + blocked.shape[0],
                            radius_cells + dx : radius_cells + dx + blocked.shape[1],
                        ]
            blocked = result
    return np.where(blocked, 100, np.where(grid < 0, -1, 0)).astype(np.int16)
