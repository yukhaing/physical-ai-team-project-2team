from __future__ import annotations

import numpy as np


def sample_grid() -> np.ndarray:
    """A*/inflation 실습용 60x40 Occupancy Grid."""

    grid = np.zeros((40, 60), dtype=np.int16)
    grid[5:35, 28] = 100
    grid[20:25, 28] = 0
    grid[8:12, 8:22] = 100
    grid[28:32, 38:52] = 100
    return grid
