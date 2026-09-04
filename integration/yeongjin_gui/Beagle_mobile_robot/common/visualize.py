from __future__ import annotations

import math

import matplotlib.pyplot as plt
from matplotlib.patches import Circle

Segment = tuple[float, float, float, float]

STATE_COLORS = {
    "WAIT_SIGNAL": "#888888",
    "GOTO_DEFECT": "#2C74F5",
    "DWELL_DEFECT": "#D9534F",
    "GOTO_RECEIVING": "#3B9C4C",
    "ALIGN_RECEIVING": "#C9A227",
    "SENSOR_FAIL": "#B00020",
}


class ShuttleVisualizer:
    """Live top-down plot of the shuttle mission: walls, zones, robot pose, and trail."""

    def __init__(self, settings, segments: list[Segment]) -> None:
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(6, 5))
        self.ax.set_aspect("equal")
        pad = 0.1
        self.ax.set_xlim(-pad, settings.boundary_x_m + pad)
        self.ax.set_ylim(-pad, settings.boundary_y_m + pad)
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")

        for x1, y1, x2, y2 in segments:
            self.ax.plot([x1, x2], [y1, y2], color="#333333", linewidth=2)

        rx, ry = settings.receiving_m
        dx, dy = settings.defect_m
        self.ax.plot(rx, ry, "s", color="#2C74F5", markersize=12, label="receiving")
        self.ax.plot(dx, dy, "s", color="#D9534F", markersize=12, label="defect")
        self.ax.legend(loc="upper right", fontsize=8)

        self._trail_x: list[float] = []
        self._trail_y: list[float] = []
        (self._trail_line,) = self.ax.plot([], [], "-", color="#7A62F6", linewidth=1.2, alpha=0.7)
        self._robot_dot = Circle((rx, ry), 0.03, color="#3B9C4C", zorder=5)
        self.ax.add_patch(self._robot_dot)
        (self._heading_line,) = self.ax.plot([], [], "-", color="#111111", linewidth=2, zorder=6)
        self._status_text = self.ax.text(
            0.02, 0.98, "", transform=self.ax.transAxes, va="top", fontsize=9, family="monospace"
        )
        self.fig.tight_layout()
        self.fig.canvas.draw()
        self._background = self.fig.canvas.copy_from_bbox(self.ax.bbox)
        self._dynamic_artists = (self._trail_line, self._robot_dot, self._heading_line, self._status_text)

    def update(self, row: dict) -> None:
        x, y = row["x_cm"] / 100.0, row["y_cm"] / 100.0
        theta = math.radians(row["theta_deg"])
        self._trail_x.append(x)
        self._trail_y.append(y)
        self._trail_line.set_data(self._trail_x, self._trail_y)
        self._robot_dot.center = (x, y)
        self._robot_dot.set_color(STATE_COLORS.get(row["state"], "#3B9C4C"))
        hx, hy = x + 0.06 * math.cos(theta), y + 0.06 * math.sin(theta)
        self._heading_line.set_data([x, hx], [y, hy])
        self._status_text.set_text(
            f"t={row['t']:6.1f}s\nstate={row['state']}\navoid={row['avoid']}\nsignal={row['signal']}"
        )
        self.fig.canvas.restore_region(self._background)
        for artist in self._dynamic_artists:
            self.ax.draw_artist(artist)
        self.fig.canvas.blit(self.ax.bbox)
        self.fig.canvas.flush_events()

    def finish(self, result: str) -> None:
        self.ax.set_title(f"mission result: {result}")
        self.fig.canvas.draw_idle()
        plt.ioff()
        plt.show()
