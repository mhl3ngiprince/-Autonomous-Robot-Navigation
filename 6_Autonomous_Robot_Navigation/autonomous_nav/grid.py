"""Pure-Python occupancy grid with a real inverse sensor model.

No ROS dependency, so it is fully unit-testable on any machine. The ROS node
wraps this.

Model
-----
Each cell stores a **log-odds** value L. A lidar hit raises the odds; free
cells along the beam lower them. Probabilities are recovered with
``p = 1 - 1/(1+e^L)`` and cells are classified obstacle / free / unknown.

Updates use Bresenham line traversal so the exact cells the beam crosses are
updated (the original code used a sloppy integer loop that skipped cells).
"""
from __future__ import annotations

import math

import numpy as np

OCCUPIED = 100
FREE = 0
UNKNOWN = -1

# Log-odds increments (tuned for a typical 2-D lidar)
L_OCC = 0.85
L_FREE = -0.40
L_MIN = -4.0
L_MAX = 4.0


class OccupancyGrid:
    def __init__(self, size=400, resolution=0.05):
        self.size = int(size)
        self.res = float(resolution)
        self.log_odds = np.zeros((self.size, self.size), dtype=np.float32)
        self.origin = (self.size // 2, self.size // 2)   # world (0,0) cell

    # ----------------------------------------------------------- coordinates --
    def world_to_cell(self, x, y):
        cx = self.origin[0] + int(round(x / self.res))
        cy = self.origin[1] + int(round(y / self.res))
        return cx, cy

    def cell_to_world(self, cx, cy):
        x = (cx - self.origin[0]) * self.res
        y = (cy - self.origin[1]) * self.res
        return x, y

    def in_bounds(self, cx, cy):
        return 0 <= cx < self.size and 0 <= cy < self.size

    # --------------------------------------------------------------- updating -
    def integrate_scan(self, pose, scan):
        """Fuse one lidar sweep.

        ``pose`` = (x, y, theta) in metres/radians (robot frame == map frame
        here; the ROS node supplies the real TF pose).
        ``scan`` = iterable of (angle_rad, range_m, hit: bool).
        """
        rx, ry, rtheta = pose
        origin = self.world_to_cell(rx, ry)
        for angle, rng, hit in scan:
            if rng is None or not math.isfinite(rng) or rng <= 0:
                continue
            a = rtheta + angle
            ex = rx + rng * math.cos(a)
            ey = ry + rng * math.sin(a)
            end = self.world_to_cell(ex, ey)
            self._update_beam(origin, end, hit)

    def _update_beam(self, origin, end, hit):
        for (cx, cy) in _bresenham(origin[0], origin[1], end[0], end[1]):
            if not self.in_bounds(cx, cy):
                continue
            is_end = (cx, cy) == end
            if is_end and hit:
                self.log_odds[cy, cx] = min(
                    L_MAX, self.log_odds[cy, cx] + L_OCC)
            else:
                self.log_odds[cy, cx] = max(
                    L_MIN, self.log_odds[cy, cx] + L_FREE)

    # --------------------------------------------------------------- readout --
    def probabilities(self):
        return 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))

    def occupancy_array(self):
        """int8 map in the ROS OccupancyGrid convention."""
        p = self.probabilities()
        out = np.full((self.size, self.size), UNKNOWN, dtype=np.int8)
        touched = self.log_odds != 0.0
        out[touched & (p > 0.5)] = OCCUPIED
        out[touched & (p < 0.5)] = FREE
        return out

    def is_blocked(self, cx, cy, inflate=1):
        occ = self.occupancy_array()
        if not self.in_bounds(cx, cy):
            return True
        # inflate obstacles so the robot keeps clearance
        x0, x1 = max(0, cx - inflate), min(self.size, cx + inflate + 1)
        y0, y1 = max(0, cy - inflate), min(self.size, cy + inflate + 1)
        return bool((occ[y0:y1, x0:x1] == OCCUPIED).any())

    def free_mask(self, inflate=1):
        """Boolean mask of cells the robot may occupy (obstacles inflated)."""
        occ = (self.occupancy_array() == OCCUPIED)
        if inflate <= 0:
            return ~occ
        # cheap dilation via shifts
        blocked = occ.copy()
        for _ in range(int(inflate)):
            shifted = blocked.copy()
            shifted[1:, :] |= blocked[:-1, :]
            shifted[:-1, :] |= blocked[1:, :]
            shifted[:, 1:] |= blocked[:, :-1]
            shifted[:, :-1] |= blocked[:, 1:]
            blocked = shifted
        return ~blocked


def _bresenham(x0, y0, x1, y1):
    """Yield every cell on the line from (x0,y0) to (x1,y1) inclusive."""
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
