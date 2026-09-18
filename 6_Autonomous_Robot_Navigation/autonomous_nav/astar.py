"""A* grid planner + path post-processing (smoothing + shortcutting).

Pure Python / numpy - no ROS dependency, fully unit-testable.
"""
from __future__ import annotations

import heapq
import math

import numpy as np

from .grid import OccupancyGrid

NEIGHBOURS_8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1),
                (0, 1), (1, -1), (1, 0), (1, 1)]


def astar(free_mask: np.ndarray, start, goal, diagonal=True):
    """A* over a boolean free-space mask. Returns a list of (x, y) cells."""
    size_y, size_x = free_mask.shape
    if not (0 <= start[0] < size_x and 0 <= start[1] < size_y):
        return []
    if not (0 <= goal[0] < size_x and 0 <= goal[1] < size_y):
        return []
    if not free_mask[goal[1], goal[0]]:
        goal = _nearest_free(free_mask, goal)
        if goal is None:
            return []

    moves = NEIGHBOURS_8 if diagonal else NEIGHBOURS_8[1::2]

    def h(p):
        return math.hypot(p[0] - goal[0], p[1] - goal[1])

    open_set = [(h(start), 0.0, start)]
    came: dict = {}
    g = {start: 0.0}
    best = 0.0
    while open_set:
        _, cost, cur = heapq.heappop(open_set)
        if cur == goal:
            return _reconstruct(came, cur)
        if cost > g.get(cur, math.inf):
            continue
        best = max(best, cost)
        for dx, dy in moves:
            nx, ny = cur[0] + dx, cur[1] + dy
            if not (0 <= nx < size_x and 0 <= ny < size_y):
                continue
            if not free_mask[ny, nx]:
                continue
            # prevent cutting diagonally through two blocked cells
            if dx and dy:
                if not free_mask[cur[1], nx] or not free_mask[ny, cur[0]]:
                    continue
            ng = cost + (math.hypot(dx, dy))
            if ng < g.get((nx, ny), math.inf):
                g[(nx, ny)] = ng
                came[(nx, ny)] = cur
                heapq.heappush(open_set, (ng + h((nx, ny)), ng, (nx, ny)))
    return []


def _reconstruct(came, cur):
    path = [cur]
    while cur in came:
        cur = came[cur]
        path.append(cur)
    return path[::-1]


def _nearest_free(free_mask, goal, radius=6):
    gy, gx = goal[1], goal[0]
    h, w = free_mask.shape
    for r in range(1, radius + 1):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                x, y = gx + dx, gy + dy
                if 0 <= x < w and 0 <= y < h and free_mask[y, x]:
                    return (x, y)
    return None


def smooth(path, free_mask, iterations=2):
    """Gradient-free shortcutting: remove points that can be skipped with a
    collision-free straight line. Produces natural, shorter paths."""
    if len(path) < 3:
        return list(path)
    out = list(path)
    for _ in range(iterations):
        i = 0
        while i < len(out) - 2:
            if _line_free(out[i], out[i + 2], free_mask):
                del out[i + 1]
            else:
                i += 1
    return out


def _line_free(a, b, free_mask):
    for (cx, cy) in _bresenham_cells(a[0], a[1], b[0], b[1]):
        h, w = free_mask.shape
        if not (0 <= cx < w and 0 <= cy < h) or not free_mask[cy, cx]:
            return False
    return True

def plan(grid: OccupancyGrid, start_world, goal_world, inflate=1, do_smooth=True):
    """Convenience: world coords in -> world path out (metres)."""
    free = grid.free_mask(inflate=inflate)
    start = grid.world_to_cell(*start_world)
    goal = grid.world_to_cell(*goal_world)
    path = astar(free, start, goal)
    if not path:
        return []
    if do_smooth:
        path = smooth(path, free)
    return [grid.cell_to_world(cx, cy) for (cx, cy) in path]


def _bresenham_cells(x0, y0, x1, y1):
    dx, dy = abs(x1 - x0), abs(y1 - y0)
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

