"""Offline navigation demo - runs the FULL stack without ROS.

Builds a synthetic room with a lidar-like scan, fuses it into the occupancy
grid, plans with A*, and drives the robot along the path with pure pursuit.
Renders a PNG so you can *see* the map, the path and the executed trajectory.

    python sim_offline.py            # writes sim_output.png + prints stats
"""
from __future__ import annotations

import math
import os

import numpy as np

from autonomous_nav.astar import plan as astar_plan
from autonomous_nav.grid import OccupancyGrid
from autonomous_nav.pure_pursuit import PurePursuit

WALLS = [  # (x0, y0, x1, y1) in metres
    (-4, -4, 4, -4), (4, -4, 4, 4), (4, 4, -4, 4), (-4, 4, -4, -4),
    (0, -4, 0, -1), (2, 0, 2, 4), (-3, 1, -1, 1),
]


def synthetic_scan(pose, walls, n_beams=360, max_range=6.0):
    """Ray-cast a 2-D scan against wall segments (robot-frame angles)."""
    px, py, ptheta = pose
    pts = []
    for i in range(n_beams):
        a = -math.pi + i * (2 * math.pi / n_beams)
        world_a = ptheta + a
        hit = max_range
        # march along the ray, checking wall proximity
        step = 0.03
        r = 0.05
        while r < max_range:
            x = px + r * math.cos(world_a)
            y = py + r * math.sin(world_a)
            if _touches_wall(x, y, walls):
                hit = r
                break
            r += step
        pts.append((a, hit, hit < max_range))
    return pts


def _touches_wall(x, y, walls, eps=0.08):
    for (x0, y0, x1, y1) in walls:
        if _point_seg_dist(x, y, x0, y0, x1, y1) < eps:
            return True
    return False


def _point_seg_dist(px, py, x0, y0, x1, y1):
    dx, dy = x1 - x0, y1 - y0
    if dx == dy == 0:
        return math.hypot(px - x0, py - y0)
    t = max(0, min(1, ((px - x0) * dx + (py - y0) * dy) / (dx * dx + dy * dy)))
    cx, cy = x0 + t * dx, y0 + t * dy
    return math.hypot(px - cx, py - cy)


def run(save="sim_output.png"):
    grid = OccupancyGrid(size=360, resolution=0.05)
    start = (-3.0, -3.0, 0.0)
    goal = (3.0, 3.0)

    # --- build the map by fusing scans from a few poses ---
    for pose in [(p, p, 0) for p in (-3, -1, 1, 3)]:
        grid.integrate_scan(pose, synthetic_scan(pose, WALLS))
    for pose in [(0, p, math.pi / 2) for p in (-3, -1, 1, 3)]:
        grid.integrate_scan(pose, synthetic_scan(pose, WALLS))

    # --- plan ---
    path = astar_plan(grid, start[:2], goal, inflate=1, do_smooth=True)
    print(f"path waypoints: {len(path)}")

    # --- drive ---
    controller = PurePursuit(lookahead=0.5, max_linear=0.4, max_angular=1.5)
    pose = list(start)
    traj = [tuple(pose[:2])]
    dt = 0.1
    for _ in range(4000):
        v, w, arrived = controller.compute(tuple(pose), path)
        if arrived:
            break
        pose[2] += w * dt
        pose[0] += v * math.cos(pose[2]) * dt
        pose[1] += v * math.sin(pose[2]) * dt
        traj.append(tuple(pose[:2]))

    reached = math.hypot(pose[0] - goal[0], pose[1] - goal[1])
    print(f"final pose: ({pose[0]:.2f}, {pose[1]:.2f})  "
          f"distance to goal: {reached:.3f} m")
    print(f"trajectory samples: {len(traj)}")
    print("cells known:", int((grid.occupancy_array() != -1).sum()))

    _render(grid, path, traj, start, goal, save)
    print(f"wrote {save}")
    return {"path": len(path), "reached": reached, "traj": len(traj)}


def _render(grid, path, traj, start, goal, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    occ = grid.occupancy_array()
    plt.figure(figsize=(8, 8))
    plt.imshow(occ, origin="lower", cmap="gray_r", interpolation="nearest")
    if path:
        px = [grid.world_to_cell(x, y)[0] for x, y in path]
        py = [grid.world_to_cell(x, y)[1] for x, y in path]
        plt.plot(px, py, "-", color="#1f6feb", lw=2, label="A* path")
    if traj:
        tx = [grid.world_to_cell(x, y)[0] for x, y in traj]
        ty = [grid.world_to_cell(x, y)[1] for x, y in traj]
        plt.plot(tx, ty, "-", color="#e0533d", lw=1.5, label="executed")
    sx, sy = grid.world_to_cell(*start[:2])
    gx, gy = grid.world_to_cell(*goal)
    plt.plot(sx, sy, "o", color="green", ms=10, label="start")
    plt.plot(gx, gy, "o", color="orange", ms=10, label="goal")
    plt.legend(loc="upper left")
    plt.title("SLAM-lite map · A* path · pure-pursuit trajectory")
    plt.tight_layout()
    plt.savefig(out, dpi=110)
    plt.close()


if __name__ == "__main__":
    run()
