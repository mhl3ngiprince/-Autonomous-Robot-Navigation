"""Load a REAL occupancy map (ROS map_server YAML+PGM) or standard image, and
plan/follow on it - so the navigation stack can be exercised against real
floor-plan data, not just the synthetic room in sim_offline.py.

Supported inputs:
* ROS ``map_server`` maps: a ``.yaml`` with ``image:``/``resolution:`` pointing
  at a ``.pgm``/``.png`` (standard occupancy colours: 0=occupied, 254=free,
  205=unknown).
* Any grayscale image where dark = obstacle, white = free.

It then:
1. converts the image to the internal log-odds grid,
2. picks start/goal (or accepts --start/--goal),
3. plans and drives with pure pursuit,
4. renders the result.

Usage:
    python run_on_map.py --map map.yaml --start 1,1 --goal 8,6
    python run_on_map.py --image floorplan.png --free-threshold 200
"""
from __future__ import annotations

import argparse
import os

import cv2
import numpy as np

from autonomous_nav.astar import plan
from autonomous_nav.grid import OccupancyGrid
from autonomous_nav.pure_pursuit import PurePursuit


def load_map(path, free_threshold=None):
    """Return (array_0_255, resolution). Dark = obstacle."""
    if path.lower().endswith((".yaml", ".yml")):
        import yaml
        with open(path, encoding="utf-8") as fh:
            meta = yaml.safe_load(fh)
        img_path = os.path.join(os.path.dirname(path), meta["image"])
        res = float(meta.get("resolution", 0.05))
        img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise FileNotFoundError(img_path)
        return img, res
    img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(path)
    return img, 0.05


def to_log_odds(img, resolution, free_threshold=None):
    """Map an occupancy image into a log-odds grid.

    ROS convention: 0 = occupied, 254 = free, 205 = unknown. For a plain
    image we treat dark pixels as obstacles and bright ones as free.
    """
    h, w = img.shape
    size = max(h, w)
    grid = OccupancyGrid(size=size, resolution=resolution)
    # unknown stays 0 (log-odds prior); occupied -> +, free -> -
    occ_mask = img < 128
    free_mask = img > (free_threshold if free_threshold is not None else 200)
    gy, gx = np.where(occ_mask)
    grid.log_odds[gy, gx] = 5.0
    fy, fx = np.where(free_mask)
    grid.log_odds[fy, fx] = -5.0
    return grid


def run(map_path, image=False, start=None, goal=None, out="map_run.png"):
    if image:
        img, res = load_map(map_path, free_threshold=200)
    else:
        img, res = load_map(map_path)
    grid = to_log_odds(img, res)
    h, w = img.shape

    def cell_of(pt):
        if pt is None:
            return None
        x, y = pt
        return int(round(x)), h - 1 - int(round(y))  # image y is top-down

    start_pt = start or _auto_free(grid, img, top=True)
    goal_pt = goal or _auto_free(grid, img, top=False)
    s_cell, g_cell = cell_of(start_pt), cell_of(goal_pt)

    path = plan(grid, _cell_world(grid, s_cell), _cell_world(grid, g_cell),
                inflate=2)
    print(f"map {w}x{h} px @ {res} m  -> path waypoints: {len(path)}")
    if not path:
        print("No path found - try different start/goal or lower inflation.")
        return

    pp = PurePursuit(lookahead=0.5 * (res * 20), max_linear=0.5, max_angular=2.0)
    pose = [*_cell_world(grid, s_cell), 0.0]
    traj = [tuple(pose[:2])]
    for _ in range(8000):
        v, w_, arrived = pp.compute(tuple(pose), path)
        if arrived:
            break
        pose[2] += w_ * 0.05
        pose[0] += v * np.cos(pose[2]) * 0.05
        pose[1] += v * np.sin(pose[2]) * 0.05
        traj.append(tuple(pose[:2]))
    reached = np.hypot(pose[0] - path[-1][0], pose[1] - path[-1][1])
    print(f"final distance to goal: {reached:.3f} m")
    _render(img, grid, path, traj, s_cell, g_cell, out)
    print("wrote", out)


def _auto_free(grid, img, top=True):
    occ = grid.occupancy_array()
    free = (occ == 0)
    ys, xs = np.where(free)
    if len(xs) == 0:
        raise RuntimeError("map has no free space")
    mid_x = int(np.median(xs))
    col = xs == mid_x
    y_sel = ys[col]
    y = y_sel.min() if top else y_sel.max()
    return (mid_x, y)


def _cell_world(grid, cell):
    return grid.cell_to_world(*cell)


def _render(img, grid, path, traj, s_cell, g_cell, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.figure(figsize=(10, 10))
    plt.imshow(img, cmap="gray", origin="upper")
    if path:
        px = [grid.world_to_cell(x, y)[0] for x, y in path]
        py = [img.shape[0] - 1 - grid.world_to_cell(x, y)[1] for x, y in path]
        plt.plot(px, py, "-", color="#1f6feb", lw=2, label="A* path")
    if traj:
        tx = [grid.world_to_cell(x, y)[0] for x, y in traj]
        ty = [img.shape[0] - 1 - grid.world_to_cell(x, y)[1] for x, y in traj]
        plt.plot(tx, ty, "-", color="#e0533d", lw=1.5, label="executed")
    plt.plot(s_cell[0], img.shape[0] - 1 - s_cell[1], "o", color="green", ms=10)
    plt.plot(g_cell[0], img.shape[0] - 1 - g_cell[1], "o", color="orange", ms=10)
    plt.legend()
    plt.title("Real occupancy map · A* · pure pursuit")
    plt.tight_layout()
    plt.savefig(out, dpi=110)
    plt.close()


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--map", required=True)
    p.add_argument("--image", action="store_true", help="treat as plain image")
    p.add_argument("--start", default=None, help="x,y in px from top-left")
    p.add_argument("--goal", default=None)
    p.add_argument("--out", default="map_run.png")
    args = p.parse_args(argv)
    conv = lambda s: tuple(int(v) for v in s.split(",")) if s else None
    run(args.map, args.image, conv(args.start), conv(args.goal), args.out)


if __name__ == "__main__":
    main()
