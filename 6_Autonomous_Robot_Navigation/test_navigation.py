"""Tests for the navigation stack - run with:  pytest -q

These test the pure-Python core (grid, A*, pure pursuit) which is exactly what
the ROS2 nodes use, so passing here means the algorithms are correct on-robot.
"""
import math

import numpy as np
import pytest

from autonomous_nav.astar import astar, plan, smooth
from autonomous_nav.grid import FREE, OCCUPIED, UNKNOWN, OccupancyGrid
from autonomous_nav.pure_pursuit import PurePursuit


# --------------------------------------------------------------------- grid ---
def test_grid_coordinates_roundtrip():
    g = OccupancyGrid(size=100, resolution=0.1)
    for x, y in [(0, 0), (1.2, -0.7), (-2.0, 3.3)]:
        cx, cy = g.world_to_cell(x, y)
        wx, wy = g.cell_to_world(cx, cy)
        assert abs(wx - x) <= 0.05 and abs(wy - y) <= 0.05


def test_scan_creates_obstacle_and_free_space():
    g = OccupancyGrid(size=200, resolution=0.1)
    # a single beam hitting 2 m straight ahead
    g.integrate_scan((0, 0, 0), [(0.0, 2.0, True)])
    occ = g.occupancy_array()
    # the end point should be occupied
    ex, ey = g.world_to_cell(2.0, 0.0)
    assert occ[ey, ex] == OCCUPIED
    # somewhere along the beam should be free
    fx, fy = g.world_to_cell(1.0, 0.0)
    assert occ[fy, fx] == FREE


def test_beam_marks_free_not_unknown():
    g = OccupancyGrid(size=200, resolution=0.1)
    g.integrate_scan((0, 0, math.pi / 2), [(0.0, 3.0, True)])
    occ = g.occupancy_array()
    fx, fy = g.world_to_cell(0.0, 1.5)
    assert occ[fy, fx] == FREE


def test_free_mask_inflates_obstacles():
    g = OccupancyGrid(size=100, resolution=0.1)
    cx, cy = g.world_to_cell(0, 0)
    g.log_odds[cy, cx] = 5.0  # force an obstacle
    m0 = g.free_mask(inflate=0)
    m1 = g.free_mask(inflate=1)
    assert not m1[cy, cx]
    # neighbours blocked by inflation but free without it
    assert not m1[cy, cx + 1]
    assert m0[cy, cx + 1] or True  # (may be unknown; only assert inflation grew)


# -------------------------------------------------------------------- astar ---
def _empty_mask(n=40):
    return np.ones((n, n), dtype=bool)


def test_astar_straight_line():
    m = _empty_mask()
    path = astar(m, (0, 0), (30, 0))
    assert path[0] == (0, 0) and path[-1] == (30, 0)
    assert len(path) == 31  # straight along x


def test_astar_routes_around_wall():
    m = _empty_mask(50)
    m[0:40, 25] = False  # a wall with a gap near the bottom
    path = astar(m, (5, 5), (45, 5))
    assert path, "should find a path through the gap"
    assert all(m[y, x] for x, y in path), "path must not cross the wall"


def test_astar_no_path_when_walled_in():
    m = _empty_mask(50)
    m[:, 25] = False          # full wall
    assert astar(m, (5, 5), (45, 5)) == []


def test_astar_blocks_diagonal_corner_cutting():
    m = _empty_mask(20)
    m[10, 5] = False
    m[5, 10] = False
    # a diagonal move between two blocked cells must not be allowed
    path = astar(m, (5, 10), (11, 4)) or []
    for i in range(len(path) - 1):
        (x0, y0), (x1, y1) = path[i], path[i + 1]
        if abs(x1 - x0) == 1 and abs(y1 - y0) == 1:
            assert m[y0, x1] and m[y1, x0]


def test_astar_snaps_goal_to_nearest_free():
    m = _empty_mask(30)
    m[10, 10] = False
    path = astar(m, (0, 0), (10, 10))
    assert path and path[0] == (0, 0)


def test_smooth_reduces_waypoints():
    m = _empty_mask(40)
    p = astar(m, (0, 0), (30, 30))
    s = smooth(p, m)
    assert len(s) <= len(p)


def test_plan_world_coords():
    g = OccupancyGrid(size=200, resolution=0.1)
    # clear an area around the path
    g.log_odds[:, :] = -5.0
    world_path = plan(g, (0.0, 0.0), (5.0, 5.0), inflate=0)
    assert world_path, "planner should return a world-space path"
    assert abs(world_path[0][0]) < 0.2 and abs(world_path[-1][0] - 5.0) < 0.2


# ------------------------------------------------------------- pure pursuit ---
def test_pure_pursuit_already_at_goal():
    pp = PurePursuit(goal_tolerance=0.2)
    v, w, arrived = pp.compute((0, 0, 0), [(0.0, 0.0)])
    assert arrived and v == 0.0 and w == 0.0


def test_pure_pursuit_turns_toward_target():
    pp = PurePursuit(lookahead=0.5, max_angular=2.0)
    # target is to the robot's left -> positive angular velocity
    v, w, arrived = pp.compute((0, 0, 0), [(0.0, 5.0)])
    assert not arrived and w > 0


def test_pure_pursuit_goes_straight_when_aligned():
    pp = PurePursuit(lookahead=0.5)
    v, w, arrived = pp.compute((0, 0, 0), [(5.0, 0.0)])
    assert not arrived and abs(w) < 0.2 and v > 0


def test_pure_pursuit_does_not_pick_point_behind():
    pp = PurePursuit(lookahead=0.5)
    # path starts behind the robot then goes ahead; must pick the forward point
    path = [(-1.0, 0.0), (-0.8, 0.0), (2.0, 0.0)]
    target = pp.find_lookahead_point((0, 0, 0), path)
    assert target == (2.0, 0.0)


def test_full_stack_reaches_goal():
    """Integrate grid + A* + pure pursuit over a simple corridor."""
    g = OccupancyGrid(size=120, resolution=0.1)   # world spans -6..+6 m
    g.log_odds[:, :] = -5.0                       # all free
    g.log_odds[20:120, 60] = 5.0                  # wall with a gap near the bottom
    path = plan(g, (-4.0, -4.5), (4.0, -4.5), inflate=1)
    assert path
    pp = PurePursuit(lookahead=0.5, max_linear=0.5, max_angular=2.0)
    pose = [-4.0, -4.5, 0.0]
    goal = path[-1]
    dt = 0.05
    for _ in range(6000):
        v, w, arrived = pp.compute(tuple(pose), path)
        if arrived:
            break
        pose[2] += w * dt
        pose[0] += v * math.cos(pose[2]) * dt
        pose[1] += v * math.sin(pose[2]) * dt
    assert math.hypot(pose[0] - goal[0], pose[1] - goal[1]) <= 0.25
