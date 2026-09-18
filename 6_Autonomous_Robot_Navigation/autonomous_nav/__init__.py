"""Autonomous navigation package: SLAM-lite occupancy grid + A* + pure pursuit.

The algorithm core (``grid``, ``astar``, ``pure_pursuit``) has no ROS
dependency and is unit-tested offline. The ROS2 nodes (``planner_node``,
``controller_node``) are thin glue layers that run on real hardware.
"""
__version__ = "2.0.0"

from .grid import OccupancyGrid
from .astar import astar, smooth, plan
from .pure_pursuit import PurePursuit

__all__ = ["OccupancyGrid", "astar", "smooth", "plan", "PurePursuit", "__version__"]
