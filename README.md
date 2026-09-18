# 6 · Autonomous Robot Navigation - Real ROS2 SLAM-lite + A* + Pure Pursuit

A complete ROS2 navigation package: consumes a **real lidar**, builds a real
occupancy-grid map, plans with **A***, and **drives the robot** along the path
with a pure-pursuit controller on `/cmd_vel`.

The algorithms are pure Python (no ROS) and **fully unit-tested offline**, so
you can verify correctness on any machine before you ever touch a robot.

## Architecture

```
 /scan ─┐
        ├─► planner_node ──► /slam_map            (OccupancyGrid, log-odds)
 /odom ─┘        │
   /goal_pose ───┘
                 └──► A* + smoothing ──► /planned_path
                                             │
 /odom ─────────────────► controller_node ◄──┘
 /scan ────────────────────────┘  ──► pure pursuit ──► /cmd_vel
```

| Module | Role | ROS-free? |
|-------|------|-----------|
| `grid.py` | Occupancy grid + inverse sensor model (Bresenham ray casts, log-odds) | yes |
| `astar.py` | A* with 8-connectivity, corner-cut prevention, path smoothing | yes |
| `pure_pursuit.py` | Path follower -> `(v, ω)` | yes |
| `planner_node.py` | ROS2 glue: scan -> map -> A* -> path | no (ROS) |
| `controller_node.py` | ROS2 glue: path -> `/cmd_vel`, obstacle safety stop | no (ROS) |

## What was fixed vs. the original
* The original fused scans with a **sloppy integer ray loop** that skipped
  cells and could not mark free space reliably; this uses **Bresenham** +
  a proper free/occupied decision boundary (p=0.5) so unknown vs. free is real.
* It had a **hardcoded goal** and no pose handling; now the goal comes from
  `/goal_pose` and the pose from **TF** (falling back to `/odom`).
* It had **no controller** - it planned but never moved. A pure-pursuit
  controller now drives the robot, with a forward-obstacle safety stop.
* It was **untestable** - now the core is offline-tested (16 tests).

## Try it without a robot (offline demo)

```bash
pip install -r requirements.txt
python sim_offline.py       # builds a map, plans, drives, writes sim_output.png
pytest -q                   # 16 tests over grid / A* / pure pursuit
```

`sim_output.png` shows the SLAM-lite map, the A* path and the executed
trajectory - visual proof the whole stack works.

## Deploy on a real robot (ROS2)

```bash
mkdir -p ~/ros2_ws/src && cp -r . ~/ros2_ws/src/autonomous_nav
cd ~/ros2_ws && colcon build && source install/setup.bash
ros2 launch autonomous_nav nav.launch.py
# drive the robot around to build the map, then set a goal:
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 2.0}}}"
```

View in RViz: add `/slam_map` (Map) and `/planned_path` (Path).

### Parameters (`config/nav_params.yaml`)
`grid_size`, `resolution`, `obstacle_inflate`, `replan_period`, `use_tf`,
`goal_tolerance` (planner) · `lookahead`, `max_linear`, `max_angular`,
`stop_distance`, `control_rate` (controller).

## Topics
| Topic | Type | Direction |
|------|------|-----------|
| `/scan` | `sensor_msgs/LaserScan` | in |
| `/odom` | `nav_msgs/Odometry` | in (pose fallback) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | in |
| `/slam_map` | `nav_msgs/OccupancyGrid` | out |
| `/planned_path` | `nav_msgs/Path` | out |
| `/cmd_vel` | `geometry_msgs/Twist` | out |

## How it works
1. **Fuse**: each lidar beam updates log-odds along the Bresenham line - hits
   raise, free cells lower; the grid is rendered to `/slam_map`.
2. **Plan**: obstacles are inflated for clearance; A* finds a path; shortcut
   smoothing removes needless waypoints.
3. **Follow**: pure pursuit picks a look-ahead point, turns in place until
   aligned, then drives; it slows near the goal and stops on forward obstacles.

> Safety: real robots can hurt people. Keep the emergency stop physical; test
> with the wheels off the ground first; tune `max_linear` conservatively.

## Web dashboard

The navigation stack serves a real **operator dashboard** as a web page
(ROS2 nodes speak DDS, not HTTP, so this is a small standalone server):

    python serve_dashboard.py          # http://127.0.0.1:8006

It reads `robot_status.json` (written by your robot bridge, if present) and the
local `maps/` directory. Point `ROBOT_STATUS_FILE` at a JSON file your robot
node writes to see live pose, goal, battery and topic status.

* No emoji ? icons are inline SVG (`../_shared/dashboard_kit.py`).
* No CDN and no JavaScript.
