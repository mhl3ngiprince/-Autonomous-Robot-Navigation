"""ROS2 SLAM-lite + A* planner node - real lidar in, real map + path out.

Subscribes
----------
``/scan``            (sensor_msgs/LaserScan)   real lidar
``/odom``            (nav_msgs/Odometry)       robot pose (fallback if no TF)
``/goal_pose``       (geometry_msgs/PoseStamped) operator picks a goal

Publishes
---------
``/slam_map``        (nav_msgs/OccupancyGrid)  fused occupancy grid
``/planned_path``    (nav_msgs/Path)           smoothed A* path

The heavy lifting lives in ``grid.py`` and ``astar.py`` (pure Python, tested
offline). This module is only the ROS glue.

Parameters (via ``ros2 run ... --ros-args -p name:=value``)
    grid_size (int)      cells per side            default 400
    resolution (float)   metres per cell           default 0.05
    obstacle_inflate     clearance cells           default 1
    replan_period (s)    replanning rate           default 1.0
    use_tf               use TF for pose           default true
"""
from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range

from .astar import plan as astar_plan
from .grid import OccupancyGrid as LogOddsGrid


class PlannerNode(Node):
    def __init__(self):
        super().__init__("planner_node")

        self.size = int(self.declare_parameter("grid_size", 400).value)
        self.res = float(self.declare_parameter("resolution", 0.05).value)
        self.inflate = int(self.declare_parameter("obstacle_inflate", 1).value)
        self.replan_period = float(self.declare_parameter("replan_period", 1.0).value)
        self.use_tf = bool(self.declare_parameter("use_tf", True).value)
        self.goal_tol = float(self.declare_parameter("goal_tolerance", 0.15).value)

        self.grid = LogOddsGrid(size=self.size, resolution=self.res)
        self.pose = (0.0, 0.0, 0.0)
        self.goal: Optional[tuple] = None
        self.last_path = []
        self._scans = 0
        self._tf_buffer = None

        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)

        self.map_pub = self.create_publisher(OccupancyGrid, "/slam_map", 10)
        self.path_pub = self.create_publisher(Path, "/planned_path", 10)

        if self.use_tf:
            self._init_tf()

        self.create_timer(self.replan_period, self.replan)
        self.get_logger().info(
            f"planner_node up - grid {self.size}x{self.size} @ {self.res}m, "
            f"pose from {'TF' if self._tf_buffer else 'odom'}")

    # ------------------------------------------------------------------ TF ----
    def _init_tf(self):
        try:
            from tf2_ros import Buffer, TransformListener
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self.create_timer(0.1, self._pose_from_tf)
        except Exception as exc:  # tf2 not available -> fall back to /odom
            self.get_logger().warn(f"TF unavailable ({exc}); using /odom pose")
            self._tf_buffer = None

    def _pose_from_tf(self):
        try:
            from rclpy.time import Time
            tf = self._tf_buffer.lookup_transform("map", "base_link", Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y ** 2 + q.z ** 2))
            self.pose = (t.x, t.y, yaw)
        except Exception:
            pass  # keep the last known pose

    # -------------------------------------------------------------- callbacks --
    def on_odom(self, msg: Odometry):
        if self._tf_buffer is not None:
            return  # TF is the source of truth
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y ** 2 + q.z ** 2))
        self.pose = (p.x, p.y, yaw)

    def on_goal(self, msg: PoseStamped):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(
            f"new goal ({self.goal[0]:.2f}, {self.goal[1]:.2f})")

    def on_scan(self, scan: LaserScan):
        """Fuse one real lidar sweep into the log-odds grid."""
        ranges = scan.ranges
        n = len(ranges)
        scan_pts = []
        for i in range(n):
            r = ranges[i]
            angle = scan.angle_min + i * scan.angle_increment
            if math.isinf(r) or math.isnan(r):
                scan_pts.append((angle, None, False))
            elif r > scan.range_max:
                scan_pts.append((angle, None, False))
            else:
                scan_pts.append((angle, float(r), True))
        self.grid.integrate_scan(self.pose, scan_pts)
        self._scans += 1
        self.publish_map(scan)

    # -------------------------------------------------------------- publishing -
    def publish_map(self, scan: LaserScan):
        occ = self.grid.occupancy_array()
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = self.res
        grid.info.width = grid.info.height = self.size
        # place the map origin so cell (0,0) corresponds to the world offset
        ox, oy = self.grid.cell_to_world(0, 0)
        grid.info.origin.position.x = ox
        grid.info.origin.position.y = oy
        grid.data = occ.flatten().tolist()
        self.map_pub.publish(grid)

    def replan(self):
        if self.goal is None:
            return
        path_world = astar_plan(self.grid, self.pose[:2], self.goal,
                                inflate=self.inflate, do_smooth=True)
        if not path_world:
            self.get_logger().warn("no path found to goal", throttle_duration_sec=5.0)
            return
        self.last_path = path_world
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        for (x, y) in path_world:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.path_pub.publish(msg)
        self.get_logger().info(f"published path, {len(path_world)} waypoints",
                               throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
