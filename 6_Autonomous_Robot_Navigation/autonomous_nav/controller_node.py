"""ROS2 path-following controller node.

Subscribes ``/planned_path`` (nav_msgs/Path) + ``/odom`` (or TF), publishes
``/cmd_vel`` (geometry_msgs/Twist) to drive the robot along the path using the
pure-pursuit controller in ``pure_pursuit.py``.

Safety:
* stops when the path is empty or the goal is reached,
* stops if a forward Range/scan sector reports an obstacle closer than
  ``stop_distance`` (a last-line-of-defence bumper).

Parameters:
    lookahead (float)      default 0.6 m
    max_linear (float)     default 0.4 m/s
    max_angular (float)    default 1.2 rad/s
    stop_distance (float)  default 0.25 m
"""
from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from .pure_pursuit import PurePursuit


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_node")
        self.lookahead = float(self.declare_parameter("lookahead", 0.6).value)
        self.max_linear = float(self.declare_parameter("max_linear", 0.4).value)
        self.max_angular = float(self.declare_parameter("max_angular", 1.2).value)
        self.stop_distance = float(self.declare_parameter("stop_distance", 0.25).value)
        self.rate_hz = float(self.declare_parameter("control_rate", 10.0).value)

        self.controller = PurePursuit(lookahead=self.lookahead,
                                      max_linear=self.max_linear,
                                      max_angular=self.max_angular)
        self.pose = (0.0, 0.0, 0.0)
        self.path: list = []
        self.front_min = math.inf

        self.create_subscription(Path, "/planned_path", self.on_path, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(1.0 / self.rate_hz, self.tick)
        self.get_logger().info(
            f"controller_node up - lookahead={self.lookahead} "
            f"max_v={self.max_linear} max_w={self.max_angular}")

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y ** 2 + q.z ** 2))
        self.pose = (p.x, p.y, yaw)

    def on_path(self, msg: Path):
        self.path = [(ps.pose.position.x, ps.pose.position.y)
                     for ps in msg.poses]

    def on_scan(self, scan: LaserScan):
        """Track the closest obstacle in a forward ±30° cone."""
        best = math.inf
        for i, r in enumerate(scan.ranges):
            if math.isinf(r) or math.isnan(r) or r <= 0:
                continue
            a = scan.angle_min + i * scan.angle_increment
            if abs(a) <= math.radians(30):
                best = min(best, float(r))
        self.front_min = best

    def tick(self):
        twist = Twist()
        if not self.path:
            self.cmd_pub.publish(twist)
            return
        if self.front_min < self.stop_distance:
            self.get_logger().warn(
                f"obstacle at {self.front_min:.2f} m - stopping",
                throttle_duration_sec=1.0)
            self.cmd_pub.publish(twist)
            return

        v, w, arrived = self.controller.compute(self.pose, self.path)
        if arrived:
            self.get_logger().info("goal reached", throttle_duration_sec=2.0)
            self.path = []
            self.cmd_pub.publish(twist)
            return
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
