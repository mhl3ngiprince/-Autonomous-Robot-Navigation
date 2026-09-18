"""Pure-pursuit path follower.

Converts a planned path (metres, map frame) + the robot pose into a
``(linear_x, angular_z)`` velocity command - the real output a ROS robot's
``/cmd_vel`` expects. Pure Python so it is unit-testable without ROS.
"""
from __future__ import annotations

import math


class PurePursuit:
    def __init__(self, lookahead=0.6, max_linear=0.4, max_angular=1.2,
                 goal_tolerance=0.15, slow_radius=0.8):
        self.lookahead = lookahead
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.goal_tolerance = goal_tolerance
        self.slow_radius = slow_radius

    def find_lookahead_point(self, pose, path):
        """First path point at least `lookahead` ahead of the robot.

        "Ahead" means within +/-90 deg of the robot's heading. Naively taking
        the first point that is `lookahead` away can pick a point *behind* the
        robot and make it circle; we prefer forward points and only fall back
        to the final goal when the robot must swing around.
        """
        px, py, ptheta = pose
        for (x, y) in path:
            dx, dy = x - px, y - py
            if math.hypot(dx, dy) < self.lookahead:
                continue
            err = abs(_wrap(math.atan2(dy, dx) - ptheta))
            if err <= math.pi / 2:
                return (x, y)
        return path[-1]

    def compute(self, pose, path):
        """Return (linear_x, angular_z, arrived: bool)."""
        px, py, ptheta = pose
        if not path:
            return 0.0, 0.0, True

        goal = path[-1]
        if math.hypot(goal[0] - px, goal[1] - py) <= self.goal_tolerance:
            return 0.0, 0.0, True

        tx, ty = self.find_lookahead_point(pose, path)

        # target in the robot frame
        dx, dy = tx - px, ty - py
        cos_t, sin_t = math.cos(-ptheta), math.sin(-ptheta)
        lx = dx * cos_t - dy * sin_t
        ly = dx * sin_t + dy * cos_t
        heading_err = math.atan2(ly, lx)

        # Slow near the goal and while turning; drive forward only once roughly
        # aligned (prevents the classic pure-pursuit spiral-in-place failure).
        dist_goal = math.hypot(goal[0] - px, goal[1] - py)
        speed = self.max_linear
        if dist_goal < self.slow_radius:
            speed = max(0.05, self.max_linear * (dist_goal / self.slow_radius))
        if abs(heading_err) > math.pi / 4:
            speed = 0.0
        elif abs(heading_err) > math.pi / 8:
            speed *= 0.5

        look = max(math.hypot(lx, ly), 1e-3)
        if abs(speed) < 1e-6:
            angular = 2.0 * heading_err              # turn in place
        else:
            angular = (2.0 * ly / (look ** 2)) * speed
        angular = max(-self.max_angular, min(self.max_angular, angular))
        return speed, angular, False


def _wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a
