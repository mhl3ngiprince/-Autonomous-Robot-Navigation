#!/bin/bash
# Bring up the navigation stack on a real robot.
#   * planner_node    : fuses /scan into /slam_map, plans A* -> /planned_path
#   * controller_node : follows /planned_path, drives /cmd_vel
set -e
source /opt/ros/humble/setup.bash      # change to your distro if needed
source ~/ros2_ws/install/setup.bash
ros2 launch autonomous_nav nav.launch.py
