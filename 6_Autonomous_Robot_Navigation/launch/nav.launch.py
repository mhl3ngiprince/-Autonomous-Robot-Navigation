"""Launch the full navigation stack (planner + controller).

    ros2 launch autonomous_nav nav.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("autonomous_nav")
    params = os.path.join(share, "config", "nav_params.yaml")

    planner = Node(
        package="autonomous_nav",
        executable="planner_node",
        name="planner_node",
        parameters=[params],
        output="screen",
    )
    controller = Node(
        package="autonomous_nav",
        executable="controller_node",
        name="controller_node",
        parameters=[params],
        output="screen",
    )

    return LaunchDescription([planner, controller])
