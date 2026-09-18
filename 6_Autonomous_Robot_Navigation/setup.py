from setuptools import setup, find_packages
import os
from glob import glob

package_name = "autonomous_nav"

setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="Autonomous navigation: SLAM-lite occupancy grid + A* + pure pursuit",
    license="MIT",
    entry_points={
        "console_scripts": [
            "planner_node = autonomous_nav.planner_node:main",
            "controller_node = autonomous_nav.controller_node:main",
        ],
    },
)
