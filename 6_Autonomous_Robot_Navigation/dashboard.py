"""Server-rendered dashboard for the Autonomous Robot Navigation stack.

No emoji (SVG icons only). A ROS2 robot does not serve HTTP itself, so this is
a small status dashboard for operators: it reads the local occupancy map files
and any run artefacts (planned path, executed trajectory) produced by
``sim_offline.py`` / ``run_on_map.py`` and shows topic/parameter status.

When running on a real robot you can point ``ROBOT_STATUS_FILE`` at a JSON file
a companion node writes (see README) and this dashboard renders it.
"""
from __future__ import annotations

import glob
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "_shared"))

import dashboard_kit as kit   # noqa: E402

STATUS_FILE = os.environ.get("ROBOT_STATUS_FILE", "robot_status.json")
MAP_DIR = os.path.join(os.path.dirname(__file__), "maps")


def _load_status():
    try:
        with open(STATUS_FILE, encoding="utf-8") as fh:
            return json.load(fh)
    except Exception:
        return None


def _maps():
    files = []
    for ext in ("*.yaml", "*.yml", "*.pgm", "*.png"):
        files.extend(glob.glob(os.path.join(MAP_DIR, ext)))
    return sorted(os.path.basename(f) for f in files)


def render(conn=None) -> str:
    status = _load_status() or {}
    maps = _maps()
    runs = sorted(os.path.basename(p) for p in
                  glob.glob(os.path.join(MAP_DIR, "*_run.png")))

    pose = status.get("pose", {})
    goal = status.get("goal", {})
    topics = status.get("topics", {})

    cards = [
        kit.card("Robot status", "robot",
                 kit.stat("Mode", status.get("mode", "offline"), "", "robot")
                 + kit.stat("Battery",
                            status.get("battery_pct", "-"), "%", "power")
                 + kit.stat("State", status.get("state", "idle"), "", "signal"),
                 note="Source: robot_status.json (written by the robot bridge)"),
        kit.card("Current pose (map frame)", "route",
                 kit.stat("x", pose.get("x", "-"), "m", "route")
                 + kit.stat("y", pose.get("y", "-"), "m", "route")
                 + kit.stat("heading", pose.get("theta", "-"), "rad", "gauge"),
                 note="Source: TF map -> base_link on the robot"),
        kit.card("Navigation goal", "map",
                 kit.stat("x", goal.get("x", "-"), "m", "map")
                 + kit.stat("y", goal.get("y", "-"), "m", "map")
                 + kit.stat("distance", status.get("distance_to_goal", "-"),
                            "m", "route"),
                 note="Source: /goal_pose topic"),
        kit.card("Topics", "signal",
                 kit.table(["topic", "type", "status"],
                           [(t, v.get("type", ""), v.get("status", "unknown"))
                            for t, v in topics.items()])
                 or '<p class="muted">No live topic status (running offline).</p>',
                 span=2,
                 note="Source: ROS2 graph on the robot"),
        kit.card("Available maps", "map",
                 kit.table(["file"], [(m,) for m in maps])
                 if maps else '<p class="muted">No maps downloaded yet. '
                              'See README to fetch real maps.</p>',
                 note="Source: maps/ directory (real ROS2 occupancy maps)"),
        kit.card("Planner runs", "route",
                 kit.table(["output"], [(r,) for r in runs])
                 if runs else '<p class="muted">Run sim_offline.py or '
                              'run_on_map.py to produce a run image.</p>',
                 note="Source: maps/*_run.png (A* + pure-pursuit results)"),
    ]
    return kit.page("Autonomous Robot Navigation",
                    "SLAM-lite map, A* planner and pure-pursuit control",
                    "".join(cards),
                    footer="Algorithms run offline and on ROS2; "
                           "core logic is unit-tested without a robot.")
