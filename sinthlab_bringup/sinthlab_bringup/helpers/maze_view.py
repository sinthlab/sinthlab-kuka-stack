"""The maze, live: what the dashboard's maze view draws (experiment_ctrl_gui/static/maze_view.js).

    <ns>/maze_view/geometry   std_msgs/String (JSON), latched   the path, checkpoints and goal, once
    <ns>/maze_view/state      std_msgs/String (JSON), RATE_HZ   where the arm is; checkpoints visited

Everything is in the maze's own plane coordinates (a, b) -- metres from the anchored start when the
maze is relative -- the same a/b as the trial data's rel_a/rel_b and the plots. It only reads the
fixture and the checkpoint monitor; it changes nothing about the experiment.

geometry: profile, restricted_axis, relative, on_rail_tol_m, corner_radius_m, closed,
          pieces [[[a, b], ...] per piece of the path], checkpoints [[a, b, r], ...], goal [a, b, r],
          goal_after_all
state:    t, trial, anchored, and once anchored: ee [a, b], dist_m (to the path), active (the fixture
          is holding the arm to the path), target [a, b] (where the spring pulls, while active),
          visited [checkpoint indices], goal_armed, monitoring, completed
"""
from __future__ import annotations

import json
import time
from typing import Callable

from rclpy.node import Node as rclpyNode
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

GEOMETRY_TOPIC = "maze_view/geometry"
STATE_TOPIC = "maze_view/state"
RATE_HZ = 15.0


class MazeView:
    def __init__(self, node: rclpyNode, fixture, monitor, trial: Callable[[], int]) -> None:
        self._fixture, self._monitor, self._trial = fixture, monitor, trial
        ia, ib = fixture.plane_axes()

        def plane(xyz_r):
            return [round(xyz_r[ia], 4), round(xyz_r[ib], 4), round(xyz_r[3], 4)]

        wp = monitor.view_geometry()
        geometry = {**fixture.view_geometry(), "checkpoints": [plane(c) for c in wp["checkpoints"]],
                    "goal": plane(wp["goal"]), "goal_after_all": wp["goal_after_all"]}

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        node.create_publisher(String, GEOMETRY_TOPIC, latched).publish(String(data=json.dumps(geometry)))
        self._pub = node.create_publisher(String, STATE_TOPIC, 1)
        node.create_timer(1.0 / RATE_HZ, self._publish)

    def _publish(self) -> None:
        state = {"t": round(time.time(), 3), "trial": int(self._trial())}
        arm = self._fixture.view_state()
        state["anchored"] = arm is not None
        if arm is not None:
            state.update(arm)
            state.update(self._monitor.view_state())
        self._pub.publish(String(data=json.dumps(state)))
