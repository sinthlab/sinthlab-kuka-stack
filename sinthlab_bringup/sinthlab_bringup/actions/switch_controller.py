#!/usr/bin/env python3
"""Switch the active ros2_control controller (e.g. joint-position <-> CLIK).

The restricted-plane / maze experiments drive the arm to a precise start posture on the
``lbr_joint_position_command_controller`` (exact joints, like apple-pluck), then hand off to the
``kuka_clik_controller`` for the Cartesian virtual-fixture phase. This action wraps the
controller_manager ``switch_controller`` service so the orchestrator can do that hand-off (and the
reverse, before the recover move) as just another step in the trial.
"""
from __future__ import annotations

from typing import Callable, List

from rclpy.node import Node as rclpyNode
from controller_manager_msgs.srv import SwitchController


class SwitchControllerAction:
    """Deactivate one set of controllers and activate another, then call ``on_complete``."""

    def __init__(self, node: rclpyNode, *, activate: List[str], deactivate: List[str],
                 on_complete: Callable[[], None], name: str = "switch_controller") -> None:
        self._node = node
        self._activate = list(activate)
        self._deactivate = list(deactivate)
        self._on_complete = on_complete
        self._name = name

        robot = node.get_namespace().strip("/")
        self._srv_name = (
            f"/{robot}/controller_manager/switch_controller" if robot
            else "/controller_manager/switch_controller"
        )
        self._cli = node.create_client(SwitchController, self._srv_name)

    def start(self) -> None:
        if not self._cli.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().error(
                f"{self._name}: service {self._srv_name} unavailable; cannot switch controllers."
            )
            return
        req = SwitchController.Request()
        req.activate_controllers = self._activate
        req.deactivate_controllers = self._deactivate
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        self._node.get_logger().info(
            f"{self._name}: activating {self._activate}, deactivating {self._deactivate}."
        )
        self._cli.call_async(req).add_done_callback(self._on_response)

    def _on_response(self, future) -> None:
        try:
            ok = future.result().ok
        except Exception as exc:
            self._node.get_logger().error(f"{self._name}: switch service call failed: {exc}")
            return
        if not ok:
            self._node.get_logger().error(
                f"{self._name}: controller switch returned not-ok (check controller names/states)."
            )
            return
        self._node.get_logger().info(f"{self._name}: switch complete.")
        self._on_complete()
