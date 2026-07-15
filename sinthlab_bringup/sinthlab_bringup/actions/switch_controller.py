#!/usr/bin/env python3
"""Switch the active ros2_control controller (e.g. joint-position <-> CLIK).

The restricted-plane / maze experiments drive the arm to a precise start posture on the
``lbr_joint_position_command_controller`` (exact joints, like apple-pluck), then hand off to the
``kuka_clik_controller`` for the Cartesian virtual-fixture phase. This action wraps the
controller_manager ``switch_controller`` service so the orchestrator can do that hand-off (and the
reverse, before the recover move) as just another step in the trial.

The CLIK is spawned ``--inactive`` in PARALLEL with the orchestrator, so at the first hand-off it may
not be loaded yet — especially when ``move_to_start`` finishes fast (start pose near the parked pose).
A STRICT switch to a not-yet-loaded controller aborts with "no controller with this name exists" and
stalls the trial. So this action first POLLS ``list_controllers`` until every controller it needs is
loaded (in any state), up to ``load_timeout_sec``, and only then switches.
"""
from __future__ import annotations

from typing import Callable, List, Optional

from rclpy.duration import Duration
from rclpy.node import Node as rclpyNode
from controller_manager_msgs.srv import SwitchController, ListControllers


class SwitchControllerAction:
    """Deactivate one set of controllers and activate another, then call ``on_complete``."""

    def __init__(self, node: rclpyNode, *, activate: List[str], deactivate: List[str],
                 on_complete: Callable[[], None], name: str = "switch_controller",
                 load_timeout_sec: float = 20.0, poll_period_sec: float = 0.3) -> None:
        self._node = node
        self._activate = list(activate)
        self._deactivate = list(deactivate)
        self._on_complete = on_complete
        self._name = name

        robot = node.get_namespace().strip("/")
        base = f"/{robot}/controller_manager" if robot else "/controller_manager"
        self._srv_name = f"{base}/switch_controller"
        self._cli = node.create_client(SwitchController, self._srv_name)
        self._list_cli = node.create_client(ListControllers, f"{base}/list_controllers")

        # Every controller involved must be LOADED before a STRICT switch will accept it.
        self._need_loaded = set(self._activate) | set(self._deactivate)
        self._load_timeout = float(load_timeout_sec)
        self._poll_period = float(poll_period_sec)
        self._poll_timer = None
        self._list_pending = False
        self._deadline = None

    def start(self) -> None:
        self._cancel_poll()  # trials repeat; never leave a previous poll timer running
        if not self._cli.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().error(
                f"{self._name}: service {self._srv_name} unavailable; cannot switch controllers."
            )
            return
        # Poll until the controllers we touch are loaded, THEN switch (see module docstring).
        self._deadline = self._node.get_clock().now() + Duration(seconds=self._load_timeout)
        self._list_pending = False
        self._poll_timer = self._node.create_timer(self._poll_period, self._poll_loaded)
        self._poll_loaded()  # check immediately so the common (already-loaded) case has no delay

    def _cancel_poll(self) -> None:
        if self._poll_timer is not None:
            self._poll_timer.cancel()
            self._node.destroy_timer(self._poll_timer)
            self._poll_timer = None

    def _poll_loaded(self) -> None:
        if self._node.get_clock().now() > self._deadline:
            self._cancel_poll()
            self._node.get_logger().error(
                f"{self._name}: controllers {sorted(self._need_loaded)} not loaded within "
                f"{self._load_timeout:.0f}s; cannot switch."
            )
            return
        if self._list_pending or not self._list_cli.service_is_ready():
            return  # try again on the next tick
        self._list_pending = True
        self._list_cli.call_async(ListControllers.Request()).add_done_callback(self._on_list)

    def _on_list(self, future) -> None:
        self._list_pending = False
        try:
            loaded = {c.name for c in future.result().controller}
        except Exception:
            return  # transient; the timer will retry
        if self._need_loaded.issubset(loaded):
            self._cancel_poll()
            self._do_switch()

    def _do_switch(self) -> None:
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
