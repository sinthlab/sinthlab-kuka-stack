#!/usr/bin/env python3
"""Switch the active ros2_control controller (e.g. joint-position <-> CLIK).

The restricted-plane / maze experiments drive the arm to a precise start posture on the
``lbr_joint_position_command_controller``, then hand off to the ``kuka_clik_controller`` for the
virtual-fixture phase. This action wraps the controller_manager ``switch_controller`` service so the
orchestrator can do that hand-off (and the reverse, before the recover move) as just another step in
the trial.

The fixture controller is spawned ``--inactive`` in PARALLEL with the orchestrator, so at the first
hand-off it may not be ready yet — especially when ``move_to_start`` finishes at once because the arm
is already at the start (the pre-training tasks and the maze share one start pose). The spawner
LOADS the controller and only then CONFIGURES it, and a STRICT switch aborts ("Aborting, no
controller is switched!") if a controller to activate is loaded but still ``unconfigured``, or one to
deactivate is not ``active``. So this action POLLS ``list_controllers`` until every controller is in
the state the switch needs -- to activate: ``inactive``; to deactivate: ``active`` -- and only then
switches. A controller already in its target state is left out of the request. A rejected switch is
retried while ``load_timeout_sec`` lasts, and the states are logged if it still fails.
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

        # Every controller involved must be LOADED, and in the right state, before a STRICT switch.
        self._need_loaded = set(self._activate) | set(self._deactivate)
        self._load_timeout = float(load_timeout_sec)
        self._poll_period = float(poll_period_sec)
        self._poll_timer = None
        self._list_pending = False
        self._deadline = None
        self._last_loaded = set()  # most recent list_controllers result, for a precise timeout message
        self._last_states = {}     # name -> state from the same result

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
            # Report only what is actually MISSING -- printing the whole required set makes an
            # unbuilt/unspawned controller look like a broader failure than it is.
            missing = sorted(self._need_loaded - self._last_loaded)
            if missing:
                self._node.get_logger().error(
                    f"{self._name}: controller(s) {missing} not loaded within {self._load_timeout:.0f}s; "
                    f"cannot switch. (loaded: {sorted(self._last_loaded)}). A missing controller is "
                    f"usually not built/installed, or absent from the controllers YAML.")
            else:
                self._node.get_logger().error(
                    f"{self._name}: controllers not in a switchable state within "
                    f"{self._load_timeout:.0f}s; cannot switch. States: {self._states_text()}. "
                    f"Needed: {self._activate} inactive, {self._deactivate} active. An 'unconfigured' "
                    f"controller usually failed to configure -- see the controller_manager log above.")
            return
        if self._list_pending or not self._list_cli.service_is_ready():
            return  # try again on the next tick
        self._list_pending = True
        self._list_cli.call_async(ListControllers.Request()).add_done_callback(self._on_list)

    def _on_list(self, future) -> None:
        self._list_pending = False
        try:
            states = {c.name: c.state for c in future.result().controller}
        except Exception:
            return  # transient; the timer will retry
        self._last_states = states
        self._last_loaded = set(states)
        if not self._need_loaded.issubset(self._last_loaded):
            return
        # What the switch still has to do; a controller already in its target state is left out.
        todo_on = [c for c in self._activate if states[c] != "active"]
        todo_off = [c for c in self._deactivate if states[c] != "inactive"]
        if not todo_on and not todo_off:
            self._cancel_poll()
            self._node.get_logger().info(f"{self._name}: already in place ({self._states_text()}).")
            self._on_complete()
            return
        ready = (all(states[c] == "inactive" for c in todo_on)
                 and all(states[c] == "active" for c in todo_off))
        if ready:
            self._cancel_poll()
            self._do_switch(todo_on, todo_off)

    def _states_text(self) -> str:
        return ", ".join(f"{n}={self._last_states.get(n, 'not loaded')}"
                         for n in sorted(self._need_loaded))

    def _do_switch(self, activate: List[str], deactivate: List[str]) -> None:
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        self._node.get_logger().info(
            f"{self._name}: activating {activate}, deactivating {deactivate}."
        )
        self._cli.call_async(req).add_done_callback(self._on_response)

    def _on_response(self, future) -> None:
        try:
            ok = future.result().ok
        except Exception as exc:
            self._node.get_logger().error(f"{self._name}: switch service call failed: {exc}")
            return
        if not ok:
            if self._node.get_clock().now() < self._deadline:
                # Rejected (e.g. a state changed between the list and the switch): poll and retry.
                self._node.get_logger().warn(
                    f"{self._name}: switch rejected ({self._states_text()}); retrying.")
                self._poll_timer = self._node.create_timer(self._poll_period, self._poll_loaded)
                return
            self._node.get_logger().error(
                f"{self._name}: controller switch returned not-ok; states {self._states_text()}. "
                f"See the controller_manager log above for its reason.")
            return
        self._node.get_logger().info(f"{self._name}: switch complete.")
        self._on_complete()
