#!/usr/bin/env python3
"""Lightweight ROS 2 node for running the admittance controller without gating."""

from __future__ import annotations

from typing import Dict

import numpy as np

import rclpy
from rclpy.node import Node as rclpyNode
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from rcl_interfaces.msg import ParameterEvent, SetParametersResult

from actions.admittance_controller import AdmittanceControlAction


class AdmittanceTuningNode(rclpyNode):
    """Minimal wrapper around `AdmittanceControlAction` for gain tuning."""

    def __init__(self) -> None:
        super().__init__(
            "admittance_controller",
            automatically_declare_parameters_from_overrides=True,
        )

        self._started = False
        self._action = AdmittanceControlAction(
            self,
            to_start=lambda: True,
            in_action=self._mark_started,
            on_complete=self._noop,
        )

        self._node_fqn = self.get_fully_qualified_name()
        self._pending_param_updates: Dict[str, np.ndarray | float] = {}
        self._pending_gain_update = False
        
        # Humble does not have native API to apply the parameters in callback
        self._param_cb_handle = self.add_on_set_parameters_callback(self._on_parameters_update)
        self._parameter_event_sub = self.create_subscription(
            ParameterEvent,
            "/parameter_events",
            self._on_parameter_event,
            QoSProfile(depth=10),
        )

    def _mark_started(self) -> None:
        if self._started:
            return
        self._started = True
        self.get_logger().info("Admittance control active; adjust gains via parameters.")

    @staticmethod
    def _noop() -> None:
        return

    def _on_parameters_update(self, params) -> SetParametersResult:
        pending_updates: Dict[str, np.ndarray | float] = {}
        pending_gain_update = False
        try:
            for param in params:
                if param.name == "dq_gains":
                    if param.type_ != Parameter.Type.DOUBLE_ARRAY:
                        raise ValueError("dq_gains must be a list of floats")
                    values = np.array(param.value, dtype=float)
                    if values.shape[0] != 7:
                        raise ValueError("dq_gains must contain 7 elements")
                    pending_updates["dq_gains"] = values
                    pending_gain_update = True
                elif param.name == "dx_gains":
                    if param.type_ != Parameter.Type.DOUBLE_ARRAY:
                        raise ValueError("dx_gains must be a list of floats")
                    values = np.array(param.value, dtype=float)
                    if values.shape[0] != 6:
                        raise ValueError("dx_gains must contain 6 elements")
                    pending_updates["dx_gains"] = values
                    pending_gain_update = True
                elif param.name == "exp_smooth":
                    if param.type_ not in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                        raise ValueError("exp_smooth must be numeric")
                    value = float(param.value)
                    if not 0.0 <= value <= 1.0:
                        raise ValueError("exp_smooth must be within [0, 1]")
                    pending_updates["exp_smooth"] = value

            self._pending_param_updates = pending_updates
            self._pending_gain_update = pending_gain_update
            return SetParametersResult(successful=True)
        except ValueError as exc:
            self._pending_param_updates = {}
            self._pending_gain_update = False
            return SetParametersResult(successful=False, reason=str(exc))

    def _on_parameter_event(self, event: ParameterEvent) -> None:
        if event.node != self._node_fqn:
            return
        if not self._pending_param_updates:
            return

        changed_names = {
            param.name for param in list(event.new_parameters) + list(event.changed_parameters)
        }
        if not any(name in changed_names for name in self._pending_param_updates):
            return

        updates = self._pending_param_updates

        if "dq_gains" in updates:
            self._action.set_dq_gains_base(updates["dq_gains"])  # type: ignore[arg-type]
        if "dx_gains" in updates:
            self._action.set_dx_gains_base(updates["dx_gains"])  # type: ignore[arg-type]
        if "exp_smooth" in updates:
            self._action.set_exp_smooth(float(updates["exp_smooth"]))

        if self._pending_gain_update:
            self._action.apply_gains()

        self._pending_param_updates = {}
        self._pending_gain_update = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AdmittanceTuningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
