#!/usr/bin/env python3

import time
import numpy as np

import rclpy
from rclpy.node import Node as rclpyNode
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from std_msgs.msg import Bool
from rcl_interfaces.msg import ParameterEvent, SetParametersResult

from helpers.common_threshold import DoneGate, create_transient_bool_publisher, get_required_param
from actions.admittance_controller import AdmittanceControlAction


class AdmittanceControlNode(rclpyNode):
    """ROS 2 wrapper that hosts the admittance control action."""

    def __init__(self) -> None:
        super().__init__(
            "admittance_control",
            automatically_declare_parameters_from_overrides=True,
        )

        self._move_done_topic = str(get_required_param(self, "move_done_topic"))
        self._move_done_gate = DoneGate(self, self._move_done_topic)

        self._force_bias_done_topic = str(get_required_param(self, "force_bias_done_topic"))
        self._force_bias_gate = DoneGate(self, self._force_bias_done_topic)

        self._admittance_in_action_topic = str(get_required_param(self, "admittance_start_topic"))
        self._admittance_in_action = create_transient_bool_publisher(self, self._admittance_in_action_topic)
        self._admittance_in_action_published = False

        self._force_release_topic = str(get_required_param(self, "force_release_done_topic"))
        self._force_release_gate = DoneGate(self, self._force_release_topic)
        self._hold_ready_topic = str(get_required_param(self, "hold_ready_topic"))
        self._hold_ready_gate = DoneGate(self, self._hold_ready_topic)

        self._action = AdmittanceControlAction(
            self,
            to_start=self._to_start_action,
            in_action=self._in_action,
            on_complete=self._on_action_complete,
        )
        
        # Ros2 dynamic parameter handling
        # https://docs.ros.org/en/humble/Concepts/Basic/About-Parameters.html
        self._node_fqn = self.get_fully_qualified_name()
        self._pending_param_updates = {}
        self._pending_gain_update = False
        self._param_cb_handle = self.add_on_set_parameters_callback(self._on_parameters_update)

        self._parameter_event_cb_handle = None
        self._parameter_event_sub = None
        # Humble does not have native API to apply the parameters in callback
        qos = QoSProfile(depth=10)
        self._parameter_event_sub = self.create_subscription(
            ParameterEvent,
            "/parameter_events",
            self._on_parameter_event,
            qos,
        )
    
    def _to_start_action(self) -> bool:
        return self._move_done_gate.done and self._force_bias_gate.done
	
    def _in_action(self) -> None:
        if self._admittance_in_action_published:
            return
        self._admittance_in_action_published = True
        try:
            self._admittance_in_action.publish(Bool(data=True))
            self.get_logger().info(f"Admittance control action has started; published done=true in {self._admittance_in_action_topic}.")
            # Allow some time for subscribers to latch onto the message
            time.sleep(get_required_param(self, "subscriber_latch_delay_sec"))
        except Exception:
            self.get_logger().warn(f"Failed to publish to {self._admittance_in_action_topic};")
    
    def _on_action_complete(self) -> None:
        if self._hold_ready_gate.done:
            self.get_logger().info(
                f"Detected displacement hold ready on '{self._hold_ready_topic}'; stopping admittance control."
            )
            self._shutdown()
            return

        if not self._force_release_gate.done:
            return
        self.get_logger().info(
            f"Detected force release on '{self._force_release_topic}'; stopping admittance control."
        )
        time.sleep(get_required_param(self, "subscriber_latch_delay_sec"))
        self._shutdown()

    def _shutdown(self) -> None:
        self.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def _on_parameters_update(self, params) -> SetParametersResult:
        pending_updates = {}
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
                elif param.name == "stiffness_scale":
                    if param.type_ not in (Parameter.Type.DOUBLE, Parameter.Type.INTEGER):
                        raise ValueError("stiffness_scale must be numeric")
                    value = float(param.value)
                    if not 0.0 < value <= 1.0:
                        raise ValueError("stiffness_scale must be in (0, 1]")
                    pending_updates["stiffness_scale"] = value
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
            self._action.set_dq_gains_base(updates["dq_gains"])
        if "dx_gains" in updates:
            self._action.set_dx_gains_base(updates["dx_gains"])
        if "stiffness_scale" in updates:
            self._action.set_stiffness_scale(updates["stiffness_scale"])
        if "exp_smooth" in updates:
            self._action.set_exp_smooth(updates["exp_smooth"])

        if self._pending_gain_update:
            self._action.apply_gains()

        self._pending_param_updates = {}
        self._pending_gain_update = False
	
def main(args=None) -> None:
	rclpy.init(args=args)
	node = AdmittanceControlNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	node.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
	main()
