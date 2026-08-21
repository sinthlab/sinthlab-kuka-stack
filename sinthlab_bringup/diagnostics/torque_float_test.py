#!/usr/bin/env python3
"""Rung-1 foundation test for FRI TORQUE mode: command ZERO torque and see if the arm behaves.

Streams LBRTorqueCommand{joint_position = measured, torque = 0} at the state rate. That exercises the
ENTIRE torque path -- FRI commanding, the per-cycle joint position command, session stability -- while
asking the arm to do nothing. With TorqueControl.java's zero-stiffness JointImpedanceControlMode, zero
commanded torque means the arm is in pure CABINET GRAVITY COMPENSATION.

PASS looks like: the arm holds its pose (or drifts only slowly), FRI stays COMMANDING_ACTIVE, and you
can push it by hand and it stays where you leave it. That proves the cabinet, TorqueControl.java, the
tool load data and the FRI torque path are all sound, with none of our control logic involved.

FAIL looks like: the arm sags/drifts on its own (gravity compensation is wrong -> tool load data), or
FRI drops COMMANDING_ACTIVE / StateGuard trips (cabinet-side setup) -- and no amount of ROS-side
tuning will fix either.

NOTE ON lbr's torque_sine_overlay demo: do NOT use it as this test. It applies a 15 Nm sine expecting
the cabinet to be HOLDING the pose. Our app sets cabinet stiffness to zero, so that torque simply
accelerates the joint away -- an aggressive swing that says nothing about whether the setup is sound.

Run with the hardware brought up on ctrl:=lbr_torque_command_controller (see README section 7).
"""
import rclpy
from rclpy.node import Node
from lbr_fri_idl.msg import LBRState, LBRTorqueCommand


class TorqueFloatTest(Node):
    def __init__(self) -> None:
        super().__init__("torque_float_test")
        # WHICH position to echo back as the FRI joint position command:
        #   "measured" - where the arm physically is (what lbr's demos use)
        #   "ipo"      - the CABINET's interpolated setpoint, i.e. the pose positionHold is holding
        # This matters because FRI rejects a commanded position that strays too far from its own
        # reference ("illegal axis delta"). With TorqueControl.java's ZERO cabinet stiffness nothing
        # holds the arm, so it drifts, "measured" drifts with it, and the delta grows until the
        # cabinet aborts. "ipo" always matches the cabinet's reference, so it cannot drift apart.
        self.declare_parameter("position_source", "measured")
        self._source = self.get_parameter("position_source").value
        # Max change of the COMMANDED joint position per FRI cycle [rad]. FRI rejects a commanded
        # position that jumps too much between cycles ("illegal axis delta"), and a raw echo of the
        # measured position jumps as fast as your hand does when you push the arm. 0 disables the
        # limit (raw echo, the upstream behaviour). 0.004 rad/cycle at 5 ms ~= 0.8 rad/s.
        self.declare_parameter("max_delta_rad", 0.004)
        self._max_delta = float(self.get_parameter("max_delta_rad").value)
        self._cmd_pos = None  # rate-limited command state
        self._cmd = LBRTorqueCommand()
        self._pub = self.create_publisher(LBRTorqueCommand, "command/lbr_torque_command", 1)
        self._sub = self.create_subscription(LBRState, "state", self._on_state, 1)
        self._n = 0
        self.get_logger().info(
            f"Zero-torque float test running (position_source='{self._source}'). Expect: arm holds "
            "pose, FRI stays COMMANDING_ACTIVE. Push it by hand -- it should move freely and stay put."
        )

    def _on_state(self, state: LBRState) -> None:
        # Echo a configuration back as the position command, add no torque at all.
        target = list(
            state.ipo_joint_position if self._source == "ipo" else state.measured_joint_position
        )
        if self._cmd_pos is None:
            self._cmd_pos = list(target)
        if self._max_delta > 0.0:
            for i in range(7):
                d = target[i] - self._cmd_pos[i]
                d = max(-self._max_delta, min(self._max_delta, d))
                self._cmd_pos[i] += d
        else:
            self._cmd_pos = list(target)
        self._cmd.joint_position = self._cmd_pos
        self._cmd.torque = [0.0] * 7
        self._pub.publish(self._cmd)
        self._n += 1
        if self._n % 400 == 0:  # ~ every 2 s at 200 Hz
            self.get_logger().info(
                f"still floating ({self._n} cycles, zero torque, position_source='{self._source}')")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TorqueFloatTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
