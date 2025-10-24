#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class ApplePluckImpedanceControlNode(Node):
    """
    Placeholder for impedance controller node. This will be started after move_to_start has finished.
    Currently, it only logs that it's running. Add impedance logic here later.
    """

    def __init__(self) -> None:
        super().__init__(
            "apple_pluck_impedance_control",
            automatically_declare_parameters_from_overrides=True,
        )
        self.get_logger().info("ApplePluckImpedanceControlNode started (impedance logic not yet implemented).")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ApplePluckImpedanceControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
