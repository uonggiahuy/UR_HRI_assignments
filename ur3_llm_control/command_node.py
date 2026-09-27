"""Minimal command node used to validate the M1 package skeleton."""

from typing import List, Optional

import rclpy
from rclpy.node import Node


SUPPORTED_UR_TYPES = ("ur3", "ur3e")


class CommandNode(Node):
    """Expose basic package parameters without commanding a robot."""

    def __init__(self) -> None:
        super().__init__("ur3_llm_command")

        ur_type = str(self.declare_parameter("ur_type", "ur3e").value)
        if ur_type not in SUPPORTED_UR_TYPES:
            supported = ", ".join(SUPPORTED_UR_TYPES)
            raise ValueError(f"Unsupported ur_type '{ur_type}'. Expected one of: {supported}")

        # rclpy's time source declares use_sim_time for every node.
        use_sim_time = bool(self.get_parameter("use_sim_time").value)

        self.get_logger().info("ur3_llm_control started")
        self.get_logger().info(f"robot type: {ur_type}")
        self.get_logger().info(f"use_sim_time: {str(use_sim_time).lower()}")
        self.get_logger().info("M1 package skeleton ready")


def main(args: Optional[List[str]] = None) -> None:
    """Start the node and shut it down cleanly on keyboard interrupt."""
    rclpy.init(args=args)
    node: Optional[CommandNode] = None
    try:
        node = CommandNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
