"""Run the M3 open-close-open gripper acceptance sequence."""

import rclpy
from rclpy.node import Node

from ur3_perception_llm_control.gripper import ParallelJawGripper


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Node("gripper_open_close_test")
    gripper = ParallelJawGripper(node)
    try:
        node.get_logger().info("Starting open -> close -> open test")
        gripper.open()
        gripper.close()
        gripper.open()
        node.get_logger().info("Open -> close -> open test PASSED")
    except Exception as exc:
        node.get_logger().error(f"Open -> close -> open test FAILED: {exc}")
        raise
    finally:
        gripper.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
