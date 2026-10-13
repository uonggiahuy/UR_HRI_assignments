"""Controlled keyboard teleoperation wrapper with safe stopping and QoS handling."""

import argparse
import math
import os
import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import Twist
import rclpy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from turtlebot4.simulation_launch import validate_namespace

try:
    from irobot_create_msgs.action import Undock
    from irobot_create_msgs.msg import DockStatus
    HAVE_CREATE_MSGS = True
except ImportError:
    HAVE_CREATE_MSGS = False


MIN_LINEAR = 0.05
MAX_LINEAR = 0.50
MIN_ANGULAR = 0.10
MAX_ANGULAR = 1.50
DEFAULT_LINEAR = 0.25
DEFAULT_ANGULAR = 0.50

TELEOP_BANNER = """
====================================================================
TURTLEBOT 4 KEYBOARD TELEOPERATION (ĐIỀU KHIỂN BẰNG BÀN PHÍM)
====================================================================
Điều khiển di chuyển / Moving around:
       u (tiến trái)    i (tiến thẳng)    o (tiến phải)
       j (quay trái)    k (dừng lại)      l (quay phải)
       m (lùi trái)     , (lùi thẳng)     . (lùi phải)

Điều chỉnh tốc độ / Speed adjustment:
  w / x : tăng / giảm 10% tốc độ tịnh tiến (linear speed +/- 10%)
  e / c : tăng / giảm 10% tốc độ quay (angular speed +/- 10%)

Dừng an toàn & Thoát / Safe stop & Quit:
  Phím space (dấu cách) / k : dừng khẩn cấp (stop)
  q hoặc Ctrl+C             : dừng robot và thoát (stop & quit)
====================================================================
"""


def clamp_speed(value: float, min_val: float, max_val: float) -> float:
    return max(min_val, min(max_val, round(value, 3)))


def calculate_twist(key: str, linear_speed: float, angular_speed: float) -> tuple[float, float]:
    """Map key press to (linear_x, angular_z) velocities."""
    bindings = {
        'i': (1.0, 0.0),
        ',': (-1.0, 0.0),
        'j': (0.0, 1.0),
        'l': (0.0, -1.0),
        'u': (1.0, 1.0),
        'o': (1.0, -1.0),
        'm': (-1.0, 1.0),
        '.': (-1.0, -1.0),
        'k': (0.0, 0.0),
        ' ': (0.0, 0.0),
    }
    if key in bindings:
        scale_x, scale_z = bindings[key]
        return scale_x * linear_speed, scale_z * angular_speed
    return 0.0, 0.0


def adjust_speed(key: str, linear: float, angular: float, factor: float = 1.1) -> tuple[float, float]:
    """Adjust linear and angular velocity bounds safely."""
    if key == 'w':
        linear = clamp_speed(linear * factor, MIN_LINEAR, MAX_LINEAR)
    elif key == 'x':
        linear = clamp_speed(linear / factor, MIN_LINEAR, MAX_LINEAR)
    elif key == 'e':
        angular = clamp_speed(angular * factor, MIN_ANGULAR, MAX_ANGULAR)
    elif key == 'c':
        angular = clamp_speed(angular / factor, MIN_ANGULAR, MAX_ANGULAR)
    return linear, angular


def cmd_vel_qos() -> QoSProfile:
    """QoS profile matching TurtleBot 4 /cmd_vel subscriber."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=5,
    )


class TeleopNode(Node):
    def __init__(self, namespace: str = '', cmd_topic: str | None = None):
        super().__init__('turtlebot4_teleop')
        self.namespace = validate_namespace(namespace)
        if cmd_topic is None:
            self.cmd_topic = f'/{self.namespace}/cmd_vel' if self.namespace else '/cmd_vel'
        else:
            self.cmd_topic = cmd_topic
            
        self.publisher = self.create_publisher(Twist, self.cmd_topic, cmd_vel_qos())
        self.is_docked = False
        self.dock_status_received = False
        self.dock_sub = None

        if HAVE_CREATE_MSGS:
            dock_topic = f'/{self.namespace}/dock_status' if self.namespace else '/dock_status'
            self.dock_sub = self.create_subscription(
                DockStatus, dock_topic, self._dock_cb,
                QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                           durability=DurabilityPolicy.VOLATILE,
                           history=HistoryPolicy.KEEP_LAST, depth=5)
            )

    def _dock_cb(self, msg):
        self.is_docked = getattr(msg, 'is_docked', False)
        self.dock_status_received = True

    def publish_twist(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.publisher.publish(msg)

    def stop_robot(self, repetitions: int = 5):
        """Repeatedly publish zero velocity to ensure the robot stops."""
        for _ in range(repetitions):
            self.publish_twist(0.0, 0.0)
            time.sleep(0.02)


def undock_robot(node: TeleopNode, timeout: float = 10.0) -> bool:
    """Send undock action goal if Create 3 undock action is available."""
    if not HAVE_CREATE_MSGS:
        print("[teleop] irobot_create_msgs not available; cannot verify undock.")
        return False

    deadline = time.monotonic() + timeout
    while not node.dock_status_received and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(.1, deadline - time.monotonic()))
    if not node.dock_status_received:
        print('[teleop] Dock status unavailable; cannot verify undock.')
        return False
    if not node.is_docked:
        print('[teleop] Robot is already undocked.')
        return True

    action_name = f'/{node.namespace}/undock' if node.namespace else '/undock'
    client = ActionClient(node, Undock, action_name)
    def finish(success, message):
        print(message)
        client.destroy()
        return success
    print(f"[teleop] Connecting to undock action server '{action_name}'...")
    if not client.wait_for_server(timeout_sec=max(0., deadline - time.monotonic())):
        return finish(False, f"[teleop] Undock server not found on {action_name}.")

    print("[teleop] Sending undock goal...")
    goal_msg = Undock.Goal()
    future = client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, future, timeout_sec=max(0., deadline - time.monotonic()))
    if not future.done() or future.result() is None or not future.result().accepted:
        return finish(False, "[teleop] Undock goal rejected or timed out.")

    print("[teleop] Undock goal accepted. Waiting for result...")
    result_future = future.result().get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=max(0., deadline - time.monotonic()))
    if not result_future.done() or result_future.result() is None or result_future.result().status != GoalStatus.STATUS_SUCCEEDED:
        return finish(False, '[teleop] Undock action failed or timed out.')
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(.1, deadline - time.monotonic()))
        if node.dock_status_received and not node.is_docked:
            return finish(True, '[teleop] Undock completed; dock_status confirms is_docked=false.')
    return finish(False, '[teleop] Undock action succeeded but dock_status still reports docked.')


def get_key_nonblocking(settings=None, timeout: float = 0.1) -> str:
    """Read a single keypress from standard input non-blockingly."""
    if select.select([sys.stdin], [], [], timeout)[0]:
        return sys.stdin.read(1)
    return ''


def run_interactive_loop(node: TeleopNode, linear: float, angular: float):
    print(TELEOP_BANNER)
    print(f"Current limits: Linear = {linear:.2f} m/s | Angular = {angular:.2f} rad/s")
    print(f"Publishing to:  {node.cmd_topic}\n")

    if not sys.stdin.isatty():
        print("[teleop] Standard input is not a terminal; exiting interactive mode.")
        return

    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        target_linear = 0.0
        target_angular = 0.0

        while rclpy.ok():
            key = get_key_nonblocking(timeout=0.1)
            rclpy.spin_once(node, timeout_sec=0.01)

            if key:
                if key in ('\x03', 'q'):  # Ctrl+C or q
                    break
                elif key in ('w', 'x', 'e', 'c'):
                    linear, angular = adjust_speed(key, linear, angular)
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                    print(f"\r[teleop] Tốc độ cập nhật: Linear = {linear:.2f} m/s | Angular = {angular:.2f} rad/s  ", end='', flush=True)
                    tty.setraw(sys.stdin.fileno())
                elif key in ('i', ',', 'j', 'l', 'u', 'o', 'm', '.', 'k', ' '):
                    target_linear, target_angular = calculate_twist(key, linear, angular)
                    node.publish_twist(target_linear, target_angular)
                    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
                    action = "TIẾN" if target_linear > 0 else "LÙI" if target_linear < 0 else "DỪNG"
                    turn = "QUAY TRÁI" if target_angular > 0 else "QUAY PHẢI" if target_angular < 0 else ""
                    print(f"\r[teleop] Vận tốc: {target_linear:+.2f} m/s, {target_angular:+.2f} rad/s ({action} {turn})       ", end='', flush=True)
                    tty.setraw(sys.stdin.fileno())
            else:
                # Keep active motion alive if continuing
                if target_linear != 0.0 or target_angular != 0.0:
                    node.publish_twist(target_linear, target_angular)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print("\n\n[teleop] Dừng robot và thoát an toàn...")
        node.stop_robot(repetitions=10)
        print("[teleop] Robot đã dừng lại (velocity = 0). Kết thúc điều khiển.")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Keyboard teleoperation wrapper for TurtleBot 4.'
    )
    parser.add_argument('--namespace', default='', help='Robot namespace')
    parser.add_argument(
        '--linear-speed', type=float, default=DEFAULT_LINEAR,
        help=f'Initial linear speed in m/s ({MIN_LINEAR}..{MAX_LINEAR})'
    )
    parser.add_argument(
        '--angular-speed', type=float, default=DEFAULT_ANGULAR,
        help=f'Initial angular speed in rad/s ({MIN_ANGULAR}..{MAX_ANGULAR})'
    )
    parser.add_argument('--cmd-topic', default=None, help='Override /cmd_vel topic')
    parser.add_argument('--undock', action='store_true', help='Trigger undock before teleop')
    parser.add_argument('--non-interactive', action='store_true', help='Exit immediately after validation')

    args = parser.parse_args(argv)

    linear = clamp_speed(args.linear_speed, MIN_LINEAR, MAX_LINEAR)
    angular = clamp_speed(args.angular_speed, MIN_ANGULAR, MAX_ANGULAR)

    rclpy.init()
    node = TeleopNode(namespace=args.namespace, cmd_topic=args.cmd_topic)
    try:
        if args.undock:
            if not undock_robot(node):
                print('[teleop] FAIL: robot did not undock.', file=sys.stderr)
                return 1

        if not args.non_interactive:
            run_interactive_loop(node, linear, angular)
        else:
            print("[teleop] Non-interactive mode: stopping robot and exiting.")
            node.stop_robot(repetitions=3)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
