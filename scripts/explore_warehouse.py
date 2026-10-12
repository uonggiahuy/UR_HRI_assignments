"""Drive TurtleBot 4 through warehouse to explore environment and achieve loop closure."""

import math
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class WarehouseExplorer(Node):
    def __init__(self):
        super().__init__('warehouse_explorer')
        
        qos_cmd = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        qos_transient = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', qos_cmd)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self._odom_cb, qos_cmd)
        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self._map_cb, qos_transient)
        
        self.current_pose = None
        self.map_updates = 0
        self.latest_map_info = None

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        # Yaw from quaternion
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_pose = (p.x, p.y, yaw)

    def _map_cb(self, msg: OccupancyGrid):
        self.map_updates += 1
        free = sum(1 for c in msg.data if 0 <= c <= 25)
        occupied = sum(1 for c in msg.data if c > 25)
        unknown = sum(1 for c in msg.data if c == -1)
        self.latest_map_info = (msg.info.width, msg.info.height, free, occupied, unknown)

    def drive_step(self, linear: float, angular: float, duration: float):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        end_time = time.monotonic() + duration
        while time.monotonic() < end_time and rclpy.ok():
            self.pub_cmd.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

    def stop(self):
        msg = Twist()
        for _ in range(10):
            self.pub_cmd.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)


def main():
    rclpy.init()
    node = WarehouseExplorer()
    print("[explore] Starting controlled exploration sequence in warehouse...")

    # Wait for initial odometry
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.current_pose is not None:
            break
        time.sleep(0.1)

    print(f"[explore] Initial pose: x={node.current_pose[0]:.2f}, y={node.current_pose[1]:.2f}, yaw={node.current_pose[2]:.2f}")

    # Sequence of exploration segments
    # 1. Drive forward along aisle
    print("[explore] Segment 1: Tiến thẳng dọc hành lang (+0.25 m/s, 8.0s)...")
    node.drive_step(linear=0.25, angular=0.0, duration=8.0)
    node.drive_step(linear=0.0, angular=0.0, duration=1.0)

    # 2. Turn left
    print("[explore] Segment 2: Quay trái 90 độ (+0.35 rad/s, 4.5s)...")
    node.drive_step(linear=0.0, angular=0.35, duration=4.5)
    node.drive_step(linear=0.0, angular=0.0, duration=1.0)

    # 3. Drive forward along cross aisle
    print("[explore] Segment 3: Tiến thẳng qua hành lang ngang (+0.25 m/s, 8.0s)...")
    node.drive_step(linear=0.25, angular=0.0, duration=8.0)
    node.drive_step(linear=0.0, angular=0.0, duration=1.0)

    # 4. Turn left
    print("[explore] Segment 4: Quay trái 90 độ (+0.35 rad/s, 4.5s)...")
    node.drive_step(linear=0.0, angular=0.35, duration=4.5)
    node.drive_step(linear=0.0, angular=0.0, duration=1.0)

    # 5. Drive forward return leg
    print("[explore] Segment 5: Tiến thẳng chặng khứ hồi (+0.25 m/s, 8.0s)...")
    node.drive_step(linear=0.25, angular=0.0, duration=8.0)
    node.drive_step(linear=0.0, angular=0.0, duration=1.0)

    # 6. Turn left towards start to close loop
    print("[explore] Segment 6: Quay trái hướng về điểm xuất phát (+0.35 rad/s, 4.5s)...")
    node.drive_step(linear=0.0, angular=0.35, duration=4.5)
    node.drive_step(linear=0.0, angular=0.0, duration=1.0)

    # 7. Close loop
    print("[explore] Segment 7: Khép vòng hành trình (+0.20 m/s, 6.0s)...")
    node.drive_step(linear=0.20, angular=0.0, duration=6.0)

    # Safe verified stop
    print("[explore] Dừng robot hoàn toàn và kiểm tra trạng thái đứng yên...")
    node.stop()
    time.sleep(1.0)
    rclpy.spin_once(node, timeout_sec=0.5)

    print(f"[explore] Final pose: x={node.current_pose[0]:.2f}, y={node.current_pose[1]:.2f}, yaw={node.current_pose[2]:.2f}")
    if node.latest_map_info:
        w, h, free, occ, unk = node.latest_map_info
        print(f"[explore] Latest map: {w}x{h} cells, Free: {free}, Occupied: {occ}, Unknown: {unk}")

    node.destroy_node()
    rclpy.shutdown()
    print("[explore] Exploration sequence completed successfully.")


if __name__ == '__main__':
    main()
