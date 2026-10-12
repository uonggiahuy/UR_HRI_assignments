# Bài tập lớn 4 — Mô phỏng TurtleBot 4 & Lập bản đồ SLAM (Milestone M2)

Tài liệu hướng dẫn thực hành chi tiết bằng tiếng Việt cho gói phần mềm `turtlebot4` trên nền tảng **ROS 2 Humble** và **Ubuntu 22.04**.

---

## 1. Tổng quan và Kiến trúc Hệ thống

Milestone M2 triển khai kiến trúc **Mô phỏng Bền vững (Persistent Base)** kết hợp với khối **SLAM Toolbox** hoạt động độc lập:
- **Terminal 1 (Persistent Base):** Khởi chạy một phiên mô phỏng duy nhất gồm Gazebo Fortress, một robot TurtleBot 4, các cầu nối cảm biến (LiDAR `/scan`, Odometry `/odom`), cây biến đổi tọa độ (TF) và giao diện trực quan RViz2. Phiên này được duy trì liên tục xuyên suốt buổi thực hành và không khởi động lại khi chuyển đổi trạng thái mapping hay navigation.
- **Terminal 2 (Mapping Mode):** Khởi chạy riêng biệt node `slam_toolbox` (chế độ mapping đồng bộ/bất đồng bộ). SLAM Toolbox là đơn vị độc quyền phát sóng transform `map -> odom` và xuất bản bản đồ `/map` (`nav_msgs/msg/OccupancyGrid`).
- **Terminal 3 (Teleoperation):** Điều khiển di chuyển robot bằng bàn phím qua `/cmd_vel` với cơ chế dừng khẩn cấp và thiết lập QoS tương thích (`best-effort`, `volatile`). Hỗ trợ thao tác undock an toàn.
- **Terminal 4 (Map Saver & Diagnostics):** Lưu trữ nguyên tử bản đồ vào `maps/map.yaml` và `maps/map.pgm`, đồng thời cung cấp công cụ chẩn đoán toàn diện `check_mapping`.

```text
               +---------------------------------------------------------+
               |        Terminal 1: Persistent Base Simulator           |
               |  ros2 launch turtlebot4 turtlebot4.launch.py rviz:=true |
               |                                                         |
               |  [Gazebo Fortress] ----> /clock, /scan, /odom, /tf      |
               |  [RViz2 (odom frame)]                                   |
               +---------------------------+-----------------------------+
                                           |
                   +-----------------------+-----------------------+
                   |                                               |
                   v                                               v
+------------------------------------+           +----------------------------------+
|      Terminal 2: Mapping           |           |     Terminal 3: Teleoperation    |
|   ros2 run turtlebot4 start_slam   |           |    ros2 run turtlebot4 teleop    |
|                                    |           |                                  |
| [slam_toolbox]                     |           | Điều khiển bàn phím              |
|  - Thu nhận /scan & /odom          |           | Xuất bản /cmd_vel                |
|  - Phát sóng map -> odom           |           | Tự động gửi zero twist khi dừng |
|  - Xuất bản /map                   |           +----------------------------------+
+------------------+-----------------+
                   |
                   v
+------------------------------------+
|      Terminal 4: Map Saving        |
|    ros2 run turtlebot4 save_map    |
|                                    |
| [nav2_map_server map_saver_cli]    |
|  - Kiểm tra lưới OccupancyGrid     |
|  - Lưu map.yaml & map.pgm          |
|  - Xác thực nguyên tử file ảnh     |
+------------------------------------+
```

---

## 2. Cấu trúc Gói phần mềm (`turtlebot4`)

```text
turtlebot4/
├── launch/
│   ├── turtlebot4.launch.py       # Điểm khởi chạy mô phỏng bền vững (persistent base)
│   ├── simulation.launch.py       # Điểm tương thích M1
│   └── README.md
├── turtlebot4/
│   ├── simulation_launch.py       # Logic ghép nối mô phỏng và quản lý tiến trình
│   ├── slam_runner.py             # Điều phối vòng đời SLAM Toolbox độc lập
│   ├── teleop.py                  # Điều khiển bàn phím an toàn và QoS
│   ├── map_saver.py               # Lưu và kiểm định nguyên tử OccupancyGrid
│   ├── mapping_diagnostics.py     # Bộ công cụ chẩn đoán toàn diện SLAM
│   └── diagnostics.py             # Chẩn đoán cảm biến và đồ thị ROS (M1)
├── maps/
│   ├── map.yaml                   # File cấu hình metadata bản đồ đã lưu
│   └── map.pgm                    # File ảnh nhị phân chiếm dụng thực tế
├── rviz/
│   └── simulation.rviz            # Cấu hình RViz2 bền vững (RobotModel, LaserScan, TF, Map)
├── config/
│   └── simulation.yaml            # Cấu hình lọc và hòa trộn trạng thái khớp
├── test/
│   ├── test_m1.py                 # 37 bài kiểm thử hồi quy M1
│   └── test_m2.py                 # 15 bài kiểm thử offline M2
├── docs/
│   ├── M1_ENGINEERING_REPORT.md   # Báo cáo kỹ thuật M1
│   ├── M2_ENGINEERING_REPORT.md   # Báo cáo kỹ thuật M2
│   └── evidence/                  # Minh chứng đo đạc thời gian thực
├── package.xml
├── setup.py
└── README.md
```

---

## 3. Biên dịch và Kiểm thử Tự động

Mọi lệnh được thực thi trong môi trường **Ubuntu 22.04 / ROS 2 Humble**:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select turtlebot4
source install/setup.bash
colcon test --packages-select turtlebot4
colcon test-result --verbose
```

Kết quả kỳ vọng: **52 tests passed** (37 kiểm thử M1 + 15 kiểm thử M2), 0 lỗi, 0 thất bại.

---

## 4. Hướng dẫn Thực thi Từng Terminal (Step-by-Step)

### Terminal 1 — Khởi chạy Mô phỏng Bền vững (Persistent Base)
Mở cửa sổ dòng lệnh thứ nhất và chạy:

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch turtlebot4 turtlebot4.launch.py rviz:=true
```

- **Tiến trình:** Khởi chạy Gazebo Fortress với thế giới `warehouse`, tạo một robot TurtleBot 4 Standard, kích hoạt các cầu nối cảm biến `/scan`, `/odom`, `/clock`, `/cmd_vel` và mở giao diện RViz2.
- **Trạng thái:** Giữ terminal này chạy liên tục trong suốt toàn bộ phiên làm việc.
- **Quan sát RViz2:** Khung tọa độ cố định (Fixed Frame) là `odom`. Robot model, dữ liệu quét LaserScan (chấm đỏ) và cây tọa độ TF hiển thị chuẩn xác. Mục `Map` sẽ ở trạng thái chờ cho tới khi SLAM được bật.

---

### Terminal 2 — Khởi chạy SLAM Toolbox
Mở cửa sổ dòng lệnh thứ hai và chạy:

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 run turtlebot4 start_slam
```

- **Tiến trình:** Kiểm tra sự sẵn sàng của mô phỏng, xác minh không có node SLAM hoặc AMCL nào đang chạy trùng lặp, sau đó khởi chạy node `slam_toolbox` với `use_sim_time:=true`.
- **Đầu ra kỳ vọng:**
  ```text
  [start_slam] Checking base simulator readiness (timeout 15.0s)...
  [start_slam] Base simulator verified (/clock and LiDAR active).
  [start_slam] Launching: ros2 launch turtlebot4_navigation slam.launch.py use_sim_time:=true sync:=true params:=...
  [sync_slam_toolbox_node-1] [INFO] [slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
  [sync_slam_toolbox_node-1] Registering sensor: [Custom Described Lidar]
  ```
- **Quan sát:** Trên RViz2 (Terminal 1), lớp bản đồ `Map` lập tức nhận dữ liệu từ `/map` và hiển thị các vùng không gian tự do (màu trắng), chướng ngại vật (màu đen) và vùng chưa khám phá (màu xám).

---

### Terminal 3 — Undock và Điều khiển Bàn phím (Teleoperation)
Mở cửa sổ dòng lệnh thứ ba để điều khiển robot khám phá kho hàng:

**Bước 3.1: Undock robot ra khỏi trạm sạc (nếu đang ở trạm sạc)**
```bash
source /opt/ros/humble/setup.bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
```
*Kết quả:* Robot sẽ tự động lùi ra khỏi dock sạc khoảng 0.35m và xoay nhẹ, trạng thái hoàn thành: `SUCCEEDED`.

**Bước 3.2: Khởi động giao diện điều khiển bàn phím**
```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 run turtlebot4 teleop
```

- **Phím điều khiển di chuyển:**
  - `i`: Tiến thẳng (+linear)
  - `,`: Lùi thẳng (-linear)
  - `j`: Quay trái (+angular)
  - `l`: Quay phải (-angular)
  - `u` / `o`: Tiến cua trái / phải
  - `m` / `.`: Lùi cua trái / phải
  - `k` hoặc Phím cách (Space): Dừng khẩn cấp (stop)
- **Phím điều chỉnh tốc độ:**
  - `w` / `x`: Tăng / giảm 10% tốc độ tịnh tiến
  - `e` / `c`: Tăng / giảm 10% tốc độ quay
- **Thoát an toàn:** Nhấn `q` hoặc `Ctrl+C`. Chương trình tự động gửi vận tốc 0 (`linear=0, angular=0`) liên tục xuống `/cmd_vel` để đảm bảo robot đứng yên tuyệt đối trước khi kết thúc.

*Gợi ý điều khiển:* Điều khiển robot đi chậm qua các hành lang chính, đi vòng quanh các kệ hàng và quay trở lại điểm xuất phát để thuật toán SLAM thực hiện khép vòng (loop closure).

---

### Terminal 4 — Kiểm tra Chẩn đoán và Lưu Bản đồ

**Bước 4.1: Kiểm tra chất lượng lập bản đồ thời gian thực**
Mở cửa sổ dòng lệnh thứ tư và chạy:

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 run turtlebot4 check_mapping
```

Đầu ra kỳ vọng:
```text
=================================================================
 TURTLEBOT 4 SLAM MAPPING DIAGNOSTICS REPORT
=================================================================
Overall Status:        [PASS]
-----------------------------------------------------------------
Stack & Nodes:
  SLAM Toolbox Active: YES
  AMCL Conflict:       NONE (Good)
  Stack Isolation:     PASS
-----------------------------------------------------------------
Sensors & Timing:
  /clock Active:       YES
  /scan Valid:         YES
  /odom Valid:         YES
-----------------------------------------------------------------
Transforms (TF):
  map -> odom:         YES
  odom -> base_link:   YES
  map -> base_link:    CONNECTED
-----------------------------------------------------------------
Occupancy Grid (/map):
  Map Received:        YES
  Map Progressing:     YES
  Map Valid:           YES
  Dimensions:          491 x 447 cells (24.55m x 22.35m)
  Resolution:          0.05 m/cell
  Free Cells:          80090 (36.5%)
  Occupied Cells:      1938 (0.9%)
  Unknown Cells:       137449 (62.6%)
=================================================================
```

**Bước 4.2: Lưu bản đồ nguyên tử**
Sau khi robot đã khám phá đủ diện tích và dừng hẳn, chạy lệnh:

```bash
ros2 run turtlebot4 save_map
```

Đầu ra kỳ vọng:
```text
[save_map] Listening for live OccupancyGrid on '/map'...
[save_map] Live map validated: 491x447 cells (24.6m x 22.4m), Free: 80090 (36.5%), Occupied: 1938 (0.9%), Unknown: 137449 (62.6%)
[save_map] Invoking Nav2 map saver: ros2 run nav2_map_server map_saver_cli -t /map -f ...
[save_map] Map files written atomically to:
  - YAML: /root/ros2_ws/src/turtlebot4/maps/map.yaml
  - PGM:  /root/ros2_ws/src/turtlebot4/maps/map.pgm

[save_map] SUCCESS: Bản đồ đã được lưu và kiểm tra thành công!
```

---

## 5. Xác thực File Bản đồ Đã Lưu

Kiểm tra nội dung file cấu hình:
```bash
cat /root/ros2_ws/src/turtlebot4/maps/map.yaml
```
Nội dung chuẩn:
```yaml
image: map.pgm
mode: trinary
resolution: 0.05
origin: [-13.4, -12.6, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

Kiểm tra file ảnh nhị phân PGM:
```bash
ls -lh /root/ros2_ws/src/turtlebot4/maps/map.pgm
```
Kích thước file ảnh xấp xỉ `219 KB`, tương ứng với kích thước ma trận `491 x 447` pixel.

---

## 6. Dừng SLAM và Giữ nguyên Mô phỏng Gazebo

Khi hoàn thành quá trình lập bản đồ:
1. Tại **Terminal 3:** Nhấn `q` để thoát teleop. Robot dừng tại chỗ.
2. Tại **Terminal 2:** Nhấn `Ctrl+C` để dừng `start_slam`.
   Thông báo xác nhận:
   ```text
   [start_slam] Received shutdown signal. Stopping SLAM Toolbox cleanly...
   [start_slam] SLAM Toolbox stopped. Simulator and RViz remain running.
   ```
3. **Kiểm tra tính bền vững:** Tại Terminal 4, chạy lệnh kiểm tra:
   ```bash
   ros2 run turtlebot4 check_simulation --timeout 5
   ```
   *Kết quả:* Mô phỏng Gazebo Fortress, cầu nối cảm biến LiDAR, Odometry và RViz2 tại **Terminal 1** vẫn tiếp tục chạy hoàn hảo, sẵn sàng cho việc nạp bản đồ đã lưu vào AMCL ở giai đoạn tiếp theo mà không cần khởi động lại mô phỏng.

---

## 7. Kịch bản Xác thực Khởi động lại (Restart Scenario)

Ngoài chế độ phiên liên tục, đề bài yêu cầu robot có thể được khởi động lại hoàn toàn từ đầu trong cùng thế giới kho hàng và nạp lại bản đồ vừa lưu:
1. Tắt toàn bộ các terminal bằng `Ctrl+C`.
2. Khởi chạy lại Terminal 1: `ros2 launch turtlebot4 turtlebot4.launch.py rviz:=true`.
3. Bản đồ đã được lưu cố định tại `maps/map.yaml` và `maps/map.pgm` trong mã nguồn gói và sẽ được sử dụng trực tiếp bởi Map Server trong Milestone M3.

---

## 8. Xử lý Sự cố Thường gặp (Troubleshooting)

| Hiện tượng | Nguyên nhân | Cách khắc phục |
|---|---|---|
| `start_slam` báo lỗi `Duplicate SLAM stack rejected` | Có một tiến trình `slam_toolbox` khác đang chạy nền | Dừng terminal SLAM cũ bằng `Ctrl+C` hoặc kiểm tra `pkill -f slam_toolbox` |
| `start_slam` báo lỗi `Incompatible stack rejected` | Node `amcl` đang hoạt động (xung đột thẩm quyền `map -> odom`) | Tắt `amcl` trước khi bật SLAM |
| `save_map` báo lỗi `0 free cells observed` | Robot chưa di chuyển hoặc cảm biến chưa quét được không gian | Điều khiển robot di chuyển bằng `teleop` để quét các khoảng trống |
| RViz2 báo lỗi TF frame `map` không tồn tại trước khi bật SLAM | Trước khi SLAM chạy, chưa có đơn vị xuất bản `map -> odom` | Đây là trạng thái bình thường; cấu hình RViz sử dụng Fixed Frame là `odom` để tránh lỗi |
| Robot không di chuyển khi bấm phím teleop | Robot đang bị khóa trên dock sạc Create 3 | Chạy lệnh undock: `ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"` |

---

## 9. Định hướng Các Milestone Tiếp theo

- **Milestone M3: AMCL Localization (Định vị trên Bản đồ Đã lưu)** — *CHƯA TRIỂN KHAI (NOT YET IMPLEMENTED)*:
  Nạp file `maps/map.yaml` qua `nav2_map_server`, kích hoạt `amcl` để định vị vị trí robot và đặt điểm ước lượng ban đầu (2D Pose Estimate).
- **Milestone M4: Nav2 Autonomous Navigation (Điều hướng Tự động)** — *CHƯA TRIỂN KHAI (NOT YET IMPLEMENTED)*:
  Kích hoạt ngăn xếp Nav2, costmap toàn cục/cục bộ, bộ lập quỹ đạo và thực hiện tự động 3 mục tiêu điều hướng liên tiếp.
