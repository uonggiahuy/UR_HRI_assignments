# Bài tập lớn 4 — Mô phỏng TurtleBot 4, Lập bản đồ SLAM & Định vị AMCL (Milestone M3)

Tài liệu hướng dẫn thực hành chi tiết bằng tiếng Việt cho gói phần mềm `turtlebot4` trên nền tảng **ROS 2 Humble** và **Ubuntu 22.04**.

---

## 1. Tổng quan và Kiến trúc Hệ thống

Milestone M3 hoàn thiện và tích hợp kiến trúc **Mô phỏng Bền vững (Persistent Base)**, **Bản đồ Chiếm dụng Thực tế (Saved Map)** từ M2 và ngăn xếp **Định vị Thích nghi Monte Carlo (AMCL Localization)** hoạt động độc lập:

- **Terminal 1 (Persistent Base):** Khởi chạy một phiên mô phỏng duy nhất gồm Gazebo Fortress, một robot TurtleBot 4, các cầu nối cảm biến (LiDAR `/scan`, Odometry `/odom`), cây biến đổi tọa độ (TF) và giao diện trực quan RViz2. Phiên này chạy liên tục xuyên suốt buổi thực hành và không bao giờ bị khởi động lại khi chuyển đổi giữa chế độ mapping và localization.
- **Terminal 2 (Stack Chức năng Độc lập — Mapping hoặc Localization):**
  - *Khi Lập bản đồ (M2):* Chạy `ros2 run turtlebot4 start_slam`. SLAM Toolbox độc quyền phát sóng `map -> odom`.
  - *Khi Định vị (M3):* Dừng SLAM bằng `Ctrl+C`, sau đó chạy `ros2 run turtlebot4 start_localization`. Map Server nạp bản đồ đã lưu và AMCL trở thành thẩm quyền phát sóng duy nhất của `map -> odom`.
- **Terminal 3 (Teleoperation & Khởi tạo Vị trí):** Điều khiển di chuyển robot bằng bàn phím qua `/cmd_vel` với cơ chế dừng khẩn cấp, hoặc gửi mục tiêu undock khi robot còn trên dock sạc.
- **Terminal 4 (Kiểm tra & Chẩn đoán Toàn diện):** Chạy `ros2 run turtlebot4 check_localization` để xác thực trạng thái vòng đời Map Server, AMCL, độ hội tụ của hạt `/amcl_pose`, tính toàn vẹn TF và tính tương thích của bản đồ.

```text
+-----------------------------------------------------------------------------------------+
|                                    PERSISTENT BASE                                      |
|                 ros2 launch turtlebot4 turtlebot4.launch.py rviz:=true                  |
|                                                                                         |
|  [Ignition Gazebo Fortress]                                                             |
|    - Physics & World: warehouse.sdf                                                     |
|    - TurtleBot 4 Standard + Create 3 Base                                               |
|    - Clock Bridge: /clock (333 Hz)                                                      |
|    - LiDAR Bridge: /scan (62.5 Hz, sensor_msgs/msg/LaserScan)                           |
|    - DiffDrive Controller: /odom (62 Hz, nav_msgs/msg/Odometry)                         |
|    - Motion Control: /cmd_vel (best-effort subscriber)                                  |
|    - TF static & dynamic publishers: odom -> base_link -> links                         |
|  [RViz2]                                                                                |
|    - Fixed frame: odom (mặc định) -> chuyển sang map khi định vị                        |
|    - Displays: RobotModel, LaserScan, TF, Map, AMCL Pose, Particle Swarm                |
|    - Tool: 2D Pose Estimate (xuất bản tới /initialpose)                                 |
+-----------------------------------------------------------------------------------------+
                                          |
                +-------------------------+-------------------------+
                |                                                   |
                v (Sau khi dừng SLAM)                               v
+---------------------------------------+         +---------------------------------------+
|      LOCALIZATION (Terminal 2)        |         |      TELEOPERATION (Terminal 3)       |
| ros2 run turtlebot4 start_localization|         |       ros2 run turtlebot4 teleop      |
|                                       |         |                                       |
|  - Tiền kiểm tra: loại trừ SLAM Toolbox|        |  - Undock Create 3 an toàn            |
|  - Map Server: nạp maps/map.yaml      |         |  - Ramping vận tốc bàn phím           |
|  - AMCL: bộ lọc hạt thích nghi        |         |  - QoS khớp chuẩn /cmd_vel            |
|  - Phát sóng duy nhất: map -> odom    |         |  - Dừng xe tức thời khi thoát         |
|  - Thu nhận /initialpose              |         +---------------------------------------+
+---------------------------------------+
                |
                v
+---------------------------------------+
|     CHẨN ĐOÁN AMCL (Terminal 4)       |
|  ros2 run turtlebot4 check_localization|
|                                       |
|  - Xác thực Map Server & AMCL ACTIVE  |
|  - Kiểm tra tính hợp lệ /amcl_pose    |
|  - Kiểm tra chuỗi TF map->odom->base  |
|  - Xuất báo cáo JSON / Text tự động   |
+---------------------------------------+
```

---

## 2. Cấu trúc Gói phần mềm (`turtlebot4`)

```text
turtlebot4/
├── launch/
│   ├── turtlebot4.launch.py          # Điểm khởi chạy mô phỏng bền vững (persistent base)
│   ├── simulation.launch.py          # Điểm tương thích M1
│   └── README.md
├── turtlebot4/
│   ├── simulation_launch.py          # Logic ghép nối mô phỏng và quản lý tiến trình
│   ├── slam_runner.py                # Điều phối vòng đời SLAM Toolbox độc lập (M2)
│   ├── teleop.py                     # Điều khiển bàn phím an toàn và QoS (M2)
│   ├── map_saver.py                  # Bản vá giao dịch nguyên tử lưu bản đồ (M2/M3)
│   ├── mapping_diagnostics.py        # Bộ công cụ chẩn đoán SLAM (M2)
│   ├── localization_runner.py        # Trình khởi chạy AMCL & Map Server độc lập (M3)
│   ├── localization_diagnostics.py   # Chẩn đoán AMCL, TF và kiểm định bản đồ (M3)
│   └── diagnostics.py                # Chẩn đoán cảm biến và đồ thị ROS (M1)
├── maps/
│   ├── map.yaml                      # File metadata bản đồ kho hàng tạo từ M2
│   └── map.pgm                       # File ảnh chiếm dụng (491x447 ô, 0.05 m/ô)
├── rviz/
│   ├── simulation.rviz               # RViz2 cấu hình bền vững (Map, AMCL Pose, Swarm, 2D Pose)
│   └── README.md
├── config/
│   └── simulation.yaml               # Cấu hình lọc và hòa trộn trạng thái khớp
├── test/
│   ├── test_m1.py                    # 37 bài kiểm thử hồi quy M1
│   ├── test_m2.py                    # 15 bài kiểm thử offline M2
│   └── test_m3.py                    # 12 bài kiểm thử offline M3
├── docs/
│   ├── M1_ENGINEERING_REPORT.md      # Báo cáo kỹ thuật M1
│   ├── M2_ENGINEERING_REPORT.md      # Báo cáo kỹ thuật M2
│   ├── M3_ENGINEERING_REPORT.md      # Báo cáo kỹ thuật M3
│   └── evidence/m3/                  # Minh chứng đo đạc thời gian thực M3
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

Kết quả kiểm thử tự động toàn diện: **64 tests passed, 0 errors, 0 failures, 0 skipped** (37 kiểm thử M1 + 15 kiểm thử M2 + 12 kiểm thử M3).

---

## 4. Hướng dẫn Thực hành — Quy trình Kép (Dual Workflows)

Milestone M3 hỗ trợ và xác thực thành công cả hai quy trình vận hành:

### Quy trình A: Trình diễn Phiên liên tục (Continuous Demonstration)

Đây là quy trình khuyến nghị khi quay video demo và đánh giá thực tế:

#### Bước 1: Khởi chạy Mô phỏng Bền vững (Terminal 1)
```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 launch turtlebot4 turtlebot4.launch.py rviz:=true
```
*Ghi chú:* Terminal này được giữ chạy liên tục. Gazebo Fortress và RViz2 hiển thị thế giới kho hàng (`warehouse.sdf`) với Create 3 và TurtleBot 4 tại điểm xuất phát.

#### Bước 2: Bật SLAM, Quét Bản đồ và Lưu (Terminal 2 & 3)
```bash
# Terminal 2: Bật SLAM
ros2 run turtlebot4 start_slam

# Terminal 3: Điều khiển quét bản đồ
ros2 run turtlebot4 teleop

# Terminal 4: Lưu bản đồ khi hoàn thành
ros2 run turtlebot4 save_map
```

#### Bước 3: Dừng SLAM an toàn và Chuyển đổi Thẩm quyền TF
1. Tại **Terminal 3:** Nhấn `q` để dừng teleop. Robot đứng yên tại chỗ.
2. Tại **Terminal 2:** Nhấn `Ctrl+C` để dừng SLAM Toolbox.
   Xác nhận đầu ra:
   ```text
   [start_slam] Received shutdown signal. Stopping SLAM Toolbox cleanly...
   [start_slam] SLAM Toolbox stopped. Simulator and RViz remain running.
   ```
3. Xác nhận không còn node SLAM nào bằng `ros2 node list | grep slam` (phải rỗng).

#### Bước 4: Khởi chạy Định vị AMCL (Terminal 2)
Trong cùng Terminal 2, chạy lệnh khởi tạo AMCL và Map Server:
```bash
ros2 run turtlebot4 start_localization
```
Trình điều phối sẽ tự động:
- Kiểm tra tính tồn tại và hợp lệ của `maps/map.yaml` và `maps/map.pgm`.
- Xác nhận không có tiến trình SLAM nào đang chạy (nếu SLAM còn bật, lệnh lập tức từ chối và thoát an toàn).
- Nạp bản đồ qua `nav2_map_server` và kích hoạt `amcl` ở chế độ mô phỏng (`use_sim_time:=true`).
- Cả hai node đạt trạng thái lifecycle `ACTIVE`.

#### Bước 5: Đặt Điểm Ước lượng Vị trí Ban đầu (2D Pose Estimate)
Trong cửa sổ RViz2:
1. Đổi **Global Options -> Fixed Frame** từ `odom` sang `map`.
2. Quan sát thấy bản đồ kho hàng hiển thị rõ nét với các bức tường và lối đi.
3. Chọn công cụ **2D Pose Estimate** trên thanh công cụ phía trên (hoặc phím tắt `p`).
4. Nhấp và kéo chuột tại vị trí thực tế hiện tại của robot trên bản đồ theo hướng đầu xe.
   - Nếu robot vẫn ở vị trí xuất phát: tọa độ x ≈ 0.0, y ≈ 0.0, yaw ≈ 0.0 rad.
   - Nếu robot vừa khám phá xong M2: tọa độ x ≈ -2.23, y ≈ -1.51, yaw ≈ 2.53 rad.

*(Tùy chọn CLI):* Có thể gửi vị trí ban đầu qua dòng lệnh bằng cách:
```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}, covariance: [0.25, 0, 0, 0, 0, 0, 0, 0.25, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.068]}}'
```

Ngay sau khi nhận `/initialpose`, AMCL thiết lập các hạt định vị, giải phóng cảnh báo và bắt đầu phát sóng transform `map -> odom`.

#### Bước 6: Chẩn đoán Định vị AMCL (Terminal 4)
Chạy bộ công cụ kiểm tra độc lập:
```bash
ros2 run turtlebot4 check_localization
```
Đầu ra chuẩn:
```text
====================================================================
 TURTLEBOT 4 AMCL LOCALIZATION DIAGNOSTICS REPORT
====================================================================
Overall Status:        [PASS]
--------------------------------------------------------------------
Stack & Nodes:
  AMCL Active:         YES
  Map Server Active:   YES
  SLAM Conflict:       NONE (Good)
  Stack Isolation:     PASS
--------------------------------------------------------------------
Sensors & Timing:
  /clock Active:       YES
  /scan Valid:         YES
  /odom Valid:         YES
--------------------------------------------------------------------
Transforms (TF):
  map -> odom:         YES (AMCL)
  odom -> base_link:   YES
  map -> base_link:    CONNECTED
--------------------------------------------------------------------
Map Server (/map):
  Map Received:        YES
  Map Valid:           YES
  Dimensions:          491 x 447 cells (24.55m x 22.35m)
  Resolution:          0.05 m/cell
  Free Cells:          217539
--------------------------------------------------------------------
AMCL Localization (/amcl_pose):
  Pose Received:       YES (1 msgs)
  Pose Valid:          YES
  Position (x,y,z):    [0.004, -0.022, 0.0]
  Heading (yaw):       0.3 deg (0.005 rad)
  Variance (x, y, yaw): {'x': 0.24259, 'y': 0.17904, 'yaw': 0.06277}
====================================================================
```

---

### Quy trình B: Khởi động lại Toàn bộ và Định vị (Restart and Relocalize)

Quy trình này chứng minh tính độc lập hoàn toàn của ngăn xếp định vị: robot được sinh ra trong một phiên mô phỏng hoàn toàn mới, không phụ thuộc vào dữ liệu TF hay node SLAM cũ:

1. **Tắt toàn bộ:** Dừng mọi terminal bằng `Ctrl+C`.
2. **Khởi chạy mô phỏng mới (Terminal 1):**
   ```bash
   ros2 launch turtlebot4 turtlebot4.launch.py rviz:=true
   ```
3. **Bật thẳng AMCL Localization (Terminal 2):**
   ```bash
   ros2 run turtlebot4 start_localization
   ```
   *Lưu ý:* Tuyệt đối không bật SLAM. Map Server tự động nạp `maps/map.yaml` đã lưu từ trước.
4. **Đặt vị trí xuất phát:**
   Trong RViz2 (Fixed Frame: `map`), chọn **2D Pose Estimate** nhấp vào điểm gốc `(0, 0)` hướng về phía trước, hoặc chạy lệnh pub `/initialpose` với `x=0, y=0, w=1`.
5. **Điều khiển xe chạy thử (Terminal 3):**
   Khởi chạy `ros2 run turtlebot4 teleop`, thực hiện undock và điều khiển xe di chuyển vài mét.
6. **Kiểm tra độ hội tụ (Terminal 4):**
   Chạy `ros2 run turtlebot4 check_localization`. Vị trí robot trên bản đồ di chuyển chính xác theo chuyển động xe, đám mây hạt (particle cloud) co cụm quanh vị trí thực và phương sai (variance) giảm dần.

---

## 5. Bản vá Giao dịch Lưu bản đồ (Transactional Map Saver Patch)

Trong Milestone M3, hàm `save_map` đã được nâng cấp cơ chế ghi file từ sao chép thông thường sang cơ chế **Giao dịch Nguyên tử (Transactional Replacement)**:
1. **Tiền kiểm tra (Pre-validation):** Kiểm tra nội dung OccupancyGrid trực tiếp và các file tạm thời trong thư mục phân lập (`/tmp/tb4_map_save_...`) trước khi tác động vào thư mục đích.
2. **Sao lưu an toàn (Backup staging):** Tạo file dự phòng `.bak` của `map.yaml` và `map.pgm` hiện hữu.
3. **Thay thế nguyên tử (Atomic `os.replace`):** Ghi đè file trong cùng một hệ thống tệp tin qua lệnh nguyên tử của hệ điều hành.
4. **Hậu kiểm tra & Tự động hoàn tác (Post-validation & Rollback):** Xác minh kích thước byte PGM và liên kết ảnh YAML sau khi ghi. Nếu có bất kỳ lỗi đĩa hoặc sai lệch dữ liệu nào, hệ thống tự động phục hồi lại cặp file `.bak` nguyên vẹn và xóa bỏ file tạm.

---

## 6. Xử lý Sự cố Thường gặp (Troubleshooting)

| Hiện tượng | Nguyên nhân | Cách khắc phục |
|---|---|---|
| `start_localization` báo lỗi `Active SLAM Toolbox detected` | Tiến trình `slam_toolbox` vẫn đang chạy nền từ giai đoạn trước | Nhấn `Ctrl+C` tại terminal SLAM hoặc kiểm tra `pkill -f slam_toolbox` |
| AMCL báo cảnh báo `AMCL cannot publish a pose or update the transform. Please set the initial pose...` | AMCL đang chờ người dùng cung cấp điểm ước lượng ban đầu | Sử dụng công cụ `2D Pose Estimate` trên RViz2 hoặc gửi `/initialpose` |
| RViz2 báo lỗi TF frame `map` bị đứt gãy hoặc không hiển thị LaserScan | Chưa đặt initial pose nên AMCL chưa phát sóng `map -> odom` | Đặt 2D Pose Estimate trong RViz2 để kích hoạt chuỗi transform `map -> odom -> base_link` |
| LaserScan không khớp với các bức tường trên bản đồ | Tọa độ ban đầu đặt bị lệch góc quay hoặc vị trí | Dùng công cụ `2D Pose Estimate` nhấp và kéo lại theo đúng vị trí và hướng nhìn của robot |
| `check_localization` báo `No OccupancyGrid received` | Map Server chưa kích hoạt hoặc đường dẫn file YAML sai | Kiểm tra sự tồn tại của `maps/map.yaml` và trạng thái lifecycle của `map_server` |

---

## 7. Định hướng Milestone Tiếp theo

- **Milestone M4: Nav2 Autonomous Navigation (Điều hướng Tự động)** — *CHƯA TRIỂN KHAI (NOT YET IMPLEMENTED)*:
  Tích hợp ngăn xếp Nav2 hoàn chỉnh (Costmap toàn cục/cục bộ, Planner, Controller, Recovery Behaviors) tái sử dụng phiên AMCL bền vững đã được hoàn thiện trong M3, thực thi tuần tự 3 mục tiêu điều hướng tự động trong kho hàng.
