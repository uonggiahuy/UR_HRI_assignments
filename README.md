# Bài tập lớn 4 — TurtleBot 4, SLAM, AMCL và Nav2 (M4.3)

Tài liệu tiếng Việt cho ROS 2 Humble / Ubuntu 22.04 trong container hiện có.

## Trạng thái hiện tại — M4.3: một A→B

**Một action A→B đã SUCCEEDED; M4.3 vẫn FAIL / chưa xác nhận tolerance vật lý.**
Robot đi thực 1,034 m, không recovery, không tiếp xúc bumper hay hazard bị cấm,
và dừng. Sai số pose Nav2 0,239 m / 0,246 rad nằm trong tolerance giữ nguyên
0,25 m / 0,25 rad. Sai số vị trí vật lý đối chiếu độc lập là **0,312 m**, cộng
bất định tham chiếu 0,15 m nên chưa chứng minh đến trong tolerance.
Không gửi goal khác để che kết quả. **M4.3 chỉ yêu cầu một goal A→B**, không yêu
cầu ba goal. M5/M6 chưa triển khai.

AMCL đã hội tụ qua hai phiên restart với quy trình undock/quay giới hạn; chỉ
đứng yên sau initialization vẫn có thể cho covariance nhỏ nhưng pose sai.
Config assignment đổi đúng hai tham số: `laser_model_type=likelihood_field_prob`
và `transform_tolerance=0.2`; noise odom, seed covariance, safety, Unknown và goal
tolerance giữ nguyên. **Build PASS, 195 tests PASS (168 cũ + 27 mới).**
Bản đồ người dùng 603 × 1002 giữ nguyên byte và không commit.
Xem [báo cáo M4.3](docs/M4_3_AMCL_CONVERGENCE_NAVIGATION_REPORT.md),
[bằng chứng action](docs/evidence/m4_3/a_to_b.json) và
[nghiệm thu vật lý](docs/evidence/m4_3/acceptance.json).
Các kết quả ba goal M4 Recovery bên dưới là lịch sử trên bản đồ cũ.

### Quy trình đã kiểm chứng trong mô phỏng

Trong **mọi terminal**, source theo thứ tự, đặc biệt source TF2 sau mỗi build:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source /root/.cache/turtlebot4/tf2-0.25.24/install/setup.bash
```

**Terminal 1**, giữ chạy, sở hữu một Gazebo/robot/RViz:

```bash
ros2 launch turtlebot4 turtlebot4.launch.py rviz:=true rviz_fixed_frame:=map
```

**Terminal 2**, sau khi SLAM đã dừng, giữ chạy, sở hữu Map Server/AMCL:

```bash
ros2 run turtlebot4 start_localization --timeout 60
```

**Terminal 4**, lệnh một lần: đợi cả hai node ACTIVE trước initialization.
Không khởi chạy base/localization lần thứ hai khi chuyển stack.

```bash
ros2 lifecycle get /amcl
ros2 lifecycle get /map_server
```

Xác định pose map **hiện tại** độc lập. Chỉ với default spawn mới restart đã
được xác minh trong báo cáo M4.3, lệnh đã chạy là:

```bash
ros2 run turtlebot4 set_initial_pose --x 0.18782 --y 0.04360 --yaw-deg -0.1463 --timeout 20
```

Không dùng tọa độ này khi robot đã di chuyển hoặc cho phiên mapping đang tiếp tục.
Map origin [-15,-25.1,0] không chứng minh world=map. `set_initial_pose` PASS chỉ
xác nhận initialization, chưa xác nhận hội tụ.

**Terminal 4**, chỉ trong mô phỏng và sau kiểm tra môi trường/dock an toàn:

```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}' --feedback
python3 src/turtlebot4/scripts/observe_amcl_convergence.py --mode rotate --duration 12 --registration src/turtlebot4/docs/evidence/m4_3/registration.json --output /tmp/rotation.json
python3 src/turtlebot4/scripts/observe_amcl_convergence.py --mode stationary --duration 10 --registration src/turtlebot4/docs/evidence/m4_3/registration.json --output /tmp/stability.json
ros2 run turtlebot4 check_localization --timeout 8 --registration src/turtlebot4/docs/evidence/m4_3/registration.json --json
```

Observer là công cụ thử nghiệm có motion rõ ràng, chặn nếu dock/clearance/sensor/
hazard không hợp lệ, xuất zero và kiểm tra odometry dừng. Không dùng để ép PASS.
Đọc `convergence.pass` trong JSON: hoàn tất experiment không đồng nghĩa hội tụ.
Không chạy observer motion/teleop khi Nav2 đang điều khiển. Các ngưỡng XY sigma
≤ 0,30 m, yaw sigma ≤ 15° và freshness vẫn giữ nguyên; gate reference còn cộng
bất định vào residual vị trí và yêu cầu bound ≤ 0,30 m.

Reference độc lập dùng SDF/mesh và scan held-out; chỉ kiểm chứng vùng map
x∈[-4,4], y∈[-4,2], gắn với SHA-256 của map này. Không tự đổi `verified`, origin,
covariance hoặc tạo TF giả. Chưa kiểm chứng toàn warehouse. Nếu localization,
scan/map hay reference FAIL, **dừng trước goal**.

Một phiên trước dừng sau translation còn pose retained lệch TF do update_min_d.
Sau xác nhận robot đã dừng, có thể cần **một** measurement thật qua service đã
phát hiện; không gọi lặp để thu nhỏ covariance:

```bash
ros2 service call /request_nomotion_update std_srvs/srv/Empty '{}'
```

Phiên A→B cuối đạt mà không cần lệnh refresh này. Luôn chẩn đoán lại sau refresh.

**Terminal 3**, chỉ sau toàn bộ gate PASS, giữ chạy và sở hữu Nav2:

```bash
ros2 run turtlebot4 start_navigation --registration src/turtlebot4/docs/evidence/m4_3/registration.json --timeout 45
```

**Terminal 4**, readiness một lần rồi một action A→B trong mô phỏng:

```bash
ros2 run turtlebot4 check_navigation --registration src/turtlebot4/docs/evidence/m4_3/registration.json --timeout 30 --json
python3 src/turtlebot4/scripts/verify_navigation.py --registration src/turtlebot4/docs/evidence/m4_3/registration.json --goals src/turtlebot4/docs/evidence/m4_3/destination.json --goal-timeout 100 --output /tmp/a_to_b.json
```

A là pose hiện tại được đo ngay trước action; file destination có đúng một B
mong muốn. Script chọn ô Free lân cận đủ clearance, kiểm tra Global Path và phạm
vi reference, ghi TF/costmap đúng timestamp, motion, DWB/recovery, action result,
final pose và dừng. Không retry tự động. Lần chạy thật đã SUCCEEDED; reporting
cuối còn tách tolerance vật lý với pose Nav2 và trả NOT VERIFIED khi chưa đủ
chứng cứ. Mở rộng reporting này đã regression test, không gửi thêm action.
Không coi status PASS trong raw evidence cũ là PASS toàn milestone.

Ctrl+C **Terminal 3 → Terminal 2 → Terminal 1**, chỉ dừng process mình sở hữu.
Giữ các owner sống lâu hơn deadline goal; timeout AMCL trước Nav2 có thể gây lỗi
TF giả tạo trong phép thử. Restart riêng phải khởi động lại base/localization,
xác minh spawn và chạy lại gate; không lấy pose cuối phiên trước. Quy trình
mapping liên tục ở các mục M2/M3 vẫn giữ nguyên, nhưng chưa được nghiệm thu
end-to-end trên map hiện tại trong M4.3.

Video M4.3 cần cho thấy một A→B: covariance và scan/map trước goal, registration,
Global Path/costmap, robot đi thật, action SUCCEEDED, sai số pose Nav2 **và**
sai số vật lý, hazard/recovery và dừng. Hiện chưa có link video nghiệm thu; không
quay hoặc mô tả residual 0,312 m như đã đạt tolerance 0,25 m.

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
│   ├── test_m3.py                    # 12 bài kiểm thử offline M3
│   └── test_m3_5.py                  # 19 kiểm thử bảo toàn ô Unknown và Nav2 thật
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

Kết quả kiểm thử tự động toàn diện: **83 tests passed, 0 errors, 0 failures, 0 skipped** (64 kiểm thử M1–M3 + 19 kiểm thử M3.5, bao gồm Map Server và Map Saver Nav2 thật).

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

#### Bước 5: Đọc pose cuối SLAM và khởi tạo bằng terminal (M4.1)

**Thực hiện phần đọc pose trước Bước 3 (dừng SLAM).** Dừng teleop, xác nhận
robot đã đứng yên. Trong Terminal 4, khi SLAM vẫn hoạt động:

```bash
# Chỉ đọc TF thật; hết 5 giây timeout trả mã 124 là bình thường.
timeout 5 ros2 run tf2_ros tf2_echo map base_link
```

Ghi lại `Translation` x/y và `Rotation: in RPY (degree)` yaw từ mẫu cuối có
thời gian đang tăng. Đây là pose của robot hiện tại trong bản đồ SLAM, không
phải `origin` của map.yaml. Không lấy tọa độ từ `/odom` thay cho pose map.
Nếu bản đồ mới đã được lưu bằng `save_map`, giữ nguyên cặp file đó; nếu đang
chẩn đoán bản đồ đã lưu, **không chạy save_map lại chỉ để thử initialization**.

Chỉ dùng pose vừa đọc khi bản đồ đang nạp chính là bản đồ tương ứng, Gazebo
vẫn cùng phiên và robot không di chuyển sau lúc ghi. Nếu không xác nhận được
ba điều kiện này, xác định lại vị trí từ cảnh Gazebo/RViz và nhập thủ công.
M4.1 không tự lưu/nạp lại snapshot: không có cơ chế tự động tái sử dụng pose cũ.
Không dùng pose từ phiên khác hoặc tọa độ M2 lịch sử.

Sau Bước 3 (Ctrl+C terminal SLAM) và Bước 4 (start_localization đang chạy),
Terminal 4 chạy:

```bash
ros2 service call /amcl/get_state lifecycle_msgs/srv/GetState '{}'
ros2 service call /map_server/get_state lifecycle_msgs/srv/GetState '{}'
# Cả hai phải trả id=3, label=active.
read -r -p "Nhập x y yaw(độ) hiện tại trong map: " INITIAL_X INITIAL_Y INITIAL_YAW_DEG
ros2 run turtlebot4 set_initial_pose --x "$INITIAL_X" --y "$INITIAL_Y" --yaw-deg "$INITIAL_YAW_DEG" --timeout 20
ros2 run turtlebot4 check_localization --timeout 15
```

`set_initial_pose` là lệnh một lần, độc lập Nav2. Nó dùng simulation timestamp,
frame `map`, quaternion chuẩn hóa và đủ 36 phần tử covariance. Mặc định gọi
service AMCL `/set_initial_pose`; nếu service không tồn tại, chỉ dùng topic khi
đã tìm thấy subscriber AMCL đúng kiểu. Không fallback sau service timeout.
`--interface topic` là đường topic đã kiểm chứng, có thể chọn rõ khi chẩn đoán.
`--map` và `--namespace` hỗ trợ cấu hình khác; không tự đoán pose (0,0,0).
PASS chỉ sau pose AMCL mới và TF động mới `map -> odom -> base_link`; phản hồi
service rỗng hoặc một lần publish không đủ. FAIL trả mã khác 0; xem log AMCL.

#### Bước 6: RViz và xác minh trước Nav2

1. Mở **Displays → Global Options**, nhấp đúp **ô giá trị bên phải** của
   **Fixed Frame**, gõ `map`, nhấn Enter. Có thể gõ tên dù chưa có trong danh sách.
   Trước initialization, cảnh báo thiếu TF là bình thường; không tạo TF giả.
2. Khi SLAM đang chạy hoặc sau CLI initialization PASS, dùng `map`. Trước SLAM/
   AMCL, dùng `odom` để giữ visualization M1. Camera Target Frame trong cấu hình
   mới là `<Fixed Frame>`, không phụ thuộc `base_link` trước localization.
3. Nếu Map vẫn `No map received`, **bỏ tick rồi tick lại Map** sau khi đổi frame.
   Bản đồ retained từng bị RViz MessageFilter loại do timestamp cũ khi chưa có
   `map -> odom`; đổi frame không luôn phát lại message đó. Không cần mở RViz thứ hai.
   Nếu AMCL Pose cũng bị drop, bỏ/tick lại display đó. Particle Cloud xuất hiện
   khi AMCL xử lý scan; nó không phải topic retained.
4. Quan sát Map, LaserScan, RobotModel, TF, AMCL Pose và Particle Cloud. Scan phải
   khớp tường; nếu lệch, xác định lại pose, không gửi goal.
5. Với GUI đã chọn `map`, **2D Pose Estimate** có thể dùng để chỉnh pose. Nếu dùng
   lúc Fixed Frame=`odom`, RViz gửi `frame_id=odom` và AMCL **từ chối**. Kiểm chứng
   bằng `timeout 10 ros2 topic echo /initialpose --once` trong terminal riêng,
   rồi nhấp/kéo công cụ; frame phải là `map` và log AMCL có `Setting pose`.

Terminal 4 (các lệnh chẩn đoán một lần/có giới hạn):

```bash
ros2 run turtlebot4 check_localization --timeout 15 --json
timeout 5 ros2 topic echo /amcl_pose --once --qos-durability transient_local
timeout 5 ros2 run tf2_ros tf2_echo map base_link
```

`check_localization` kiểm tra lifecycle thật bằng GetState, map đúng nội dung,
clock tăng và TF động còn mới. `/amcl_pose` retained có thể giữ timestamp cũ
khi robot đứng yên; đó không phải heartbeat. CLI initialization yêu cầu output
mới sau request, còn chẩn đoán stationary dựa thêm vào TF/scan đang chạy.
Bản đồ người dùng hiện tại: **603 × 1002**, 0.05 m/cell, origin [-15, -25.1, 0],
505.728 Free / 15.141 Occupied / 83.337 Unknown. Số liệu thay đổi khi chủ động
lập bản đồ mới; các báo cáo M2–M4 cũ là lịch sử.

Chỉ khi localization PASS **và scan/map khớp**, Terminal 2 giữ AMCL, Terminal 3
đã dừng teleop mới chạy `ros2 run turtlebot4 start_navigation --timeout 45` theo
mục M4. Không cần gửi goal để kiểm tra initialization. Dừng Nav2 trước AMCL,
sau đó Ctrl+C Terminal 1 để tắt base và RViz thuộc phiên đó.

---

### Quy trình B: Khởi động lại Toàn bộ và Định vị (Restart and Relocalize)

1. Dừng Nav2, teleop, AMCL/SLAM và base bằng Ctrl+C ở terminal sở hữu tương ứng.
2. Source ROS, assignment và TF2 overlay như mục chuẩn bị; Terminal 1 chạy lại
   `ros2 launch turtlebot4 turtlebot4.launch.py rviz:=true` trong cùng world.
3. Terminal 2 chạy `ros2 run turtlebot4 start_localization`, **không bật SLAM**,
   giữ bản đồ đã lưu. Xác định vị trí thực mới trong hệ map từ cảnh Gazebo.
4. Terminal 4 nhập x/y/yaw hiện tại vào `set_initial_pose` như Bước 5. Không dùng
   pose cuối phiên trước. Chỉ nhập (0,0,0) nếu đã kiểm chứng spawn và hệ map tương ứng.
5. Thực hiện Bước 6: check_localization PASS, Fixed Frame `map`, refresh Map nếu
   cần và xem scan khớp tường. Nếu chạy thử bằng teleop, undock an toàn, di chuyển
   ngắn/chậm rồi dừng; xác minh pose/TF theo chuyển động. Không chạy teleop cùng goal.

M4.1 đã kiểm chứng CLI service/topic, handoff ngắn trong cùng phiên và moving
localization với bản đồ mới; GUI chọn frame `map` và refresh retained map đã
quan sát thật. Xem [báo cáo M4.1](docs/M4_1_LOCALIZATION_INITIAL_POSE_FIX_REPORT.md)
để biết phạm vi replay và giới hạn; không coi đó là video hoàn chỉnh của người dùng.

---

## 5. Bản vá Giao dịch Lưu bản đồ (Transactional Map Saver Patch)

Trong Milestone M3, hàm `save_map` đã được nâng cấp cơ chế ghi file từ sao chép thông thường sang cơ chế **Giao dịch Nguyên tử (Transactional Replacement)**:
1. **Tiền kiểm tra (Pre-validation):** Kiểm tra nội dung OccupancyGrid trực tiếp và các file tạm thời trong thư mục phân lập (`/tmp/tb4_map_save_...`) trước khi tác động vào thư mục đích.
2. **Sao lưu an toàn (Backup staging):** Tạo file dự phòng `.bak` của `map.yaml` và `map.pgm` hiện hữu.
3. **Thay thế nguyên tử (Atomic `os.replace`):** Ghi đè file trong cùng một hệ thống tệp tin qua lệnh nguyên tử của hệ điều hành.
4. **Hậu kiểm tra & Tự động hoàn tác (Post-validation & Rollback):** Xác minh kích thước byte PGM và liên kết ảnh YAML sau khi ghi. Nếu có bất kỳ lỗi đĩa hoặc sai lệch dữ liệu nào, hệ thống tự động phục hồi lại cặp file `.bak` nguyên vẹn và xóa bỏ file tạm.

---

### M3.5 — Bảo toàn vùng chưa quan sát (Unknown)

Bản đồ kho hàng gốc giữ nguyên PGM: **80.090 ô Free, 1.938 ô Occupied,
137.449 ô Unknown**. Metadata dùng `mode: trinary`, `negate: 0`,
`free_thresh: 0.196`, `occupied_thresh: 0.65`. Không đặt lại `free_thresh: 0.25`:
Nav2 sẽ đọc pixel xám 205 thành Free, làm mất toàn bộ vùng chưa quan sát.
`save_map` dùng mặc định đã sửa và từ chối ngưỡng không bảo toàn ba loại ô.
`start_localization` kiểm tra ngữ nghĩa PGM/YAML trước khi khởi chạy;
`check_localization` đối chiếu cả số lượng và bố trí ô của `/map` với PGM.

Sau khi lưu bản đồ và dừng SLAM, tại Terminal 4 đã source môi trường:

```bash
ros2 run turtlebot4 check_localization --timeout 15 --json
```

Kiểm tra `map_server.statistics.free_cells`, `occupied_cells`, `unknown_cells` trong báo cáo.
Các số lượng nêu trên thuộc bản đồ M2 lịch sử; bản đồ mới của người dùng
có số liệu khác. Luôn so sánh `/map` với cặp PGM/YAML hiện tại. Nếu báo lỗi sai lệch, kiểm tra YAML và đường dẫn ảnh,
không biến ô Unknown thành Free để bỏ qua lỗi. Các minh chứng M2/M3 cũ được giữ
làm lịch sử; nhận định M3 về “fully resolved trinary map” đã bị bác bỏ bởi M3.5.
Xem [báo cáo M3.5](docs/M3_5_MAP_CORRECTNESS_REPORT.md) và
[minh chứng AMCL thật](docs/evidence/m3_5/localization.json).

---

## 6. Xử lý Sự cố Thường gặp (Troubleshooting)

| Hiện tượng | Nguyên nhân | Cách khắc phục |
|---|---|---|
| `start_localization` báo lỗi `Active SLAM Toolbox detected` | Tiến trình `slam_toolbox` vẫn đang chạy nền từ giai đoạn trước | Nhấn `Ctrl+C` tại terminal SLAM và xác nhận node SLAM đã dừng; không kill tiến trình không thuộc terminal của mình |
| AMCL báo cảnh báo `AMCL cannot publish a pose or update the transform. Please set the initial pose...` | AMCL đang chờ người dùng cung cấp điểm ước lượng ban đầu | Chạy `set_initial_pose` với pose map hiện tại; chỉ dùng RViz sau khi Fixed Frame=`map` |
| RViz2 báo lỗi TF frame `map` bị đứt gãy hoặc không hiển thị LaserScan | Chưa đặt initial pose nên AMCL chưa phát sóng `map -> odom` | Dùng CLI `set_initial_pose`, chờ PASS rồi gõ `map` vào ô Fixed Frame; refresh Map nếu cần |
| LaserScan không khớp với các bức tường trên bản đồ | Tọa độ ban đầu đặt bị lệch góc quay hoặc vị trí | Dùng công cụ `2D Pose Estimate` nhấp và kéo lại theo đúng vị trí và hướng nhìn của robot |
| `check_localization` báo `No OccupancyGrid received` | Map Server chưa kích hoạt hoặc đường dẫn file YAML sai | Kiểm tra sự tồn tại của `maps/map.yaml` và trạng thái lifecycle của `map_server` |

---

## 7. M4 — Nav2 độc lập sau khôi phục TF2

**Khôi phục runtime: ba goal A/B/C đã trả về `SUCCEEDED`.** Build và **116 kiểm
thử PASS** gồm 109 kiểm thử cũ và 7 kiểm thử recovery. TF2 0.25.23 trong image
hiện tại có deadlock giữa callback scan và listener TF. Dùng dependency upstream
**0.25.24**, không sửa mã vendor hoặc `/opt/ros`. Ngoài ra đã sửa cách đọc timestamp
costmap Humble và bỏ đếm toàn bộ costmap giữa từng callback khi giám sát goal.

Sai số goal theo pose Nav2: 0.234 / 0.235 / 0.240 m (tolerance 0.25 m).
Sai số định vị lớn nhất so với Gazebo: 0.229 m. Riêng goal C, khoảng cách ground
truth tới target là 0.434 m; không coi tolerance theo AMCL là bảo đảm sai số vật
lý 0.25 m. Chưa có video thành công hoặc kiểm chứng lại toàn bộ demo restart.
Xem [báo cáo recovery](docs/M4_RECOVERY_REPORT.md) và
[bằng chứng action thật](docs/evidence/m4_recovery/three_goals.json).
Không gửi goal khi readiness báo FAIL. M5/M6 chưa triển khai.

### Chuẩn bị dependency TF2 (một lần trong container Humble)

```bash
cd /root/ros2_ws
bash src/turtlebot4/scripts/prepare_tf2_runtime.sh
```

Script lấy đúng commit upstream `404b7224d623d614f18fa9738dbf1716403d857e`
(tag 0.25.24), build riêng package `tf2` vào cache người dùng. Nó không cài lại
ROS, không thay middleware, không sửa vendor. Chạy lại script chỉ build cùng
revision. Cần Git, C++ compiler và dependency build Humble đã cài trong container.

Trong **mọi terminal mới**, source theo đúng thứ tự bên dưới; nếu quên overlay
TF2, deadlock có thể xuất hiện lại. Khi thay dependency, dừng stack cũ trước rồi
khởi chạy stack mới; source môi trường không thay thư viện của process đang chạy.

### Thứ tự terminal và quyền sở hữu tiến trình

Trong **mỗi terminal**, dùng môi trường ROS 2 Humble hiện có:

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
source "${XDG_CACHE_HOME:-$HOME/.cache}/turtlebot4/tf2-0.25.24/install/setup.bash"
```

**Terminal 1 — giữ chạy liên tục:**

```bash
ros2 launch turtlebot4 turtlebot4.launch.py rviz:=true
```

Terminal này sở hữu một Gazebo, một robot Standard và một RViz. Không chạy lại
lệnh này khi chuyển stack. Các cảnh báo frame `map` trước khi có AMCL/SLAM là
bình thường; giữ Fixed Frame `odom` ở giai đoạn này.

**Terminal 2 — localization, giữ chạy:**

```bash
ros2 run turtlebot4 start_localization
```

Trong RViz, chuyển **Global Options → Fixed Frame → map**, rồi dùng
**2D Pose Estimate** tại vị trí/hướng thực tế của robot. Nếu vừa restart mô phỏng,
spawn world gần (0,0,0) vẫn phải được đăng ký sang hệ map; không giả định
world=map. Dùng gate và reference M4.3 ở đầu README cho bản đồ hiện tại.

**Terminal 3 — undock trước khi bật Nav2:**

```bash
ros2 run turtlebot4 teleop --undock --non-interactive
```

Đây là lệnh một lần đã chạy trong các chức năng M2/M3. Xác nhận robot thực sự
ra khỏi dock và đứng yên; không chỉ tin dòng chữ “Undock completed”. Không giữ
teleop chạy khi thực thi goal. Kiểm thử M4 dùng action undock có kiểm tra kết quả
và chuyển động tịnh tiến giới hạn để đối chiếu AMCL với ground truth.

**Terminal 4 — kiểm tra localization trước:**

```bash
ros2 run turtlebot4 check_localization --timeout 15 --json
```

Báo cáo này hỗ trợ kiểm tra bản đồ/TF; nhãn node “active” riêng lẻ không thay thế
lifecycle và kiểm tra định vị khi robot di chuyển. Nếu robot ở ô Unknown/Occupied
hoặc định vị không ổn định, dừng lại để phục hồi định vị trước khi bật navigation.

**Terminal 3 — navigation, giữ chạy nếu READY:**

```bash
ros2 run turtlebot4 start_navigation --timeout 45
```

Lệnh này dùng `config/nav2.yaml` của assignment và chỉ include Nav2 navigation
upstream. Nó yêu cầu AMCL/Map Server có sẵn, không khởi chạy Gazebo, RViz, SLAM
hoặc localization lần thứ hai. Thời gian chờ được giới hạn; nếu lỗi, nó dọn các
process Nav2 do chính nó sở hữu và báo lý do. Nó kiểm tra toàn bộ danh sách
lifecycle của launch upstream, action và dữ liệu costmap đang cập nhật.

**Terminal 4 — kiểm tra Nav2, lệnh một lần:**

```bash
ros2 run turtlebot4 check_navigation --timeout 30
ros2 run turtlebot4 check_navigation --timeout 30 --json
```

Kiểm tra lifecycle, tham số thực, TF, footprint, timestamp costmap và action.
`PASS` ở đây chỉ nghĩa là readiness tại thời điểm quan sát, không phải đã chạy
thành công một goal. Bản đồ người dùng hiện tại có **505.728 Free, 15.141 Occupied,
83.337 Unknown**; số liệu M4 cũ chỉ là lịch sử. Costmap có thể có số lượng Unknown khác do layer cảm biến và
inflation; không sửa PGM/YAML để biến vùng chưa quan sát thành Free.

### RViz và xác minh một A→B — chỉ thực hiện khi mọi gate đạt

RViz hiện có **Global Path**, **Global Costmap**, **Local Costmap** và công cụ
**Nav2 Goal**. Không cần mở RViz thứ hai. Khi các điều kiện an toàn đều đạt:

1. Bật Map, LaserScan, AMCL Pose, Particle Swarm và các costmap; kiểm tra scan
   khớp môi trường và footprint theo đúng robot.
2. Chọn **Nav2 Goal**, nhấp và kéo mũi tên trong vùng **Free đã lập bản đồ**, có
   khoảng cách an toàn tới tường/kệ. Không chọn ô Unknown/Occupied.
3. Quan sát Global Path, costmap và chuyển động thật. Ghi start pose, goal pose,
   kết quả action, final pose, dừng xe và dữ liệu va chạm.
4. Chỉ ghi PASS nếu action trả `SUCCEEDED`, robot thực sự tịnh tiến và final pose
   đạt tolerance thực của controller (mặc định 0,25 m / 0,25 rad).
5. M4.3 chạy đúng một A→B có ý nghĩa; không dùng teleop trong goal.
   Goal được nhận, có đường đi, quay tại chỗ hoặc timeout đều không phải thành công.

`GridBased.allow_unknown=false`, `track_unknown_space=true` và planner
`tolerance=0.0` ngăn planner thay goal không hợp lệ bằng một điểm lân cận.
Bộ kiểm tra đường đi kiểm tra cả các đoạn nội suy và từ chối đi qua Unknown,
Occupied hoặc chi phí collision. Không tăng tolerance TF để che dữ liệu cũ.

Script kiểm chứng thực **có gửi action làm robot mô phỏng di chuyển**, chỉ dùng
khi đã undock, AMCL đáng tin cậy và readiness đạt. Nó chọn tọa độ từ bản đồ thật,
kiểm tra clearance/planner và ghi JSON. Dùng quy trình M4.3 ở đầu README; một action đã
SUCCEEDED nhưng tolerance vật lý chưa được xác nhận:

```bash
cd /root/ros2_ws
python3 src/turtlebot4/scripts/verify_navigation.py --help
# Trước action luôn cần reference registration hợp lệ và readiness PASS.
```

Không chạy script này để cố vượt qua lỗi readiness. Không dùng trên robot thật.

### Dừng và quay lại mapping

Dừng goal đang chạy bằng chức năng cancel của Nav2 trước; xác nhận odometry xe
đã dừng. Tại **Terminal 3**, nhấn `Ctrl+C` để dừng Nav2. Terminal 1 và 2 vẫn sở
hữu mô phỏng/RViz và localization. Dừng **Terminal 2** bằng `Ctrl+C`, chờ Map
Server/AMCL thoát, xác nhận `ros2 node list` không còn AMCL/SLAM đang sống. Sau đó
có thể dùng lại lệnh M2 `ros2 run turtlebot4 start_slam`, rồi teleop và save_map;
không khởi chạy thêm Gazebo. Trước khi đóng toàn bộ, dừng stack chức năng trước,
cuối cùng `Ctrl+C` ở Terminal 1. Quy trình restart ở phần M3 vẫn giữ nguyên.

### Hướng dẫn quay video M4.3 (chưa có video nghiệm thu)

Ghi đồng thời Gazebo, RViz và terminal diagnostics/action result. Cho thấy một
robot, một simulator và một RViz; bản đồ Unknown vẫn còn. Quay bước initialize,
undock, kiểm tra định vị khi di chuyển, start_navigation và tham số thực. Với
một A→B, quay mũi tên goal, Global Path, costmap, chuyển động, `SUCCEEDED`,
final pose và xe dừng; không cắt bỏ lỗi/va chạm. Nếu lỗi TF/costmap hoặc cancel,
ghi rõ FAIL thay vì ghi video như đã thành công. Ảnh hiện có ở
[RViz thử nghiệm thất bại](docs/evidence/m4/rviz_failed_trial.png) và
[Gazebo thử nghiệm thất bại](docs/evidence/m4/gazebo_failed_trial.png) là ảnh thật,
không phải minh chứng M4.3 PASS. Chưa có link video nghiệm thu.

M5/M6 chưa triển khai. Các ảnh M4 cũ giữ nguyên trạng thái FAIL; chỉ evidence
`m4_recovery` chứa lần chạy ba goal lịch sử. `m4_3` ghi một A→B SUCCEEDED và
sai số vật lý chưa đạt chứng nhận tolerance. Chưa có video nghiệm thu mới.
