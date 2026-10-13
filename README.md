# Bài tập 4 — TurtleBot 4

Package `turtlebot4` mô phỏng TurtleBot 4 Standard trong kho Gazebo trên ROS 2 Humble. SLAM Toolbox tạo bản đồ từ LiDAR; Map Server và AMCL định vị trên bản đồ đã lưu; Nav2 lập đường và đưa robot từ A tới một đích B. `launch/turtlebot4.launch.py` giữ một Gazebo, một robot và RViz; `start_slam`, `start_localization`, `prepare_navigation` và `start_navigation` là các lệnh chạy riêng. Cấu hình ở `config/`, bản đồ ở `maps/`, giao diện ở `rviz/`.

## Cài đặt lần đầu

Trong container ROS 2 Humble, chuẩn bị TF2 0.25.24 **một lần** (lần sau chỉ cần source):

```bash
cd /root/ros2_ws
bash src/turtlebot4/scripts/prepare_tf2_runtime.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select turtlebot4
```

## Chuẩn bị bốn terminal

Ở **mỗi terminal mới**, chạy:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source "${XDG_CACHE_HOME:-$HOME/.cache}/turtlebot4/tf2-0.25.24/install/setup.bash"
```

Các lệnh dưới đây dùng `maps/map.yaml` mặc định. Nếu muốn dùng bản đồ khác, đặt **cùng một biến trong Terminal 2, 3 và 4** trước khi chạy các lệnh ROS; tên file được tìm trong `maps/` và kiểm tra cùng ảnh PGM:

```bash
export TURTLEBOT4_MAP=map_test.yaml
```

Không dùng biến đó khi vẫn muốn chạy bản đồ mặc định. Sau khi lưu map mới, phải dừng SLAM trước khi bật AMCL.

## Cách 1 — Quét bản đồ lần đầu

**Terminal 1 — giữ chạy** Gazebo và RViz:

```bash
ros2 launch turtlebot4 turtlebot4.launch.py rviz:=true
```

**Terminal 2 — giữ chạy** SLAM Toolbox:

```bash
ros2 run turtlebot4 start_slam
```

**Terminal 3 — lái robot** sau khi Gazebo sẵn sàng:

```bash
ros2 run turtlebot4 teleop --undock
```

Chọn terminal này để bấm `i` tiến, `,` lùi, `j/l` quay, `u/o` tiến và rẽ, `k` hoặc Space dừng, `q` thoát. Đi qua các lối an toàn và quan sát map lớn dần trong RViz. Dừng robot, rồi thoát teleop trước khi lưu.

**Terminal 4 — kiểm tra và lưu** khi SLAM còn chạy:

```bash
ros2 run turtlebot4 check_mapping
ros2 run turtlebot4 save_map --map-name map_test
ls -lh src/turtlebot4/maps/map_test.yaml src/turtlebot4/maps/map_test.pgm
```

`map_test` giữ nguyên cặp `map.yaml`/`map.pgm` hiện có. Chỉ dùng `ros2 run turtlebot4 save_map` nếu chủ ý thay bản đồ mặc định. Sau khi lưu xong, nhấn Ctrl+C ở Terminal 2 để dừng SLAM. Có thể giữ Terminal 1 cho phiên định vị liên tục, hoặc dừng và khởi động lại cùng world.

## Cách 2 — Có bản đồ, chạy A → B

Nếu đã có Terminal 1 từ lúc quét map, giữ nguyên Gazebo và chuyển Fixed Frame trong RViz sang `map`. Nếu bắt đầu phiên mới, **Terminal 1 — giữ chạy**:

```bash
ros2 launch turtlebot4 turtlebot4.launch.py rviz:=true rviz_fixed_frame:=map
```

**Terminal 2 — giữ chạy** Map Server và AMCL. Bảo đảm SLAM và teleop đã dừng:

```bash
ros2 run turtlebot4 start_localization --timeout 60
```

**Terminal 4 — chạy một lần** lệnh chuẩn bị:

```bash
ros2 run turtlebot4 prepare_navigation --timeout 45
```

Nếu báo chưa có initial pose, dùng **2D Pose Estimate** trong RViz đặt đúng vị trí và hướng **hiện tại** của robot trên map, rồi chạy lại lệnh trên. Không mặc định pose map là `(0,0,0)` theo Gazebo. Nếu đã biết độc lập tọa độ map, có thể dùng `prepare_navigation --x X --y Y --yaw-deg GOC`; cả ba số phải cùng được cung cấp. Nếu AMCL còn bất định, chỉ lái một đoạn ngắn bằng `teleop` khi đường trống, dừng và thoát teleop, rồi chạy lại lệnh chuẩn bị. Không gửi goal cho đến khi thấy `LOCALIZATION READY`. Lệnh này không tự chạy robot để ép hội tụ. Độ chính xác vật lý so với Gazebo là **NOT VERIFIED** nếu không cấp reference độc lập.

**Terminal 3 — giữ chạy** Nav2 sau khi localization READY:

```bash
ros2 run turtlebot4 start_navigation --timeout 45
```

**Terminal 4 — kiểm tra một lần**:

```bash
ros2 run turtlebot4 check_navigation --timeout 30
```

Chỉ khi `PASS`, trong RViz kiểm tra Map, LaserScan, RobotModel và hai costmap, rồi chọn **Nav2 Goal** và đặt **một** đích B ở vùng Free đã biết, có lối đi an toàn từ pose A hiện tại. Theo dõi Global Path, robot trong Gazebo, kết quả `NavigateToPose` **SUCCEEDED**, vận tốc về 0 và không va chạm. Nếu đường vòng bất thường, DWB thất bại lặp lại hoặc có recovery không kiểm soát, hủy goal và kiểm tra lại map/pose/costmap. Kết quả action không tự chứng nhận sai số vật lý; phép đo đó cần reference độc lập được cung cấp rõ ràng qua `--registration /duong/dan/ngoai/repository.json` và được kiểm tra hash/phạm vi.

Sau khi robot dừng, kiểm tra lại AMCL; nếu FAIL thì chưa đạt nghiệm thu A→B dù action báo SUCCEEDED:

```bash
ros2 run turtlebot4 check_localization --timeout 8
```

## Dừng và xử lý lỗi

Nhấn Ctrl+C lần lượt ở **Terminal 3 (Nav2), Terminal 2 (AMCL), Terminal 1 (Gazebo/RViz)**. Nếu đang quét map, dừng teleop rồi SLAM trước khi dừng Gazebo.

- Không thấy map: kiểm tra Map Server/SLAM đã chạy, Fixed Frame `map`, rồi mở lại RViz với `rviz_fixed_frame:=map` nếu cần.
- `LOCALIZATION FAIL`: đặt lại pose thật bằng 2D Pose Estimate; kiểm tra scan chồng đúng vật cản và chờ AMCL hội tụ. Không ép goal.
- Không undock: kiểm tra `/dock_status`, `/undock` và vùng trống quanh dock; lệnh thất bại sẽ có mã thoát khác 0.
- Map không khớp: kiểm tra `TURTLEBOT4_MAP` trong các terminal và khởi động lại AMCL với cùng map; không trộn hai bản đồ.
- Nav2 chưa READY: giữ Terminal 2 chạy, kiểm tra lại `prepare_navigation` và `check_navigation`; không bật SLAM đồng thời.
