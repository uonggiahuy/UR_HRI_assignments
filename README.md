# Bài tập 4 — TurtleBot 4: lập bản đồ và điều hướng

Package `turtlebot4` chạy TurtleBot 4 Standard trong kho hàng Gazebo trên ROS 2 Humble. Robot dùng LiDAR và odometry để quét bản đồ bằng SLAM Toolbox; sau đó Map Server nạp bản đồ đã lưu, AMCL định vị robot và Nav2 lập đường đi, điều khiển robot tới đích. Gazebo và RViz chạy trong một terminal riêng, còn SLAM, AMCL và Nav2 được bật khi cần.

## Các file chính

| File/thư mục | Vai trò |
| --- | --- |
| `launch/turtlebot4.launch.py` | Lệnh khởi chạy chính: một Gazebo, một robot, cảm biến và RViz tùy chọn. |
| `launch/simulation.launch.py`, `turtlebot4/simulation_launch.py` | Launch mô phỏng ban đầu và phần dùng chung để dựng Gazebo/robot. |
| `turtlebot4/slam_runner.py`, `teleop.py`, `map_saver.py` | Các lệnh `start_slam`, `teleop`, `save_map` cho quy trình quét bản đồ. |
| `turtlebot4/localization_runner.py`, `initial_pose.py` | Các lệnh `start_localization`, `set_initial_pose` để nạp bản đồ và khởi tạo AMCL. |
| `turtlebot4/navigation_runner.py` | Lệnh `start_navigation` bật Nav2 trên mô phỏng và AMCL đang chạy. |
| `turtlebot4/diagnostics.py`, `turtlebot4/*_diagnostics.py` | Các lệnh kiểm tra `check_simulation`, `check_mapping`, `check_localization`, `check_navigation`. |
| `config/localization.yaml`, `config/nav2.yaml` | Cấu hình AMCL và Nav2. |
| `maps/map.yaml`, `maps/map.pgm` | Bản đồ kho hàng đã lưu; YAML tham chiếu ảnh PGM. |
| `rviz/simulation.rviz`, `rviz/navigation.rviz` | Giao diện RViz cho lúc quét map và lúc định vị/điều hướng. |

## Chuẩn bị terminal

Chạy các lệnh ROS trong container Ubuntu 22.04/ROS 2 Humble của workspace `/root/ros2_ws`. Nếu vừa sửa source, build package **một lần**:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select turtlebot4
```

Trong **mỗi terminal mới**, source theo thứ tự:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source /root/.cache/turtlebot4/tf2-0.25.24/install/setup.bash
```

### Quy trình 1: Quét và lưu map

1. **Terminal 1 — giữ chạy:** mở Gazebo với một robot và RViz. Mặc định mô phỏng là `warehouse`, robot `standard`.

   ```bash
   ros2 launch turtlebot4 turtlebot4.launch.py rviz:=true
   ```

2. **Terminal 2 — giữ chạy:** bật SLAM Toolbox. Trong RViz, theo dõi `/map` lớn dần khi robot di chuyển.

   ```bash
   ros2 run turtlebot4 start_slam
   ```

3. **Terminal 3 — giữ chạy khi lái:** undock rồi điều khiển bằng bàn phím. Đặt focus vào terminal này, đi chậm qua các lối đi và quan sát vật cản trong Gazebo/RViz. Khi quét đủ vùng cần dùng, nhấn **Ctrl+C** để teleop dừng robot.

   ```bash
   ros2 run turtlebot4 teleop --undock
   ```

4. **Terminal 4 — lệnh chạy một lần:** kiểm tra bản đồ đang xuất hiện và lưu khi SLAM vẫn chạy, robot đã đứng yên.

   ```bash
   ros2 run turtlebot4 check_mapping --json
   ros2 run turtlebot4 save_map
   ls -lh src/turtlebot4/maps/map.yaml src/turtlebot4/maps/map.pgm
   ```

   `save_map` mặc định ghi vào `maps/map.yaml` và `maps/map.pgm`, **thay thế bản đồ hiện có** sau khi kiểm tra. Nếu muốn giữ bản đồ hiện tại, dùng `ros2 run turtlebot4 save_map --map-name map_moi`; khi định vị với bản đồ đó, truyền `--map /root/ros2_ws/src/turtlebot4/maps/map_moi.yaml` cho các lệnh AMCL/Nav2 tương ứng.

5. Nhấn **Ctrl+C ở Terminal 2** sau khi lưu thành công để dừng SLAM. Giữ Terminal 1 nếu tiếp tục cùng phiên mô phỏng; không chạy SLAM và AMCL cùng lúc.

### Quy trình 2: Định vị và chạy Nav2 trên map đã lưu

Các lệnh dưới đây dùng `maps/map.yaml` mặc định. Nếu bắt đầu một phiên riêng, **Terminal 1 — giữ chạy** mở lại Gazebo/RViz với Fixed Frame `map`:

```bash
ros2 launch turtlebot4 turtlebot4.launch.py rviz:=true rviz_fixed_frame:=map
```

Nếu Terminal 1 từ quy trình quét map vẫn đang chạy, giữ nguyên simulator đó; chuyển Fixed Frame trong RViz sang `map` sau khi AMCL có TF. Nếu Map display không hiện lại bản đồ đã lưu, dùng chức năng refresh Map trong RViz.

Với **bản đồ hiện tại**, Terminal 3 và Terminal 4 cùng đặt đường dẫn reference độc lập dưới đây. `docs/` chỉ có trên máy này, không được đẩy lên GitHub. Nếu file không có hoặc vừa quét map mới, cần tạo và xác minh reference cho đúng map trước khi gửi goal.

```bash
export TB4_MAP_REF=/root/ros2_ws/src/turtlebot4/docs/evidence/m4_3/registration.json
test -f "$TB4_MAP_REF"
```

1. **Terminal 2 — giữ chạy:** bảo đảm SLAM và teleop đã dừng, rồi nạp cùng bản đồ qua Map Server và bật AMCL. Chờ Map Server/AMCL đạt trạng thái `active`.

   ```bash
   ros2 run turtlebot4 start_localization --timeout 60
   ```

2. **Terminal 4 — lệnh chạy một lần:** kiểm tra trạng thái, rồi trong RViz dùng **2D Pose Estimate** đặt pose theo **vị trí và hướng hiện tại của robot trong map**. Spawn gần `(0,0)` trong Gazebo không có nghĩa pose map là `(0,0)`. Nếu pose map đã được xác minh độc lập, có thể dùng `ros2 run turtlebot4 set_initial_pose --x X --y Y --yaw-deg GOC` với các số thực thay cho `X`, `Y`, `GOC`.

   ```bash
   ros2 lifecycle get /map_server
   ros2 lifecycle get /amcl
   ```

3. **Terminal 4 — kiểm tra hội tụ:** xem robot/scan trong RViz có khớp map không. AMCL có thể cần một chuyển động ngắn, có kiểm soát để hội tụ: chỉ undock/lái khi đường trống, rồi dừng teleop trước khi kiểm tra lại. `set_initial_pose` thành công hoặc covariance nhỏ riêng lẻ chưa chứng minh robot ở đúng chỗ.

   ```bash
   ros2 run turtlebot4 check_localization --timeout 8 --registration "$TB4_MAP_REF" --json
   ```

   Xác nhận `overall_pass` và `ground_truth_accuracy_verified` đều là `true`; đồng thời kiểm tra robot/scan khớp vật cản trên map. Covariance nhỏ riêng lẻ không đủ. Nếu một gate chưa đạt, **không gửi goal**.

4. **Terminal 3 — giữ chạy:** sau khi định vị đạt và không còn teleop, bật Nav2.

   ```bash
   ros2 run turtlebot4 start_navigation --registration "$TB4_MAP_REF" --timeout 45
   ```

5. **Terminal 4 — lệnh chạy một lần:** kiểm tra Nav2. Nếu kết quả PASS, chọn **một đích B** có ý nghĩa trong vùng Free đã quét bằng công cụ **Nav2 Goal** của RViz. Với reference M4.3, chỉ chọn trong vùng map x từ -4 đến 4 m, y từ -4 đến 2 m. Theo dõi Global Path, costmap và robot di chuyển trong Gazebo; yêu cầu action `NavigateToPose` trả `SUCCEEDED`, robot đến trong goal tolerance, không va chạm và đứng yên sau khi đến.

   ```bash
   ros2 run turtlebot4 check_navigation --registration "$TB4_MAP_REF" --timeout 30 --json
   ```

   Sau kết quả action, Terminal 4 có thể xem `/odom` để kiểm tra vận tốc đã về gần 0:

   ```bash
   ros2 topic echo /odom --once
   ```

   Chỉ có đường đi hoặc goal được nhận chưa đủ để kết luận điều hướng thành công. Trong thử nghiệm M4.3, một goal A→B đã `SUCCEEDED`, nhưng sai số đến đo bằng Gazebo khoảng **0,312 m**, nên chưa xác nhận nằm trong tolerance vị trí **0,25 m**.

Dừng theo thứ tự **Terminal 3 (Nav2) → Terminal 2 (AMCL) → Terminal 1 (Gazebo/RViz)** bằng Ctrl+C. Nếu khởi động lại robot, đặt lại initial pose theo vị trí mới và chạy lại các kiểm tra; không dùng pose của phiên trước.
