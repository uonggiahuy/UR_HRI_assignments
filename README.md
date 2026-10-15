# Bài tập 5 — TurtleBot 4: ba đích tự hành và nhận diện YOLO

Package ROS 2 Humble `turtlebot4` chạy TurtleBot 4 Standard trong kho Gazebo Fortress, dùng lại bản đồ Bài 4, định vị AMCL và đi tuần tự **Goal 1 → Goal 2 → Goal 3** bằng Python. YOLO11n xử lý ảnh camera thật của mô phỏng đồng thời với Nav2, vẽ khung, nhãn và độ tin cậy. Không cần bấm Nav2 Goal trong RViz.

Nhánh nộp bài: [assignments_5](https://github.com/uonggiahuy/UR_HRI_assignments/tree/assignments_5). Nhánh `assignments_4` được giữ nguyên.

## Cài đặt một lần

Thực hiện trong container ROS Ubuntu 22.04, tại `/root/ros2_ws`; không cài ROS Humble lên host Ubuntu 24.04. Cần TurtleBot 4 Simulator, Fortress, Nav2, AMCL, SLAM Toolbox, RViz2 và các dependency trong `package.xml`.

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
sudo apt-get update
sudo apt-get install -y python3.10-venv python3-pip
rosdep install --ignore-src --from-paths src/turtlebot4 -r -y
bash src/turtlebot4/scripts/prepare_tf2_runtime.sh
bash src/turtlebot4/scripts/prepare_yolo_runtime.sh
colcon build --symlink-install --packages-select turtlebot4
source install/setup.bash
```

Nếu đang là `root`, bỏ `sudo`. TF2 0.25.24 được build ngoài repository để tránh lỗi callback của TF2 cũ; mỗi terminal phải source overlay này **sau** workspace. Script YOLO tạo venv riêng có quyền đọc các package ROS của hệ thống; không nâng cấp NumPy/OpenCV/PyTorch toàn cục. Bộ phiên bản đã kiểm tra: Python 3.10, NumPy 1.26.4, OpenCV 4.10.0.84, PyTorch 2.5.1 CPU, torchvision 0.20.1, Ultralytics 8.3.40. Inference dùng hai luồng CPU, giới hạn xuất khoảng 5 FPS để dành tài nguyên cho Nav2.

YOLO11n được tải một lần từ release chính thức Ultralytics vào `~/.cache/turtlebot4/models/yolo11n.pt`; cần Internet khi chuẩn bị lần đầu. Không lưu weights trong Git. SHA-256 của weights đã chạy: `0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1`.

Mã nền giữ Apache-2.0; node `yolo_detector.py` dùng AGPL-3.0-or-later, kèm văn bản tại `licenses/AGPL-3.0.txt`. Ultralytics và weights áp dụng [giấy phép của Ultralytics](https://www.ultralytics.com/license). Mô hình người/ghế CC BY 4.0 được ghi nguồn tại `models/ATTRIBUTION.md`.

## Chuẩn bị cả bốn terminal

Chạy đoạn này trong **mỗi terminal mới**:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
source "${XDG_CACHE_HOME:-$HOME/.cache}/turtlebot4/tf2-0.25.24/install/setup.bash"
```

Không chạy thêm simulator, SLAM hoặc teleop khi đang thực hiện nhiệm vụ tự hành.

### Terminal 1 — giữ Gazebo, robot, vật thể và RViz chạy

```bash
ros2 launch turtlebot4 turtlebot4.launch.py \
  rviz:=true rviz_fixed_frame:=map assignment5_objects:=true
```

Đợi robot, LiDAR và `/clock` hoạt động. Chưa bật AMCL thì RViz có thể báo chưa có frame `map`. Scene Bài 5 chỉ hỗ trợ `warehouse`, `standard`, namespace rỗng. Mặc định `assignment5_objects:=false` giữ tương thích Bài 4; `simulation.launch.py` vẫn hoạt động.

### Terminal 2 — giữ Map Server và AMCL chạy

```bash
ros2 run turtlebot4 start_localization
```

Đợi lifecycle `map_server` và `amcl` ACTIVE trước bước tiếp theo. Không bật SLAM đồng thời.

### Terminal 3 — chuẩn bị định vị, sau đó giữ Nav2 chạy

```bash
ros2 run turtlebot4 prepare_navigation
```

Lệnh một lần này undock nếu cần, rồi kiểm tra định vị. **Nếu chưa có initial pose:** sau khi undock đã kết thúc, dùng **2D Pose Estimate** trong RViz đặt đúng vị trí và hướng **hiện tại** trên bản đồ; chạy lại `prepare_navigation`. Undock có thể đổi hướng robot, vì vậy không dùng lại hướng lúc vừa spawn. Không mặc định `(0,0,0)` trong Gazebo cũng là `(0,0,0)` trên map.

Nếu AMCL còn bất định, kiểm tra scan chồng đúng vật cản. Khi vùng quanh robot thật sự trống và Nav2 chưa chạy, có thể dùng `ros2 run turtlebot4 teleop` để quay chậm/đi ngắn; dừng và thoát bằng `q`, rồi kiểm tra lại. Không bỏ qua cổng covariance hoặc freshness. Nếu đã đo độc lập tọa độ hiện tại, có thể truyền đủ `--x X --y Y --yaw-deg GOC` vào `prepare_navigation`.

Chỉ khi thấy **LOCALIZATION READY**, chạy:

```bash
ros2 run turtlebot4 start_navigation
```

Giữ terminal này chạy. Chờ thông báo READY cho lifecycle, action và costmaps. Không gửi goal thủ công.

### Terminal 4 — chạy toàn bộ nhiệm vụ

Có thể kiểm tra trước bằng lệnh một lần:

```bash
ros2 run turtlebot4 check_navigation --timeout 30
```

Sau khi PASS:

```bash
ros2 launch turtlebot4 assignment5.launch.py
```

Launch chạy `waypoint_navigator` và `yolo_detector` đồng thời. Navigator chỉ gửi goal sau khi localization/Nav2 đạt các kiểm tra hiện có, robot undocked, không có teleop cạnh tranh, và đã nhận hai ảnh YOLO mới. Các đường nối ba goal được kiểm tra với planner và costmap trước khi chạy; từng goal được lập đường lại từ pose hiện tại.

Mỗi goal có timeout 120 giây; readiness có timeout 60 giây. Goal bị từ chối/thất bại, cảm biến/TF/costmap cũ, camera mất, nguy cơ va chạm hoặc recovery/DWB lỗi lặp lại sẽ dừng nhiệm vụ. Node không bỏ qua goal lỗi. Sau mỗi goal, robot phải dừng; một lần `/request_nomotion_update` yêu cầu AMCL xử lý scan thật để cập nhật đoạn dịch chuyển cuối nhỏ hơn ngưỡng odometry. Sau Goal 3, kiểm tra vận tốc dừng, lưu báo cáo và tắt cả hai node. Gazebo, AMCL và Nav2 vẫn thuộc ba terminal trước.

Báo cáo JSON mặc định ở `~/.local/state/turtlebot4/assignment5/latest.json`, ngoài repository; có danh sách action results, recoveries, vận tốc dừng và detections khớp timestamp odometry trong khi chuyển động. `PASS` cần cả ba action SUCCEEDED và ít nhất một detection thuộc lớp mục tiêu khi robot chuyển động. Action SUCCEEDED không chứng nhận sai số vật lý nếu chưa có phép đăng ký map/world độc lập.

## Bản đồ và ba đích

Cặp `maps/map.yaml` / `maps/map.pgm` là bản đồ warehouse Bài 4 được giữ nguyên, không tạo lại. Resolution 0.05 m/cell, origin `[-15, -25.1, 0]`, kích thước 603×1002. Các giá trị PGM hiện có: Free 505728, Occupied 15141, Unknown 83337 cell.

| File | SHA-256 |
| --- | --- |
| map.yaml | `b1be9283ae2c966e54ce6000133263b62fcf7daa2a84cce61c94b210622bf8c9` |
| map.pgm | `915c88f67974aa4e160f2679cc17c538efceddc1e2ed2f9eb0cf05000b3a4955` |

`config/goals.yaml` chứa ba pose frame `map`, đơn vị mét/radian:

| Thứ tự | Tên | x | y | yaw | Nội dung quan sát |
| --- | --- | --- | --- | --- | --- |
| 1 | north_of_dock | -0.525 | 1.225 | -0.140 | Rời vùng dock, quay về ghế |
| 2 | chair_view | 0.775 | 1.325 | -0.300 | Tiếp cận khu vực nhìn rõ ghế |
| 3 | person_view | -0.525 | 2.425 | 3.100 | Quay về người ở bên tuyến |

Ba cell đích đều Free; khoảng cách tới cell non-Free lần lượt khoảng 2.61, 2.26, 3.68 m. Khoảng trống này chỉ tính map gốc; planner/costmap đang chạy kiểm tra thêm dock và vật thể mới. Không giảm robot radius, inflation, unknown-space restriction hoặc dung sai đích 0.25 m / 0.25 rad.

Các runner Bài 4 vẫn hỗ trợ `TURTLEBOT4_MAP` hoặc `--map`. Tuy nhiên tuyến Bài 5 khóa theo hash bản đồ: chọn map khác mà không đo lại và tạo cấu hình goal tương ứng sẽ bị từ chối. Không sửa hash chỉ để bỏ qua kiểm tra.

## Vật thể và camera

Không sửa world SDF hay vendor packages. Assets người/ghế có mesh, texture và collision được đóng gói sẵn; không cần Fuel cache riêng cho hai vật thể mới. Warehouse gốc vẫn dùng các tài nguyên upstream.

| Tên Gazebo duy nhất | Model | World pose (x,y,z,yaw) |
| --- | --- | --- |
| assignment5_chair | Open Robotics Chair v3 | `(2.5, 0.8, 0.0, 2.8)` |
| assignment5_person | MaleVisitorOnPhone v2 | `(-2.0, 2.5, 0.0, -0.7)` |

Dock gốc ở khoảng `(0.157, 0, 0, π)` trong world. Hai vật thể ở bên tuyến phía bắc, giữ vùng undock và bố cục kệ. Chúng là obstacles mới để costmap nhận bằng LiDAR; không chèn vào map đã lưu.

Camera OAK-D gắn trên robot, không dùng ảnh/video thay thế: RGB 320×240, khoảng 30 FPS, horizontal FOV 1.25 rad, optical frame `oakd_rgb_camera_optical_frame`.

- Ảnh vào: `/oakd/rgb/preview/image_raw` (`sensor_msgs/msg/Image`, `rgb8`).
- Ảnh YOLO: `/assignment5/yolo/image_annotated` (`sensor_msgs/msg/Image`, `bgr8`).
- JSON detections: `/assignment5/yolo/detections` (`std_msgs/msg/String`).
- Trạng thái goal: `/assignment5/mission/status` (`std_msgs/msg/String`).

RViz có sẵn panel **YOLO Camera**. Khi Terminal 4 chạy, ảnh có bounding boxes, class và confidence xuất hiện. Nếu thêm thủ công: Add → Image, Topic `/assignment5/yolo/image_annotated`, Reliability **Best Effort**, Durability **Volatile**. Khi detector dừng, panel có thể trở lại No Image. Log thật đã có `chair` khoảng 0.91 và `person` khoảng 0.93 trong lúc di chuyển; các nhãn nhầm như `bench`/`kite` cũng có thể xuất hiện, không được coi là vật thể mới đã xác nhận.

Cấu hình model, confidence, image size, device, số luồng và max FPS nằm trong `config/yolo.yaml`. Một worker giữ model duy nhất, chỉ xử lý frame mới nhất, bỏ ảnh quá cũ và giữ nguyên timestamp/frame ID. Detector không xuất `/cmd_vel`. Có thể đổi input bằng `assignment5.launch.py image_topic:=/topic_da_kiem_tra`.

## Kiểm tra và kết quả mô phỏng

```bash
colcon build --symlink-install --packages-select turtlebot4
source install/setup.bash
colcon test --packages-select turtlebot4
colcon test-result --verbose
python3 -m pytest -q src/turtlebot4/tests
```

Đã chạy thực tế: scene có đúng một robot, một dock, một ghế mới và một người mới; undock thành công; RGB và annotated images có timestamp hợp lệ; AMCL và Nav2 READY; ba action SUCCEEDED; zero recoveries, zero bumper contacts, không ghi nhận hazard va chạm, vận tốc cuối về 0. Có detection người/ghế trong lúc robot chuyển động, không chỉ khi đứng yên. Kiểm tra localization sau nhiệm vụ PASS. Độ chính xác vật lý tới đích chưa được chứng nhận bằng reference độc lập trong nhiệm vụ này.

## Dừng, chạy lại và xử lý lỗi

Dừng theo thứ tự **Terminal 4 → Terminal 3 → Terminal 2 → Terminal 1** bằng Ctrl+C. Nếu ngắt giữa goal, chờ navigator xác nhận hủy goal và vận tốc dừng trước khi tắt Nav2. Không chạy thêm teleop cùng nhiệm vụ. Muốn kiểm tra restart: dừng sạch cả bốn, khởi động lại cùng world/map, undock, đặt lại initial pose hiện tại, chờ READY rồi chạy lại Terminal 4; không dùng trạng thái AMCL cũ để suy ra restart đã đạt.

- **AMCL chưa ACTIVE:** đợi lifecycle khởi động xong rồi chạy lại preparation.
- **LOCALIZATION FAIL:** sửa initial pose hiện tại, kiểm tra scan/map, dừng teleop rồi thử lại; không thay covariance để ép PASS.
- **No Image / YOLO FAIL:** kiểm tra camera topic, venv và weights; chạy lại script chuẩn bị YOLO nếu thiếu. Mất camera khiến launch dừng nhiệm vụ.
- **Sai TF / RViz treo:** kiểm tra thứ tự source TF2 0.25.24 trong cả bốn terminal.
- **Goal/costmap bị chặn:** xem vật cản, vị trí robot và costmap; không tắt collision checking hoặc tăng dung sai để hoàn thành.
- **Launch thất bại:** chờ các tiến trình do launch sở hữu dừng hẳn trước khi chạy lại; không mở Gazebo thứ hai.

## Quay video nộp bài

Chỉ ghi cửa sổ Gazebo/RViz và terminal nhiệm vụ, tránh desktop hay ứng dụng riêng tư. Bắt đầu ghi trước Terminal 4; cho thấy hai vật thể cạnh dock, map/AMCL/costmap, ba lần goal ACCEPTED/SUCCEEDED, robot thực sự di chuyển, ảnh YOLO có nhãn người/ghế khi đang chạy và robot dừng cuối nhiệm vụ. Kết thúc bằng kết quả action và báo cáo JSON. Có thể quay riêng bước restart/đặt initial pose. Không đưa weights, video, ảnh riêng tư hay log tạm vào Git. Tự tải video lên nơi nộp bài và thêm URL thật sau khi upload; repository không giả định một liên kết video đã tồn tại.
