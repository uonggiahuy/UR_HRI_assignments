# 1. Giới thiệu Assignment 5

TurtleBot 4 Standard dùng lại bản đồ warehouse của Assignment 4, định vị bằng AMCL, tự hành tuần tự **Goal 1 → Goal 2 → Goal 3** qua Nav2 và nhận diện vật thể bằng YOLO11n từ camera RGB thật trong mô phỏng Gazebo Fortress. Luồng tự động là chức năng chính; không cần bấm đích bằng chuột. RViz2 và cửa sổ camera YOLO riêng giúp quan sát đồng thời đường đi và kết quả nhận diện.

Mã nguồn ROS 2 Humble: [nhánh assignments_5](https://github.com/uonggiahuy/UR_HRI_assignments/tree/assignments_5). Không cần tạo package hay workspace mới.

## 2. Cài đặt và chuẩn bị môi trường lần đầu

Thực hiện **trong container ROS Ubuntu 22.04**, không cài ROS Humble lên host Ubuntu 24.04. Cần các gói TurtleBot 4 Simulator, Ignition Fortress, Nav2, AMCL và RViz2. Với tài khoản `root`, bỏ `sudo` trong hai lệnh cài đặt:

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

TF2 **0.25.24** được build ngoài repository để khắc phục lỗi callback của phiên bản cũ. YOLO dùng venv riêng có quyền đọc package ROS hệ thống, không nâng cấp NumPy/OpenCV toàn cục. Các phiên bản đã dùng: NumPy 1.26.4, OpenCV 4.10.0.84, PyTorch 2.5.1 CPU, torchvision 0.20.1, Ultralytics 8.3.40. Viewer dùng OpenCV hệ thống có HighGUI; chỉ detector chạy trong venv. Không cần kích hoạt venv cho cả terminal.

Lần đầu cần Internet để tải YOLO11n vào `~/.cache/turtlebot4/models/yolo11n.pt`. Không đưa weights vào Git. Detector dùng hai luồng CPU, giới hạn khoảng 5 ảnh/giây. Mã detector áp dụng AGPL-3.0-or-later (`licenses/AGPL-3.0.txt`); mã nền Apache-2.0. Nguồn và giấy phép CC BY 4.0 của người/ghế có trong `models/ATTRIBUTION.md`.

**Chạy đoạn sau trong mỗi terminal mới**, theo đúng thứ tự source:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
source "${XDG_CACHE_HOME:-$HOME/.cache}/turtlebot4/tf2-0.25.24/install/setup.bash"
```

## 3. Chạy tự động 3 Goal + YOLO — CHỨC NĂNG CHÍNH

### Terminal 1 — Gazebo + TurtleBot 4 + vật thể + RViz2

```bash
ros2 launch turtlebot4 turtlebot4.launch.py \
  rviz:=true rviz_fixed_frame:=map assignment5_objects:=true
```

Giữ terminal này chạy suốt buổi demo. Đợi warehouse, một robot, một người và một ghế xuất hiện; `/clock`, LiDAR và odometry phải hoạt động. Trước khi AMCL chạy, RViz có thể tạm báo chưa có frame `map`. Scene Assignment 5 dùng `warehouse`, model `standard`, namespace rỗng. Không khởi động thêm simulator. Lệnh `simulation.launch.py` của Assignment 4 vẫn được giữ.

### Terminal 2 — Map Server + AMCL

```bash
ros2 run turtlebot4 start_localization
```

Giữ terminal này chạy. Đợi Map Server và AMCL đạt trạng thái ACTIVE; không chạy SLAM đồng thời. Dùng đúng cặp `maps/map.yaml` và `maps/map.pgm` của warehouse Assignment 4, không tạo lại bản đồ.

### Terminal 3 — Chuẩn bị định vị, rồi khởi động Nav2

```bash
ros2 run turtlebot4 prepare_navigation
```

Đây là lệnh **chạy một lần**: undock nếu cần và kiểm tra định vị. Nếu chưa biết initial pose, sau khi undock kết thúc hãy dùng **2D Pose Estimate** trong RViz đặt vị trí và hướng **hiện tại** của robot, rồi chạy lại lệnh trên. Undock có thể đổi hướng robot; tọa độ world Gazebo không mặc nhiên bằng tọa độ map.

Nếu AMCL chưa hội tụ, kiểm tra LiDAR chồng đúng vật cản trên map. Chỉ khi Nav2 chưa chạy và vùng quanh robot trống, có thể dùng `ros2 run turtlebot4 teleop` để quay chậm/đi ngắn; nhấn `q` để dừng và thoát, rồi chạy lại preparation. Không bỏ qua kiểm tra covariance/freshness.

**Chỉ sau thông báo `LOCALIZATION READY`**, chạy:

```bash
ros2 run turtlebot4 start_navigation
```

Giữ terminal này chạy; đợi **READY** của Nav2, gồm lifecycle, action server và costmaps. Nếu định vị chưa sẵn sàng, navigator không gửi goal.

### Terminal 4 — Tự động ba đích và YOLO

Có thể kiểm tra sẵn sàng bằng lệnh một lần:

```bash
ros2 run turtlebot4 check_navigation --timeout 30
```

Sau khi kiểm tra đạt, chạy:

```bash
ros2 launch turtlebot4 assignment5.launch.py
```

Launch khởi động `waypoint_navigator`, `yolo_detector` và viewer tùy chọn. Python đọc tọa độ/yaw từ `config/goals.yaml`, gửi lần lượt các goal bằng action **Nav2 NavigateToPose**. YOLO chạy đồng thời trên ảnh RGB của robot; viewer chỉ đọc ảnh đã được detector vẽ bounding boxes.

| Goal | Tên | x (m) | y (m) | yaw (rad) |
| --- | --- | --- | --- | --- |
| 1 | north_of_dock | -0.525 | 1.225 | -0.14 |
| 2 | chair_view | 0.775 | 1.325 | -0.30 |
| 3 | person_view | -0.525 | 2.425 | 3.10 |

Có thể thêm nhãn ba đích trong RViz bằng `assignment5.launch.py show_goal_markers:=true`. Marker đọc cùng `config/goals.yaml`, chỉ hiển thị tâm đích và số thứ tự; không thay thế Global Path và không gửi goal.

Các pose thuộc frame `map`; tuyến được kiểm tra với hash bản đồ, vùng Free và costmap đang chạy. Không sửa tọa độ, dung sai hay cấu hình tránh vật cản để ép thành công. Navigator kiểm tra readiness, camera mới, robot undocked và không có teleop cạnh tranh trước khi chạy. Mỗi goal có timeout 120 giây; readiness có timeout 60 giây. Goal lỗi không được bỏ qua.

**Cách quan sát và xác nhận:**

- Trong Gazebo: robot thực sự di chuyển qua ba đích, người/ghế có mặt và không xảy ra va chạm.
- Trong RViz: Fixed Frame `map`, robot/scan/costmap hợp lý, **Global Path** hiện khi Nav2 lập đường.
- Trong cửa sổ **TurtleBot 4 — YOLO Object Detection**: ảnh camera sống có khung, tên lớp và confidence. Có thể xếp Gazebo, RViz và camera cạnh nhau để quay video.
- Trong Terminal 4: cả **Goal 1, Goal 2, Goal 3 phải trả về `SUCCEEDED`**. Đường vẽ trên RViz chưa chứng minh goal thành công. Robot phải dừng sau goal cuối; launch kết thúc detector và viewer, ba terminal đầu vẫn chạy.

Báo cáo JSON tại `~/.local/state/turtlebot4/assignment5/latest.json` chứa kết quả action, recoveries, vận tốc dừng và detection khớp timestamp odometry. PASS của nhiệm vụ còn yêu cầu detection lớp mục tiêu khi robot đang chuyển động. Action thành công không chứng nhận sai số vật lý tuyệt đối. Khi camera/TF/costmap mất hoặc có nguy cơ va chạm, nhiệm vụ dừng theo kiểm tra hiện có.

Để kiểm tra lại từ đầu: dừng sạch bốn terminal theo mục 6, mở lại cùng world/map, undock, đặt initial pose đúng vị trí hiện tại, chờ READY rồi chạy nhiệm vụ. Không dùng trạng thái AMCL của phiên trước làm bằng chứng restart thành công.

## 4. Điều hướng thủ công bằng Nav2 Goal trên RViz2 — CHỨC NĂNG PHỤ

Chế độ này dùng để gỡ lỗi và kiểm tra điều hướng bằng tay. **Dùng cùng Terminal 1, 2, 3 như mục 3**, gồm preparation và chờ Nav2 READY. **Không chạy `assignment5.launch.py`** khi thử thủ công; `waypoint_navigator` phải không hoạt động.

1. Mở cửa sổ RViz2 đã có từ Terminal 1, không mở thêm một RViz khác.
2. Đặt **Fixed Frame = map**.
3. Chọn **Navigation 2 Panel**; nếu chưa có, vào Panels → Add New Panel → Navigation 2.
4. Chọn công cụ **Nav2 Goal** trên thanh công cụ.
5. Nhấn vào một đích trong vùng **Free** trên map, tránh vật cản/vùng Unknown; kéo chuột để chọn hướng rồi thả.
6. Quan sát **Global Path**, costmaps và robot di chuyển trong Gazebo.
7. Xác nhận kết quả **NavigateToPose = SUCCEEDED** trong panel/log Nav2 và robot đã dừng. Có thể quan sát status action từ terminal trống:

```bash
ros2 topic echo /navigate_to_pose/_action/status
```

Mã status `4` là SUCCEEDED; phải đối chiếu UUID của goal vừa gửi, tránh nhầm với kết quả lưu của goal cũ. Kết thúc lệnh theo dõi bằng Ctrl+C.

**Không gửi goal Nav2 thủ công từ RViz trong khi nhiệm vụ tự động ba goal đang chạy.** Không dùng teleop trong lúc Nav2 đang thực hiện goal. Nếu cần hủy, dùng Cancel trong panel và chờ robot dừng.

Muốn quan sát YOLO khi thử thủ công, chỉ chạy detector/viewer riêng trong các terminal đã source môi trường; hai lệnh này giữ terminal chạy và không gửi goal:

```bash
ros2 run turtlebot4 yolo_detector --ros-args -p use_sim_time:=true
```

```bash
ros2 run turtlebot4 yolo_viewer
```

## 5. Camera và YOLO

Camera OAK-D của TurtleBot 4 phát ảnh RGB 320×240 từ mô phỏng, không dùng video thay thế:

| Nội dung | Topic | Kiểu |
| --- | --- | --- |
| Ảnh RGB đầu vào | `/oakd/rgb/preview/image_raw` | `sensor_msgs/msg/Image`, rgb8 |
| Ảnh có chú thích | `/assignment5/yolo/image_annotated` | `sensor_msgs/msg/Image`, bgr8 |
| Detections dạng JSON | `/assignment5/yolo/detections` | `std_msgs/msg/String` |
| Trạng thái nhiệm vụ | `/assignment5/mission/status` | `std_msgs/msg/String` |

YOLO11n dùng các lớp COCO; mục tiêu demo là **person** và **chair**, không chỉ giới hạn hai lớp này. Khung, tên lớp và confidence đều là kết quả suy luận thật; có thể có nhãn nhầm như bench/kite, cần đối chiếu cảnh thực tế. Cấu hình tại `config/yolo.yaml`: confidence 0.35, ảnh suy luận 640, CPU, tối đa 5 FPS. Giữ nguyên timestamp/frame ID của camera.

**Hai cách hiển thị cùng hoạt động:**

- **RViz2 — YOLO Camera**: display Image có sẵn. Nếu thêm lại, chọn topic `/assignment5/yolo/image_annotated`, Reliability **Best Effort**, Durability **Volatile**.
- **Cửa sổ OpenCV riêng**: tự mở mặc định cùng launch Assignment 5; ảnh 320×240 được phóng thành 640×480, giữ tỉ lệ. Viewer dùng sensor-data QoS, hàng đợi một ảnh, xử lý sự kiện GUI trên luồng chính. Không chạy YOLO lần hai và không xuất `/cmd_vel`. Nhấn **q**, **Esc** hoặc nút đóng cửa sổ chỉ tắt viewer, nhiệm vụ và detector tiếp tục.

Tắt riêng cửa sổ khi chạy không có màn hình:

```bash
ros2 launch turtlebot4 assignment5.launch.py \
  show_yolo_window:=false
```

Lệnh này vẫn chạy suy luận, xuất ảnh chú thích và điều hướng tự động. Nó không tắt Gazebo/RViz do Terminal 1 sở hữu. Muốn tắt RViz, dùng `rviz:=false` ở Terminal 1; tùy chọn viewer không thay đổi cấu hình GUI của Gazebo. Nếu GUI không truy cập được, viewer cảnh báo rồi thoát độc lập; việc viewer thoát không hủy nhiệm vụ.

### Docker/X11

Môi trường kiểm tra hiện tại có `DISPLAY=:1`, socket `/tmp/.X11-unix/X1`, `XAUTHORITY` không được đặt và OpenCV hệ thống dùng **GTK3**. Giữ nguyên cấu hình GUI đang dùng cho Gazebo/RViz; không cần cấp quyền host bổ sung khi các cửa sổ mở bình thường.

Nếu chuyển sang container khác, lấy **giá trị thật** của `DISPLAY` và `XAUTHORITY` từ terminal phiên đồ họa host. Container cần chuyển tiếp `DISPLAY`, mount `/tmp/.X11-unix:/tmp/.X11-unix:rw` và quyền X11 hợp lệ. Khi host dùng cookie, mount file `$XAUTHORITY` của host vào `/tmp/host.xauthority:ro` và đặt `XAUTHORITY=/tmp/host.xauthority` trong container. Không mặc định file cookie luôn tồn tại, không dùng `xhost +` để mở quyền toàn cục. Đây là cấu hình lúc tạo container; không tạo lại container đang làm việc chỉ để chạy viewer.

Kiểm tra trong container:

```bash
printenv DISPLAY XAUTHORITY
ls -l /tmp/.X11-unix
python3 -c "import cv2; print([s for s in cv2.getBuildInformation().splitlines() if 'GUI:' in s])"
```

Viewer tự thử mở/đóng HighGUI trong tiến trình con có timeout trước khi mở cửa sổ chính, tránh lỗi native của backend làm ảnh hưởng các node khác. Không đặt OpenCV headless làm thư viện của viewer nếu muốn có cửa sổ.

## 6. Dừng chương trình và xử lý lỗi

**Ctrl+C theo thứ tự Terminal 4 → Terminal 3 → Terminal 2 → Terminal 1.** Nếu đang chạy goal, đợi navigator xác nhận hủy goal và vận tốc dừng rồi mới tắt Nav2. Ở chế độ thủ công, hủy goal và chờ dừng trước khi tắt Terminal 3; dừng detector/viewer riêng nếu đã mở. Không để runner cũ còn sống rồi mở phiên trùng.

| Triệu chứng | Cách xử lý |
| --- | --- |
| AMCL chưa ACTIVE | Chờ lifecycle; kiểm tra map đúng world và không có SLAM chạy cùng. |
| LOCALIZATION FAIL | Đặt initial pose hiện tại bằng 2D Pose Estimate, kiểm tra scan/map, dừng teleop rồi chạy lại preparation. Không nới covariance để ép READY. |
| Không có ảnh YOLO | Kiểm tra camera đầu vào, venv/weights và log detector. Chạy lại script chuẩn bị YOLO nếu thiếu dependency. Mất camera có thể dừng nhiệm vụ. |
| Viewer báo không mở được GUI | Kiểm tra DISPLAY/socket/cookie X11 và HighGUI; dùng `show_yolo_window:=false` nếu không cần cửa sổ. RViz vẫn nhận cùng topic chú thích khi detector chạy. |
| Viewer đã đóng, muốn mở lại | Chạy `ros2 run turtlebot4 yolo_viewer` trong terminal mới khi detector còn hoạt động. |
| RViz không có map/TF hoặc treo | Kiểm tra thứ tự source ROS → workspace → TF2 0.25.24; chờ AMCL tạo frame map, không thêm TF giả. |
| Goal bị chặn hoặc thất bại | Kiểm tra pose, costmap và vật cản thực; không tắt tránh va chạm hoặc tăng dung sai để ép thành công. |

Kiểm tra phần mềm sau thay đổi, trong terminal đã source môi trường và khi không có nhiệm vụ điều hướng đang chạy (regression suite có tạo node teleop tạm để kiểm tra QoS):

```bash
cd /root/ros2_ws
python3 -m compileall -q src/turtlebot4/turtlebot4 src/turtlebot4/launch
colcon build --symlink-install --packages-select turtlebot4
source install/setup.bash
colcon test --packages-select turtlebot4 --event-handlers console_direct+
colcon test-result --test-result-base build/turtlebot4 --verbose
python3 -m pytest -q src/turtlebot4/tests
```

Khi quay video: bắt đầu trước Terminal 4; ghi Gazebo có người/ghế, RViz có map và Global Path, cửa sổ YOLO có nhãn/confidence, ba lần SUCCEEDED và robot dừng cuối nhiệm vụ. Chỉ ghi các cửa sổ liên quan, tránh ứng dụng riêng tư. Dùng kết quả/log thật; không đưa video, logs, weights hoặc tệp tạm vào Git.
