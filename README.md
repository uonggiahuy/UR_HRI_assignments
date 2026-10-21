# Assignment 6 — Điều khiển TurtleBot 4 bằng cử chỉ tay

ROS 2 Humble / Ubuntu 22.04, Gazebo Fortress, webcam laptop/USB thật và
MediaPipe Tasks Gesture Recognizer. Chọn hành vi trước, giơ ngón cái để xác nhận;
robot không di chuyển chỉ vì vừa nhận ra một cử chỉ chọn.

**Trạng thái kiểm chứng (21/10/2026):** webcam thật đã đọc ảnh 640×480,
hiển thị landmarks và nhận đủ sáu cử chỉ từ thao tác người dùng. Wave → Thumb Up
đã hoàn tất chào hỏi trong Gazebo (khoảng 6 giây, về hướng cũ, odometry dừng).
Open Palm đã dừng một vòng tròn đang chạy; mất dấu tay cũng dừng và không tự chạy lại.
Gazebo render bằng NVIDIA RTX 4060; đo đồng thời webcam khoảng 17–18 FPS,
LiDAR khoảng 62 Hz, real-time factor khoảng 0,997 trong world `assignment6_open`.
Chưa nghiệm thu đủ bốn chu kỳ và một vòng tròn khép kín bằng tay thật.
Không dùng camera mô phỏng, video ghi sẵn hoặc JSON giả làm bằng chứng.

## Kiến trúc

```text
Webcam V4L2 thật → gesture_recognition_node (MediaPipe + cửa sổ OpenCV)
                          ↓ /detected_gesture (String chứa JSON)
                  robot_behavior_node (FSM + watchdog)
                          ↓ /cmd_vel (Twist)
                  Create 3 motion_control → TurtleBot 4 / Gazebo
                          ↑ /odom /scan /clock /dock_status /hazard_detection
```

Launch riêng không khởi động SLAM, AMCL, Nav2 hoặc YOLO. Bộ nhận dạng dùng
môi trường Python riêng, độc lập với môi trường YOLO đang có.

## Cài đặt và build

Thực hiện **trong container ROS**, tại workspace hiện có:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
bash src/turtlebot4/scripts/prepare_gesture_runtime.sh
colcon build --symlink-install --packages-select turtlebot4
source install/setup.bash
# Các regression kế thừa dùng bản TF2 đã chuẩn bị của workspace (nếu có).
if [ -f "$HOME/.cache/turtlebot4/tf2-0.25.24/install/setup.bash" ]; then
  source "$HOME/.cache/turtlebot4/tf2-0.25.24/install/setup.bash"
fi
colcon test --packages-select turtlebot4
colcon test-result --test-result-base build/turtlebot4 --verbose
```

Script tạo `~/.cache/turtlebot4/gesture-venv` với Python 3.10,
MediaPipe **0.10.21**, NumPy **1.26.4**, OpenCV contrib **4.10.0.84**.
Model chính thức `gesture_recognizer/float16/1` được tải từ Google Storage,
kiểm SHA256, lưu tại `~/.cache/turtlebot4/models/gesture_recognizer.task`.
Không nâng cấp pip/packages của ROS toàn cục hoặc venv YOLO.
Cần có `python3-venv`, pip, kết nối mạng và quyền ghi thư mục cache.

Trước khi chạy, kiểm tra **trong container**:

```bash
ls -l /dev/video*
echo "$DISPLAY"
~/.cache/turtlebot4/gesture-venv/bin/python -c 'import cv2; c=cv2.VideoCapture(0, cv2.CAP_V4L2); ok,f=c.read(); print("Live frame:",ok, None if f is None else f.shape); c.release()'
```

Lệnh phải báo `Live frame: True`; đóng chương trình kiểm tra camera trước
khi mở demo. Container hiện đã truy cập được `/dev/video0`.

Kiểm tra render NVIDIA trong container (nếu đã có các tiện ích này):

```bash
nvidia-smi
glxinfo -B
```

`OpenGL renderer` phải hiện NVIDIA khi dùng GPU NVIDIA. Trong lần kiểm tra này,
log Ogre xác nhận `NVIDIA GeForce RTX 4060 Laptop GPU/PCIe/SSE2`.
MediaPipe dùng CPU/XNNPACK và OpenCV giới hạn hai luồng; đây là phần nhận dạng,
độc lập với GPU render Gazebo. Không cần đổi driver khi log render đã đúng.

## Chạy chương trình

Terminal 1, giữ chạy; đóng simulator cũ trước:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch turtlebot4 assignment6.launch.py
```

Mặc định dùng **`assignment6_open`**, một world riêng có sàn và tường va chạm,
một TurtleBot 4 Standard và cửa sổ **TurtleBot 4 — Hand Gesture Control**.
World này tránh tải texture lớn của depot/warehouse gây stall GPU trong lần thử.
Launch không mở RViz mặc định. Đợi robot xuất hiện và dữ liệu cảm biến ổn định.
Không chạy thêm simulator, teleop, navigator hoặc publisher vận tốc khác.
Launch kiểm tra webcam/model/runtime và từ chối khi phát hiện simulator đang chạy,
kể cả Gazebo server còn sót mà chưa có `/clock`.

Terminal 2, sau khi Gazebo đã sẵn sàng: quan sát vùng sau dock trống, gửi
lệnh undock chính thức, đợi kết quả `SUCCEEDED` và `/dock_status` báo false:

```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}' --feedback
ros2 topic echo /dock_status --once
```

Đây là thao tác một lần, không phải teleop. Không gửi hành vi trước khi undock
thành công. Controller không gửi Twist rỗng liên tục khi đang rảnh để tránh can thiệp
action undock; sau chuyển động do nó điều khiển vẫn gửi lặp lệnh dừng ngắn hạn.
Nếu vùng hiện tại thiếu khoảng trống, controller từ chối chạy;
không tắt bảo vệ, dịch chuyển robot tức thời hoặc giả lập tín hiệu cảm biến.
Chỉ bắt đầu chọn cử chỉ sau kết quả undock; kiểm tra dòng `Ready` trong cửa sổ.

Có thể đổi camera, model hoặc giới hạn riêng cho lần chạy:

```bash
ros2 launch turtlebot4 assignment6.launch.py webcam_device:=0 show_camera:=true \
  world:=assignment6_open speed:=0.06 distance:=0.12 radius:=0.35 angular_speed:=0.25
```

Các giá trị nhận dạng, làm mượt, cooldown, tái kích hoạt và watchdog nằm trong
`config/assignment6.yaml`. `signal_timeout` mặc định 0,7 giây.
Để debug với **một simulator đã chạy**, dùng `start_simulation:=false`.
Hai executable riêng:

```bash
ros2 run turtlebot4 gesture_recognition_node
ros2 run turtlebot4 robot_behavior_node
```

Không chạy hai executable này cùng lúc với các node tương ứng trong launch.

## Sáu cử chỉ và xác nhận

| Cử chỉ | Cách làm | Kết quả |
| --- | --- | --- |
| Wave | Xòe tay, vẫy ngang qua lại rõ ràng, ít nhất 3 lần đổi chiều | Chọn chào hỏi |
| Pointing_Up | Chỉ ngón trỏ lên | Chọn **4 chu kỳ** tiến–lùi, mỗi đoạn 0,12 m |
| Victory | Giơ hai ngón hình V | Chọn một vòng tròn bán kính 0,35 m |
| Thumb_Up | Giơ ngón cái sau khi đã chọn | Xác nhận, thực thi đúng một lần |
| Open_Palm | Xòe tay | Dừng và xóa lựa chọn ngay, không cần xác nhận |
| Closed_Fist | Nắm tay | Hủy, xóa lựa chọn, về IDLE |

Cửa sổ hiển thị ảnh thật, landmarks, nhãn, confidence, FPS, lựa chọn và trạng thái.
Sau khi chọn, chờ `PENDING_CONFIRMATION / WAITING FOR THUMBS UP`, rồi giơ ngón cái
trong 8 giây. Giữ nguyên cử chỉ không tạo thêm lần chạy. Hạ tay hoặc đưa về cử
chỉ trung tính để tái kích hoạt. **Giữ Thumb Up và toàn bộ bàn tay trong ảnh
suốt hành vi**, tránh sát mép ảnh, ngược sáng và chuyển động tay quá nhanh.
Mất dấu tay dù camera vẫn có ảnh sẽ dừng hành vi, không tự tiếp tục.
Bốn chu kỳ cần khoảng 16–20 giây, chào khoảng 6 giây, vòng tròn khoảng 37 giây;
thời gian thực tế phụ thuộc vật lý và giới hạn an toàn. Nếu bị dừng, chọn lại
Pointing Up/Victory/Wave rồi xác nhận lại; chỉ Thumb Up sẽ không chạy tiếp.

Wave được suy ra từ chuyển động tâm bàn tay/ngón giữa theo thời gian, có bù dịch
chuyển nền ảnh, không phải nhãn có sẵn trong model. Dòng `Wave: …/3 direction changes`
cho biết số lần đổi chiều; vẫy đủ ba lần trong 2,4 giây, biên độ tối thiểu khoảng
4,5% chiều ngang ảnh. Có thể vẫy bằng xoay cổ tay; giữ bàn tay trong ảnh.
Xòe tay đứng yên vẫn là STOP.
**Trong lúc robot chạy, xòe tay luôn ưu tiên STOP kể cả khi đang vẫy.**
Chào hỏi quay trái 0,35 rad, phải 0,70 rad rồi trái 0,35 rad về hướng ban đầu.

## Cơ chế an toàn

FSM: `IDLE → PENDING_CONFIRMATION → EXECUTING → SETTLING → IDLE`, hoặc `STOPPED` khi lỗi.
Lệnh xác nhận thiếu lựa chọn, lựa chọn hết hạn và xác nhận lặp bị từ chối.
Vận tốc mặc định 0,06 m/s, quay 0,25 rad/s. Hoàn tất dựa vào odometry;
vòng tròn cần đủ góc quay, quãng đường và trở lại gần điểm đầu, không chỉ hết giờ.
Sau đó phải quan sát odometry đứng yên trước khi báo hoàn tất.

Watchdog dùng đồng hồ monotonic, vẫn hoạt động khi Gazebo pause. Mất webcam,
mất tiến trình nhận dạng, mất tay, clock ngừng, odometry/scan/dock/hazard cũ,
bumper/hazard hoặc publisher vận tốc cạnh tranh đều ngăn/dừng chuyển động.
Kiểm tra toàn bộ LiDAR 360° trước và trong hành vi; vùng trống tối thiểu tính cả
robot và toàn bộ quỹ đạo: khoảng 0,52 m cho tiến–lùi và 1,10 m cho vòng tròn
mặc định. Dữ liệu scan không hợp lệ bị từ chối. Giới hạn lùi Create 3 được giữ.
Trong bản Humble này, `BACKUP_LIMIT` là cảnh báo khi buffer lùi còn ≤5 cm;
`motion_control` chặn lùi khi buffer hết. Controller giữ đường điều khiển qua
`motion_control`, không đổi `safety_override`; nếu odometry không tiến triển
trong 1 giây thì dừng với `No measured progress`. Không coi cảnh báo sớm là
va chạm để tránh dừng nhầm giữa một chu kỳ hợp lệ.
Các ngưỡng bảo thủ có thể từ chối vùng gần dock; đó là lý do phải kiểm chứng
vùng chạy thật trước buổi trình diễn.

Kiểm thử phần mềm mới nhất: **313 test PASS**, gồm regression kế thừa.
Đây là kiểm thử tự động, không thay thế nghiệm thu đủ hành vi bằng tay thật.

## Kiểm tra ROS

Terminal phụ, chỉ đọc:

```bash
ros2 topic echo /detected_gesture
ros2 topic echo /assignment6/state
ros2 topic info /cmd_vel --verbose
ros2 topic echo /odom --once
```

`/detected_gesture`: `std_msgs/msg/String`, JSON phiên bản 1 chứa `session`,
`sequence`, `stamp` (Unix time), `gesture`, `raw_gesture`, `confidence`, `event`,
`hand_present`, `camera_ok`, `open_hand`. Mỗi frame xử lý là heartbeat; không thấy tay phát
`gesture=None`, `hand_present=false`. `/assignment6/state` công bố lựa chọn,
FSM, lý do, readiness, `backup_warning` và `readiness_reason` cụ thể
(dock, mất tay, cảm biến cũ, thiếu khoảng trống…). `/cmd_vel` là
`geometry_msgs/msg/Twist`, robot nhận
best-effort/volatile; bridge `cmd_vel_bridge` của simulator được cho phép.
Không tự publish JSON giả để chứng minh webcam hoặc khởi động robot.

## Dừng và xử lý lỗi

Xòe tay để STOP; nắm tay để CANCEL. Nhấn `q`/đóng cửa sổ hoặc Ctrl+C ở Terminal 1
để dừng các tiến trình launch sở hữu. Đợi launch thoát hoàn toàn trước khi chạy lại.
Launch đánh dấu Gazebo server theo phiên và dọn cả server con bị tách khỏi launcher.
Đã kiểm tra tắt/chạy lại không còn server cũ. Một node upstream TurtleBot có thể
cần launch gửi SIGTERM sau 5 giây; không mở bản thứ hai trong lúc đang tắt.
Kiểm tra odometry trở về vận tốc 0 trước khi đóng nếu cần ghi bằng chứng.
Khi dùng simulator ngoài (`start_simulation:=false`), đóng nó ở terminal chủ.

- **Lag/Gazebo đứng:** dùng world mặc định `assignment6_open`, đóng các demo cũ.
  Xem `pgrep -af 'ign gazebo|gz sim'`, `nvidia-smi` và `glxinfo -B`.
  Đã gặp `Texture memory budget exceeded. Stalling GPU.` ở world lớn dù render
  đúng NVIDIA. Không khởi chạy thêm một Gazebo để chữa phiên đang treo.
  Dừng ở terminal sở hữu bằng Ctrl+C; nếu có server cũ ngoài phiên, xác định đúng
  PID/owner trước khi xử lý, tránh kill toàn bộ ROS hoặc ứng dụng NVIDIA.
- **Chọn được nhưng không chạy/hay dừng:** xem dòng `Ready` và `/assignment6/state`.
  `No live hand control signal` nghĩa là không thấy tay đủ tin cậy; giữ tay rõ,
  không hạ tay sau xác nhận. `Docked` cần undock; dữ liệu cũ/clock dừng cần kiểm tra
  simulator. Sau STOP phải chọn lại rồi xác nhận, không tự chạy tiếp.
- **Wave khó nhận:** xòe ít nhất ba ngón, giữ bàn tay rõ và vẫy ngang qua lại 3–4 lần
  trong khoảng 2,4 giây, biên độ rõ. Chờ thấy `Wave/PENDING_CONFIRMATION` rồi
  chuyển nhanh sang Thumb Up trước khi lòng bàn tay đứng yên thành STOP.
- **Không có `/dev/video*`:** trên **host**, kiểm tra
  `ls -l /dev/video*` và `docker inspect ros2_workspace`. Cần bổ sung đúng
  `--device=/dev/video0:/dev/video0` vào cấu hình chạy container hiện hữu;
  với Docker Compose thêm `devices: ["/dev/video0:/dev/video0"]`. Nếu chạy
  non-root, thêm numeric group của device vào `group_add`. Device `video1`
  thường là metadata, không mặc định dùng làm ảnh. Docker không thêm device
  vào container đang chạy bằng `docker update`: người quản trị cần áp dụng
  cấu hình khi tạo lại container theo cấu hình và volume đã lưu. Chưa thực
  hiện thay đổi này; không xóa container hoặc mất dữ liệu để thử camera.
- **Permission denied:** trên host xem group/quyền device, cấp group `video`
  cho tài khoản phù hợp; tránh `chmod 777` và tránh `--privileged`.
- **Camera bận:** đóng ứng dụng đang giữ webcam, chọn đúng `webcam_device`.
- **Lỗi cửa sổ:** kiểm tra DISPLAY, mount `/tmp/.X11-unix`, quyền X11 của
  container và thư viện xcb. `show_camera:=false` phục vụ debug, không chứng
  minh đã đạt yêu cầu cửa sổ demo. Lỗi HighGUI kết thúc node và dừng launch.
- **MediaPipe/model lỗi:** chạy lại script setup; model checksum phải đúng;
  không cài lẫn `opencv-python` và `opencv-contrib-python` trong venv gesture.
- **Readiness false:** kiểm tra dock, hazard, clock, scan và publisher `/cmd_vel`;
  chọn lại rồi xác nhận sau khi xử lý nguyên nhân. Không có tự động tiếp tục.

Hướng dẫn API: [MediaPipe Tasks Gesture Recognizer](https://ai.google.dev/edge/mediapipe/solutions/vision/gesture_recognizer/python).
Mã nguồn: [nhánh assignments_6](https://github.com/uonggiahuy/UR_HRI_assignments/tree/assignments_6).
