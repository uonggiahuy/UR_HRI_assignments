# Bài 8 — Điều khiển TurtleBot 4 bằng biểu cảm khuôn mặt

## 1. Giới thiệu

Webcam thật nhận dạng sáu biểu cảm bằng FER trên CPU, hiển thị khung mặt,
độ tin cậy, tốc độ xử lý, hành động và trạng thái an toàn. Đây là dự đoán
**biểu cảm nhìn thấy**, không phải kết luận về cảm xúc thật của con người.
Chỉ dùng mô phỏng TurtleBot 4 Standard trong **đúng world Bài 6**
`worlds/assignment6_open.sdf`, giữ nguyên tệp; không cần bản đồ hay điều hướng.

## 2. Cài đặt và build

Từ máy chủ, mở hai terminal vào container ROS hiện có:

```bash
docker exec -it ros_humble_env bash
```

Trong container, chạy một lần, khi webcam chưa bị chương trình khác sử dụng:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
bash src/turtlebot4/scripts/prepare_assignment8_runtime.sh
colcon build --symlink-install --packages-select turtlebot4
source install/setup.bash
```

Môi trường FER riêng ở `~/.cache/turtlebot4/emotion-venv`; không thay môi trường
Whisper, MediaPipe hay Python hệ thống. Webcam cần được cấp vào container ở
`/dev/video0`; cửa sổ cần kết nối màn hình X11 sẵn có.

## 3. Cách chạy

**Terminal 1 — giữ chạy Gazebo và một robot bằng quy trình mô phỏng Bài 6:**

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source src/turtlebot4/scripts/simulation_environment.bash
ros2 launch turtlebot4 turtlebot4.launch.py world:=assignment6_open model:=standard rviz:=false
```

**Terminal 2 — chờ robot xuất hiện, undock một lần rồi chạy Bài 8:**

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source src/turtlebot4/scripts/simulation_environment.bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}'
ros2 launch turtlebot4 assignment8.launch.py
```

Chờ undock trả về `SUCCEEDED`. Nếu robot đã undock thì bỏ qua lệnh đó.
Launch Bài 8 chỉ mở nhận dạng và bộ điều khiển, không mở thêm Gazebo/robot.
Không chạy bộ điều khiển Bài 6, Bài 5/7, Nav2 hoặc teleop cùng lúc.

Nhìn thẳng webcam với **Neutral** ổn định để chuẩn bị. Giữ biểu cảm tiếp theo
ít nhất bốn lần suy luận có độ tin cậy từ 0,65. Mỗi biểu cảm chỉ kích hoạt
một chuyển động; sau khi robot dừng, trở về Neutral rồi mới chọn biểu cảm khác.
Neutral trong lúc chạy hủy hành động và kiểm tra robot đã đứng yên.
Mất mặt quá 0,7 giây, nhiều mặt, mất webcam hoặc dữ liệu an toàn không hợp lệ
sẽ ngăn/dừng chuyển động. Nếu vừa dừng do lỗi, trở về Neutral để chuẩn bị lại.

Tham số tùy chọn: `camera_device:=/dev/video0`, `emotion_model:=fer`,
`confidence_threshold:=0.65`, `show_camera:=true`, `use_sim_time:=true`.
Giới hạn chuyển động và thời gian nằm trong `config/assignment8.yaml`.
Không hạ các điều kiện an toàn để ép robot chạy.

## 4. Các biểu cảm và hành động

| Biểu cảm | Hành động mặc định |
|---|---|
| Happy | Cung trái nhẹ, dài 0,25 m, bán kính 0,50 m |
| Sad | Cung phải nhẹ, dài 0,25 m, bán kính 0,50 m |
| Neutral | Dừng, giữ đứng yên |
| Angry | Lùi 0,10 m nếu đủ khoảng trống phía sau |
| Surprise | Quay trái tại chỗ khoảng 360°, đo bằng odometry |
| Fear | Tiến 0,10 m |

Vận tốc tiến/lùi tối đa 0,06 m/s, quay tại chỗ 0,25 rad/s. LiDAR phải đủ
360° và toàn bộ vùng quét phải có khoảng trống; dữ liệu không biết sẽ bị từ chối.
Robot vẫn chịu các bảo vệ va chạm và giới hạn lùi của Create 3.

## 5. Kiểm tra và dừng chương trình

Trong terminal phụ đã source cùng môi trường:

```bash
ros2 topic echo /detected_emotion
ros2 topic echo /assignment8/state
ros2 topic info /cmd_vel --verbose
```

Dừng các lệnh theo dõi bằng Ctrl+C. Kiểm tra `ready`, `readiness_reason` và
`reason` nếu robot không nhận lệnh. Không coi việc xuất hiện `/cmd_vel` là
bằng chứng robot đã thực hiện hành động; cần quan sát Gazebo và odometry.
Bộ nhận dạng có thể nhầm hoặc không nhận ra một số biểu cảm, tùy ánh sáng/góc mặt.

Nhấn **q/Esc** trong cửa sổ webcam hoặc **Ctrl+C ở Terminal 2** để đóng Bài 8
và gửi lệnh dừng; sau đó **Ctrl+C ở Terminal 1** để dừng mô phỏng.

Kiểm thử tự động dùng dữ liệu giả lập, tách biệt với kiểm chứng webcam thật:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source src/turtlebot4/scripts/simulation_environment.bash
source /root/.cache/turtlebot4/tf2-0.25.24/install/setup.bash
colcon test --packages-select turtlebot4 --event-handlers console_direct+
colcon test-result --test-result-base build/turtlebot4 --verbose
```

Dừng mô phỏng và các chương trình Bài 8 trước khi chạy toàn bộ bộ kiểm thử.
