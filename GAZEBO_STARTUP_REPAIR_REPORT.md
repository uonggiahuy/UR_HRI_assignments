# Sửa lỗi Gazebo Fortress chờ vô hạn khi khởi động

Ngày kiểm tra: 10/11/2026 (Asia/Bangkok). Nhánh: `assignments_7`.
Baseline Git: `05590c2`. **Kết quả sửa lỗi khởi động/GUI: PASS**.

## Hiện tượng và nguyên nhân

Lệnh thông thường mở cửa sổ nhưng không tải world; GUI báo không tìm thấy
subscriber `/gazebo/starting_world`, liên tục chờ danh sách world; hai tiến trình
spawn cũng chờ world. Không có ROS `/clock`, `/scan`, `/odom`. Server mồ côi PID
2849 có PPID 1, không công bố world/service; đã dừng đúng PID sau khi xác nhận
trạng thái. Một lần chạy lại độc lập, sau khi dọn các phiên thử nghiệm, vẫn lỗi:
server cũ không phải nguyên nhân duy nhất.

Hai lớp discovery bị ảnh hưởng trong môi trường nhiều giao diện mạng:

1. Ignition GUI/server không hoàn tất handshake khi tự chọn mạng. Chỉ đặt
   `IGN_IP=127.0.0.1` đã tải warehouse và cho phép thao tác camera, nhưng chưa spawn
   robot: spawn tiếp tục chờ `robot_description` của ROS.
2. Cyclone DDS tự chọn `wlp18s0` (192.168.0.109), trong khi còn `virbr0`,
   `tailscale0`, `CloudflareWARP`, `docker0`. Trace xác nhận lựa chọn Wi-Fi;
   các node ROS không khám phá được nhau. Chọn loopback và bật multicast rõ ràng
   trong XML khôi phục discovery, spawn và bridges. Thử `ROS_LOCALHOST_ONLY=1`
   đơn thuần thất bại với `Failed to find a free participant index for domain 0`:
   Humble tắt multicast trên `lo`, không đủ participant cho toàn bộ stack.

Kết luận: lỗi discovery của Ignition Transport và ROS/DDS trong cấu hình mạng
hiện tại. Chưa xác định riêng VPN hay routing/firewall nào gây mất multicast;
không quy kết thay đổi âm thanh là nguyên nhân. Cơ chế chọn giao diện `IGN_IP`
được mô tả tại https://gazebosim.org/api/transport/11/envvars.html.

## Bản sửa nhỏ, giới hạn trong terminal mô phỏng

- `scripts/simulation_environment.bash`: source sau setup ROS/workspace trong
  mỗi terminal; mặc định `IGN_IP=127.0.0.1`, `CYCLONEDDS_URI` trỏ đến XML cạnh
  script; giữ hai biến này nếu người dùng đã cấu hình riêng. Đặt
  `ROS_LOCALHOST_ONLY=0` để XML quản lý multicast trên loopback.
- `config/cyclonedds_simulation.xml`: chỉ chọn `lo`, bật multicast rõ ràng.
- `README.md`: thêm một dòng source và giải thích phạm vi cùng máy.
- Báo cáo này.

Không sửa launch/thuật toán SLAM, AMCL, Nav2, world, goals, vùng ngữ nghĩa,
upstream hoặc driver. Không thay `.bashrc`, Docker, NVIDIA, Python/Whisper,
`LD_LIBRARY_PATH`, socket X11/PulseAudio hay bind mounts. Không dùng render phần
mềm. Script phục vụ mô phỏng cùng máy, không dùng cho robot thật/ROS phân tán.
Hoàn tác môi trường bằng shell mới không source script.

## Kiểm tra thực tế

| Hạng mục | Kết quả và bằng chứng |
| --- | --- |
| Headless riêng | PASS: Fortress 6.18.0 tải đúng world warehouse upstream; 3 mẫu `/world/warehouse/clock` tăng 4.158 → 4.161 → 4.164 s |
| GUI riêng | PASS: GUI nối server headless, hiển thị kho; không cần đổi renderer |
| Lệnh đầy đủ sau sửa | PASS: chạy lại bằng đúng lệnh README, một world, một robot, một dock |
| GUI tương tác | PASS: gửi wheel events qua X11 trong viewport; ảnh trước/sau xác nhận zoom camera thay đổi cảnh, cửa sổ 1200×1000; kiểm tra lại ở phiên cuối |
| ROS | PASS: `/clock`, `/scan`, `/odom`, `/tf`, `/dock_status` có dữ liệu; dock=true; `check_simulation` xác nhận dữ liệu hợp lệ, sim time, TF và không trùng node |
| Ổn định | PASS: phiên đầy đủ 180 s và lần khởi động lại 90 s có giới hạn; clock/sensor tiến đều, không treo/crash tức thì; không phát lệnh di chuyển robot |
| GPU | PASS: `glxinfo -B` báo direct rendering NVIDIA RTX 4060, OpenGL 4.6, driver 580.178.04; trong mô phỏng VRAM khoảng 1.36 GiB/8 GiB, GPU 38–42%, 44°C |
| CPU/RAM | Server khoảng 1.1–1.25 lõi, GUI 1.4–1.5 lõi trên 20 CPU; RSS khoảng 0.58–0.61 GB/0.89–0.90 GB. RAM khả dụng khoảng 6.7–7.1 GiB, swap sử dụng 0; không thấy tăng mất kiểm soát trong khoảng đo |
| Map Server/AMCL | PASS startup: lifecycle ACTIVE, map đã lưu; khởi tạo pose từ vị trí spawn mặc định và đăng ký world/map đã kiểm chứng trong M4.3, nhận pose/TF mới từ AMCL |
| Nav2 | PASS startup/readiness: `start_navigation --timeout 8` báo READY; controller và BT navigator ACTIVE, một server `/navigate_to_pose`, costmaps được wrapper xác nhận. Không chạy autonomous goal hay khẳng định độ chính xác điều hướng mới |
| Assignment 7 | PASS interface: `--show-args`; launch offline, speech=false chạy semantic/voice nodes và state IDLE. Không kiểm tra lại nhận dạng Whisper hoặc nhiệm vụ giọng nói end-to-end |
| Webcam | PASS: `/dev/video0` mở bằng V4L2, đọc được frame 640×480 |
| Micro/PipeWire | PASS kết nối/thu: `pactl info` nhận PulseAudio on PipeWire 1.0.5; capture mono 16 kHz có 64,000 mẫu, RMS khác 0. Không đánh giá chất lượng nhận dạng lời nói |
| Bản đồ/cấu hình goals | Giữ nguyên: Git không có thay đổi map/regions/goals; kiểm tra SHA-256 map.yaml/map.pgm/regions.yaml trước và sau phần xác minh cuối khớp |

Lượt quan sát ROS đầu 12 s nhận 3977 clock (~332 Hz), 746 scan (~62 Hz),
739 odom (~62 Hz), không stamp regression; scan có 640 tia và dữ liệu hợp lệ.
Phiên khởi động cuối chạy lại `check_simulation --timeout 15`: toàn bộ bảy nhóm
kiểm tra PASS. Ảnh GUI, JSON và log thô giữ ngoài Git tại
`/tmp/gazebo-startup-audit/`.

Một lần thử ban đầu có hai launch chồng thời gian; kết quả đó không dùng để
kết luận. Đã dừng cả hai và kiểm chứng lại bằng các phiên độc lập.

## Build và tests đã thực thi

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select turtlebot4
source install/setup.bash
source src/turtlebot4/scripts/simulation_environment.bash
source /root/.cache/turtlebot4/tf2-0.25.24/install/setup.bash
colcon test --packages-select turtlebot4 --event-handlers console_direct+
colcon test-result --test-result-base build/turtlebot4 --verbose
```

Build PASS (1 package); kết quả cuối **438 tests, 0 errors, 0 failures, 0 skipped**.
Overlay TF2 đã tồn tại trước phiên sửa, không build/cài lại. Lượt đầu chưa nạp
môi trường mạng và TF2: 435 pass/3 fail (hai timeout discovery của map server,
một TF2 callback reentry). Lượt kế tiếp đã nạp mạng: 436 pass/2 fail (TF2 và
offline launch bị timeout khi phiên Assignment 7 thử nghiệm còn chạy).
Sau khi dừng stack thử nghiệm trùng và nạp overlay có sẵn: 438/438 pass.
`bash -n`, kiểm tra giữ nguyên biến môi trường đã đặt và `git diff --check` PASS.

## Lệnh khởi động đã kiểm chứng

Trong container hiện có, chạy trong mỗi terminal liên quan:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source src/turtlebot4/scripts/simulation_environment.bash
ros2 launch turtlebot4 turtlebot4.launch.py world:=warehouse model:=standard rviz:=false
```

Các lệnh kiểm tra giới hạn thời gian đã dùng:

```bash
IGN_IP=127.0.0.1 IGN_PARTITION=gazebo_startup_audit \
IGN_GAZEBO_RESOURCE_PATH=/opt/ros/humble/share/turtlebot4_ignition_bringup/worlds:/opt/ros/humble/share \
timeout --signal=INT --kill-after=5s 25s ign gazebo -s -r -v 4 \
  /opt/ros/humble/share/turtlebot4_ignition_bringup/worlds/warehouse.sdf
timeout --signal=INT --kill-after=12s 90s ros2 launch turtlebot4 turtlebot4.launch.py \
  world:=warehouse model:=standard rviz:=false
ros2 run turtlebot4 check_simulation --timeout 15
```

## Giới hạn và bàn giao

Không có Docker CLI/socket trong container nên không xác minh được tên container,
image `audio_ready`, host `ros.sh`, cấu hình runtime/mount bằng `docker inspect`,
hoặc hành vi sau restart container. Không thay đổi các mục này. Không cần cài
lại Gazebo/ROS hay driver. Các cảnh báo QML `control is not defined`, controller
period 1 ms so với physics 3 ms, và service `oakd/stop_camera`/`stop_motor` không
có vẫn xuất hiện; chúng không chặn startup/clock/GUI trong thử nghiệm.

Đã dừng các stack thử nghiệm bằng signal đến đúng launch/runner sở hữu và timeout;
không dùng killall/pkill. Chỉ commit bốn tệp nêu trên, không commit log, ảnh, bản
thu âm hay workspace build/install/log. SHA commit và xác minh push được ghi
trong phần bàn giao cuối; có thể tra bằng
`git log -1 --format=%H -- GAZEBO_STARTUP_REPAIR_REPORT.md`.
