# Bài tập 7 — Phân vùng ngữ nghĩa kho và điều hướng bằng giọng nói tiếng Việt

TurtleBot 4 Standard, ROS 2 Humble / Ubuntu 22.04, Ignition Fortress,
AMCL và Nav2. Bài này mở rộng package `turtlebot4` hiện có trên nhánh
`assignments_7`, kế thừa `assignments_6` tại `e71a1e1`.
Không thay thế simulator, bộ điều khiển hay bản đồ đã đo.

## Kiến trúc và phạm vi kiểm chứng

```text
Micro thật → sounddevice / faster-whisper (CPU INT8, tiếng Việt)
                    ↓ /recognized_speech (std_msgs/String)
          Bộ phân tích tất định → nhiệm vụ tuần tự
                    ↓ kiểm tra map, AMCL, TF, clock, dock, costmap
          ComputePathToPose → NavigateToPose → Nav2 → robot
                    ↓ /assignment7/state (String JSON)
regions.yaml → semantic_map_node → /assignment7/regions (MarkerArray)
```

Tái sử dụng `simulation_launch`, `turtlebot4.launch.py`, `start_localization`,
`set_initial_pose`, `prepare_navigation`, `start_navigation`, `NavigationProbe`,
kiểm tra covariance/thời gian AMCL, kiểm tra đường đi và costmap của bài trước.
Launch bài 7 chỉ mở ba thành phần mới; không mở Gazebo, AMCL hoặc Nav2 lần hai.
Mọi executable/launch của bài 4–6 được giữ nguyên. Không chạy bộ waypoint,
teleop hoặc điều khiển cử chỉ đồng thời với nhiệm vụ bài 7.

**Kết quả phải phân biệt:** kiểm thử tự động, đường Nav2, robot đến đích và
nhận dạng micro thật. Trong môi trường thực hiện, không có `/dev/snd`, socket
PulseAudio/PipeWire hoặc thiết bị đầu vào PortAudio. Nghiệm thu micro thật còn
**BỊ CHẶN**; văn bản bơm vào topic chỉ là kiểm thử tích hợp ngoại tuyến.
Không coi đường lập kế hoạch thành công là robot đã đến nơi.

## Chín vùng ngữ nghĩa

![Bản đồ phân vùng](maps/segmented_map.png)

| Vùng | Tên chính | Tên gọi khác |
| --- | --- | --- |
| A | Khu vực cửa ra vào | khu A, khu vực A, cửa ra vào, khu cửa ra vào, lối vào kho |
| B | Kệ linh kiện cảm biến | khu B, khu cảm biến, kệ cảm biến, khu linh kiện cảm biến |
| C | Kệ linh kiện truyền động | khu C, khu truyền động, kệ truyền động, khu linh kiện truyền động |
| D | Kệ linh kiện điều khiển | khu D, khu điều khiển, kệ điều khiển, khu linh kiện điều khiển |
| E | Khu vực đóng gói và xuất kho | khu E, khu đóng gói, khu xuất kho, khu đóng gói và xuất kho |
| F | Khu vực trung tâm | khu F, khu trung tâm, khu vực trung tâm, khu trung chuyển, khu giữa |
| G | Kệ linh kiện nguồn và pin | khu G, khu nguồn, khu pin, khu linh kiện nguồn, khu pin và nguồn |
| H | Sảnh vào phòng họp | khu H, sảnh phòng họp, sảnh vào phòng họp, hành lang phòng họp |
| I | Phòng họp kỹ thuật | khu I, phòng họp, phòng họp kỹ thuật |

## Phân vùng và đích tiếp cận

Đọc `maps/map.yaml` và `map.pgm` gốc, phân loại ô trống/vật cản/chưa biết theo
`negate`, `free_thresh`, `occupied_thresh`. Đa giác thủ công tất định bám theo
ảnh A–I: tường ngoài, ba dãy kệ và phòng họp là các mốc căn chỉnh.
Ảnh tham chiếu quay 90° theo chiều kim đồng hồ so với PGM; tọa độ chuẩn hóa
tham chiếu `(u,v)` chuyển thành `(v,1-u)` trên PGM. PNG xuất giữ hướng PGM gốc.
Không dùng pixel ảnh chụp làm mét ROS. Chuyển đổi dùng tâm pixel, đảo trục Y,
resolution, tịnh tiến origin và cả yaw origin.

Đa giác chứa cả kệ; đích chọn trong lối đi trống gần tâm vùng, với clearance
ít nhất **0,55 m** tới cả vật cản và ô chưa biết, thuộc thành phần liên thông
lớn nhất sau kiểm tra clearance. Tâm ngữ nghĩa và dấu cộng đỏ là hai loại điểm
riêng. Đích B/C/D/G nằm trong lối đi của vùng tương ứng, không đặt trên kệ.
A/H/I được kiểm tra bằng cùng tiêu chí; không sửa map để mở cửa giả.

`regions.yaml` ghi SHA256 của cả YAML và PGM, metadata, đa giác pixel/map,
tâm vùng, đích, khoảng trống và trạng thái kiểm tra hình học.
`GEOMETRIC_ONLY` chưa chứng minh Nav2 đi được; node yêu cầu đường Nav2 thật
trước từng chặng. File map thay đổi sẽ bị từ chối. Sinh lại chỉ khi đã kiểm tra
các mốc căn chỉnh của bản đồ mới; không tự coi căn chỉnh cũ là còn đúng.

Trong container, lệnh một lần để tái tạo hai tệp đầu ra:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run turtlebot4 segment_warehouse_map \
  --map /root/ros2_ws/src/turtlebot4/maps/map.yaml \
  --output-dir /root/ros2_ws/src/turtlebot4/maps
```

Đích được kiểm tra lại với toàn bộ footprint tròn bảo thủ 0,22 m trong global
costmap hiện tại; Nav2 vẫn kiểm tra footprint, vật cản và điều khiển vận tốc.

## Chuẩn bị nhận dạng tiếng nói

Cài runtime riêng, không nâng cấp Python hệ thống hoặc môi trường YOLO/MediaPipe:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
bash src/turtlebot4/scripts/prepare_assignment7_runtime.sh
```

Script dùng Python 3.10, venv `~/.cache/turtlebot4/speech-venv` với
`--system-site-packages` để dùng được ABI `rclpy` Humble.
Các phiên bản được ghim trong `config/assignment7-requirements.txt`;
faster-whisper 1.1.1, CTranslate2 4.5.0, NumPy 1.24.4, sounddevice 0.5.1.
Nếu thiếu PortAudio, script tải và giải nén thư viện Ubuntu vào venv, không cài
đè thư viện hệ thống. Cần `python3-venv`, pip, `apt-get`, `dpkg-deb`, trình liên
kết hệ thống và kết nối mạng. Tải model đa ngôn ngữ `base` lần đầu cần mạng và
cache ghi được; có thể dùng `small` với cùng giao diện. CPU INT8, hai luồng,
không chiếm VRAM của Gazebo. Tham khảo [faster-whisper chính thức](https://github.com/SYSTRAN/faster-whisper)
và [sounddevice chính thức](https://github.com/spatialaudio/python-sounddevice).

Trong container, kiểm tra đầu vào thật:

```bash
ls -l /dev/snd
cat /proc/asound/cards
ls -l /run/user/1000/pulse/native
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
ros2 run turtlebot4 speech_recognition_node
```

Node liệt kê micro thực, ghi âm mono 16 kHz theo khung 20 ms, chờ 0,7 giây
im lặng để kết thúc câu, giới hạn câu 8 giây và hàng đợi 2 giây.
Câu quá dài bị loại thay vì tách thành nhiều nhiệm vụ; nhận dạng rỗng không phát
lệnh. Mất micro, tràn hàng đợi hoặc lỗi recognizer làm tiến trình báo lỗi và
launch hủy nhiệm vụ. Không có thiết bị đầu vào thì dừng với lý do cụ thể.
Ngưỡng âm lượng trong `config/assignment7.yaml` cần chỉnh theo micro thực;
không khẳng định đã kiểm chứng ba câu nói khi chưa có người nói vào micro.

### Kết nối âm thanh Docker khi còn thiếu

Thực hiện các lệnh sau **trên host**, không phải trong container:

```bash
arecord -l
pactl info
pactl list short sources
docker inspect ros_humble_env --format '{{json .HostConfig.Devices}}'
docker inspect ros_humble_env --format '{{json .Mounts}}'
docker inspect ros_humble_env --format '{{.HostConfig.NetworkMode}}'
```

Tên container do đề bài cung cấp là `ros_humble_env`; kiểm tra `docker ps`
nếu host dùng tên khác. Phiên làm việc này không có Docker CLI/socket nên
không xác nhận được cấu hình host. Cả hai đường âm thanh đều chưa được nối
vào môi trường đang chạy; webcam hoạt động không chứng minh micro hoạt động.

Nếu micro ALSA trực tiếp còn trống, cấu hình khởi động container phải có
`--device=/dev/snd:/dev/snd --group-add "$(getent group audio | cut -d: -f3)"`.
Nếu host đang quản lý micro bằng PulseAudio hoặc PipeWire-Pulse, ưu tiên
chia sẻ server đang dùng: các tùy chọn cho **lần khởi động do chủ máy quản lý**:

```bash
--mount type=bind,src="$XDG_RUNTIME_DIR/pulse",dst=/run/host-pulse \
--mount type=bind,src="$HOME/.config/pulse/cookie",dst=/run/host-pulse-cookie,readonly \
--env PULSE_SERVER=unix:/run/host-pulse/native \
--env PULSE_COOKIE=/run/host-pulse-cookie
```

Chỉ dùng đường cookie khi host thực sự có file này; quyền UID/GID và plugin
ALSA Pulse (`libasound2-plugins` trong image) phải khớp. Đây là tùy chọn cấu hình,
không phải lệnh tự động tạo lại container. Không chạy cả hai phương án một cách
mù quáng, không `chmod 777`, không xóa/thay container hoặc đổi 9Router.
Sau khi chủ máy nối âm thanh, chạy lại script và node để xác nhận danh sách
micro, rồi nói ít nhất ba câu thật; hiện chưa nghiệm thu phương án host nào.

## Build và kiểm thử

Trong container ROS:

```bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select turtlebot4
source install/setup.bash
# Dùng overlay TF2 đã có nếu môi trường bài trước yêu cầu.
if [ -f "$HOME/.cache/turtlebot4/tf2-0.25.24/install/setup.bash" ]; then
  source "$HOME/.cache/turtlebot4/tf2-0.25.24/install/setup.bash"
fi
colcon test --packages-select turtlebot4 --event-handlers console_direct+
colcon test-result --test-result-base build/turtlebot4 --verbose
```

Không sửa thư viện ROS upstream để làm test vượt qua. Unit test dùng âm thanh
và ActionClient giả được đánh dấu rõ, không phải bằng chứng nhận dạng thật.

## Chạy mô phỏng, định vị và Nav2

Mỗi terminal ROS mới cần source `/opt/ros/humble/setup.bash` rồi
`/root/ros2_ws/install/setup.bash`. Dùng cùng map đã lưu và world `warehouse`.
Các terminal giữ chạy ở foreground; Ctrl+C chỉ dừng phần terminal đó sở hữu.

**Terminal 1 — giữ một Gazebo và một robot:**

```bash
ros2 launch turtlebot4 turtlebot4.launch.py world:=warehouse model:=standard rviz:=false
```

**Terminal 2 — giữ Map Server và AMCL:**

```bash
ros2 run turtlebot4 start_localization
```

**Terminal 3 — giữ RViz ngữ nghĩa:**

```bash
rviz2 -d "$(ros2 pkg prefix turtlebot4)/share/turtlebot4/rviz/assignment7.rviz" \
  --ros-args -r __node:=assignment7_rviz -p use_sim_time:=true
```

Không mở thêm RViz Nav2 trùng lặp: plugin điều hướng tạo node phụ có cùng tên
và kiểm tra graph sẽ từ chối. Tách RViz khỏi base để đóng cửa sổ không kéo theo
simulator. Trước AMCL được khởi tạo, cảnh báo thiếu `map`/TF có thể xuất hiện.

**Terminal 4 — thao tác một lần, sau khi cảm biến đã ổn định:** quan sát khu vực
sau dock trống trước khi yêu cầu undock; đợi kết quả `SUCCEEDED`, trạng thái false.

```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock '{}' --feedback
ros2 topic echo /dock_status --once
```

Trong RViz chọn **2D Pose Estimate**, đặt vị trí/hướng **hiện tại** của robot
sau undock, đối chiếu LiDAR với map. Không mặc định tọa độ spawn là tọa độ hiện tại.
Có thể dùng `set_initial_pose --x ... --y ... --yaw-deg ...` chỉ với tọa độ đã đo
độc lập, không sao chép một pose ví dụ. Kiểm tra một lần:

```bash
ros2 run turtlebot4 prepare_navigation --timeout 45
ros2 run turtlebot4 check_localization --timeout 20 --json
```

Nếu covariance chưa hội tụ, kiểm tra map/scan và khởi tạo lại đúng pose;
chỉ dùng teleop giám sát khi cần rồi dừng hoàn toàn trước Nav2. Không tắt bảo vệ.

**Terminal 4 — sau readiness, giữ Nav2:**

```bash
ros2 run turtlebot4 start_navigation --timeout 45
```

Không giả định lệnh này khởi động AMCL; nó tái sử dụng localization đã chạy.

**Terminal 5 — giữ ba node bài 7, mặc định dùng micro thật:**

```bash
ros2 launch turtlebot4 assignment7.launch.py
```

Có thể chọn micro theo index/tên do PortAudio liệt kê và model đa ngôn ngữ:

```bash
ros2 launch turtlebot4 assignment7.launch.py whisper_model:=base \
  microphone_device:=0 navigation_timeout:=180.0 show_markers:=true
```

`regions_file` và `map_file` có thể trỏ đến cặp cấu hình đã kiểm tra khác;
node từ chối hash không khớp. Node không tự undock hoặc tự đoán initial pose.

## Lệnh nói và nhiệm vụ nhiều vùng

Ví dụ: “Đi đến khu vực A/B/C/F” (nói một chữ đích mỗi câu), “Đi đến khu cảm biến”,
“Đi đến khu truyền động”, “Đi đến khu điều khiển”, “Đi đến khu trung tâm”,
“Đi đến khu nguồn”, “Đi đến phòng họp”, “Đi đến sảnh phòng họp”.

- “Đi từ khu vực A đến khu vực B” tạo tuyến `[A, B]`.
- “Di chuyển từ khu cảm biến đến khu nguồn” tạo `[B, G]`.
- “Đi từ khu trung tâm đến phòng họp” tạo `[F, I]`.
- “Đi từ khu A qua khu F rồi đến khu G” tạo `[A, F, G]`.
- “Dừng lại” hoặc “Hủy nhiệm vụ” yêu cầu hủy action Nav2.

Robot luôn đến A trước B trong tuyến `[A,B]`; không đoán rằng nó đang ở A.
Chỉ gửi đích tiếp theo sau `SUCCEEDED`. Bị từ chối, thất bại, timeout hoặc mất
readiness sẽ dừng tuyến. Lệnh mới trong lúc bận bị từ chối; câu lặp trong bốn
giây không tạo nhiệm vụ mới. Câu hủy được ưu tiên, không bị chặn bởi cooldown.
Hủy chưa được xác nhận sẽ khóa executor và yêu cầu khởi động lại.

Bộ phân tích bỏ dấu tiếng Việt, chuẩn hóa hoa/thường/khoảng trắng/dấu câu và
khớp toàn bộ tên đích; tên “sảnh phòng họp” không bị nhầm thành “phòng họp”.
Chỉ hỗ trợ một số biến thể chữ đọc rõ; chữ mơ hồ như “dê” bị từ chối.
“Đi đến khu vực Z”, “Đi đến phòng không tồn tại” hoặc “Đi đến khu A và khu B”
không tạo goal. Hãy nói lại tên vùng rõ ràng khi bị từ chối.

## Quan sát và kiểm thử ngoại tuyến

Terminal phụ, chỉ đọc:

```bash
ros2 topic echo /recognized_speech
ros2 topic echo /assignment7/state
ros2 topic echo /assignment7/regions_status --qos-durability transient_local
ros2 topic info /cmd_vel --verbose
ros2 run turtlebot4 check_navigation --timeout 20 --json
```

Trạng thái JSON phân biệt `IDLE`, `RECOGNIZED`, `VALIDATING`, `NAVIGATING`,
`SUCCEEDED`, `FAILED`, `CANCELED`; có transcript, tuyến, đích hiện tại,
distance_remaining, kết quả hoặc lý do lỗi. `SUCCEEDED` từng chặng khác với
`Mission completed` của cả tuyến. Cần kiểm tra robot đã chuyển động và dừng,
không chỉ xem topic hoặc một path.

RViz hiển thị đường viền, chín tên vùng và mũi tên đích; vùng đang chọn được tô
nổi. Topic dùng reliable/transient-local, chỉ cập nhật khi lựa chọn thay đổi.
Nếu dùng RViz riêng, thêm `MarkerArray` từ `/assignment7/regions`, chọn
Durability **Transient Local**, Fixed Frame `map`. RViz gốc không vẽ đủ glyph tiếng Việt. Vì vậy package cung cấp thêm
`semantic_legend.png` và mặt phẳng `semantic_legend.dae` sinh từ cùng chín định
nghĩa; legend có đủ dấu, còn text marker vẫn giữ nguyên UTF-8 cho các node đọc.

Khi **chưa có micro**, chỉ kiểm thử tích hợp bằng văn bản có nhãn ngoại tuyến:

```bash
# Terminal 5, thay cho launch dùng micro; vẫn có thể gây robot mô phỏng di chuyển.
ros2 launch turtlebot4 assignment7.launch.py test_mode:=offline speech_enabled:=false
# Terminal phụ; dữ liệu kiểm thử, KHÔNG phải bằng chứng nói thật.
ros2 topic pub --once /recognized_speech std_msgs/msg/String "{data: 'Đi đến khu F'}"
```

Chỉ chạy khi vùng mô phỏng an toàn và Nav2 đã sẵn sàng. Không trình bày log này
như nhận dạng âm thanh thật. Để kiểm tra từ chối mà không điều hướng, dùng câu
“Đi đến khu vực Z”. Dùng các đích khác nhau đã kiểm tra đường đi để nghiệm thu
ít nhất ba lần đến đích và một tuyến tuần tự; ghi riêng kết quả từng vùng.

## Lỗi thường gặp, dừng và ghi hình

- Không thấy micro: kiểm tra kết nối host/container như trên; không thay bằng
  bàn phím để tuyên bố micro hoạt động.
- Clock/scan/odom/TF cũ: kiểm tra simulator còn sống; không gửi thêm goal.
- AMCL chưa khởi tạo: đặt đúng pose hiện tại và kiểm tra covariance, scan/map.
- Vùng bị chặn: kiểm tra costmap, không sửa PGM hoặc bỏ collision checking.
- Có teleop/gesture/waypoint: dừng tiến trình cạnh tranh trước nhiệm vụ bài 7.
- Hash không khớp: xác nhận đúng map; chỉ sinh lại sau kiểm tra căn chỉnh.
- Hủy/timeout: đợi action kết thúc; nếu báo không xác nhận được cancellation,
  kiểm tra robot đã dừng trước khi khởi động lại node.

Dừng: nói “Dừng lại”, đợi `CANCELED` và robot đứng yên; Ctrl+C Terminal 5,
Terminal 4 (Nav2), Terminal 2 (AMCL), Terminal 3 (RViz), cuối cùng Terminal 1.
Không để tiến trình tách nền. Sau một lần **restart simulator**, chạy lại
localization, undock, đặt pose hiện tại và readiness; không dùng state AMCL cũ.

Checklist video tiếng Việt: quay map gốc và PNG A–I; RViz có đủ marker; micro
thật với ba câu và transcript; câu Z bị từ chối; Gazebo di chuyển tới ba vùng;
một tuyến nhiều vùng chỉ chuyển sau thành công; kết quả action và robot dừng;
cuối cùng shutdown sạch. Chỉ dùng ảnh/video thật. Hiện không có video hoặc
ảnh nhận dạng micro thật để công bố; không đặt liên kết giả.

## Kết quả kiểm chứng ngày 31/10/2026

Build thực tế và **410 test PASS** (313 regression kế thừa, 97 test bài 7). Map gốc giữ nguyên SHA256. Các giao diện bài 4–6 được kiểm tra regression; không tuyên bố đã chạy lại toàn bộ demo webcam/YOLO.

| Vùng | Đã phân vùng | Đích x, y, yaw (m, m, rad) | Ô đích | Đường Nav2 | Robot đến đích |
| --- | --- | --- | --- | --- | --- |
| A | YES | 0.225, -23.225, -1.569 | VALID | VALID | NOT TESTED |
| B | YES | -10.475, -12.625, 0.693 | VALID | VALID | NOT TESTED |
| C | YES | -3.375, -12.625, 0.017 | VALID | VALID | NOT TESTED |
| D | YES | 4.625, -12.625, 0.022 | VALID | VALID | NOT TESTED |
| E | YES | 12.125, -12.625, 2.247 | VALID | VALID | NOT TESTED |
| F | YES | 2.925, 1.925, 0.205 | VALID | VALID | PASS |
| G | YES | 2.625, 13.075, -0.951 | VALID | VALID | PASS |
| H | YES | 0.075, 21.975, -1.571 | VALID | VALID | PASS |
| I | YES | -12.425, 11.475, -1.009 | VALID | VALID | NOT TESTED |

Chín đường Nav2 được tính từ pose kiểm tra ban đầu; không phải chứng chỉ đường luôn mở ở mọi thời điểm. F/G/H có action thành công và odometry đứng yên. Tuyến **G → H hoàn thành tuần tự**. Lượt F → G → H trước đó dừng an toàn ở H khi graph publisher thay đổi lúc khởi động lại RViz; không coi lượt đó là thành công toàn tuyến. Lệnh Z bị từ chối và lệnh “Dừng lại” hủy action, đã quan sát robot đứng yên. Toàn bộ lệnh trong nghiệm thu này đến từ **văn bản kiểm thử ngoại tuyến**, không phải micro.

Model đa ngôn ngữ `base` đã tải và nạp CPU INT8; runtime micro báo **không có thiết bị đầu vào**. Chưa có ba transcript tiếng Việt từ micro thật, chưa có demo giọng nói đầu-cuối. Vì vậy trạng thái tổng thể **PARTIAL**. `regions.yaml` cố ý lưu trạng thái hình học tái tạo được; kết quả action trực tiếp nằm ở topic trạng thái, không giả mạo chứng chỉ điều hướng trong YAML.
