# ur3_perception_llm_control

Bài tập 03 kế thừa workcell UR3e Gazebo của Bài tập 02 trên ROS 2 Humble với giao diện tác vụ ngôn
ngữ tự nhiên hoạt động theo nguyên tắc fail-closed. Toàn bộ mã đặc thù của bài
tập nằm trong package này; các repository Universal Robots được dùng như hạ
tầng upstream và không bị chỉnh sửa.

## Kiến trúc và ranh giới an toàn

```text
Yêu cầu ngôn ngữ tự nhiên
  -> LLMPlanner / 9Router
  -> JSON ứng viên không đáng tin cậy
  -> TaskValidator
  -> SkillExecutor
  -> RobotSkills
  -> MoveIt 2 + gripper controller
  -> Gazebo UR3e workcell
```

LLM chỉ đóng vai trò lập kế hoạch. Nó có thể chọn các skill công khai cấp cao,
nhưng không bao giờ gửi giá trị khớp, quỹ đạo, tọa độ Cartesian, lệnh
controller, lệnh gripper hoặc primitive cấp thấp. Mọi JSON ứng viên đều được
kiểm tra trước khi `SkillExecutor` được phép gọi robot skill. Các kế hoạch
không hợp lệ, không được hỗ trợ, mơ hồ, sai định dạng hoặc cũ sẽ fail-closed và
không thực thi robot.

Các robot skill công khai duy nhất là:

- `pick(red_cube|yellow_cube|blue_cube)`
- `place(red_cube|yellow_cube|blue_cube, zone_a|zone_b|zone_c)`
- `home()`

MoveIt là nguồn thẩm quyền cho lập kế hoạch có kiểm tra va chạm và thực thi
quỹ đạo. `config/scene.yaml` là scene workcell chuẩn. Trong lúc gắp, vòng đời
của vật thể được xác minh là MoveIt WORLD -> ATTACHED -> WORLD; Gazebo chỉ cho
vật thể đi theo gripper trong trạng thái attached và khôi phục model động, có
va chạm khi thả.

## Gói cài đặt bắt buộc

- Môi trường Ubuntu/ROS 2 Humble đã có source workspace và các phụ thuộc ROS.
- Các package mô phỏng UR chính thức có trong workspace: `ur_description`,
  `ur_robot_driver` và `ur_simulation_gz`.
- Cài đặt các phụ thuộc Python trong môi trường ROS Humble:

  ```bash
  python3 -m pip install -r src/ur3_perception_llm_control/requirements.txt
  ```

- Có endpoint 9Router tương thích OpenAI và thông tin xác thực được cấp qua
  biến môi trường. Thông tin xác thực chủ ý không được lưu trong repository.

## Build

Chạy tại thư mục gốc của ROS 2 workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ur3_perception_llm_control
source install/setup.bash
```

## Chạy workcell

Thiết lập các biến môi trường runtime cục bộ bắt buộc trong mỗi terminal ROS:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
export IGN_IP=127.0.0.1
```

Khởi động toàn bộ workcell bằng lệnh launch duy nhất được hỗ trợ:

```bash
ros2 launch ur3_perception_llm_control workcell.launch.py
```

`workcell.launch.py` khởi động mô phỏng UR, Planning Scene của bài tập,
gripper controller và đúng một tiến trình MoveIt `move_group`. **Không** chạy
riêng `moveit.launch.py`.

Chạy không giao diện:

```bash
ros2 launch ur3_perception_llm_control workcell.launch.py gazebo_gui:=false launch_rviz:=false
```

## M2: Camera RGB và vùng đích trong RViz

`config/scene.yaml` định nghĩa camera cố định ở world pose `(0, 0.38, 1.25)` m,
RPY `(0, π/2, 0)`, 640×480, horizontal FOV 1.2 rad (vertical FOV khoảng
0.948 rad), 10 Hz. Gazebo xuất
`/camera/image_raw` và `/camera/camera_info`; frame trong hai header là
`overhead_rgb_camera/camera_link/rgb`. Camera chỉ có ảnh RGB, không có depth.

Sau khi chạy `workcell.launch.py`, kiểm tra một ảnh RGB thật qua `cv_bridge`
và OpenCV (không chạy robot):

```bash
ros2 run ur3_perception_llm_control camera_test
```

RViz tự hiển thị ba vùng đích bằng MarkerArray trên `/workcell/zone_markers`.
Pose, kích thước và màu được đọc từ `config/scene.yaml`; các marker không là
vật cản MoveIt.

## M3: Đồng nhất mặt bàn

`config/camera_calibration.yaml` chứa bốn cặp góc mặt bàn không thẳng hàng,
được dùng để fit homography OpenCV cho mặt phẳng `world z=0.300 m`. Gốc ảnh ở
góc trên trái; +u tương ứng world -y, +v tương ứng world -x. Bộ chuyển đổi
`planar_mapper.py` chỉ nhận điểm trong vùng mặt bàn đã hiệu chuẩn, trả về XY
và không suy Z từ ảnh RGB.

Sau khi chạy workcell, kiểm tra ảnh, CameraInfo, ba tâm zone cố định và sai
số khứ hồi mà không di chuyển robot:

```bash
ros2 run ur3_perception_llm_control homography_test
```

## Cấu hình 9Router

Trong terminal chạy lệnh ngôn ngữ tự nhiên, cung cấp đủ ba biến môi
trường. Placeholder API key dưới đây phải được thay thế trong shell hoặc file
môi trường riêng của người dùng; không commit key.

```bash
export NINEROUTER_BASE_URL='http://127.0.0.1:20128/v1'
export NINEROUTER_API_KEY='<private-key>'
export NINEROUTER_MODEL='<configured-model>'
```

Dùng file môi trường phát triển riêng, chỉ source file đó trong chính shell
trước khi chạy lệnh:

```bash
source /path/to/private/9router.env
```

## Demo ngôn ngữ tự nhiên 

Chạy các lệnh sau trong terminal thứ hai sau khi workcell sẵn sàng và môi
trường ROS/9Router ở trên đã được cấu hình.

Yêu cầu pick/place cơ bản:

```bash
ros2 run ur3_perception_llm_control m12_demo \
  --command "Đặt khối đỏ vào vùng B rồi về home."
```

Kế hoạch skill hợp lệ dự kiến:

```text
pick(red_cube)
place(red_cube, zone_b)
home()
```

Sắp xếp theo mã số sinh viên một cách tất định, dùng cấu hình nộp bài:

```bash
ros2 run ur3_perception_llm_control m12_demo \
  --command "Arrange all objects according to my student ID."
```

Mã số sinh viên (23020746) trong cấu hình nộp bài cho biến thể P4. Ánh xạ bắt buộc là:

```text
zone_a <- blue_cube
zone_b <- red_cube
zone_c <- yellow_cube
```

Ngoài mã số sinh viên được cấu hình, chương trình đọc mã số dưới dạng chuỗi, dùng hai chữ số cuối, tính `P = int(XX) % 6` và xác định ánh xạ này. Có thể truyền mã kiểm thử tạm thời ở
runtime mà không sửa cấu hình:

```bash
ros2 run ur3_perception_llm_control m12_demo \
  --student-id "12345600" \
  --command "Arrange all objects according to my student ID."
```

Với mọi biến thể mã số sinh viên, áp dụng thứ tự chuyển vật thể tất định bằng Python trước
khi thực thi. Điều này không thay đổi ánh xạ mã số sinh viên và không cho phép
LLM chọn routing hình học hoặc tạo tọa độ.

Một yêu cầu cấp thấp không được hỗ trợ dùng để minh họa hành vi fail-closed:

```bash
ros2 run ur3_perception_llm_control m12_demo \
  --command "Move joint 2 to 30 degrees."
```

Lệnh này phải bị từ chối trước khi bất kỳ RobotSkills action nào bắt đầu.

## Kiểm thử và công cụ chẩn đoán hữu ích

Chạy các kiểm thử tĩnh/unit tại thư mục gốc workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m unittest discover \
  -s src/ur3_perception_llm_control/ur3_perception_llm_control \
  -p '*_test.py'
```

`m12_ordering_diagnostic` được giữ lại một cách chủ ý. Đây là công cụ giới
hạn, không dùng LLM, để tái hiện việc khảo sát thứ tự chuyển vật thể M12 trong
một tiến trình ROS liên tục. Nó không cần thiết cho demo thông thường:

```bash
ros2 run ur3_perception_llm_control m12_ordering_diagnostic --case A
ros2 run ur3_perception_llm_control m12_ordering_diagnostic --case B
ros2 run ur3_perception_llm_control m12_ordering_diagnostic --case C
```

## Khắc phục sự cố

Nếu các planning service bị trùng hoặc không thể thực thi trajectory, hãy dừng
các tiến trình launch trùng lặp và chỉ khởi động `workcell.launch.py`. Xác nhận
có đúng một server cho mỗi action và mọi controller đang active:

```bash
ros2 action list | grep -E '^/(move_action|execute_trajectory)$'
ros2 control list_controllers
```

Các controller bắt buộc ở trạng thái active là `joint_state_broadcaster`,
`joint_trajectory_controller` và `gripper_controller`. Không khởi động một
`moveit.launch.py` riêng để khôi phục hệ thống; hãy dùng workcell launch của
bài tập và chẩn đoán việc khởi động controller trước.

## M4: RGB five-cube localization

`config/perception.yaml` records HSV ranges measured from the Gazebo RGB frame,
blob shape/area limits, and an eight-frame 5 mm stability limit. The detector
uses the centroid of each segmented square **top face**. A pixel first maps to
the calibrated tabletop by the M3 homography; similar triangles along the ray
from the fixed camera center then intersect the known cube-top plane at
`z = 0.345 m`. The tabletop homography is unchanged. The RGB result contains
cube identity, pixel center, world XY, area, quality, and timestamp; it does
not estimate Z or infer zone occupancy. Missing, duplicate, or invalid cube
blobs cause an explicit failure.

Run after `workcell.launch.py` is ready:

```bash
ros2 run ur3_perception_llm_control cube_detection_test \
  --output /tmp/ur3_m4_annotated.png
```

For a **test-only** Gazebo pose error check, save one settled pose sample and
pass it to the diagnostic. These poses are never passed to the detector:

```bash
ign topic -t /world/empty/dynamic_pose/info -e -n 1 --json-output > /tmp/m4_gazebo_poses.json
ros2 run ur3_perception_llm_control cube_detection_test \
  --oracle-poses-json /tmp/m4_gazebo_poses.json \
  --output /tmp/ur3_m4_annotated.png
```

`config/scene_m4_layout_b.yaml` is a test-only alternate launch scene. Launch
with `scene_config:=/root/ros2_ws/src/ur3_perception_llm_control/config/scene_m4_layout_b.yaml`
to reproduce the green-on-zone-C and purple-on-zone-B overlap check. No
perception configuration or code change is needed between layouts. Layout A
error mean/max was 2.048/2.561 mm; layout B was 1.447/1.804 mm against
settled Gazebo XY. Both used eight complete frames at 10 Hz with zero measured
XY spread. The existing zone colors remain unchanged.

## M5: Camera-derived symbolic state

`perception_state.py` converts one complete five-cube M4 detection result into
an immutable `PerceptionSnapshot`. Fixed table/zone rectangles and cube X/Y
sizes come from the scene YAML. A cube is in a zone only when its full
footprint lies inside that zone with 3 mm clearance; partial overlap is an
explicit `PERCEPTION_AMBIGUOUS` error. Two blocks in one zone, missing blocks,
non-finite positions, and detections from mixed image timestamps also fail.
The snapshot records camera-derived XY, symbolic locations, zone occupancy,
and its ROS image timestamp. `require_fresh` rejects data older than 1.0 s by
default with `PERCEPTION_STALE`. It represents a static scene and sets
`held_object=None`; the legacy task `WorldState` remains separate.

With the workcell running, inspect live state using:

```bash
ros2 run ur3_perception_llm_control perception_state_test
```

For the accepted alternate scene, launch `workcell.launch.py` with
`scene_config:=/root/ros2_ws/src/ur3_perception_llm_control/config/scene_m4_layout_b.yaml`
and run:

```bash
ros2 run ur3_perception_llm_control perception_state_test \
  --scene /root/ros2_ws/src/ur3_perception_llm_control/config/scene_m4_layout_b.yaml
```

The `--scene` input supplies fixed geometry and cube dimensions. Movable cube
spawn poses do not populate the snapshot. No depth data or Gazebo pose topic is
read by the M5 runtime diagnostic.

## M6: Camera-derived MoveIt collision scene

`perception_scene.py` accepts a fresh, complete M5 `PerceptionSnapshot`,
checks its symbolic geometry, reads authoritative MoveIt attachments, then
applies one Planning Scene diff for non-attached cubes. Cube XY is copied from
the camera snapshot. Center Z comes from fixed table or zone top plus half the
configured cube height. Equal-edge collision boxes use identity orientation;
RGB does not provide cube orientation or Z. The synchronizer queries MoveIt
after applying and verifies each WORLD ID, dimensions, pose within 1 mm,
attached exclusions, unchanged table/pedestal, and absence of zone collision
objects. Service success alone is insufficient.

Run the no-motion live diagnostic after the workcell starts:

```bash
ros2 run ur3_perception_llm_control perception_scene_test
```

For Layout B, launch `workcell.launch.py` with
`scene_config:=/root/ros2_ws/src/ur3_perception_llm_control/config/scene_m4_layout_b.yaml`
and pass that same path as `--scene` to `perception_scene_test`. Both layouts
returned 0.000 mm requested-to-authoritative MoveIt pose error for all five
cubes. The legacy Assignment 02 scene initialization remains available to its
existing execution path.

## M7: Physical Gazebo grasp

The Assignment 03 grasp uses Gazebo Fortress `DetachableJoint` constraints
between `wrist_3_link` and each cube's physical `link`. `PhysicalGraspManager`
first detaches the plugin's initially attached joints, then verifies measured
closed fingers, gripper-to-cube proximity, and each attach/release state event.
`RobotSkills` coordinates that physical state with MoveIt's WORLD/ATTACHED
collision state. The physical path never calls the legacy Assignment 02
`GazeboAttachmentSynchronizer` or writes cube poses for transport.

In a fresh workcell, check the camera-derived scene and run the isolated
red-cube physical test:

```bash
ros2 run ur3_perception_llm_control perception_scene_test
ros2 run ur3_perception_llm_control m7_physical_grasp_test
ros2 run ur3_perception_llm_control perception_scene_test
```

The M7 test performs HOME, pick, a 3.2-second hold, collision-checked lateral
motion, a second hold, placement in zone A, physical release, retreat, and
HOME. Gazebo poses in this diagnostic are read-only physical-state oracles;
they never supply camera perception or motion targets. Run it once per fresh
workcell because `wait_until_ready()` establishes the initial detached state
from plugin transition events.
