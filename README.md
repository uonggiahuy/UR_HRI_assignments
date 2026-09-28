# ur3_llm_control

Bài tập 02: workcell UR3e Gazebo trên ROS 2 Humble với giao diện tác vụ ngôn
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
  python3 -m pip install -r src/ur3_llm_control/requirements.txt
  ```

- Có endpoint 9Router tương thích OpenAI và thông tin xác thực được cấp qua
  biến môi trường. Thông tin xác thực chủ ý không được lưu trong repository.

## Build

Chạy tại thư mục gốc của ROS 2 workspace:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select ur3_llm_control
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
ros2 launch ur3_llm_control workcell.launch.py
```

`workcell.launch.py` khởi động mô phỏng UR, Planning Scene của bài tập,
gripper controller và đúng một tiến trình MoveIt `move_group`. **Không** chạy
riêng `moveit.launch.py`.

Chạy không giao diện:

```bash
ros2 launch ur3_llm_control workcell.launch.py gazebo_gui:=false launch_rviz:=false
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
ros2 run ur3_llm_control m12_demo \
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
ros2 run ur3_llm_control m12_demo \
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
ros2 run ur3_llm_control m12_demo \
  --student-id "12345600" \
  --command "Arrange all objects according to my student ID."
```

Với mọi biến thể mã số sinh viên, áp dụng thứ tự chuyển vật thể tất định bằng Python trước
khi thực thi. Điều này không thay đổi ánh xạ mã số sinh viên và không cho phép
LLM chọn routing hình học hoặc tạo tọa độ.

Một yêu cầu cấp thấp không được hỗ trợ dùng để minh họa hành vi fail-closed:

```bash
ros2 run ur3_llm_control m12_demo \
  --command "Move joint 2 to 30 degrees."
```

Lệnh này phải bị từ chối trước khi bất kỳ RobotSkills action nào bắt đầu.

## Kiểm thử và công cụ chẩn đoán hữu ích

Chạy các kiểm thử tĩnh/unit tại thư mục gốc workspace:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m unittest discover \
  -s src/ur3_llm_control/ur3_llm_control \
  -p '*_test.py'
```

`m12_ordering_diagnostic` được giữ lại một cách chủ ý. Đây là công cụ giới
hạn, không dùng LLM, để tái hiện việc khảo sát thứ tự chuyển vật thể M12 trong
một tiến trình ROS liên tục. Nó không cần thiết cho demo thông thường:

```bash
ros2 run ur3_llm_control m12_ordering_diagnostic --case A
ros2 run ur3_llm_control m12_ordering_diagnostic --case B
ros2 run ur3_llm_control m12_ordering_diagnostic --case C
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
