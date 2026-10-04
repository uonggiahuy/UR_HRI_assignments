# Bài tập 03 — Điều khiển UR3 bằng LLM dựa trên Thị giác (Perception-Driven)

Package ROS 2 Humble dành cho ô làm việc (workcell) mô phỏng robot UR3. Mã nguồn riêng của bài tập nằm trong package này; các repository của Universal Robots là hạ tầng upstream và cần cài đặt riêng.

## Kiến trúc và an toàn

Yêu cầu bằng ngôn ngữ tự nhiên -> LLM nhận biết môi trường 9Router (scene-aware LLM) -> JSON chưa xác thực (untrusted JSON) -> TaskValidator, kiểm tra mục tiêu (goal check), kiểm tra thế giới lỗi thời (stale-world check) -> SkillExecutor -> RobotSkills -> MoveIt 2/tay gắp (gripper) -> Gazebo DetachableJoint -> Xác thực qua camera RGB.

Ô làm việc gồm năm khối hộp màu đỏ (red), vàng (yellow), xanh dương (blue), xanh lá (green), và tím (purple) cùng các Vùng (Zone) A, B, và C. OpenCV xử lý hình ảnh từ camera RGB đặt từ trên cao nhìn xuống (overhead camera). Một phép biến đổi đồng điều (calibrated homography) ánh xạ tâm đỉnh khối hộp trên ảnh sang toạ độ XY; các phát hiện mới hoàn chỉnh sẽ trở thành các ảnh chụp nhanh trạng thái thế giới (WorldState) mang tính biểu tượng bất biến (immutable symbolic snapshots) và đồng bộ hóa không gian va chạm WORLD có thẩm quyền của MoveIt.

Khớp DetachableJoint của Gazebo Fortress tạo ràng buộc vật lý giữa tay gắp và khối hộp. MoveIt xác thực chuyển đổi WORLD -> ATTACHED -> WORLD; Gazebo duy trì tính chất động học (dynamic) và khả năng va chạm của khối hộp, sau đó tách (detach) nó khi nhả ra. Quá trình vận chuyển đang hoạt động của Bài tập 03 không can thiệp ghi đè trực tiếp vị trí/tư thế (pose) của vật thể trong Gazebo.

Các kỹ năng LLM công khai (public) chỉ gồm: home(), pick(object), place(object, zone), và place_temp(object). LLM chỉ nhận trạng thái dạng biểu tượng: không nhận tọa độ, tư thế (poses), giá trị khớp (joint values), quỹ đạo (trajectories), bộ điều khiển (controllers), hay ID vị trí tạm thời. Mọi kế hoạch đề xuất đều phải vượt qua bước kiểm tra lược đồ chính xác (exact-schema validation), mô phỏng mục tiêu, và chốt kiểm tra thế giới lỗi thời qua RGB trước khi SkillExecutor có thể thực thi.

Đối với đích đến đã bị chiếm chỗ, Python sẽ tự động tính toán và chọn một vị trí tạm thời đã được kiểm tra va chạm từ bên trong:

    pick(blue_cube) -> place_temp(blue_cube) -> pick(red_cube)
    -> place(red_cube, zone_b) -> home()

## Biên dịch và khởi chạy

    cd /root/ros2_ws
    source /opt/ros/humble/setup.bash
    colcon build --symlink-install --packages-select
    ur3_perception_llm_control
    source install/setup.bash

Terminal 1 — khởi động workcell một lần:

    ros2 launch ur3_perception_llm_control workcell.launch.py


Terminal 2 — source cùng môi trường ROS và thiết lập các biến môi trường 9router (đã config sẵn trên máy):

    source /path/to/private/9router.env
    ros2 run ur3_perception_llm_control assignment3_runtime 

Môi trường runtime sẽ khởi tạo một lần duy nhất: ROS, thu thập dữ liệu RGB, MoveIt, tay gắp, cơ chế kẹp vật lý, bộ đồng bộ hóa planning-scene, các kỹ năng của robot, và bộ lập kế hoạch 9Router. Mỗi dấu nhắc Command sẽ tạo một ảnh chụp RGB mới, WorldState mới, kết quả xác thực (validator result), kiểm tra thế giới lỗi thời, và một giao dịch SkillExecutor; các kế hoạch LLM trước đó và các vị trí tạm thời đã giữ chỗ sẽ bị hủy sau mỗi lệnh.

    Assignment 03 ready.
    Command> Put the red cube in Zone B.
    TASK SUCCESS
    Command> Put the green cube in Zone B.
    TASK SUCCESS
    Command> Put the purple cube in Zone A.
    TASK SUCCESS
    Command> quit

Dùng exit, quit, hoặc Ctrl-D để tắt an toàn. Các yêu cầu cấp thấp không được hỗ trợ sẽ bị từ chối trước khi gửi truy vấn LLM hoặc chuyển động robot và quay trở lại `Command>`. Runtime chỉ quay lại `Command>` sau khi gặp lỗi nếu trạng thái của MoveIt và cơ cấu kẹp vật lý được xác minh an toàn; nếu không, hệ thống sẽ kết thúc theo cơ chế đóng an toàn khi lỗi (fail-closed).
