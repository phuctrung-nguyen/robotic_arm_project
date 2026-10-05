# Mô phỏng ROS 2 (Gazebo + RViz)

Workspace `rprr_ws` gồm 3 package:

```
rprr_ws/src/
├── robotic_arm_description/   # URDF/Xacro, mesh STL (xuất từ SolidWorks), cấu hình RViz
├── robotic_arm_bringup/       # Launch file, cấu hình Gazebo bridge
└── robotic_arm_control/       # trajectory_client.py: sinh quỹ đạo LSPB + động học ngược
```

## Yêu cầu

- Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic (gz sim 8) — đã chạy thử trên cấu hình này
- **Không hỗ trợ ROS 2 Humble**: Humble đi kèm Gazebo Fortress, không nạp được các plugin `gz-sim-*` dùng trong
  `robotic_arm_gazebo.xacro` nên robot không có `/joint_states` và không điều khiển được
- `ros_gz_sim`, `ros_gz_bridge`, `robot_state_publisher`, `xacro`, `joint_state_publisher_gui`, `rviz2`
- Python: `numpy`

Cài các gói còn thiếu bằng `rosdep install --from-paths src --ignore-src -y` (trong `rprr_ws`).

## Build

```bash
cp -r rprr_ws ~/rprr_ws
cd ~/rprr_ws
colcon build
source install/setup.bash
```

## Chạy

Xem mô hình trên RViz (kéo thanh trượt để chỉnh từng khớp):

```bash
ros2 launch robotic_arm_bringup display.launch.py
```

Mô phỏng trên Gazebo + RViz:

```bash
ros2 launch robotic_arm_bringup gazebo.launch.py
```

Ở terminal thứ hai, chạy quỹ đạo hàn mẫu (Home → B → dừng → A → dừng → về Home):

```bash
cd ~/rprr_ws
source install/setup.bash
ros2 run robotic_arm_control trajectory_client.py
```

Quan sát Gazebo hoặc RViz, robot sẽ di chuyển theo đúng quỹ đạo đã thiết kế.

## Cách điều khiển trong Gazebo

`trajectory_client.py` tính trước quỹ đạo (LSPB trong không gian Cartesian + động học ngược) rồi cứ 10 ms phát
một điểm đặt xuống các topic `/q1_cmd_pos` ... `/q4_cmd_pos`. `ros_gz_bridge` chuyển các topic này sang Gazebo,
nơi mỗi khớp có một plugin `JointPositionController` (PID) riêng, khai báo trong
`robotic_arm_description/urdf/robotic_arm_gazebo.xacro`:

| Khớp | P | I | D |
|---|---|---|---|
| q1 | 12 | 0.2 | 1.0 |
| q2 | 120 | 80 | 20 |
| q3 | 300 | 200 | 4.0 |
| q4 | 15 | 0.3 | 1.5 |

Trạng thái khớp `/joint_states` do plugin `JointStatePublisher` của Gazebo phát và được bridge sang ROS 2 cho
RViz.

Trình tự chạy của node:

1. Chờ bản tin `/joint_states` đầu tiên để biết tư thế hiện tại của robot (lúc spawn là q = 0).
2. Đưa robot về HOME bằng đa thức bậc 3 trong không gian khớp (vận tốc đầu/cuối bằng 0, vận tốc đỉnh không quá
   50% giới hạn URDF), từ q = 0 mất khoảng 8.4 s.
3. Giữ tại HOME 1 s cho các bộ PID ổn định.
4. Chạy chu trình hàn 29.60 s (2961 điểm, trùng với `Vitridat.mat` của Simulink), xong thì node tự thoát.

Kết quả đo trong Gazebo: sai số vị trí gốc hệ O4 so với quỹ đạo đặt dưới 2 mm suốt chu trình, trung bình 0.1 mm
trên đoạn hàn B→A.
