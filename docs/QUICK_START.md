# Quick Start — OpenArm Skeleton v1.2 on Jazzy

## 1. Yêu cầu

- Ubuntu 24.04 (Noble).
- ROS 2 Jazzy tại `/opt/ros/jazzy`.
- `ros-jazzy-ros-gz`, `ros-jazzy-navigation2`,
  `ros-jazzy-nav2-bringup` và `ros-jazzy-slam-toolbox`.

Xem lệnh cấu hình ROS apt repository và cài package trong `README.md`.

## 2. Build sạch

### Demo một-chạm

Trên Ubuntu 24.04, chạy file duy nhất ở thư mục gốc để tự tải/cập nhật source,
cài dependency còn thiếu, build, source và mở toàn bộ stack:

```bash
./START_OPENARM.sh
```

Launcher không ghi đè working tree có thay đổi local và không dùng force-pull.
Khi được đặt ngoài repository trên máy mới, source mặc định được clone vào
`~/openarm_skeleton_v1.2_ws`.

### Build thủ công

Không dùng artifact Humble cũ.

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
./scripts/build_workspace.sh
source install/setup.bash
```

## 3. Smoke test Gazebo

```bash
./scripts/run_hotel_demo.sh auto_run:=false
```

Terminal thứ hai:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic hz /scan
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
ros2 topic echo /camera/camera_info --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_footprint
```

Kỳ vọng: `/scan`, ảnh RGB và depth image xuất liên tục; camera được cấu hình
target 10 Hz nhưng tần số thực tế phụ thuộc CPU/GPU. Camera info, `/odom` có
dữ liệu và TF tồn tại. Có thể xem hai luồng ảnh bằng
`rqt_image_view` tại `/camera/color/image_raw` và
`/camera/depth/image_raw`.

Cấu hình RViz của workspace có sẵn hai display `RGB Camera` và `Depth Camera`
với QoS `Best Effort`. Display depth dùng dải cố định `0–10 m` và tắt
`Normalize Range`; cách này tránh các pixel `Inf` ngoài tầm đo làm toàn ảnh
`32FC1` bị hiển thị đen. Nếu tự thêm Image display mới, phải đặt `Reliability
Policy = Best Effort`, `Normalize Range = false`, `Min Value = 0` và
`Max Value = 10`. Giá trị QoS mặc định `Reliable` không tương thích với
sensor-data publisher và sẽ làm RViz báo `No Image`.

Point cloud không được bridge sang ROS và không có display RViz nhằm giảm tải.
Ứng dụng perception nên lấy vùng vật thể trên depth image rồi kết hợp
`camera_info` để tính tọa độ 3D.

RViz mặc định dùng view controller `Orbit` để thấy map và robot trong không
gian 3D. Giữ chuột trái và kéo để xoay, con lăn để zoom, kéo chuột giữa để dịch
tâm nhìn. Có thể chọn `TopDownOrtho` trong panel `Views` khi chỉ cần quan sát
SLAM/costmap từ trên xuống.

## 4. Nav2 mapping

```bash
./scripts/run_nav2_sim.sh
```

Wrapper gọi launch tổng:

```bash
ros2 launch openarm_skeleton_v1_2_navigation all_in_one.launch.py
```

Launch này đưa hotel Gazebo, robot/lidar, watchdog, SLAM hoặc localization,
Nav2 và RViz vào cùng một entry point. Dùng `slam:=false
map:=/absolute/path/to/map.yaml` để chuyển sang localization bằng map đã lưu.

Wrapper này cũng làm sạch GTK/GIO environment do Snap VS Code truyền vào.
Không gọi trực tiếp `ros2 launch` từ Snap terminal nếu RViz báo lỗi thư viện
`/snap/core20/.../libpthread.so.0`.

TF bắt buộc:

```text
map -> odom -> base_footprint -> base_link -> base_scan
```

Trong RViz kiểm tra LaserScan và costmap, sau đó di chuyển robot để tạo map.
Có thể chạy acceptance goal có kiểm chứng odometry ở terminal thứ hai:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run openarm_skeleton_v1_2_navigation check_nav2_goal.py
```

Không full-reset world khi SLAM/Nav2 đang chạy: robot được spawn động sẽ bị
xóa và `/clock` nhảy về 0. GUI hotel đã ẩn nút này. Muốn reset sạch, nhấn
`Ctrl+C` rồi chạy lại `./scripts/run_nav2_sim.sh`.

Lưu map:

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f "$(pwd)/maps/hotel"
```

## 5. Nav2 localization

```bash
./scripts/run_nav2_sim.sh \
  slam:=false map:="$(pwd)/maps/hotel.yaml"
```

Trong RViz:

1. Chờ AMCL tự khởi tạo tại pose map `(0, 0, 0)` của hotel spawn cố định.
2. Chọn `Nav2 Goal` để gửi goal.
3. Quan sát global plan, local costmap và `/cmd_vel_safe`.

`2D Pose Estimate` vẫn có thể ghi đè pose tự động. Nếu dùng map hoặc vị trí
spawn khác hotel profile, phải đặt lại initial pose đúng thay vì giữ `(0,0,0)`.

## 6. Safety command path

```text
controller_server / behavior_server
  -> /cmd_vel_nav
  -> velocity_smoother
  -> /cmd_vel
  -> wall-time watchdog (0.5 s)
  -> /cmd_vel_safe
  -> Gazebo Harmonic DiffDrive
```

Không remap Nav2 trực tiếp tới `/cmd_vel_safe`.

## 7. Test

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Runtime Jazzy chỉ được coi là đạt sau khi `/scan`, TF, SLAM, costmap, goal Nav2
và watchdog đều được kiểm tra.

Lidar trong profile simulation có dead zone 0,35 m và được đặt ở giữa/phía
trên base để loại self-return từ mesh. Khi chuyển sang hardware phải thay bằng
transform đã đo và cấu hình filter của lidar thật.
