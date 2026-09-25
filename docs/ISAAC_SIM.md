# Isaac Sim 5.0 + ROS 2 Jazzy — OpenArm Skeleton v1.2

## Trạng thái tích hợp vào workspace chính — 2026-09-22

Package `openarm_skeleton_v1_2_isaac`, các script và tên file được giữ nguyên
từ workspace Isaac tham chiếu để thuận tiện so sánh và debug. Chúng được tạo
thành bản sao trong workspace OpenArm chính; workspace tham chiếu không bị di
chuyển hoặc sửa đổi.

Máy thực hiện migration này không cài và không chạy Isaac Sim. Đã xác nhận
build đủ bốn ROS package, chuẩn bị được URDF `38 links / 37 joints` hiện tại,
compile Python và toàn bộ contract test. Các kết quả runtime Isaac ở phần dưới
là bằng chứng lịch sử từ máy nguồn, chưa phải runtime acceptance sau migration.

Việc build workspace không tự tải hay cài Isaac Sim. Chỉ
`run_isaac_nav2.sh` mới yêu cầu một bộ Isaac Sim 5.0 đã tồn tại.

## Trạng thái máy nguồn đã kiểm tra ngày 2026-09-01

- Ubuntu 24.04.4 LTS và ROS 2 Jazzy: đúng nền tảng được NVIDIA hỗ trợ.
- GPU NVIDIA GeForce RTX 4060 Laptop, kernel driver 580.173.02 đã được nạp.
- Khoảng 16 GB RAM, 4 GB swap và hơn 230 GB dung lượng trống.
- Isaac Sim 5.0 standalone đã cài tại `~/isaacsim`.
- Factory headless smoke test, ROS 2 Jazzy bridge, SLAM/Nav2 lifecycle và goal
  0,60 m đã PASS trong cả scene hotel và restaurant trước migration.

NVIDIA ghi mức RAM tối thiểu của Isaac Sim 5.0 là 32 GB và GPU tối thiểu là
RTX 4080. Máy này thấp hơn mức chính thức dù trước đây đã từng chạy được 5.0.
Vì vậy integration này không texture/camera và chỉ dùng một RTX lidar. Scene
`hotel` hiện được dựng từ chính `hotel_lobby_demo.sdf` của Gazebo để saved map
và live scan dùng cùng geometry; scene `restaurant` vẫn là primitive nhẹ độc
lập. Không mở đồng thời Gazebo hoặc ứng dụng GPU nặng.

Nguồn chính thức:

- [Download Isaac Sim 5.0](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/download.html)
- [Cài workstation](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/install_workstation.html)
- [ROS 2 Jazzy trên Ubuntu 24.04](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/install_ros.html)
- [Yêu cầu phần cứng](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/requirements.html)

## 1. Cài Isaac Sim 5.0 standalone (chỉ khi cần runtime)

Tải file Linux `isaac-sim-standalone-5.0.0-linux-x86_64.zip` từ trang NVIDIA
ở trên. Sau khi file đã nằm trong `~/Downloads`, chạy:

```bash
mkdir -p "$HOME/isaacsim"
unzip "$HOME/Downloads/isaac-sim-standalone-5.0.0-linux-x86_64.zip" \
  -d "$HOME/isaacsim"
cd "$HOME/isaacsim"
./post_install.sh
./isaac-sim.selector.sh
```

Lần đầu shader cache có thể mất nhiều phút. Trong App Selector chọn
**Isaac Sim Full** để xác nhận GUI cơ bản mở được, rồi đóng nó trước khi chạy
project. Nếu file zip có tên khác, thay đúng tên file đã tải.

Không cần cài ROS riêng bên trong Isaac. Tiến trình simulator dùng thư viện
Jazzy/Python 3.11 đóng gói trong Isaac; các node workspace dùng Jazzy hệ thống
với Python 3.12 và trao đổi qua DDS. Launch của project tự tách hai môi trường
để tránh trộn binary Python.

## 2. Build và kiểm tra host

Từ workspace:

```bash
./scripts/build_workspace.sh
export ISAAC_SIM_PATH="$HOME/isaacsim"
./scripts/check_isaac_host.sh
```

`check_isaac_host.sh` phải tìm thấy `python.sh`. Chạy `nvidia-smi` trong
terminal desktop bình thường; nếu lệnh này lỗi ở đó thì sửa driver trước.
Isaac Sim 5.x trên máy này phải dùng driver nhánh 580. Driver 595.84 đã được
runtime-test và làm cả Isaac nguyên bản lẫn project segfault trong RTX renderer;
host checker sẽ trả lỗi rõ ràng thay vì tiếp tục mở Kit.

## 3. Một launch chạy toàn bộ

Lệnh khuyến nghị:

```bash
export ISAAC_SIM_PATH="$HOME/isaacsim"
./scripts/run_isaac_nav2.sh scene:=hotel
```

Scene nhà hàng:

```bash
./scripts/run_isaac_nav2.sh scene:=restaurant
```

`scene:=hotel` là mặc định. Hotel được đọc từ file SDF đang dùng bởi Gazebo,
bao gồm hành lang spawn, tường, phòng khách, bếp, phòng ngủ và các object hiện
tại. Loader giữ visual/collision riêng, hỗ trợ box, cylinder, sphere và plane,
đồng thời đổi toàn bộ pose từ Gazebo world sang frame khởi đầu SLAM tại robot.
Các plane nền trong SDF không được tạo trong scene hotel; một ground collider
vô hình của Isaac vẫn đỡ robot. Nhờ đó map đã quét từ Gazebo có thể dùng cho
Isaac nếu SDF không đổi.
Restaurant có khu bếp/quầy phục vụ, bục đón khách và bốn cụm bàn ghế, vẫn là
scene primitive nhẹ độc lập. Hai scene chưa phải asset photorealistic.

Wrapper trên gọi đúng một ROS launch:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch openarm_skeleton_v1_2_isaac isaac_nav2.launch.py \
  isaac_sim_path:="$HOME/isaacsim"
```

Launch thực hiện theo thứ tự:

1. chuẩn bị URDF Isaac từ base-demo đã kiểm chứng;
2. mở scene nhẹ, robot và RTX lidar trong Isaac Sim;
3. mở `robot_state_publisher`, watchdog, bộ bù ma sát bánh và bộ hiệu chỉnh
   odometry phẳng;
4. đợi `/clock`, `/scan`, `/odom`, `/joint_states` và TF đầy đủ;
5. chỉ khi checker PASS mới khởi động SLAM Toolbox, Nav2 và RViz.

Trong terminal cần thấy `OPENARM ISAAC READY`, sau đó
`PASS: Isaac topics and TF contract are ready for SLAM/Nav2` và
`Managed nodes are active`. RViz được mở trễ 15 giây để không làm timeout
lifecycle trên máy 16 GB. Chỉ gửi `Nav2 Goal` sau marker active; mỗi lần gửi
một goal và chờ robot hoàn thành trước khi chọn goal tiếp theo.

Headless:

```bash
./scripts/run_isaac_nav2.sh \
  scene:=hotel headless:=true use_rviz:=false
```

Trong chế độ mặc định `slam:=true`, RViz xây occupancy map từ `/scan`. Sau khi
điều khiển robot quét đủ scene, lưu riêng từng map:

```bash
# Khi đang chạy scene:=hotel
mkdir -p maps
ros2 run nav2_map_server map_saver_cli \
  -f "$(pwd)/maps/isaac_hotel"

# Khi đang chạy scene:=restaurant
ros2 run nav2_map_server map_saver_cli \
  -f "$(pwd)/maps/isaac_restaurant"
```

Localization bằng đúng scene/map đã lưu:

```bash
./scripts/run_isaac_nav2.sh \
  scene:=hotel slam:=false \
  map:="$(pwd)/maps/isaac_hotel.yaml"

./scripts/run_isaac_nav2.sh \
  scene:=restaurant slam:=false \
  map:="$(pwd)/maps/isaac_restaurant.yaml"
```

Không đổi `scene` trong khi launch đang chạy và không dùng chéo hai file map.
Muốn chuyển scene, nhấn `Ctrl+C`, chờ Isaac đóng rồi chạy lệnh scene còn lại.

## 4. Hợp đồng ROS

```text
Nav2 -> /cmd_vel_nav -> velocity_smoother (Isaac: OPEN_LOOP) -> /cmd_vel
     -> wall-time watchdog (0.5 s) -> /cmd_vel_safe
     -> Isaac static-friction conditioner -> /isaac_cmd_vel -> controller

Isaac -> /clock
Isaac -> /scan             frame base_scan
Isaac -> /isaac_odom_raw -> planar twist corrector -> /odom (frame odom)
Isaac -> /joint_states
Isaac -> odom -> base_footprint
robot_state_publisher -> base_footprint -> base_link -> base_scan -> robot
```

Isaac không subscribe trực tiếp `/cmd_vel`. Nếu Nav2 hoặc teleop chết, watchdog
vẫn phát zero sau 0,5 giây. Hai drive joint dùng bán kính bánh `0.03810 m`,
khoảng cách bánh `0.45 m`; bốn caster joint được giữ passive. Lệnh quay khác
zero nhưng nhỏ hơn `0.25 rad/s` được nâng tới ngưỡng này để thắng ma sát tĩnh
cả khi GUI/RTX đang tải GPU;
lệnh zero vẫn được giữ nguyên nên watchdog luôn có quyền dừng robot.

Isaac 5 báo thành phần twist của `IsaacComputeOdometry` không nhất quán với
pose phẳng của robot này. Node hiệu chỉnh lấy đạo hàm pose theo simulation time,
đổi vận tốc world sang frame `base_footprint` và xuất `/odom` chuẩn cho Nav2.

## 5. Kiểm tra độc lập

Khi simulator đang chạy:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run openarm_skeleton_v1_2_isaac check_isaac_runtime.py --timeout 45
```

Kiểm tra command ngắn, watchdog sẽ tự dừng sau khi publisher kết thúc:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10}, angular: {z: 0.0}}"
```

Nhấn `Ctrl+C` sau khoảng 2 giây và xác nhận robot dừng. Sau đó chạy acceptance
goal Nav2 trong vùng trống:

```bash
ros2 run openarm_skeleton_v1_2_navigation check_nav2_goal.py
```

Để bắt buộc kiểm tra cả quay và đi ngang sang một vị trí khác, có thể gửi goal
90 độ trong frame `odom`:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: odom}, pose: {position: {x: 0.0, y: 0.60}, orientation: {z: 0.70710678, w: 0.70710678}}}}"
```

## 6. Nếu không chạy được

- `Isaac Sim python.sh not found`: đặt đúng `ISAAC_SIM_PATH` tới thư mục có
  `python.sh`, không trỏ tới file zip.
- GUI đóng khi compile shader hoặc máy swap mạnh: đóng Gazebo/browser, thử
  `headless:=true use_rviz:=false`; giới hạn RAM 16 GB vẫn có thể làm runtime
  không ổn định.
- Log có `cuDeviceGetUuid`, `librtx.scenedb.plugin.so` và exit 139: driver
  595.x không tương thích với Isaac Sim 5.x/Kit 107 trên máy này. Chuyển về
  nhánh NVIDIA 580, reboot, xác nhận `nvidia-smi`, rồi chạy lại host checker.
- Không có topic ROS: bảo đảm mọi terminal dùng cùng `ROS_DOMAIN_ID`; không
  source workspace ROS bên trong một terminal dùng để chạy `python.sh` thủ công.
- Cache/version conflict: dùng `./isaac-sim.sh --reset-user` hoặc
  `./clear_caches.sh` theo hướng dẫn NVIDIA, rồi thử lại.
- Không đổi launch sang Isaac Sim 6.0 mà không migration: ROS OmniGraph và
  sensor API 6.0 có thay đổi; profile hiện được khóa cho 5.0.
