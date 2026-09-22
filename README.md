# OpenArm Skeleton v1.2 — ROS 2 Jazzy / Nav2

Workspace tự chứa cho robot `OpenArm Skeleton v1.2`, gồm URDF, 34 mesh STL,
Gazebo Harmonic, lidar 2D, camera RGB-D mô phỏng và bringup Nav2. Không source
hoặc dùng package từ `sim-workspace`.

Workspace cũng chứa package Isaac Sim tùy chọn được tích hợp từ workspace tham
chiếu riêng. Package này vẫn build và chạy contract test khi máy không cài
Isaac Sim; chỉ các phép thử runtime Isaac mới cần bộ cài simulator.

## 1. Nền tảng

- Mục tiêu: Ubuntu 24.04 x86_64, ROS 2 Jazzy, Gazebo Harmonic.
- Bản gốc đã được kiểm chứng trên Humble/Fortress ngày 2026-08-06.
- Source đã được chuyển sang Jazzy/Harmonic ngày 2026-08-18.
- Build, test, hotel route, skeleton demo và Nav2 runtime đã PASS trên
  Jazzy/Harmonic ngày 2026-08-19; xem `docs/TEST_REPORT.md`.

## 2. Cài ROS 2 Jazzy trên Ubuntu 24.04

Khuyến nghị: chạy installer đã được kiểm tra từ tài khoản desktop bình thường
(không thêm `sudo` trước script). Script sẽ hỏi mật khẩu sudo tại máy:

```bash
./scripts/install_jazzy_dependencies.sh
```

Các lệnh thủ công tương đương được giữ bên dưới để học và kiểm tra từng bước.

Máy mới phải cấu hình ROS apt repository trước. Theo hướng dẫn ROS chính thức:

```bash
sudo apt install software-properties-common curl
sudo add-apt-repository universe
sudo apt update

export ROS_APT_SOURCE_VERSION="$(curl -s \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F 'tag_name' | awk -F'"' '{print $4}')"
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update
```

Cài runtime và công cụ cần cho workspace:

```bash
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-ros-gz \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pytest
```

Nếu máy chưa khởi tạo `rosdep`:

```bash
sudo rosdep init
rosdep update
```

## 3. Build

Không copy `build/`, `install/` hoặc `log/` từ máy Humble.

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
./scripts/build_workspace.sh
source install/setup.bash
```

Build thông thường không tải, cài hoặc khởi động Isaac Sim. Hướng dẫn riêng và
giới hạn kiểm chứng nằm tại `docs/ISAAC_SIM.md`.

## 4. Kiểm tra simulation cơ bản

```bash
./scripts/run_hotel_demo.sh auto_run:=false
```

Headless:

```bash
./scripts/run_hotel_demo.sh auto_run:=false headless:=true
```

Pipeline command vẫn giữ ranh giới an toàn:

```text
/cmd_vel -> wall-time watchdog -> /cmd_vel_safe -> Gazebo
```

Watchdog phát lệnh dừng nếu mất command quá 0,5 giây.

## 5. Nav2 + SLAM

### Một file tự tải, build và chạy toàn bộ

File `START_OPENARM.sh` dùng cho demo một-chạm trên Ubuntu 24.04. Có thể đặt
riêng file này ở máy mới; lần chạy đầu nó tự clone nhánh `main` từ GitHub vào
`~/openarm_skeleton_v1.2_ws`, kiểm tra/cài ROS 2 Jazzy và dependency, build,
source môi trường rồi mở Gazebo, robot, SLAM, Nav2 và RViz:

```bash
chmod +x START_OPENARM.sh
./START_OPENARM.sh
```

Nếu double-click từ file manager, launcher sẽ tự mở một Terminal để hiển thị
tiến trình và nơi xảy ra lỗi. Những lần sau nó dùng `git fetch` và chỉ cập nhật
fast-forward khi source local sạch. Nếu có file đang sửa hoặc commit local,
launcher không ghi đè mà giữ source hiện tại để build/chạy. Có thể đổi nơi tải
workspace hoặc truyền launch argument:

```bash
OPENARM_WORKSPACE_DIR="$HOME/demo/openarm_ws" ./START_OPENARM.sh
./START_OPENARM.sh headless:=true use_rviz:=false
```

Việc `source` chỉ áp dụng cho tiến trình launcher và các node được nó mở; file
không chỉnh sửa vĩnh viễn `.bashrc` của người dùng.

### Chạy từ workspace đã có

Khởi động hotel world, lidar, SLAM Toolbox, Nav2 và RViz:

```bash
./scripts/run_nav2_sim.sh
```

Đây là wrapper an toàn cho launch tổng duy nhất:

```bash
ros2 launch openarm_skeleton_v1_2_navigation all_in_one.launch.py
```

`all_in_one.launch.py` là entry point công khai cho toàn bộ simulation stack:
hotel world, robot CAD, lidar, command watchdog, SLAM hoặc localization, Nav2
và RViz. Nên tiếp tục dùng wrapper `run_nav2_sim.sh` khi chạy từ Snap VS Code
vì wrapper còn làm sạch GTK/GIO environment trước khi mở GUI.

Gazebo được khởi động trước để nạp model CAD; Nav2 bắt đầu sau 7 giây và RViz
sau 9 giây. Hãy chờ robot xuất hiện trên ô xanh thay vì gửi goal ngay khi cửa
sổ đầu tiên vừa mở. Thứ tự này tránh Gazebo, Nav2 và RViz cùng nạp CPU/GPU lúc
model 32 MB đang được dựng.

Nên dùng wrapper trên khi terminal nằm trong Snap VS Code. Wrapper tự loại các
GTK/GIO path của Snap core20 trước khi mở RViz/Gazebo, tránh lỗi
`libpthread.so.0 ... GLIBC_PRIVATE` trên Ubuntu 24.04.

Chạy không có GUI:

```bash
./scripts/run_nav2_sim.sh headless:=true use_rviz:=false
```

Trong RViz cần xác nhận `/scan`, TF `map -> odom -> base_footprint -> base_link`
và hai costmap hoạt động. Cấu hình RViz của workspace bật sẵn `RobotModel` và
đặt camera tại vị trí spawn. Khi map còn trống, điều khiển robot khám phá bằng
`/cmd_vel`; Nav2 output vẫn đi qua velocity smoother và watchdog.

Map SLAM là lát cắt occupancy 2D tại độ cao lidar, không phải bản sao hình ảnh
3D của Gazebo: trắng là vùng trống, đen là vật cản tia laser nhìn thấy và xám
là vùng chưa biết. Màu sắc, texture và vật nằm ngoài mặt phẳng quét sẽ không
được dựng lại. Tuy nhiên tường không được xoay/chồng thành nhiều bản khi robot
quay. Nếu map đã được tạo trước bản sửa trục bánh xe ngày 2026-08-19, không tái
sử dụng map đó; hãy dừng launch và tạo map mới từ một session sạch.

Lidar simulation dùng tầm quét 20 m để quan sát hết hotel lobby ngay từ vị trí
spawn. Nav2 hiện cho controller bám trực tiếp đường NavFn; node `SmoothPath`
được giữ dưới dạng comment trong behavior tree nhưng tạm tắt vì collision check
của `SimpleSmoother` làm các goal xa hoặc cong bị abort trước khi tới
`FollowPath`. Velocity smoother và watchdog vẫn hoạt động bình thường. Robot
vẫn sẽ chủ động đi cong khi đường thẳng có vật cản; đó là hành vi đúng của
Nav2.

GUI hotel/Nav2 không hiển thị nút **Reset world**. Robot được spawn động sau
khi world SDF đã load; full reset sẽ xóa robot, đưa `/clock` về 0 và làm dữ
liệu SLAM/TF cũ không còn hợp lệ. Muốn bắt đầu lại sạch, nhấn `Ctrl+C`, chờ
launch dừng, rồi chạy lại `./scripts/run_nav2_sim.sh`.

Lưu map:

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f "$(pwd)/maps/hotel"
```

Chạy localization với map đã lưu:

```bash
./scripts/run_nav2_sim.sh \
  slam:=false map:="$(pwd)/maps/hotel.yaml"
```

Profile hotel tự khởi tạo AMCL tại pose map `(0, 0, 0)`, tương ứng vị trí
Gazebo spawn cố định nơi phiên SLAM bắt đầu, nên global costmap có thể activate
mà không cần click `2D Pose Estimate`. Công cụ `2D Pose Estimate` vẫn có thể
ghi đè pose nếu cần. Không dùng giá trị mặc định này cho map hoặc vị trí spawn
khác; khi đó phải cấu hình lại `amcl.initial_pose` theo pose khởi tạo thực tế.

Kiểm tra Nav2 tự động bằng action result và `/odom`:

```bash
ros2 run openarm_skeleton_v1_2_navigation check_nav2_goal.py
```

Kết quả chuẩn là `PASS`, action status `SUCCEEDED`, chuyển động lớn hơn ngưỡng
0,25 m và `cross_track` không quá 0,05 m. Chỉ chạy lệnh này khi phía trước robot
trong simulation trống; nếu có vật cản thì Nav2 phải được phép đi vòng.
Acceptance trong `docs/TEST_REPORT.md` còn đối chiếu riêng pose vật lý trực tiếp
từ Gazebo để tránh trường hợp odometry tự báo đúng nhưng chassis đi sai.

## 6. Kiểm thử

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Kết quả regression mới nhất ngày 2026-08-26: 3 package build thành công, 32
test, 0 lỗi. RGB-D runtime, hotel route, skeleton demo và Nav2 goal runtime gần
nhất đều PASS; xem ngày và lệnh chính xác trong `docs/TEST_REPORT.md`.

## 7. Topic và frame contract

| Topic/frame | Hướng | Ý nghĩa |
|---|---|---|
| `/cmd_vel_nav` | Nav2 internal | Command trước velocity smoother |
| `/cmd_vel` | ROS → watchdog | Command public đã được Nav2 smooth |
| `/cmd_vel_safe` | watchdog → Gazebo | Command có timeout stop |
| `/scan` (`base_scan`) | Gazebo → ROS | Lidar 2D, 720 mẫu, 10 Hz, 0.35–20 m |
| `/camera/color/image_raw` (`camera_optical_frame`) | Gazebo → ROS | Ảnh RGB 640×480, target 10 Hz |
| `/camera/depth/image_raw` (`camera_optical_frame`) | Gazebo → ROS | Depth image 640×480, target 10 Hz, 0.20–10 m |
| `/camera/camera_info` | Gazebo → ROS | Tham số nội tại của camera RGB-D |
| `/odom` | Gazebo → ROS | Odometry và velocity feedback |
| `map -> odom` | SLAM/AMCL | Global localization transform |
| `odom -> base_footprint` | Gazebo | Local odometry transform |

## 8. Thành phần workspace

- `openarm_skeleton_v1_2_description`: URDF, mesh, RViz và profile simulation.
- `openarm_skeleton_v1_2_gazebo`: Gazebo Harmonic, bridge, spawner, watchdog,
  hotel world và route demo.
- `openarm_skeleton_v1_2_navigation`: Nav2, SLAM/localization, footprint và
  costmap configuration.
- `openarm_skeleton_v1_2_isaac`: integration Isaac Sim tùy chọn; build/test
  tĩnh không yêu cầu cài Isaac Sim.
- `tools/prepare_model.py`: tái tạo URDF; thay đổi Gazebo/lidar phải sửa tại đây
  rồi chạy lại script.

## 9. Ranh giới an toàn

- `/cmd_vel` luôn phải qua wall-time watchdog.
- Hotel/Nav2 profile chỉ cho phép sáu khớp mobile base chuyển động.
- Skeleton-lift vẫn là simulation-only.
- Footprint và tuning Nav2 hiện là giá trị khởi tạo từ collision geometry; phải
  đo và tune lại trước khi dùng trên robot thật.
- Lidar simulation dùng frame `base_scan` ở giữa/phía trên base và dead zone
  0,35 m để mesh của robot không tự tạo obstacle. Đây không phải calibration
  lidar thật; hardware phải dùng transform đo được và self-filter của driver.
- RGB-D simulation dùng `camera_link` CAD và frame ROS
  `camera_optical_frame`. FOV, clip range và intrinsic hiện là thông số mô
  phỏng; phải thay bằng calibration của camera thật khi bring-up hardware.
- Các topic ảnh dùng sensor-data QoS (`Best Effort`). Hai Image display trong
  RViz đã được cấu hình cùng QoS; Image display thêm thủ công cũng phải chọn
  `Best Effort` để tránh trạng thái `No Image`.
- Depth image dùng encoding `32FC1`, đơn vị mét. RViz dùng dải hiển thị cố định
  0–10 m (`Normalize Range: false`) để pixel `Inf` ngoài tầm không làm toàn ảnh
  bị hiển thị đen.
- Point cloud không được bridge sang ROS để giảm tải. Đo vị trí vật thể nên dùng
  depth image kết hợp `camera_info`, sau đó đổi frame bằng TF2 khi cần.
- Simulator ground truth chỉ dùng cho test/evaluation.
