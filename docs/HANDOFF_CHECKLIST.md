# Checklist bàn giao OpenArm Skeleton v1.2

## Người đóng gói

- [ ] `./scripts/build_workspace.sh` hoàn tất 4 package.
- [ ] `colcon test-result --verbose` báo 0 lỗi.
- [ ] Hotel headless có dòng `INDOOR ROUTE PASS`.
- [ ] Skeleton demo có dòng `Skeleton lift demo PASS`.
- [ ] Chạy `./scripts/create_handoff_package.sh`.
- [ ] Gửi cả file `.tar.gz` và `.tar.gz.sha256`.
- [ ] Không gửi riêng `build/`, `install/`, `log/`.
- [ ] Đã xác nhận quyền phân phối các file CAD/STL cho người nhận.

## Người nhận

- [ ] Máy dùng Ubuntu 24.04 x86_64 và ROS 2 Jazzy.
- [ ] Đã cài `ros-jazzy-ros-gz`, Nav2, SLAM Toolbox, `colcon` và `rosdep`.
- [ ] `sha256sum -c ...sha256` báo `OK`.
- [ ] Giải nén archive vào thư mục có quyền ghi.
- [ ] Chạy `rosdep install --from-paths src --ignore-src -r -y`.
- [ ] Chạy `./scripts/build_workspace.sh`.
- [ ] Chạy hotel demo bằng `./scripts/run_hotel_demo.sh`.
- [ ] `/scan` khoảng 10 Hz và frame là `base_scan`.
- [ ] `/camera/color/image_raw` và `/camera/depth/image_raw` xuất liên tục ở
  640×480; target cấu hình là 10 Hz, tốc độ thực tế phụ thuộc CPU/GPU.
- [ ] `/camera/camera_info` có frame `camera_optical_frame`.
- [ ] Chạy `./scripts/run_nav2_sim.sh` và xác nhận SLAM/costmap hoạt động.
- [ ] `check_nav2_goal.py` báo PASS và `odom_motion` lớn hơn 0,25 m.
- [ ] Không source một OpenArm workspace cũ trong cùng terminal.

## Kết quả bàn giao chuẩn

```text
Build: 4 packages finished
Tests: 0 errors, 0 failures
Hotel: INDOOR ROUTE PASS
Skeleton: Skeleton lift demo PASS
Nav2: scan + TF + SLAM + costmaps + goal PASS
```

## Giới hạn phải thông báo

- Gazebo/Nav2 profile đã PASS runtime acceptance trên Jazzy/Harmonic
  ngày 2026-08-19; máy mới vẫn phải chạy lại checklist.
- Hotel là procedural demo map, không phải scan khách sạn thật.
- Skeleton-lift dùng giới hạn/controller tạm thời trong simulation.
- Đã có lidar và camera RGB-D simulation; chưa có MoveIt, VLM hoặc ACT/VLA
  runtime.
- `base_scan` simulation dùng transform/dead zone dành cho Gazebo, không phải
  calibration của lidar hardware.
- `camera_optical_frame`, FOV và clip range là cấu hình simulation, không phải
  calibration của camera RGB-D hardware.
