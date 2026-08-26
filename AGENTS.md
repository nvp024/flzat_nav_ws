# AI handoff rules for OpenArm Skeleton v1.2

## Source of truth

The directory containing this file is the active official ROS 2 / Gazebo
workspace. `/home/hai/sim-workspace` is legacy reference only: do not add
runtime, build, launch, or documentation dependencies on it.

## Read before changes

1. `README.md`
2. `docs/QUICK_START.md`
3. `docs/HANDOFF_CHECKLIST.md`
4. `docs/VLM_ROBOT_ARCHITECTURE_TREE.md`
5. `docs/TEST_REPORT.md`

## Fixed contracts

- Public base command is `/cmd_vel`; it must pass through the wall-time
  watchdog before Gazebo receives `/cmd_vel_safe`.
- `base_demo` moves only the six approved base joints. Upper body and arms are
  locked.
- `lift_demo` is provisional and simulation-only. Never infer hardware safety
  from it.
- VLM emits a schema-validated `GroundedGoal`, not joint positions, poses,
  torque, or raw controller commands.
- VLA/ACT, when used, is a closed-loop skill implementation under the skill
  executive and above the independent safety/action guard.
- Simulator ground truth is for tests/evaluation only.

## Validation

```bash
cd /path/to/openarm_skeleton_v1.2_ws
source /opt/ros/jazzy/setup.bash
./scripts/build_workspace.sh
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

For runtime acceptance, launch the hotel demo headlessly and require every
route segment plus the final `INDOOR ROUTE PASS`. For Nav2, also require
`/scan`, the full map/odom/base TF chain, SLAM or AMCL, both costmaps and a
completed goal. Record the exact commands, date and results in
`docs/TEST_REPORT.md`.
