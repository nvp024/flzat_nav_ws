"""Static and generated-asset contracts for the Isaac Sim integration."""

import ast
import importlib.util
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
DESCRIPTION = PACKAGE.parent / "openarm_skeleton_v1_2_description"
SOURCE_URDF = (
    DESCRIPTION / "urdf" / "openarm_skeleton_v1_2_base_demo.urdf"
)
MESHES = DESCRIPTION / "meshes"
PREPARE = PACKAGE / "scripts" / "prepare_isaac_urdf.py"
SIMULATOR = PACKAGE / "scripts" / "openarm_isaac_sim.py"
SCENES = PACKAGE / "scripts" / "isaac_scenes.py"
SDF_SCENE = PACKAGE / "scripts" / "isaac_sdf_scene.py"
CHECKER = PACKAGE / "scripts" / "check_isaac_runtime.py"
COMMAND_CONDITIONER = PACKAGE / "scripts" / "condition_isaac_cmd_vel.py"
ODOMETRY_CORRECTOR = PACKAGE / "scripts" / "correct_isaac_odometry.py"
LAUNCH = PACKAGE / "launch" / "isaac_nav2.launch.py"
RUNNER = WORKSPACE / "scripts" / "run_isaac_nav2.sh"
HOST_CHECK = WORKSPACE / "scripts" / "check_isaac_host.sh"
HOTEL_WORLD = (
    PACKAGE.parent
    / "openarm_skeleton_v1_2_gazebo"
    / "worlds"
    / "hotel_lobby_demo.sdf"
)

ACTIVE_BASE_JOINTS = {
    "caster_joint_1",
    "caster_wheel_joint_1",
    "caster_joint_2",
    "caster_wheel_joint_2",
    "drive_joint_1",
    "drive_joint_2",
}


def _load_prepare_module():
    spec = importlib.util.spec_from_file_location("prepare_isaac_urdf", PREPARE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_scenes_module():
    spec = importlib.util.spec_from_file_location("isaac_scenes", SCENES)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_sdf_scene_module():
    spec = importlib.util.spec_from_file_location("isaac_sdf_scene", SDF_SCENE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_prepared_urdf_is_isaac_only_and_portable(tmp_path):
    output = tmp_path / "openarm_isaac.urdf"
    module = _load_prepare_module()
    assert module.prepare_urdf(SOURCE_URDF, MESHES, output) == output
    root = ET.parse(output).getroot()
    assert not root.findall("./gazebo")
    assert len(root.findall("./link")) == 38
    assert len(root.findall("./joint")) == 37
    movable = {
        joint.get("name")
        for joint in root.findall("./joint")
        if joint.get("type") != "fixed"
    }
    assert movable == ACTIVE_BASE_JOINTS
    for mesh in root.findall(".//mesh"):
        path = Path(mesh.get("filename"))
        assert path.is_absolute()
        assert path.is_file()
        assert "package://" not in str(path)
    for name in (
        "base_footprint",
        "base_link",
        "base_scan",
        "camera_optical_frame",
    ):
        assert root.find(f"./link[@name='{name}']/inertial") is not None


def test_prepared_urdf_accepts_colcon_mesh_symlinks(tmp_path):
    installed_meshes = tmp_path / "install" / "meshes"
    installed_meshes.mkdir(parents=True)
    for source_mesh in MESHES.iterdir():
        if source_mesh.is_file():
            (installed_meshes / source_mesh.name).symlink_to(source_mesh)

    output = tmp_path / "openarm_from_install.urdf"
    module = _load_prepare_module()
    assert module.prepare_urdf(SOURCE_URDF, installed_meshes, output) == output
    for mesh in ET.parse(output).getroot().findall(".//mesh"):
        resolved = Path(mesh.get("filename"))
        assert resolved.is_file()
        assert MESHES in resolved.parents


def test_simulator_uses_verified_ros_and_drive_contracts():
    source = SIMULATOR.read_text(encoding="utf-8")
    ast.parse(source)
    assert 'COMMAND_TOPIC = "isaac_cmd_vel"' in source
    assert 'JOINT_STATE_TOPIC = "joint_states"' in source
    assert 'ODOMETRY_TOPIC = "isaac_odom_raw"' in source
    assert 'SCAN_TOPIC = "scan"' in source
    assert 'BASE_FRAME = "base_footprint"' in source
    assert 'SCAN_FRAME = "base_scan"' in source
    assert 'WHEEL_JOINTS = ["drive_joint_1", "drive_joint_2"]' in source
    assert "PASSIVE_CASTER_JOINTS" in source
    assert "_configure_joint_drives(stage, UsdPhysics)" in source
    assert "DRIVE_DAMPING = 1.0e5" in source
    assert "DRIVE_MAX_FORCE = 30.0" in source
    assert "drive.GetDampingAttr().Set(DRIVE_DAMPING)" in source
    assert "drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)" in source
    assert "WHEEL_RADIUS_METERS = 0.03810" in source
    assert "WHEEL_DISTANCE_METERS = 0.45" in source
    assert "ROS2SubscribeTwist" in source
    assert "DifferentialController" in source
    assert "ROS2PublishJointState" in source
    assert "ROS2PublishOdometry" in source
    assert "ROS2RtxLidarHelper" in source
    assert "ROS2PublishClock" in source
    assert 'default="Example_Rotary_2D"' in source
    assert "skipDroppingInvalidPoints" in source
    assert "Context.inputs:useDomainIDEnvVar" in source
    assert "PublishOdom.inputs:chassisFrameId" in source
    assert "gz::sim" not in source


def test_restaurant_procedural_scene_remains_spawn_clear():
    scenes = _load_scenes_module()
    assert scenes.SCENE_NAMES == ("hotel", "restaurant")
    assert set(scenes.SCENE_OBJECTS) == {"restaurant"}
    assert len(scenes.get_scene_objects("restaurant")) >= 28

    objects = scenes.get_scene_objects("restaurant")
    names = [item["name"] for item in objects]
    assert len(names) == len(set(names))
    assert {
        "north_wall",
        "south_wall",
        "east_wall",
        "west_wall",
    } <= set(names)
    for item in objects:
        x, y, _z = item["position"]
        width, depth, height = item["scale"]
        assert min(width, depth, height) > 0.0
        nearest_x = max(abs(x) - width / 2.0, 0.0)
        nearest_y = max(abs(y) - depth / 2.0, 0.0)
        assert math.hypot(nearest_x, nearest_y) >= (
            scenes.ROBOT_SPAWN_CLEARANCE_METERS
        )


def test_hotel_is_loaded_from_gazebo_sdf_in_slam_start_frame():
    module = _load_sdf_scene_module()
    primitives = module.load_hotel_primitives(HOTEL_WORLD)
    assert len(primitives) >= 190
    assert {item.role for item in primitives} == {"visual", "collision"}
    assert {item.shape for item in primitives} == {
        "box",
        "cylinder",
        "sphere",
        "plane",
    }
    assert len({item.name for item in primitives}) == len(primitives)
    assert not any("reception_main_counter" in item.name for item in primitives)

    by_name = {item.name: item for item in primitives}
    east_wall = by_name["visual__east_wall__link__visual"]
    assert all(
        math.isclose(actual, expected, abs_tol=1.0e-9)
        for actual, expected in zip(
            east_wall.position, (19.05, 3.5, 1.4)
        )
    )
    assert east_wall.scale == (0.30, 12.0, 2.8)
    corridor_end = by_name[
        "visual__entry_corridor_end_wall__link__visual"
    ]
    assert math.isclose(corridor_end.position[0], -0.95, abs_tol=1.0e-9)
    assert corridor_end.position[1:] == (0.0, 1.4)
    start_zone = by_name["visual__hotel_start_zone__link__visual"]
    assert start_zone.position == (0.0, 0.0, 0.015)

    source = SDF_SCENE.read_text(encoding="utf-8")
    ast.parse(source)
    assert "GAZEBO_HOTEL_SPAWN = (-11.2, -3.5, 0.0)" in source
    assert "unsupported SDF geometry" in source


def test_simulator_selects_and_reports_requested_scene():
    source = SIMULATOR.read_text(encoding="utf-8")
    assert (
        'parser.add_argument("--scene", choices=SCENE_NAMES, default="hotel")'
        in source
    )
    assert '"--hotel-world"' in source
    assert "load_hotel_primitives(hotel_world)" in source
    assert "VisualCuboid" in source
    assert "FixedCylinder" in source
    assert "FixedSphere" in source
    assert 'print(f"  scene: {args.scene}", flush=True)' in source


def test_launch_is_a_single_gated_isaac_nav2_entry_point():
    source = LAUNCH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    declared = {
        ast.literal_eval(node.args[0])
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "DeclareLaunchArgument"
        and node.args
    }
    assert {
        "start_isaac",
        "scene",
        "hotel_world",
        "isaac_sim_path",
        "headless",
        "startup_timeout",
        "slam",
        "map",
        "use_rviz",
        "use_sim_time",
        "autostart",
        "params_file",
    } <= declared
    assert "openarm_isaac_sim.py" in source
    assert '"--scene"' in source
    assert '"--hotel-world"' in source
    assert '"hotel_lobby_demo.sdf"' in source
    assert 'scene not in {"hotel", "restaurant"}' in source
    assert "correct_isaac_odometry.py" in source
    assert "condition_isaac_cmd_vel.py" in source
    assert "check_isaac_runtime.py" in source
    assert "cmd_vel_watchdog.py" in source
    assert "robot_state_publisher" in source
    assert "slam_launch.py" in source
    assert "localization_launch.py" in source
    assert '"use_composition": "False"' in source
    assert "navigation.launch.py" in source
    assert '"velocity_smoother_feedback": "OPEN_LOOP"' in source
    assert "TimerAction(period=3.0, actions=[navigation])" in source
    assert "TimerAction(period=15.0, actions=[rviz])" in source
    assert "Isaac ROS contract passed; starting SLAM/Nav2." in source
    assert 'watchdog, "Base command watchdog"' in source
    assert 'environment.pop(name, None)' in source
    assert 'environment["ROS_DISTRO"] = "jazzy"' in source
    assert 'environment["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"' in source
    assert '"isaacsim.ros2.bridge"' in source
    assert '"jazzy"' in source
    assert '"lib"' in source


def test_runtime_checker_requires_topics_and_complete_tf_chain():
    source = CHECKER.read_text(encoding="utf-8")
    ast.parse(source)
    for topic in ("/clock", "/scan", "/odom", "/joint_states"):
        assert topic in source
    assert '"odom", "base_footprint"' in source
    assert '"base_link", "base_scan"' in source
    assert "message.header.frame_id != \"base_scan\"" in source
    assert "message.header.frame_id != \"odom\"" in source
    assert "message.child_frame_id != \"base_footprint\"" in source


def test_odometry_corrector_derives_planar_twist_from_pose():
    source = ODOMETRY_CORRECTOR.read_text(encoding="utf-8")
    ast.parse(source)
    assert '"/isaac_odom_raw"' in source
    assert '"/odom"' in source
    assert "_wrapped_angle(yaw - previous_yaw) / elapsed" in source
    assert "twist.linear.z = 0.0" in source
    assert "twist.angular.z = angular_z" in source
    assert "except KeyboardInterrupt:" in source
    assert "if rclpy.ok():" in source


def test_command_conditioner_compensates_isaac_static_friction():
    source = COMMAND_CONDITIONER.read_text(encoding="utf-8")
    ast.parse(source)
    assert "MIN_ANGULAR_SPEED = 0.25" in source
    assert '"/cmd_vel_safe"' in source
    assert '"/isaac_cmd_vel"' in source
    assert "0.0 < abs(angular) < MIN_ANGULAR_SPEED" in source
    assert "except KeyboardInterrupt:" in source
    assert "if rclpy.ok():" in source


def test_public_runner_builds_and_launches_from_isaac_path():
    source = RUNNER.read_text(encoding="utf-8")
    assert RUNNER.stat().st_mode & 0o111
    subprocess.run(["bash", "-n", RUNNER], check=True)
    assert "ISAAC_SIM_PATH" in source
    assert "build_workspace.sh" in source
    assert "check_isaac_host.sh" in source
    assert "isaac_nav2.launch.py" in source
    assert "isaac_sim_path:=" in source

    host_check = HOST_CHECK.read_text(encoding="utf-8")
    subprocess.run(["bash", "-n", HOST_CHECK], check=True)
    assert "driver_major >= 595" in host_check
    assert "validated 580" in host_check


def test_python_entry_points_compile_without_importing_isaac(tmp_path):
    for script in (
        PREPARE,
        SCENES,
        SIMULATOR,
        CHECKER,
        COMMAND_CONDITIONER,
        ODOMETRY_CORRECTOR,
        LAUNCH,
    ):
        subprocess.run(
            ["python3", "-m", "py_compile", str(script)],
            cwd=tmp_path,
            check=True,
        )


def test_runtime_ready_and_error_markers_are_flushed_to_launch_log():
    source = SIMULATOR.read_text(encoding="utf-8")
    assert 'print("OPENARM ISAAC READY", flush=True)' in source
    assert 'print(f"OPENARM ISAAC ERROR: {error}", flush=True)' in source
