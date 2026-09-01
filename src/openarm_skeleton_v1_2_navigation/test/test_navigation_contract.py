"""Static contracts for the initial Jazzy Nav2 integration."""

import ast
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
PARAMS = PACKAGE / "config" / "nav2_params.yaml"
NAVIGATION_LAUNCH = PACKAGE / "launch" / "navigation.launch.py"
SIM_LAUNCH = PACKAGE / "launch" / "nav2_sim.launch.py"
ALL_IN_ONE_LAUNCH = PACKAGE / "launch" / "all_in_one.launch.py"
RUN_NAV2 = PACKAGE.parents[1] / "scripts" / "run_nav2_sim.sh"
ONE_FILE_LAUNCHER = PACKAGE.parents[1] / "START_OPENARM.sh"
GOAL_CHECK = PACKAGE / "scripts" / "check_nav2_goal.py"
RVIZ_CONFIG = PACKAGE / "config" / "nav2_openarm_view.rviz"
SMOOTHED_NAV_BT = (
    PACKAGE / "behavior_trees" / "navigate_to_pose_with_smoothing.xml"
)


def _params():
    return yaml.safe_load(PARAMS.read_text(encoding="utf-8"))


def test_nav2_uses_expected_frames_scan_and_footprint():
    params = _params()
    assert params["amcl"]["ros__parameters"]["scan_topic"] == "/scan"
    assert params["slam_toolbox"]["ros__parameters"]["scan_topic"] == "/scan"
    assert params["slam_toolbox"]["ros__parameters"]["map_frame"] == "map"
    assert params["slam_toolbox"]["ros__parameters"]["odom_frame"] == "odom"
    local = params["local_costmap"]["local_costmap"]["ros__parameters"]
    global_params = params["global_costmap"]["global_costmap"]["ros__parameters"]
    assert local["robot_base_frame"] == "base_link"
    assert local["footprint"] == global_params["footprint"]
    assert local["obstacle_layer"]["scan"]["topic"] == "/scan"
    assert params["slam_toolbox"]["ros__parameters"]["max_laser_range"] == 20.0
    assert params["amcl"]["ros__parameters"]["laser_max_range"] == 20.0
    for costmap in (local, global_params):
        scan = costmap["obstacle_layer"]["scan"]
        assert scan["inf_is_valid"] is True
        assert scan["raytrace_max_range"] == 20.0


def test_amcl_auto_initializes_at_the_fixed_hotel_spawn():
    amcl = _params()["amcl"]["ros__parameters"]
    assert amcl["set_initial_pose"] is True
    assert amcl["always_reset_initial_pose"] is False
    assert amcl["initial_pose"] == {
        "x": 0.0,
        "y": 0.0,
        "z": 0.0,
        "yaw": 0.0,
    }


def test_velocity_chain_keeps_the_independent_watchdog_boundary():
    launch = NAVIGATION_LAUNCH.read_text(encoding="utf-8")
    assert '("cmd_vel", "cmd_vel_nav")' in launch
    assert '("cmd_vel_smoothed", "cmd_vel")' in launch
    params = _params()
    for node in ("controller_server", "behavior_server", "velocity_smoother"):
        assert params[node]["ros__parameters"]["enable_stamped_cmd_vel"] is False


def test_sim_launch_supports_slam_and_saved_map_localization():
    launch = SIM_LAUNCH.read_text(encoding="utf-8")
    assert "slam_launch.py" in launch
    assert "localization_launch.py" in launch
    assert '"use_composition": "False"' in launch
    assert '"auto_run": "false"' in launch
    assert "map:=/absolute/path/to/map.yaml" in launch
    assert "nav2_openarm_view.rviz" in launch
    rviz = RVIZ_CONFIG.read_text(encoding="utf-8")
    assert "Class: rviz_default_plugins/RobotModel" in rviz
    assert "Enabled: true" in rviz
    assert "Value: /robot_description" in rviz


def test_rviz_rgbd_displays_use_sensor_data_qos():
    config = yaml.safe_load(RVIZ_CONFIG.read_text(encoding="utf-8"))
    displays = config["Visualization Manager"]["Displays"]
    images = {
        display["Name"]: display
        for display in displays
        if display.get("Class") == "rviz_default_plugins/Image"
    }
    expected = {
        "RGB Camera": "/camera/color/image_raw",
        "Depth Camera": "/camera/depth/image_raw",
    }
    assert images.keys() >= expected.keys()
    for name, topic in expected.items():
        qos = images[name]["Topic"]
        assert qos["Value"] == topic
        assert qos["Reliability Policy"] == "Best Effort"
        assert qos["Durability Policy"] == "Volatile"
    depth = images["Depth Camera"]
    assert depth["Normalize Range"] is False
    assert float(depth["Min Value"]) == 0.0
    assert float(depth["Max Value"]) == 10.0


def test_rviz_displays_completed_rag_object_coordinates():
    config = yaml.safe_load(RVIZ_CONFIG.read_text(encoding="utf-8"))
    displays = config["Visualization Manager"]["Displays"]
    markers = next(
        display
        for display in displays
        if display.get("Name") == "RAG Object Coordinates"
    )
    assert markers["Class"] == "rviz_default_plugins/MarkerArray"
    assert markers["Enabled"] is True
    assert markers["Topic"]["Value"] == "/environment_memory/object_markers"
    assert markers["Topic"]["Durability Policy"] == "Transient Local"


def test_rviz_uses_3d_orbit_view_without_pointcloud_display():
    config = yaml.safe_load(RVIZ_CONFIG.read_text(encoding="utf-8"))
    displays = config["Visualization Manager"]["Displays"]
    assert all(
        display.get("Class") != "rviz_default_plugins/PointCloud2"
        for display in displays
    )
    view = config["Visualization Manager"]["Views"]["Current"]
    assert view["Class"] == "rviz_default_plugins/Orbit"
    assert float(view["Distance"]) > 0.0
    assert 0.0 < float(view["Pitch"]) < 1.57


def test_all_in_one_is_the_public_complete_stack_entry_point():
    launch = ALL_IN_ONE_LAUNCH.read_text(encoding="utf-8")
    tree = ast.parse(launch)
    runner = RUN_NAV2.read_text(encoding="utf-8")
    declared = {
        ast.literal_eval(node.args[0])
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "DeclareLaunchArgument"
        and node.args
    }
    expected = {
        "slam",
        "map",
        "headless",
        "use_rviz",
        "use_sim_time",
        "autostart",
        "params_file",
        "transport_partition",
    }
    assert expected <= declared
    assert "nav2_sim.launch.py" in launch
    for argument in expected:
        assert f'LaunchConfiguration("{argument}")' in launch
    assert "all_in_one.launch.py" in runner
    assert "nav2_sim.launch.py" not in runner


def test_one_file_launcher_bootstraps_builds_and_runs_the_complete_stack():
    launcher = ONE_FILE_LAUNCHER.read_text(encoding="utf-8")
    assert ONE_FILE_LAUNCHER.stat().st_mode & 0o111
    subprocess.run(["bash", "-n", ONE_FILE_LAUNCHER], check=True)
    assert "DinhPhuHai/openarm_skeleton_v1.2.git" in launcher
    assert 'git clone --branch "$REPOSITORY_BRANCH"' in launcher
    assert 'merge --ff-only "origin/$REPOSITORY_BRANCH"' in launcher
    assert "install_jazzy_dependencies.sh" in launcher
    assert "rosdep install --from-paths" in launcher
    assert "build_workspace.sh" in launcher
    assert "run_nav2_sim.sh" in launcher


def test_sim_launch_gives_gazebo_time_to_load_robot_visual():
    launch = SIM_LAUNCH.read_text(encoding="utf-8")
    tree = ast.parse(launch)
    constants = {
        target.id: ast.literal_eval(node.value)
        for node in tree.body
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance((target := node.targets[0]), ast.Name)
        and target.id
        in {
            "NAVIGATION_START_DELAY_SECONDS",
            "RVIZ_START_DELAY_SECONDS",
        }
    }
    assert constants["NAVIGATION_START_DELAY_SECONDS"] >= 7.0
    assert (
        constants["RVIZ_START_DELAY_SECONDS"]
        > constants["NAVIGATION_START_DELAY_SECONDS"]
    )
    assert "period=NAVIGATION_START_DELAY_SECONDS" in launch
    assert "period=RVIZ_START_DELAY_SECONDS" in launch


def test_navigation_goal_acceptance_check_is_installed():
    cmake = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
    script = GOAL_CHECK.read_text(encoding="utf-8")
    assert "PROGRAMS scripts/check_nav2_goal.py" in cmake
    assert 'frame_id = "odom"' in script
    assert "GoalStatus.STATUS_SUCCEEDED" in script
    assert "motion >= args.min_motion" in script
    assert "abs(cross_track) <= args.max_cross_track" in script


def test_navfn_path_is_followed_directly_while_path_smoother_is_disabled():
    launch = NAVIGATION_LAUNCH.read_text(encoding="utf-8")
    tree = SMOOTHED_NAV_BT.read_text(encoding="utf-8")
    root = ET.parse(SMOOTHED_NAV_BT).getroot()
    cmake = (PACKAGE / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "default_nav_to_pose_bt_xml" in launch
    assert "navigate_to_pose_with_smoothing.xml" in launch
    assert "DIRECTORY behavior_trees config launch" in cmake
    compute_path = root.find(".//ComputePathToPose")
    follow_path = root.find(".//FollowPath")
    assert compute_path is not None
    assert compute_path.attrib["path"] == "{path}"
    assert follow_path is not None
    assert follow_path.attrib["path"] == "{path}"
    assert root.find(".//SmoothPath") is None
    assert "Disabled after repeated SMOOTHED_PATH_IN_COLLISION" in tree
    assert '<SmoothPath unsmoothed_path="{raw_path}"' in tree
    assert 'smoothed_path="{path}"' in tree
