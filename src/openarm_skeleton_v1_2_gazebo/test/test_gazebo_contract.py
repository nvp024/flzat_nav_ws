"""Static contracts for the self-contained Gazebo package."""

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = PACKAGE.parent
DESCRIPTION = WORKSPACE_SRC / "openarm_skeleton_v1_2_description"
SIMULATION_LAUNCH = PACKAGE / "launch" / "simulation.launch.py"
HOTEL_LAUNCH = PACKAGE / "launch" / "hotel_demo.launch.py"
HOTEL_WORLD = PACKAGE / "worlds" / "hotel_lobby_demo.sdf"
BRIDGE_CONFIG = PACKAGE / "config" / "bridge.yaml"
HOTEL_GUI_CONFIG = PACKAGE / "config" / "hotel_nav_gui.config"


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_package_is_self_contained_and_uses_v1_2_names():
    source_text = "\n".join(
        path.read_text(encoding="utf-8")
        for path in PACKAGE.rglob("*")
        if path.is_file()
        and "__pycache__" not in path.parts
        and "test" not in path.parts
        and path.suffix in {".py", ".xml", ".txt", ".yaml"}
    )
    assert "sim-workspace" not in source_text
    assert "openarm_mobile_gazebo" not in source_text
    assert "openarm_skeleton_description" not in source_text
    assert "openarm_skeleton_gazebo" not in source_text
    assert "openarm_skeleton_v1_2_description" in source_text
    assert "openarm_skeleton_v1_2_gazebo" in source_text


def test_simulation_profiles_are_prepared_in_description_package():
    for filename in (
        "openarm_skeleton_v1_2_base_demo.urdf",
        "openarm_skeleton_v1_2_lift_demo.urdf",
    ):
        root = ET.parse(DESCRIPTION / "urdf" / filename).getroot()
        assert root.get("name") == "openarm_skeleton_v1.2"
        assert len(root.findall("./link")) == 38
        assert len(root.findall("./joint")) == 37


def test_hotel_world_and_route_landmarks_are_local_assets():
    root = ET.parse(HOTEL_WORLD).getroot()
    world = root.find("./world")
    assert world is not None
    assert world.get("name") == "hotel_lobby"
    model_names = {
        model.get("name") for model in world.findall("./model")
    }
    assert {
        "hotel_floor",
        "indoor_dining_table",
        "lobby_sofa_west",
        "lobby_wall_clock",
        "living_kitchen_partition",
        "living_tv_unit",
        "living_side_table",
        "living_armchair_south",
        "living_floor_plant",
        "living_coffee_table_decor",
        "living_wall_art",
        "kitchen_counter_set",
        "kitchen_microwave",
        "kitchen_refrigerator",
        "kitchen_trash_bin",
        "dining_pendant_light",
        "bedroom_partition_north",
        "bedroom_partition_west",
        "bedroom_partition_west_lower",
        "bedroom_bed",
        "bedroom_nightstand",
        "bedroom_wardrobe",
        "bedroom_desk",
        "bedroom_desk_chair",
        "elevator_bank",
        "hotel_start_zone",
    }.issubset(model_names)
    assert {"dining_chair_west", "dining_chair_east"}.isdisjoint(model_names)
    partition = world.find("./model[@name='living_kitchen_partition']")
    assert partition is not None
    assert tuple(float(value) for value in partition.findtext("pose").split()) \
        == pytest.approx((1.25, 4.325, 1.20, 0.0, 0.0, 0.0))
    partition_size = partition.findtext(
        "./link/collision[@name='partition_collision']/geometry/box/size"
    )
    assert tuple(float(value) for value in partition_size.split()) \
        == pytest.approx((0.18, 2.75, 2.40))
    kitchen_door_south = 1.65 + (0.18 / 2.0)
    kitchen_door_north = 4.325 - (2.75 / 2.0)
    assert kitchen_door_north - kitchen_door_south == pytest.approx(1.21)
    assert not root.findall(".//include")
    assert not root.findall(".//uri")


def test_living_room_and_bedroom_have_deliberate_clear_layouts():
    world = ET.parse(HOTEL_WORLD).getroot().find("./world")

    def model_pose(name):
        model = world.find(f"./model[@name='{name}']")
        assert model is not None
        return tuple(float(value) for value in model.findtext("pose").split())

    # The coffee table is centered between a west sofa, north sofa and south
    # armchair, and all three seats face inward toward the television area.
    assert model_pose("lobby_sofa_west") == pytest.approx(
        (-2.70, 3.40, 0.225, 0.0, 0.0, 1.570796326794897)
    )
    assert model_pose("lobby_sofa_north") == pytest.approx(
        (-1.15, 4.75, 0.225, 0.0, 0.0, 3.141592653589793)
    )
    assert model_pose("lobby_coffee_table") == pytest.approx(
        (-1.15, 3.40, 0.12, 0.0, 0.0, 0.0)
    )
    assert model_pose("living_armchair_south") == pytest.approx(
        (-1.15, 2.05, 0.235, 0.0, 0.0, 3.141592653589793)
    )

    upper_wall = model_pose("bedroom_partition_west")
    lower_wall = model_pose("bedroom_partition_west_lower")
    upper_south_edge = upper_wall[1] - (2.45 / 2.0)
    lower_north_edge = lower_wall[1] + (3.50 / 2.0)
    lower_south_edge = lower_wall[1] - (3.50 / 2.0)
    assert upper_south_edge - lower_north_edge == pytest.approx(1.40)
    # The existing route crosses x=1.65 at y=-1.5, centered in this doorway.
    assert lower_north_edge < -1.50 < upper_south_edge
    # The lower wall now meets the building's inner south-wall face directly.
    assert lower_south_edge == pytest.approx(-5.70)
    assert model_pose("bedroom_partition_north")[1] == pytest.approx(1.65)
    assert model_pose("bedroom_bed")[1] == pytest.approx(-1.30)


def test_launches_reference_the_hotel_and_local_package():
    simulation = SIMULATION_LAUNCH.read_text(encoding="utf-8")
    hotel = HOTEL_LAUNCH.read_text(encoding="utf-8")
    assert "openarm_skeleton_v1_2_description" in simulation
    assert "openarm_skeleton_v1_2_gazebo" in simulation
    assert "hotel_lobby_demo.sdf" in hotel
    assert '"x": "-5.0"' in hotel
    assert '"y": "-3.5"' in hotel
    assert "demo_indoor_route.py" in hotel
    assert "hotel_nav_gui.config" in hotel
    gui = HOTEL_GUI_CONFIG.read_text(encoding="utf-8")
    assert "<camera_pose>-10 -3.5 6 0 0.65 0</camera_pose>" in gui
    assert 'filename="WorldControl"' not in gui
    assert 'filename="EntityTree"' in gui


def test_transport_partition_is_unique_and_validated():
    module = _load(SIMULATION_LAUNCH, "simulation_launch")
    assert (
        module._resolve_transport_partition("", launch_pid=1234)
        == "openarm_sim_1234"
    )
    assert (
        module._resolve_transport_partition("hotel_trial-01")
        == "hotel_trial-01"
    )
    with pytest.raises(RuntimeError, match="transport_partition"):
        module._resolve_transport_partition("invalid partition")


def test_runtime_targets_jazzy_and_gazebo_harmonic():
    module = _load(SIMULATION_LAUNCH, "harmonic_simulation_launch")
    command = module._gazebo_command(
        "2", "false", "true", "/tmp/world.sdf"
    )
    assert command == [
        "gz",
        "sim",
        "-v",
        "2",
        "-r",
        "-s",
        "--headless-rendering",
        "/tmp/world.sdf",
    ]
    gui_command = module._gazebo_command(
        "2", "false", "false", "/tmp/world.sdf", "/tmp/gui.config"
    )
    assert gui_command[-3:] == [
        "--gui-config",
        "/tmp/gui.config",
        "/tmp/world.sdf",
    ]
    world = HOTEL_WORLD.read_text(encoding="utf-8")
    bridge = BRIDGE_CONFIG.read_text(encoding="utf-8")
    assert "gz-sim-sensors-system" in world
    assert "gz::sim::systems::Physics" in world
    assert "gz.msgs.LaserScan" in bridge
    assert "ros_topic_name: /scan" in bridge
    assert "ros_topic_name: /camera/color/image_raw" in bridge
    assert "ros_topic_name: /camera/depth/image_raw" in bridge
    assert "ros_topic_name: /camera/camera_info" in bridge
    assert bridge.count("gz_type_name: gz.msgs.Image") == 2
    assert "PointCloud2" not in bridge
    assert "PointCloudPacked" not in bridge
    assert "gz_type_name: gz.msgs.CameraInfo" in bridge
    assert "ignition" not in world + bridge
