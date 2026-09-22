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
        "entry_corridor_floor",
        "entry_corridor_north_wall",
        "entry_corridor_south_wall",
        "entry_corridor_end_wall",
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
        "bedroom_entry_fan",
        "entry_suitcase",
        "entry_umbrella_stand",
        "elevator_bank",
        "hotel_start_zone",
    }.issubset(model_names)
    assert {"dining_chair_west", "dining_chair_east"}.isdisjoint(model_names)
    partition = world.find("./model[@name='living_kitchen_partition']")
    assert partition is not None
    assert tuple(float(value) for value in partition.findtext("pose").split()) \
        == pytest.approx((1.65, 3.60, 1.20, 0.0, 0.0, 0.0))
    partition_size = partition.findtext(
        "./link/collision[@name='partition_collision']/geometry/box/size"
    )
    assert tuple(float(value) for value in partition_size.split()) \
        == pytest.approx((0.18, 4.20, 2.40))
    kitchen_door_south = 0.00 + (0.18 / 2.0)
    kitchen_door_north = 3.60 - (4.20 / 2.0)
    assert kitchen_door_north - kitchen_door_south == pytest.approx(1.41)
    assert not root.findall(".//include")
    assert not root.findall(".//uri")


def test_entry_corridor_has_a_wide_straight_route_from_spawn():
    world = ET.parse(HOTEL_WORLD).getroot().find("./world")

    def model(name):
        result = world.find(f"./model[@name='{name}']")
        assert result is not None
        return result

    def model_pose(name):
        return tuple(float(value) for value in model(name).findtext("pose").split())

    def wall_size(name):
        value = model(name).findtext("./link/collision/geometry/box/size")
        return tuple(float(component) for component in value.split())

    corridor_floor = model("entry_corridor_floor")
    floor_size = corridor_floor.findtext(
        "./link/collision/geometry/plane/size"
    )
    assert model_pose("entry_corridor_floor") == pytest.approx(
        (-9.925, -3.50, 0.0, 0.0, 0.0, 0.0)
    )
    assert tuple(float(value) for value in floor_size.split()) == pytest.approx(
        (4.45, 2.50)
    )
    assert model_pose("hotel_start_zone")[:2] == pytest.approx((-11.2, -3.5))

    upper_wall = model_pose("west_wall")
    lower_wall = model_pose("west_wall_lower")
    upper_edge = upper_wall[1] - (wall_size("west_wall")[1] / 2.0)
    lower_edge = lower_wall[1] + (wall_size("west_wall_lower")[1] / 2.0)
    assert upper_edge - lower_edge == pytest.approx(2.20)

    north_inner_edge = model_pose("entry_corridor_north_wall")[1] - 0.15
    south_inner_edge = model_pose("entry_corridor_south_wall")[1] + 0.15
    assert north_inner_edge - south_inner_edge == pytest.approx(2.20)
    assert model_pose("entry_corridor_end_wall")[:2] == pytest.approx(
        (-12.15, -3.50)
    )


def test_living_room_and_bedroom_have_deliberate_clear_layouts():
    world = ET.parse(HOTEL_WORLD).getroot().find("./world")

    def model_pose(name):
        model = world.find(f"./model[@name='{name}']")
        assert model is not None
        return tuple(float(value) for value in model.findtext("pose").split())

    # The coffee table is centered between a west sofa, north sofa and south
    # armchair, and all three seats face inward toward the television area.
    assert model_pose("lobby_sofa_west") == pytest.approx(
        (-3.40, 3.25, 0.225, 0.0, 0.0, 1.570796326794897)
    )
    assert model_pose("lobby_sofa_north") == pytest.approx(
        (-1.35, 4.65, 0.225, 0.0, 0.0, 3.141592653589793)
    )
    assert model_pose("lobby_coffee_table") == pytest.approx(
        (-1.35, 3.15, 0.12, 0.0, 0.0, 0.0)
    )
    assert model_pose("living_armchair_south") == pytest.approx(
        (-1.35, 1.65, 0.235, 0.0, 0.0, 3.141592653589793)
    )
    # Collision-to-collision gaps around the central table are deliberately
    # wide enough to avoid the previous narrow traps between furniture.
    north_gap = (4.65 - 0.25) - (3.15 + 0.325)
    south_gap = (3.15 - 0.325) - (1.65 + 0.275)
    west_gap = (-1.35 - 0.325) - (-3.40 + 0.25)
    assert north_gap == pytest.approx(0.925)
    assert south_gap == pytest.approx(0.90)
    assert west_gap == pytest.approx(1.475)
    sofa_side_table_gap = (3.25 - (1.40 / 2.0)) - (1.40 + 0.20)
    fan_chest_gap = (-4.90 - (0.65 / 2.0)) - (-6.35 + (0.35 / 2.0))
    assert sofa_side_table_gap == pytest.approx(0.95)
    assert fan_chest_gap == pytest.approx(0.95)
    tv_east_edge = model_pose("living_tv_unit")[0] + (0.45 / 2.0)
    partition_west_edge = model_pose("living_kitchen_partition")[0] - (
        0.18 / 2.0
    )
    assert partition_west_edge - tv_east_edge == pytest.approx(0.985)

    kitchen_counter = model_pose("kitchen_counter_set")
    kitchen_microwave = model_pose("kitchen_microwave")
    kitchen_refrigerator = model_pose("kitchen_refrigerator")
    assert kitchen_counter[:2] == pytest.approx((3.80, 5.46))
    assert kitchen_microwave[:2] == pytest.approx((4.80, 5.44))
    assert kitchen_refrigerator[:2] == pytest.approx((6.55, 5.42))
    # The counter top and refrigerator meet the north inner wall at y=5.70.
    assert kitchen_counter[1] - 0.01 + (0.50 / 2.0) == pytest.approx(5.70)
    assert kitchen_refrigerator[1] + (0.56 / 2.0) == pytest.approx(5.70)

    upper_wall = model_pose("bedroom_partition_west")
    lower_wall = model_pose("bedroom_partition_west_lower")
    upper_south_edge = upper_wall[1] - (0.80 / 2.0)
    lower_north_edge = lower_wall[1] + (3.50 / 2.0)
    lower_south_edge = lower_wall[1] - (3.50 / 2.0)
    assert upper_south_edge - lower_north_edge == pytest.approx(1.40)
    # The existing route crosses x=1.65 at y=-1.5, centered in this doorway.
    assert lower_north_edge < -1.50 < upper_south_edge
    # The lower wall now meets the building's inner south-wall face directly.
    assert lower_south_edge == pytest.approx(-5.70)
    assert model_pose("bedroom_partition_north")[1] == pytest.approx(0.00)
    # Both rooms use the same east/west bounds and equal depth about y=0.
    assert model_pose("living_kitchen_partition")[0] == pytest.approx(
        upper_wall[0]
    )
    bed = model_pose("bedroom_bed")
    assert bed[:2] == pytest.approx((6.60, -0.69))
    # The bed is flush with both inner wall faces in the north-east corner.
    assert 7.70 - (bed[0] + (2.20 / 2.0)) == pytest.approx(0.0)
    assert -0.09 - (bed[1] + (1.20 / 2.0)) == pytest.approx(0.0)
    wardrobe = model_pose("bedroom_wardrobe")
    nightstand = model_pose("bedroom_nightstand")
    assert wardrobe[:2] == pytest.approx((3.00, -0.34))
    assert -0.09 - (wardrobe[1] + (0.50 / 2.0)) == pytest.approx(0.0)
    assert nightstand[:2] == pytest.approx((7.49, -1.65))
    assert 7.70 - (nightstand[0] + (0.42 / 2.0)) == pytest.approx(0.0)

    desk_books = world.find("./model[@name='bedroom_desk_books']")
    assert desk_books is not None
    book_visuals = desk_books.findall("./link/visual")
    assert len(book_visuals) == 5
    assert model_pose("entry_suitcase")[:2] == pytest.approx((-4.40, -5.38))
    assert model_pose("entry_umbrella_stand")[:2] == pytest.approx((-4.20, -1.80))
    assert model_pose("bedroom_entry_fan")[:2] == pytest.approx((0.35, -2.75))


def test_launches_reference_the_hotel_and_local_package():
    simulation = SIMULATION_LAUNCH.read_text(encoding="utf-8")
    hotel = HOTEL_LAUNCH.read_text(encoding="utf-8")
    assert "openarm_skeleton_v1_2_description" in simulation
    assert "openarm_skeleton_v1_2_gazebo" in simulation
    assert "hotel_lobby_demo.sdf" in hotel
    assert '"x": "-11.2"' in hotel
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
