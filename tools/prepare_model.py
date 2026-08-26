#!/usr/bin/env python3
"""Prepare reproducible ROS 2 and Gazebo URDFs from the SolidWorks export."""

from __future__ import annotations

import argparse
import copy
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


ROBOT_NAME = "openarm_skeleton_v1.2"
DESCRIPTION_PACKAGE = "openarm_skeleton_v1_2_description"
ORIGINAL_PACKAGE_URI = "package://openarm_skeleton_v1.2/meshes/"
NORMALIZED_PACKAGE_URI = f"package://{DESCRIPTION_PACKAGE}/meshes/"

ACTIVE_BASE_JOINTS = {
    "caster_joint_1",
    "caster_wheel_joint_1",
    "caster_joint_2",
    "caster_wheel_joint_2",
    "drive_joint_1",
    "drive_joint_2",
}
SKELETON_JOINTS = {
    f"skeleton_joint_{index}" for index in range(1, 6)
}
SKELETON_DEMO_LIMITS = {
    "skeleton_joint_1": (-0.58, 0.05),
    "skeleton_joint_2": (-0.05, 1.15),
    "skeleton_joint_3": (-1.15, 0.05),
    "skeleton_joint_4": (-0.05, 1.15),
    "skeleton_joint_5": (-0.58, 0.05),
}
DIFF_DRIVE_JOINT_AXES = {
    "drive_joint_1": "1 0 0",
    "drive_joint_2": "1 0 0",
}
SIM_LIDAR_MAX_RANGE = "20.0"
SIM_RGBD_TOPIC = "/camera/rgbd"
SIM_RGBD_WIDTH = "640"
SIM_RGBD_HEIGHT = "480"
SIM_RGBD_UPDATE_RATE = "10"
SIM_RGBD_NEAR_CLIP = "0.20"
SIM_RGBD_FAR_CLIP = "10.0"


def _element(tag: str, **attributes: str) -> ET.Element:
    return ET.Element(tag, {name: str(value) for name, value in attributes.items()})


def _rename_and_reframe(root: ET.Element) -> None:
    """Add REP-103 frames while preserving the SolidWorks CAD frame."""
    root.set("name", ROBOT_NAME)

    original_base = root.find("./link[@name='base_link']")
    if original_base is None:
        raise RuntimeError("The source URDF has no base_link")
    original_base.set("name", "cad_base_link")

    for parent in root.findall(".//joint/parent[@link='base_link']"):
        parent.set("link", "cad_base_link")

    base_footprint = _element("link", name="base_footprint")
    base_link = _element("link", name="base_link")

    footprint_joint = _element(
        "joint", name="base_footprint_joint", type="fixed"
    )
    footprint_joint.append(
        _element("origin", xyz="0 0 0.0402", rpy="0 0 0")
    )
    footprint_joint.append(_element("parent", link="base_footprint"))
    footprint_joint.append(_element("child", link="base_link"))

    cad_frame_joint = _element("joint", name="cad_frame_joint", type="fixed")
    cad_frame_joint.append(
        _element(
            "origin",
            xyz="0 0 0",
            rpy="0 0 -1.5707963267948966",
        )
    )
    cad_frame_joint.append(_element("parent", link="base_link"))
    cad_frame_joint.append(_element("child", link="cad_base_link"))

    # Keep the CAD lidar link for the visual model, but put the simulated
    # 360-degree navigation scanner above the base. The physical CAD mounting
    # point is low and front-mounted, so an unfiltered Gazebo ray sensor there
    # sees the robot itself and marks its footprint as occupied. Hardware
    # bring-up should use the measured sensor transform and driver filtering.
    base_scan = _element("link", name="base_scan")
    scan_joint = _element("joint", name="base_scan_joint", type="fixed")
    scan_joint.append(
        _element(
            "origin",
            xyz="0 0 0.34",
            rpy="0 0 0",
        )
    )
    scan_joint.append(_element("parent", link="base_link"))
    scan_joint.append(_element("child", link="base_scan"))

    # Keep a REP-103 optical frame below the camera CAD link. The SolidWorks
    # camera component uses local +Z as its physical forward axis, local +X
    # as robot-left and local +Y as robot-up at the zero lift pose. Therefore
    # the optical axes are -X right, -Y down and +Z forward in camera_link.
    if root.find("./link[@name='camera_link']") is None:
        raise RuntimeError("The source URDF has no camera_link")
    camera_optical_frame = _element("link", name="camera_optical_frame")
    camera_optical_joint = _element(
        "joint", name="camera_optical_joint", type="fixed"
    )
    camera_optical_joint.append(
        _element(
            "origin",
            xyz="0 0 0",
            rpy="0 0 3.141592653589793",
        )
    )
    camera_optical_joint.append(_element("parent", link="camera_link"))
    camera_optical_joint.append(
        _element("child", link="camera_optical_frame")
    )

    root.insert(0, camera_optical_joint)
    root.insert(0, camera_optical_frame)
    root.insert(0, scan_joint)
    root.insert(0, base_scan)
    root.insert(0, cad_frame_joint)
    root.insert(0, footprint_joint)
    root.insert(0, base_link)
    root.insert(0, base_footprint)


def _normalize_mesh_uris(root: ET.Element) -> None:
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename", "")
        if filename.startswith(ORIGINAL_PACKAGE_URI):
            mesh.set(
                "filename",
                filename.replace(
                    ORIGINAL_PACKAGE_URI, NORMALIZED_PACKAGE_URI, 1
                ),
            )


def _remove_children(element: ET.Element, tags: set[str]) -> None:
    for child in list(element):
        if child.tag in tags:
            element.remove(child)


def _lock_upper_body(
    root: ET.Element, active_upper_joints: set[str] | None = None
) -> None:
    active_upper_joints = active_upper_joints or set()
    for joint in root.findall("./joint"):
        name = joint.get("name", "")
        if (
            joint.get("type") == "fixed"
            or name in ACTIVE_BASE_JOINTS
            or name in active_upper_joints
        ):
            continue
        joint.set("type", "fixed")
        _remove_children(
            joint,
            {
                "axis",
                "limit",
                "dynamics",
                "calibration",
                "safety_controller",
                "mimic",
            },
        )


def _add_skeleton_joint_dynamics(root: ET.Element) -> None:
    """Apply conservative simulation-only limits to the five lift joints."""
    joints = {
        joint.get("name", ""): joint
        for joint in root.findall("./joint")
    }
    missing = SKELETON_JOINTS.difference(joints)
    if missing:
        raise RuntimeError(
            f"Missing skeleton joints: {sorted(missing)}"
        )

    for name in sorted(SKELETON_JOINTS):
        joint = joints[name]
        if joint.get("type") != "continuous":
            raise RuntimeError(
                f"Expected {name} to be continuous in the CAD export"
            )
        lower, upper = SKELETON_DEMO_LIMITS[name]
        joint.set("type", "revolute")
        _remove_children(joint, {"limit", "dynamics"})
        joint.append(
            _element(
                "limit",
                lower=f"{lower:g}",
                upper=f"{upper:g}",
                effort="150",
                velocity="0.5",
            )
        )
        joint.append(
            _element("dynamics", damping="0.5", friction="0.05")
        )


def _add_base_joint_dynamics(root: ET.Element) -> None:
    for joint in root.findall("./joint"):
        name = joint.get("name", "")
        if name not in ACTIVE_BASE_JOINTS:
            continue

        _remove_children(joint, {"limit", "dynamics"})
        if name.startswith("drive_joint"):
            joint.append(_element("limit", effort="30", velocity="30"))
            joint.append(_element("dynamics", damping="0.05", friction="0.01"))
        elif name.startswith("caster_wheel_joint"):
            joint.append(_element("limit", effort="5", velocity="40"))
            joint.append(_element("dynamics", damping="0.02", friction="0.005"))
        else:
            joint.append(_element("limit", effort="5", velocity="20"))
            joint.append(_element("dynamics", damping="0.10", friction="0.01"))


def _normalize_diff_drive_joint_axes(root: ET.Element) -> None:
    """Give both Gazebo drive wheels the same positive rotation direction.

    The mirrored SolidWorks export uses opposite axis signs for its two drive
    joints. Gazebo DiffDrive already commands opposite wheel velocities for a
    turn, so retaining those mirrored signs swaps the physical straight and
    angular responses while odometry still reports the requested motion.
    """
    joints = {
        joint.get("name", ""): joint
        for joint in root.findall("./joint")
    }
    missing = DIFF_DRIVE_JOINT_AXES.keys() - joints.keys()
    if missing:
        raise RuntimeError(
            f"Missing differential-drive joints: {sorted(missing)}"
        )

    for name, axis_xyz in DIFF_DRIVE_JOINT_AXES.items():
        axis = joints[name].find("axis")
        if axis is None:
            raise RuntimeError(
                f"Differential-drive joint has no axis: {name}"
            )
        axis.set("xyz", axis_xyz)


def _replace_collision_with_box(
    link: ET.Element, *, xyz: str, size: str
) -> None:
    _remove_children(link, {"collision"})
    collision = _element("collision", name=f"{link.get('name')}_collision")
    collision.append(_element("origin", xyz=xyz, rpy="0 0 0"))
    geometry = _element("geometry")
    geometry.append(_element("box", size=size))
    collision.append(geometry)
    link.append(collision)


def _replace_collision_with_cylinder(
    link: ET.Element, *, radius: str, length: str
) -> None:
    _remove_children(link, {"collision"})
    collision = _element("collision", name=f"{link.get('name')}_collision")
    collision.append(
        _element("origin", xyz="0 0 0", rpy="0 1.5707963267948966 0")
    )
    geometry = _element("geometry")
    geometry.append(_element("cylinder", radius=radius, length=length))
    collision.append(geometry)
    link.append(collision)


def _replace_collision_with_sphere(
    link: ET.Element, *, xyz: str, radius: str
) -> None:
    _remove_children(link, {"collision"})
    collision = _element("collision", name=f"{link.get('name')}_collision")
    collision.append(_element("origin", xyz=xyz, rpy="0 0 0"))
    geometry = _element("geometry")
    geometry.append(_element("sphere", radius=radius))
    collision.append(geometry)
    link.append(collision)


def _simplify_base_demo_collisions(root: ET.Element) -> None:
    """Use primitive collisions for stable and fast mobile-base simulation."""
    for link in root.findall("./link"):
        name = link.get("name", "")
        if name == "cad_base_link":
            _replace_collision_with_box(
                link,
                xyz="0 -0.000422686 0.155",
                size="0.50 0.50 0.25",
            )
        elif name in {"drive_link_1", "drive_link_2"}:
            _replace_collision_with_cylinder(
                link, radius="0.03810", length="0.02560"
            )
        elif name in {"caster_link_1", "caster_link_2"}:
            _replace_collision_with_sphere(
                link, xyz="-0.00075 -0.0115 -0.001", radius="0.019"
            )
        elif name in {"caster_wheel_link_1", "caster_wheel_link_2"}:
            _replace_collision_with_cylinder(
                link, radius="0.01929", length="0.02300"
            )
        else:
            _remove_children(link, {"collision"})


def _append_text_child(parent: ET.Element, tag: str, text: str) -> None:
    child = _element(tag)
    child.text = text
    parent.append(child)


def _add_gazebo_extensions(
    root: ET.Element, *, enable_skeleton_lift: bool = False
) -> None:
    gazebo = _element("gazebo")
    _append_text_child(gazebo, "self_collide", "false")

    diff_drive = _element(
        "plugin",
        filename="gz-sim-diff-drive-system",
        name="gz::sim::systems::DiffDrive",
    )
    for tag, value in (
        ("left_joint", "drive_joint_1"),
        ("right_joint", "drive_joint_2"),
        ("wheel_separation", "0.45"),
        ("wheel_radius", "0.03810"),
        ("topic", "/cmd_vel"),
        ("odom_topic", "/odom"),
        ("tf_topic", "/tf"),
        ("frame_id", "odom"),
        ("child_frame_id", "base_footprint"),
        ("odom_publish_frequency", "50"),
        ("min_velocity", "-0.60"),
        ("max_velocity", "0.60"),
        ("min_acceleration", "-1.50"),
        ("max_acceleration", "1.50"),
    ):
        _append_text_child(diff_drive, tag, value)
    gazebo.append(diff_drive)

    joint_state = _element(
        "plugin",
        filename="gz-sim-joint-state-publisher-system",
        name="gz::sim::systems::JointStatePublisher",
    )
    _append_text_child(joint_state, "topic", "/joint_states")
    gazebo.append(joint_state)

    if enable_skeleton_lift:
        for index in range(1, 6):
            controller = _element(
                "plugin",
                filename="gz-sim-joint-position-controller-system",
                name=(
                    "gz::sim::systems::"
                    "JointPositionController"
                ),
            )
            for tag, value in (
                ("joint_name", f"skeleton_joint_{index}"),
                ("topic", f"/skeleton_lift/joint_{index}/command"),
                ("p_gain", "2.0"),
                ("i_gain", "0.0"),
                ("d_gain", "0.1"),
                ("cmd_max", "0.5"),
                ("cmd_min", "-0.5"),
                ("use_velocity_commands", "true"),
                ("initial_position", "0.0"),
            ):
                _append_text_child(controller, tag, value)
            gazebo.append(controller)
    root.append(gazebo)

    scan_reference = _element("gazebo", reference="base_scan")
    scan_sensor = _element(
        "sensor", name="openarm_lidar", type="gpu_lidar"
    )
    for tag, value in (
        ("always_on", "true"),
        ("visualize", "false"),
        ("update_rate", "10"),
        ("topic", "/scan"),
        ("gz_frame_id", "base_scan"),
    ):
        _append_text_child(scan_sensor, tag, value)
    lidar = _element("lidar")
    scan = _element("scan")
    horizontal = _element("horizontal")
    for tag, value in (
        ("samples", "720"),
        ("resolution", "1"),
        ("min_angle", "-3.141592653589793"),
        ("max_angle", "3.141592653589793"),
    ):
        _append_text_child(horizontal, tag, value)
    scan.append(horizontal)
    lidar.append(scan)
    scan_range = _element("range")
    for tag, value in (
        # The base is about 0.6 m wide. This simulation-only dead zone removes
        # returns from the robot's own rendered structure while retaining a
        # full 360-degree field of view for Nav2.
        ("min", "0.35"),
        # Cover the complete hotel lobby. Shorter ranges leave large wedges
        # unknown at the spawn pose, which makes NavFn prefer curved paths
        # through already-observed cells even when the direct route is clear.
        ("max", SIM_LIDAR_MAX_RANGE),
        ("resolution", "0.01"),
    ):
        _append_text_child(scan_range, tag, value)
    lidar.append(scan_range)
    noise = _element("noise")
    for tag, value in (
        ("type", "gaussian"),
        ("mean", "0.0"),
        ("stddev", "0.005"),
    ):
        _append_text_child(noise, tag, value)
    lidar.append(noise)
    scan_sensor.append(lidar)
    scan_reference.append(scan_sensor)
    root.append(scan_reference)

    camera_reference = _element("gazebo", reference="camera_link")
    camera_sensor = _element(
        "sensor", name="openarm_rgbd_camera", type="rgbd_camera"
    )
    for tag, value in (
        ("always_on", "true"),
        ("visualize", "false"),
        ("update_rate", SIM_RGBD_UPDATE_RATE),
        ("topic", SIM_RGBD_TOPIC),
        ("gz_frame_id", "camera_optical_frame"),
    ):
        _append_text_child(camera_sensor, tag, value)
    # Reframe the Gazebo camera convention (+X forward, +Y left, +Z up)
    # onto the SolidWorks camera component (+Z forward, +X left, +Y up).
    _append_text_child(
        camera_sensor,
        "pose",
        "0 0 0 -1.5707963267948966 -1.5707963267948966 0",
    )
    camera = _element("camera", name="openarm_rgbd_camera")
    _append_text_child(camera, "horizontal_fov", "1.0471975512")
    image = _element("image")
    for tag, value in (
        ("width", SIM_RGBD_WIDTH),
        ("height", SIM_RGBD_HEIGHT),
        ("format", "R8G8B8"),
    ):
        _append_text_child(image, tag, value)
    camera.append(image)
    clip = _element("clip")
    for tag, value in (
        ("near", SIM_RGBD_NEAR_CLIP),
        ("far", SIM_RGBD_FAR_CLIP),
    ):
        _append_text_child(clip, tag, value)
    camera.append(clip)
    # Keep the depth output explicit. Gazebo can infer it for an RGB-D sensor,
    # but declaring it avoids renderer/version-dependent defaults and keeps the
    # depth range aligned with the RGB camera clip plane.
    depth_camera = _element("depth_camera")
    _append_text_child(depth_camera, "output", "depths")
    depth_clip = _element("clip")
    for tag, value in (
        ("near", SIM_RGBD_NEAR_CLIP),
        ("far", SIM_RGBD_FAR_CLIP),
    ):
        _append_text_child(depth_clip, tag, value)
    depth_camera.append(depth_clip)
    camera.append(depth_camera)
    _append_text_child(camera, "optical_frame_id", "camera_optical_frame")
    camera_sensor.append(camera)
    camera_reference.append(camera_sensor)
    root.append(camera_reference)

    # Keep the sensor-bearing CAD camera link as a Gazebo entity. The massless
    # ROS optical frame may be lumped because it is published separately by
    # robot_state_publisher and is used only as the image header frame.
    joint_reference = _element("gazebo", reference="camera_joint")
    _append_text_child(joint_reference, "preserveFixedJoint", "true")
    root.append(joint_reference)

    for name, mu1, mu2 in (
        ("drive_link_1", "1.2", "1.0"),
        ("drive_link_2", "1.2", "1.0"),
        ("caster_wheel_link_1", "0.7", "0.5"),
        ("caster_wheel_link_2", "0.7", "0.5"),
        ("caster_link_1", "0.3", "0.3"),
        ("caster_link_2", "0.3", "0.3"),
    ):
        reference = _element("gazebo", reference=name)
        _append_text_child(reference, "mu1", mu1)
        _append_text_child(reference, "mu2", mu2)
        root.append(reference)


def _read_source(source: Path) -> ET.Element:
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    root = ET.parse(source, parser=parser).getroot()
    _normalize_mesh_uris(root)
    _rename_and_reframe(root)
    return root


def _write(root: ET.Element, output: Path) -> None:
    ET.indent(root, space="  ")
    data = ET.tostring(root, encoding="unicode")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        '<?xml version="1.0" encoding="utf-8"?>\n' + data + "\n",
        encoding="utf-8",
    )


def _validate(root: ET.Element, expected_movable: set[str] | None = None) -> None:
    links = root.findall("./link")
    joints = root.findall("./joint")
    if len(links) != 38 or len(joints) != 37:
        raise RuntimeError(
            f"Unexpected model size: {len(links)} links, {len(joints)} joints"
        )

    movable = {
        joint.get("name", "")
        for joint in joints
        if joint.get("type") != "fixed"
    }
    if expected_movable is not None and movable != expected_movable:
        raise RuntimeError(
            f"Unexpected movable joints: {sorted(movable)}"
        )

    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename", "")
        if not filename.startswith(NORMALIZED_PACKAGE_URI):
            raise RuntimeError(f"Unnormalized mesh URI: {filename}")


def prepare(
    source: Path,
    description_output: Path,
    gazebo_output: Path,
    lift_output: Path,
) -> None:
    description = _read_source(source)
    _validate(description)
    _write(description, description_output)

    gazebo = copy.deepcopy(description)
    _lock_upper_body(gazebo)
    _normalize_diff_drive_joint_axes(gazebo)
    _add_base_joint_dynamics(gazebo)
    _simplify_base_demo_collisions(gazebo)
    _add_gazebo_extensions(gazebo)
    _validate(gazebo, ACTIVE_BASE_JOINTS)
    _write(gazebo, gazebo_output)

    lift = copy.deepcopy(description)
    _lock_upper_body(lift, SKELETON_JOINTS)
    _normalize_diff_drive_joint_axes(lift)
    _add_base_joint_dynamics(lift)
    _add_skeleton_joint_dynamics(lift)
    _simplify_base_demo_collisions(lift)
    _add_gazebo_extensions(lift, enable_skeleton_lift=True)
    _validate(lift, ACTIVE_BASE_JOINTS | SKELETON_JOINTS)
    _write(lift, lift_output)


def _parse_args() -> argparse.Namespace:
    workspace = Path(__file__).resolve().parents[1]
    package = (
        workspace / "src" / "openarm_skeleton_v1_2_description"
    )
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source",
        type=Path,
        default=(
            package
            / "urdf"
            / "reference"
            / "openarm_skeleton_v1.2.original.urdf"
        ),
    )
    parser.add_argument(
        "--description-output",
        type=Path,
        default=package / "urdf" / "openarm_skeleton_v1_2.urdf",
    )
    parser.add_argument(
        "--gazebo-output",
        type=Path,
        default=package / "urdf" / "openarm_skeleton_v1_2_base_demo.urdf",
    )
    parser.add_argument(
        "--lift-output",
        type=Path,
        default=package / "urdf" / "openarm_skeleton_v1_2_lift_demo.urdf",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    try:
        prepare(
            args.source,
            args.description_output,
            args.gazebo_output,
            args.lift_output,
        )
    except (OSError, ET.ParseError, RuntimeError) as error:
        print(f"prepare_model: {error}", file=sys.stderr)
        return 1
    print(f"Prepared {args.description_output}")
    print(f"Prepared {args.gazebo_output}")
    print(f"Prepared {args.lift_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
