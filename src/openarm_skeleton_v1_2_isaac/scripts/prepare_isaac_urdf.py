#!/usr/bin/env python3

# Copyright 2026 Simulation Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Prepare the verified base-demo URDF for the Isaac Sim 5.0 importer."""

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


DESCRIPTION_PACKAGE = "openarm_skeleton_v1_2_description"
PACKAGE_MESH_PREFIX = f"package://{DESCRIPTION_PACKAGE}/meshes/"
EXPECTED_LINK_COUNT = 38
EXPECTED_JOINT_COUNT = 37
EXPECTED_MOVABLE_JOINTS = {
    "caster_joint_1",
    "caster_wheel_joint_1",
    "caster_joint_2",
    "caster_wheel_joint_2",
    "drive_joint_1",
    "drive_joint_2",
}
REQUIRED_FRAME_LINKS = (
    "base_footprint",
    "base_link",
    "base_scan",
    "camera_optical_frame",
)
MASSLESS_FRAME_INERTIA = "1e-12"
MASSLESS_FRAME_MASS = "1e-6"


def _add_tiny_inertial(link):
    """Give a frame-only link a negligible, valid inertia for import."""
    if link.find("inertial") is not None:
        return
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(inertial, "mass", value=MASSLESS_FRAME_MASS)
    ET.SubElement(
        inertial,
        "inertia",
        ixx=MASSLESS_FRAME_INERTIA,
        ixy="0",
        ixz="0",
        iyy=MASSLESS_FRAME_INERTIA,
        iyz="0",
        izz=MASSLESS_FRAME_INERTIA,
    )


def _validate_source(root):
    links = root.findall("./link")
    joints = root.findall("./joint")
    if root.tag != "robot" or root.get("name") != "openarm_skeleton_v1.2":
        raise ValueError("unexpected OpenArm robot description")
    if (
        len(links) != EXPECTED_LINK_COUNT
        or len(joints) != EXPECTED_JOINT_COUNT
    ):
        raise ValueError(
            "expected the verified "
            f"{EXPECTED_LINK_COUNT}-link/{EXPECTED_JOINT_COUNT}-joint "
            "base profile"
        )
    movable = {
        joint.get("name")
        for joint in joints
        if joint.get("type") != "fixed"
    }
    if movable != EXPECTED_MOVABLE_JOINTS:
        raise ValueError(f"unexpected movable joints: {sorted(movable)}")
    for name in REQUIRED_FRAME_LINKS:
        if root.find(f"./link[@name='{name}']") is None:
            raise ValueError(f"required frame is missing: {name}")


def prepare_urdf(source, mesh_directory, output):
    """Write a portable Isaac-import URDF and return its output path."""
    source = Path(source).expanduser().resolve()
    mesh_directory = Path(mesh_directory).expanduser().resolve()
    output = Path(output).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"source URDF not found: {source}")
    if not mesh_directory.is_dir():
        raise FileNotFoundError(f"mesh directory not found: {mesh_directory}")

    tree = ET.parse(source)
    root = tree.getroot()
    _validate_source(root)

    # Gazebo plugins and surface tags are simulator-specific. Isaac receives
    # equivalent ROS bridges and drive control from its OmniGraph instead.
    for gazebo in list(root.findall("./gazebo")):
        root.remove(gazebo)

    for mesh in root.findall(".//mesh"):
        uri = mesh.get("filename", "")
        if not uri.startswith(PACKAGE_MESH_PREFIX):
            raise ValueError(f"unexpected mesh URI: {uri}")
        relative = Path(uri.removeprefix(PACKAGE_MESH_PREFIX))
        if relative.is_absolute() or ".." in relative.parts:
            raise ValueError(f"unsafe mesh URI: {uri}")
        candidate = mesh_directory / relative
        if not candidate.is_file():
            raise FileNotFoundError(f"mesh asset not found: {candidate}")
        # colcon --symlink-install intentionally points installed mesh entries
        # back to the source tree. The package URI and rejection of absolute /
        # parent-relative paths above keep the lookup bounded, while resolving
        # the final symlink gives Isaac an absolute path it can import.
        resolved = candidate.resolve(strict=True)
        mesh.set("filename", str(resolved))

    # Isaac's URDF importer has changed its handling of massless frame-only
    # roots between releases. Tiny inertials keep 5.0 import deterministic;
    # fixed-joint merging makes their physical effect negligible.
    for name in REQUIRED_FRAME_LINKS:
        _add_tiny_inertial(root.find(f"./link[@name='{name}']"))

    ET.indent(tree, space="  ")
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return output


def _arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, help="Base-demo URDF")
    parser.add_argument("--meshes", required=True, help="Description mesh directory")
    parser.add_argument("--output", required=True, help="Prepared URDF output")
    return parser.parse_args()


def main():
    args = _arguments()
    output = prepare_urdf(args.source, args.meshes, args.output)
    print(f"Prepared Isaac URDF: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
