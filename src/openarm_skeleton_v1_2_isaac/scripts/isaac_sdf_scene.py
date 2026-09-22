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

"""Read the Gazebo hotel SDF into simulator-neutral primitive records."""

from dataclasses import dataclass
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET


GAZEBO_HOTEL_SPAWN = (-11.2, -3.5, 0.0)
DEFAULT_COLOR = (0.65, 0.65, 0.65)


@dataclass(frozen=True)
class ScenePrimitive:
    """One visual or collision primitive expressed in the SLAM start frame."""

    name: str
    role: str
    shape: str
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    color: tuple[float, float, float]
    scale: tuple[float, float, float] | None = None
    radius: float | None = None
    height: float | None = None


def _numbers(text, expected, label):
    values = tuple(float(value) for value in (text or "").split())
    if len(values) != expected:
        raise ValueError(
            f"{label} requires {expected} numeric values, got {values}"
        )
    return values


def _pose(element):
    pose = element.find("pose")
    values = (
        _numbers(pose.text, 6, "SDF pose")
        if pose is not None
        else (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    )
    return values[:3], _quaternion_from_rpy(*values[3:])


def _quaternion_from_rpy(roll, pitch, yaw):
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def _quaternion_multiply(left, right):
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )


def _rotate(quaternion, vector):
    conjugate = (
        quaternion[0],
        -quaternion[1],
        -quaternion[2],
        -quaternion[3],
    )
    pure = (0.0, vector[0], vector[1], vector[2])
    rotated = _quaternion_multiply(
        _quaternion_multiply(quaternion, pure), conjugate
    )
    return rotated[1:]


def _compose(parent, child):
    parent_position, parent_orientation = parent
    child_position, child_orientation = child
    rotated = _rotate(parent_orientation, child_position)
    return (
        tuple(
            parent_position[index] + rotated[index]
            for index in range(3)
        ),
        _quaternion_multiply(parent_orientation, child_orientation),
    )


def _color(element):
    material = element.find("material")
    if material is None:
        return DEFAULT_COLOR
    value = material.find("diffuse")
    if value is None:
        value = material.find("ambient")
    if value is None:
        return DEFAULT_COLOR
    rgba = _numbers(value.text, 4, "SDF material color")
    return rgba[:3]


def _safe_name(*parts):
    joined = "__".join(part for part in parts if part)
    return re.sub(r"[^A-Za-z0-9_]", "_", joined)


def _geometry_record(geometry, label):
    box = geometry.find("box")
    if box is not None:
        size = box.find("size")
        if size is None:
            raise ValueError(f"SDF box has no size: {label}")
        scale = _numbers(size.text, 3, f"SDF box size for {label}")
        if min(scale) <= 0.0:
            raise ValueError(f"SDF box size must be positive: {label}")
        return {"shape": "box", "scale": scale}

    cylinder = geometry.find("cylinder")
    if cylinder is not None:
        radius = cylinder.find("radius")
        length = cylinder.find("length")
        if radius is None or length is None:
            raise ValueError(f"SDF cylinder is incomplete: {label}")
        radius_value = float(radius.text)
        height_value = float(length.text)
        if radius_value <= 0.0 or height_value <= 0.0:
            raise ValueError(f"SDF cylinder dimensions must be positive: {label}")
        return {
            "shape": "cylinder",
            "radius": radius_value,
            "height": height_value,
        }

    sphere = geometry.find("sphere")
    if sphere is not None:
        radius = sphere.find("radius")
        if radius is None:
            raise ValueError(f"SDF sphere has no radius: {label}")
        radius_value = float(radius.text)
        if radius_value <= 0.0:
            raise ValueError(f"SDF sphere radius must be positive: {label}")
        return {"shape": "sphere", "radius": radius_value}

    plane = geometry.find("plane")
    if plane is not None:
        normal = plane.find("normal")
        size = plane.find("size")
        if normal is None or size is None:
            raise ValueError(f"SDF plane is incomplete: {label}")
        normal_value = _numbers(normal.text, 3, f"SDF plane normal for {label}")
        if normal_value != (0.0, 0.0, 1.0):
            raise ValueError(f"only horizontal SDF planes are supported: {label}")
        width, depth = _numbers(size.text, 2, f"SDF plane size for {label}")
        if width <= 0.0 or depth <= 0.0:
            raise ValueError(f"SDF plane size must be positive: {label}")
        return {"shape": "plane", "scale": (width, depth, 0.01)}

    child_tags = [child.tag for child in geometry]
    raise ValueError(
        f"unsupported SDF geometry {child_tags!r}: {label}"
    )


def load_hotel_primitives(sdf_path):
    """Load the current hotel SDF relative to its fixed Gazebo spawn pose."""

    path = Path(sdf_path).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"hotel SDF not found: {path}")
    root = ET.parse(path).getroot()
    world = root.find("world")
    if world is None:
        raise ValueError(f"SDF contains no world: {path}")

    # SLAM starts with the Gazebo robot at map (0, 0, 0). Isaac imports its
    # robot at world origin, so express every world primitive relative to the
    # same fixed Gazebo spawn instead of copying absolute Gazebo coordinates.
    spawn_x, spawn_y, spawn_z = GAZEBO_HOTEL_SPAWN
    map_origin = (
        (-spawn_x, -spawn_y, -spawn_z),
        (1.0, 0.0, 0.0, 0.0),
    )
    primitives = []
    for model in world.findall("model"):
        model_name = model.get("name", "model")
        model_pose = _compose(map_origin, _pose(model))
        for link in model.findall("link"):
            link_name = link.get("name", "link")
            link_pose = _compose(model_pose, _pose(link))
            for role in ("visual", "collision"):
                for element in link.findall(role):
                    element_name = element.get("name", role)
                    label = f"{model_name}/{link_name}/{element_name}"
                    geometry = element.find("geometry")
                    if geometry is None:
                        raise ValueError(f"SDF element has no geometry: {label}")
                    record = _geometry_record(geometry, label)
                    position, orientation = _compose(link_pose, _pose(element))
                    # Represent horizontal visual planes as a 1 cm slab below
                    # z=0. Collision planes are supplied by Isaac's default
                    # ground plane and are not instantiated a second time.
                    if record["shape"] == "plane" and role == "visual":
                        position = (position[0], position[1], position[2] - 0.005)
                    primitives.append(
                        ScenePrimitive(
                            name=_safe_name(
                                role, model_name, link_name, element_name
                            ),
                            role=role,
                            position=position,
                            orientation=orientation,
                            color=_color(element),
                            **record,
                        )
                    )
    if not primitives:
        raise ValueError(f"SDF contains no scene primitives: {path}")
    return tuple(primitives)
