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

"""Procedural scene definitions that are not shared with Gazebo."""


SCENE_NAMES = ("hotel", "restaurant")
ROBOT_SPAWN_CLEARANCE_METERS = 0.90

WALL_COLOR = (0.78, 0.73, 0.64)
RESTAURANT_WALL_COLOR = (0.72, 0.63, 0.52)
RESTAURANT_TABLE_COLOR = (0.42, 0.20, 0.08)
RESTAURANT_CHAIR_COLOR = (0.65, 0.16, 0.12)
RESTAURANT_COUNTER_COLOR = (0.18, 0.38, 0.24)


def _object(name, position, scale, color):
    return {
        "name": name,
        "position": position,
        "scale": scale,
        "color": color,
    }


def _outer_walls(color):
    return (
        _object("north_wall", (0.0, 4.5, 0.6), (10.0, 0.15, 1.2), color),
        _object("south_wall", (0.0, -4.5, 0.6), (10.0, 0.15, 1.2), color),
        _object("east_wall", (5.0, 0.0, 0.6), (0.15, 9.0, 1.2), color),
        _object("west_wall", (-5.0, 0.0, 0.6), (0.15, 9.0, 1.2), color),
    )


def _dining_set(prefix, x, y):
    """One square table and four chairs, with a clear 1.5 m aisle around it."""
    return (
        _object(
            f"{prefix}_table",
            (x, y, 0.38),
            (0.90, 0.90, 0.76),
            RESTAURANT_TABLE_COLOR,
        ),
        _object(
            f"{prefix}_chair_west",
            (x - 0.78, y, 0.33),
            (0.42, 0.42, 0.66),
            RESTAURANT_CHAIR_COLOR,
        ),
        _object(
            f"{prefix}_chair_east",
            (x + 0.78, y, 0.33),
            (0.42, 0.42, 0.66),
            RESTAURANT_CHAIR_COLOR,
        ),
        _object(
            f"{prefix}_chair_south",
            (x, y - 0.78, 0.33),
            (0.42, 0.42, 0.66),
            RESTAURANT_CHAIR_COLOR,
        ),
        _object(
            f"{prefix}_chair_north",
            (x, y + 0.78, 0.33),
            (0.42, 0.42, 0.66),
            RESTAURANT_CHAIR_COLOR,
        ),
    )


RESTAURANT_OBJECTS = _outer_walls(RESTAURANT_WALL_COLOR) + (
    _object(
        "kitchen_wall_west",
        (-3.35, 3.00, 0.60),
        (3.15, 0.15, 1.20),
        RESTAURANT_WALL_COLOR,
    ),
    _object(
        "kitchen_wall_east",
        (3.35, 3.00, 0.60),
        (3.15, 0.15, 1.20),
        RESTAURANT_WALL_COLOR,
    ),
    _object(
        "service_counter",
        (0.0, 3.72, 0.48),
        (3.00, 0.62, 0.96),
        RESTAURANT_COUNTER_COLOR,
    ),
    _object(
        "host_stand",
        (-4.15, -3.55, 0.46),
        (0.62, 0.62, 0.92),
        RESTAURANT_COUNTER_COLOR,
    ),
    *_dining_set("north_west", -2.45, 1.35),
    *_dining_set("north_east", 2.45, 1.35),
    *_dining_set("south_west", -2.45, -1.75),
    *_dining_set("south_east", 2.45, -1.75),
)


SCENE_OBJECTS = {"restaurant": RESTAURANT_OBJECTS}


def get_scene_objects(scene_name):
    """Return a procedural scene; hotel is loaded from the Gazebo SDF."""
    try:
        return SCENE_OBJECTS[scene_name]
    except KeyError as error:
        choices = ", ".join(SCENE_NAMES)
        raise ValueError(
            f"unknown Isaac scene {scene_name!r}; choose one of: {choices}"
        ) from error
