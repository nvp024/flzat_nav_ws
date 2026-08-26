"""Unit contracts for bounded route, base watchdog, and lift profile."""

import importlib.util
import math
from pathlib import Path

import pytest


PACKAGE = Path(__file__).resolve().parents[1]


def _load(filename, name):
    path = PACKAGE / "scripts" / filename
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_route_is_bounded_and_totals_nine_metres():
    route = _load("demo_indoor_route.py", "indoor_route")
    assert route.validate_route(route.ROUTE) == route.ROUTE
    assert route.route_linear_distance() == pytest.approx(9.0)
    assert route.LINEAR_LIMIT == 0.30
    assert route.ANGULAR_LIMIT == 0.70
    assert route.wrap_angle(3.0 * math.pi) == pytest.approx(-math.pi)
    with pytest.raises(ValueError):
        route.validate_route((("straight", -1.0),))


def test_watchdog_stops_stale_commands_using_wall_time():
    watchdog = _load("cmd_vel_watchdog.py", "cmd_vel_watchdog")
    guard = watchdog.WallTimeCommandWatchdog(0.5)
    command = (0.2, 0.0, 0.0, 0.0, 0.0, 0.1)
    guard.update(command, 10.0)
    assert guard.output(10.49) == command
    assert guard.output(10.50) == watchdog.ZERO_COMMAND


def test_semantic_lift_profiles_are_level_and_within_limits():
    lift = _load("demo_skeleton_lift.py", "skeleton_lift")
    low = lift.validate_target_pose(lift.POSES["low"])
    high = lift.validate_target_pose(lift.POSES["high"])
    assert sum(low) == pytest.approx(0.0)
    assert sum(high) == pytest.approx(0.0)
    assert lift.estimated_camera_height(low) == pytest.approx(0.295)
    assert lift.estimated_camera_height(high) == pytest.approx(1.235)

    invalid = list(high)
    invalid[0] = -0.7
    with pytest.raises(ValueError, match="outside provisional"):
        lift.validate_target_pose(invalid)
