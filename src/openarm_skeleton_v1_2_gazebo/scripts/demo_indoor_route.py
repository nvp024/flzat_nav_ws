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

"""Drive the official robot through a bounded indoor inspection route."""

import argparse
import math
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


ROUTE = (
    ("straight", 4.0),
    ("turn", math.pi / 2.0),
    ("straight", 2.0),
    ("turn", -math.pi / 2.0),
    ("straight", 3.0),
)
LINEAR_LIMIT = 0.30
ANGULAR_LIMIT = 0.70
DISTANCE_TOLERANCE = 0.035
ANGLE_TOLERANCE = math.radians(2.0)


def clamp(value, lower, upper):
    """Clamp a finite scalar to an inclusive interval."""
    return min(max(value, lower), upper)


def wrap_angle(angle):
    """Normalize an angle to the half-open interval [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_yaw(x, y, z, w):
    """Extract planar yaw from a normalized or near-normalized quaternion."""
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def route_linear_distance(route=ROUTE):
    """Return the sum of all straight segment lengths."""
    return sum(value for kind, value in route if kind == "straight")


def validate_route(route):
    """Reject malformed, non-finite, or unbounded route definitions."""
    if not route:
        raise ValueError("route cannot be empty")
    for kind, value in route:
        if kind not in {"straight", "turn"}:
            raise ValueError(f"unsupported route segment: {kind!r}")
        if not math.isfinite(value) or value == 0.0:
            raise ValueError("route segment values must be finite and nonzero")
        if kind == "straight" and value < 0.0:
            raise ValueError("this bounded demo does not drive in reverse")
        if kind == "turn" and abs(value) > math.pi:
            raise ValueError("one turn segment cannot exceed pi radians")
    return tuple(route)


class IndoorRouteDemo(Node):
    """Close the loop on odometry while publishing guarded base velocity."""

    def __init__(self):
        super().__init__("official_robot_indoor_route_demo")
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 20)
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.pose = None

    def _on_odom(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self.pose = (
            float(position.x),
            float(position.y),
            quaternion_to_yaw(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            ),
        )

    def publish_velocity(self, linear_x=0.0, angular_z=0.0):
        """Publish one bounded base command sample."""
        message = Twist()
        message.linear.x = clamp(
            float(linear_x), -LINEAR_LIMIT, LINEAR_LIMIT
        )
        message.angular.z = clamp(
            float(angular_z), -ANGULAR_LIMIT, ANGULAR_LIMIT
        )
        self.publisher.publish(message)


def _spin_until(node, predicate, timeout):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def _hold_stop(node, duration=0.35):
    deadline = time.monotonic() + duration
    while rclpy.ok() and time.monotonic() < deadline:
        node.publish_velocity()
        rclpy.spin_once(node, timeout_sec=0.05)


def _drive_straight(node, distance, speed):
    start_x, start_y, target_heading = node.pose
    timeout = distance / speed * 2.2 + 6.0
    deadline = time.monotonic() + timeout
    progress = 0.0

    while rclpy.ok() and time.monotonic() < deadline:
        current_x, current_y, current_heading = node.pose
        progress = math.hypot(
            current_x - start_x,
            current_y - start_y,
        )
        remaining = distance - progress
        if remaining <= DISTANCE_TOLERANCE:
            _hold_stop(node)
            return progress

        heading_error = wrap_angle(target_heading - current_heading)
        commanded_speed = min(speed, max(0.08, 0.9 * remaining))
        node.publish_velocity(
            linear_x=commanded_speed,
            angular_z=clamp(
                1.8 * heading_error,
                -0.35,
                0.35,
            ),
        )
        rclpy.spin_once(node, timeout_sec=0.05)

    _hold_stop(node)
    raise RuntimeError(
        f"straight segment timed out at {progress:.3f}/{distance:.3f}m"
    )


def _turn(node, angle, angular_speed):
    target_heading = wrap_angle(node.pose[2] + angle)
    timeout = abs(angle) / angular_speed * 3.0 + 5.0
    deadline = time.monotonic() + timeout
    error = angle

    while rclpy.ok() and time.monotonic() < deadline:
        error = wrap_angle(target_heading - node.pose[2])
        if abs(error) <= ANGLE_TOLERANCE:
            _hold_stop(node)
            return error

        command = clamp(
            1.7 * error,
            -angular_speed,
            angular_speed,
        )
        if abs(command) < 0.10:
            command = math.copysign(0.10, error)
        node.publish_velocity(angular_z=command)
        rclpy.spin_once(node, timeout_sec=0.05)

    _hold_stop(node)
    raise RuntimeError(
        f"turn segment timed out with {math.degrees(error):.2f}deg error"
    )


def _parse_arguments(argv):
    parser = argparse.ArgumentParser(
        description=(
            "Run the official OpenArm mobile base through the indoor "
            "inspection route and verify every segment with odometry."
        )
    )
    parser.add_argument("--linear-speed", type=float, default=0.25)
    parser.add_argument(
        "--angular-speed",
        type=float,
        default=0.55,
    )
    parser.add_argument("--route-scale", type=float, default=1.0)
    parser.add_argument("--startup-timeout", type=float, default=20.0)
    args = parser.parse_args(remove_ros_args(args=argv)[1:])

    if (
        not math.isfinite(args.linear_speed)
        or not 0.05 <= args.linear_speed <= LINEAR_LIMIT
    ):
        parser.error(
            f"--linear-speed must be in [0.05, {LINEAR_LIMIT}]"
        )
    if (
        not math.isfinite(args.angular_speed)
        or not 0.10 <= args.angular_speed <= ANGULAR_LIMIT
    ):
        parser.error(
            f"--angular-speed must be in [0.10, {ANGULAR_LIMIT}]"
        )
    if (
        not math.isfinite(args.route_scale)
        or not 0.25 <= args.route_scale <= 1.0
    ):
        parser.error("--route-scale must be in [0.25, 1.0]")
    if (
        not math.isfinite(args.startup_timeout)
        or args.startup_timeout <= 0.0
    ):
        parser.error("--startup-timeout must be positive")
    return args


def main(argv=None):
    """Run the bounded route and return nonzero on missing or stale motion."""
    argv = sys.argv if argv is None else argv
    args = _parse_arguments(argv)
    route = tuple(
        (
            kind,
            value * args.route_scale if kind == "straight" else value,
        )
        for kind, value in validate_route(ROUTE)
    )

    rclpy.init(args=argv)
    node = IndoorRouteDemo()
    try:
        node.get_logger().info(
            "Waiting for /odom before starting indoor route..."
        )
        if not _spin_until(
            node,
            lambda: node.pose is not None,
            args.startup_timeout,
        ):
            raise RuntimeError(
                "no /odom; launch an indoor map demo first"
            )

        start_pose = node.pose
        total_target = route_linear_distance(route)
        node.get_logger().info(
            f"Starting {len(route)} route segments; "
            f"target linear distance={total_target:.2f}m"
        )

        measured_linear = 0.0
        for index, (kind, value) in enumerate(route, start=1):
            if kind == "straight":
                node.get_logger().info(
                    f"Segment {index}/{len(route)}: "
                    f"straight {value:.2f}m"
                )
                progress = _drive_straight(
                    node,
                    value,
                    args.linear_speed,
                )
                measured_linear += progress
                node.get_logger().info(
                    f"Segment {index} PASS: measured {progress:.3f}m"
                )
            else:
                node.get_logger().info(
                    f"Segment {index}/{len(route)}: "
                    f"turn {math.degrees(value):+.1f}deg"
                )
                error = _turn(node, value, args.angular_speed)
                node.get_logger().info(
                    f"Segment {index} PASS: final heading error "
                    f"{math.degrees(error):.2f}deg"
                )

        end_pose = node.pose
        displacement = math.hypot(
            end_pose[0] - start_pose[0],
            end_pose[1] - start_pose[1],
        )
        node.get_logger().info(
            "INDOOR ROUTE PASS: "
            f"measured_linear={measured_linear:.3f}m, "
            f"net_displacement={displacement:.3f}m, "
            f"final_pose=({end_pose[0]:.3f}, {end_pose[1]:.3f}, "
            f"{math.degrees(end_pose[2]):.1f}deg)"
        )
        return 0
    except RuntimeError as error:
        node.get_logger().error(f"INDOOR ROUTE FAIL: {error}")
        return 1
    finally:
        if rclpy.ok():
            _hold_stop(node, duration=0.25)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
