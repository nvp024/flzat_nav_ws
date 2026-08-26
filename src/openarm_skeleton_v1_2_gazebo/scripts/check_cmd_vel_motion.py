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

"""Bounded acceptance check for odometry and both official drive wheels."""

import argparse
import math
import sys
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState


DRIVE_JOINTS = ("drive_joint_1", "drive_joint_2")


def _qos(depth=20):
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


class MotionObserver(Node):
    """Publish a base command while retaining the latest motion evidence."""

    def __init__(self):
        super().__init__("check_official_cmd_vel_motion")
        self.odom = None
        self.joints = {}
        self.publisher = self.create_publisher(Twist, "/cmd_vel", _qos())
        self.create_subscription(
            Odometry, "/odom", self._on_odom, _qos()
        )
        self.create_subscription(
            JointState, "/joint_states", self._on_joint_state, _qos()
        )

    def _on_odom(self, message):
        self.odom = message

    def _on_joint_state(self, message):
        self.joints.update(zip(message.name, message.position))

    def ready(self):
        return self.odom is not None and all(
            name in self.joints for name in DRIVE_JOINTS
        )

    def publish_velocity(self, linear_x):
        message = Twist()
        message.linear.x = linear_x
        self.publisher.publish(message)


def _wait_until(node, predicate, timeout):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--linear-x", type=float, default=0.15)
    parser.add_argument("--startup-timeout", type=float, default=15.0)
    parser.add_argument("--min-distance", type=float, default=0.03)
    parser.add_argument("--min-wheel-motion", type=float, default=0.20)
    return parser.parse_args(remove_ros_args(sys.argv)[1:])


def main():
    args = _arguments()
    positive_values = {
        "duration": args.duration,
        "startup_timeout": args.startup_timeout,
        "min_distance": args.min_distance,
        "min_wheel_motion": args.min_wheel_motion,
    }
    if (
        not all(
            math.isfinite(value) and value > 0.0
            for value in positive_values.values()
        )
        or not math.isfinite(args.linear_x)
        or args.linear_x == 0.0
    ):
        print("FAIL: all thresholds must be finite and positive", file=sys.stderr)
        return 2

    rclpy.init()
    node = MotionObserver()
    try:
        if not _wait_until(node, node.ready, args.startup_timeout):
            print(
                "FAIL: no synchronized /odom and two-wheel /joint_states "
                f"within {args.startup_timeout:g}s",
                file=sys.stderr,
            )
            return 1

        # Let contact settling finish before taking the baseline.
        settle_deadline = time.monotonic() + 0.75
        while rclpy.ok() and time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        start_xy = (
            node.odom.pose.pose.position.x,
            node.odom.pose.pose.position.y,
        )
        start_joints = {
            name: node.joints[name] for name in DRIVE_JOINTS
        }

        command_deadline = time.monotonic() + args.duration
        while rclpy.ok() and time.monotonic() < command_deadline:
            node.publish_velocity(args.linear_x)
            rclpy.spin_once(node, timeout_sec=0.05)
        for _index in range(5):
            node.publish_velocity(0.0)
            rclpy.spin_once(node, timeout_sec=0.05)

        end_xy = (
            node.odom.pose.pose.position.x,
            node.odom.pose.pose.position.y,
        )
        distance = math.hypot(
            end_xy[0] - start_xy[0], end_xy[1] - start_xy[1]
        )
        deltas = {
            name: abs(node.joints[name] - start_joints[name])
            for name in DRIVE_JOINTS
        }
        passed = (
            distance >= args.min_distance
            and all(
                delta >= args.min_wheel_motion
                for delta in deltas.values()
            )
        )
        result = "PASS" if passed else "FAIL"
        print(
            f"{result}: /cmd_vel linear.x={args.linear_x:.3f} m/s, "
            f"odom_distance={distance:.4f} m, "
            f"delta_x={end_xy[0] - start_xy[0]:.4f} m, "
            f"delta_y={end_xy[1] - start_xy[1]:.4f} m, "
            f"drive_joint_1={deltas['drive_joint_1']:.4f} rad, "
            f"drive_joint_2={deltas['drive_joint_2']:.4f} rad"
        )
        return 0 if passed else 1
    finally:
        try:
            for _index in range(3):
                node.publish_velocity(0.0)
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
