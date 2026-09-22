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

"""Verify the Isaac Sim topics and frame chain required by SLAM/Nav2."""

import argparse
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class IsaacRuntimeCheck(Node):
    """Wait for all simulator outputs and both required TF segments."""

    def __init__(self, timeout):
        super().__init__("check_openarm_isaac_runtime")
        self.deadline = time.monotonic() + timeout
        self.received = set()
        self.failure = ""
        self.done = False
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(
            Clock, "/clock", lambda _msg: self.received.add("/clock"), 10
        )
        self.create_subscription(
            LaserScan,
            "/scan",
            self._scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry, "/odom", self._odom, qos_profile_sensor_data
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            lambda _msg: self.received.add("/joint_states"),
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(0.2, self._check)

    def _scan(self, message):
        if message.header.frame_id != "base_scan":
            self.failure = (
                "expected /scan frame_id base_scan, got "
                f"{message.header.frame_id!r}"
            )
        self.received.add("/scan")

    def _odom(self, message):
        if message.header.frame_id != "odom":
            self.failure = (
                "expected /odom frame_id odom, got "
                f"{message.header.frame_id!r}"
            )
        if message.child_frame_id != "base_footprint":
            self.failure = (
                "expected /odom child_frame_id base_footprint, got "
                f"{message.child_frame_id!r}"
            )
        self.received.add("/odom")

    def _has_transform(self, target, source):
        try:
            self.buffer.lookup_transform(
                target, source, rclpy.time.Time()
            )
            return True
        except TransformException:
            return False

    def _check(self):
        if self.failure:
            self.done = True
            return
        required = {"/clock", "/scan", "/odom", "/joint_states"}
        topics_ready = required <= self.received
        frames_ready = self._has_transform(
            "odom", "base_footprint"
        ) and self._has_transform("base_link", "base_scan")
        if topics_ready and frames_ready:
            self.done = True
            return
        if time.monotonic() >= self.deadline:
            missing = sorted(required - self.received)
            odom_base_ready = self._has_transform(
                "odom", "base_footprint"
            )
            base_scan_ready = self._has_transform(
                "base_link", "base_scan"
            )
            self.failure = (
                f"timeout; missing topics={missing}, "
                f"odom->base_footprint={odom_base_ready}, "
                f"base_link->base_scan={base_scan_ready}"
            )
            self.done = True


def _arguments(arguments=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=45.0)
    parsed, ros_arguments = parser.parse_known_args(arguments)
    if parsed.timeout <= 0.0:
        parser.error("--timeout must be greater than zero")
    return parsed, ros_arguments


def main(arguments=None):
    args, ros_arguments = _arguments(arguments)
    rclpy.init(args=ros_arguments)
    node = IsaacRuntimeCheck(args.timeout)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.failure = node.failure or "interrupted"
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if node.failure:
        print(f"FAIL: Isaac runtime contract: {node.failure}")
        return 1
    print("PASS: Isaac topics and TF contract are ready for SLAM/Nav2")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
