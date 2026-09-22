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

"""Publish planar odometry with twist derived from Isaac's world pose."""

import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


def _yaw(orientation):
    return math.atan2(
        2.0
        * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        ),
        1.0
        - 2.0
        * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        ),
    )


def _wrapped_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class IsaacOdometryCorrector(Node):
    """Replace IsaacComputeOdometry's non-planar twist with pose rates."""

    def __init__(self):
        super().__init__("openarm_isaac_odometry_corrector")
        self._previous = None
        self._publisher = self.create_publisher(Odometry, "/odom", 20)
        self.create_subscription(
            Odometry,
            "/isaac_odom_raw",
            self._correct,
            20,
        )

    def _correct(self, message):
        stamp = message.header.stamp
        seconds = stamp.sec + stamp.nanosec * 1.0e-9
        pose = message.pose.pose
        yaw = _yaw(pose.orientation)

        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0
        if self._previous is not None:
            previous_seconds, previous_x, previous_y, previous_yaw = (
                self._previous
            )
            elapsed = seconds - previous_seconds
            if elapsed > 1.0e-6:
                world_x = (pose.position.x - previous_x) / elapsed
                world_y = (pose.position.y - previous_y) / elapsed
                linear_x = math.cos(yaw) * world_x + math.sin(yaw) * world_y
                linear_y = -math.sin(yaw) * world_x + math.cos(yaw) * world_y
                angular_z = _wrapped_angle(yaw - previous_yaw) / elapsed

        self._previous = (
            seconds,
            pose.position.x,
            pose.position.y,
            yaw,
        )
        twist = message.twist.twist
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_z
        self._publisher.publish(message)


def main():
    rclpy.init()
    node = IsaacOdometryCorrector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
