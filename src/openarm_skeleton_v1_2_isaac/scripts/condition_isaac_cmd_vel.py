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

"""Compensate for Isaac wheel static friction after the safety watchdog."""

import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


MIN_ANGULAR_SPEED = 0.25


class IsaacCommandConditioner(Node):
    """Preserve stops while lifting nonzero turns above breakaway speed."""

    def __init__(self):
        super().__init__("openarm_isaac_command_conditioner")
        self._publisher = self.create_publisher(Twist, "/isaac_cmd_vel", 20)
        self.create_subscription(
            Twist,
            "/cmd_vel_safe",
            self._condition,
            20,
        )

    def _condition(self, message):
        angular = message.angular.z
        if 0.0 < abs(angular) < MIN_ANGULAR_SPEED:
            message.angular.z = math.copysign(MIN_ANGULAR_SPEED, angular)
        self._publisher.publish(message)


def main():
    rclpy.init()
    node = IsaacCommandConditioner()
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
