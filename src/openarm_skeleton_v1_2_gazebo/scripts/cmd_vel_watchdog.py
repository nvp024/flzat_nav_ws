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

"""Wall-time safety relay for Gazebo base velocity commands."""

import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


ZERO_COMMAND = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def reliable_qos(depth=10):
    """Return the explicit public command QoS."""
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def twist_values(message):
    """Extract all six finite Twist values or reject the command."""
    values = (
        float(message.linear.x),
        float(message.linear.y),
        float(message.linear.z),
        float(message.angular.x),
        float(message.angular.y),
        float(message.angular.z),
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("velocity command contains a non-finite value")
    return values


def twist_message(values):
    """Build a Twist from one six-axis tuple."""
    message = Twist()
    message.linear.x, message.linear.y, message.linear.z = values[:3]
    message.angular.x, message.angular.y, message.angular.z = values[3:]
    return message


class WallTimeCommandWatchdog:
    """Resolve the newest command to zero after a wall-time deadline."""

    def __init__(self, timeout):
        timeout = float(timeout)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("command_timeout must be finite and greater than zero")
        self.timeout = timeout
        self.command = ZERO_COMMAND
        self.received_at = -math.inf

    def update(self, values, now):
        values = tuple(float(value) for value in values)
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            raise ValueError("velocity command must contain six finite values")
        now = float(now)
        if not math.isfinite(now):
            raise ValueError("command timestamp must be finite")
        self.command = values
        self.received_at = now

    def output(self, now):
        now = float(now)
        if not math.isfinite(now):
            raise ValueError("watchdog timestamp must be finite")
        if now - self.received_at >= self.timeout:
            return ZERO_COMMAND
        return self.command


class CmdVelWatchdog(Node):
    """Relay `/cmd_vel` immediately and enforce a stale-command stop."""

    def __init__(self):
        super().__init__("cmd_vel_watchdog")
        self.declare_parameter("command_timeout", 0.50)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/cmd_vel_safe")

        timeout = float(self.get_parameter("command_timeout").value)
        publish_rate = float(self.get_parameter("publish_rate").value)
        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        if not math.isfinite(publish_rate) or publish_rate <= 0.0:
            raise ValueError("publish_rate must be finite and greater than zero")
        if not input_topic or not output_topic or input_topic == output_topic:
            raise ValueError("watchdog input/output topics must be distinct")

        self._watchdog = WallTimeCommandWatchdog(timeout)
        qos = reliable_qos()
        self._publisher = self.create_publisher(Twist, output_topic, qos)
        self._subscription = self.create_subscription(
            Twist, input_topic, self._command_callback, qos
        )
        self._timer = self.create_timer(
            1.0 / publish_rate, self._timer_callback
        )
        self.get_logger().info(
            f"Relaying {input_topic} to {output_topic}; stale commands stop "
            f"after {timeout:g}s wall time"
        )

    def _publish_current(self, now):
        self._publisher.publish(
            twist_message(self._watchdog.output(now))
        )

    def _command_callback(self, message):
        now = time.monotonic()
        try:
            self._watchdog.update(twist_values(message), now)
        except ValueError as error:
            self.get_logger().warning(f"Rejected /cmd_vel: {error}")
            self._watchdog.update(ZERO_COMMAND, now)
        # Forward immediately so the safety relay does not make teleoperation
        # feel quantized at the watchdog timer frequency.
        self._publish_current(now)

    def _timer_callback(self):
        self._publish_current(time.monotonic())


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = CmdVelWatchdog()
        rclpy.spin(node)
    except ValueError:
        exit_code = 2
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            try:
                node._publisher.publish(twist_message(ZERO_COMMAND))
                node.destroy_node()
            except (KeyboardInterrupt, RuntimeError):
                pass
        try:
            rclpy.try_shutdown()
        except RuntimeError:
            pass
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
