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

"""Spawn the generated robot through ros_gz_bridge with bounded waits."""

from dataclasses import dataclass
import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.srv import SpawnEntity
from std_msgs.msg import String


@dataclass(frozen=True)
class SpawnSpec:
    """Values used to populate a Gazebo entity factory request."""

    name: str
    x: float
    y: float
    z: float
    yaw: float
    allow_renaming: bool = False


def quaternion_from_yaw(yaw):
    """Return an XYZW quaternion for a planar yaw angle."""
    half_yaw = 0.5 * yaw
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def make_spawn_request(spec, robot_description):
    """Build a typed SpawnEntity request from a robot description."""
    request = SpawnEntity.Request()
    factory = request.entity_factory
    factory.name = spec.name
    factory.allow_renaming = spec.allow_renaming
    factory.sdf = robot_description
    factory.pose.position.x = spec.x
    factory.pose.position.y = spec.y
    factory.pose.position.z = spec.z
    quaternion = quaternion_from_yaw(spec.yaw)
    factory.pose.orientation.x = quaternion[0]
    factory.pose.orientation.y = quaternion[1]
    factory.pose.orientation.z = quaternion[2]
    factory.pose.orientation.w = quaternion[3]
    factory.relative_to = "world"
    return request


class EntitySpawner(Node):
    """Wait for the description and Gazebo service, then spawn exactly once."""

    def __init__(self):
        super().__init__("spawn_openarm_skeleton_v1_2")
        self.declare_parameter("world_name", "openarm_official")
        self.declare_parameter("robot_name", "openarm_skeleton_v1_2")
        self.declare_parameter("robot_description_topic", "/robot_description")
        self.declare_parameter("allow_renaming", False)
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.01)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("startup_timeout", 20.0)
        self.declare_parameter("service_timeout", 20.0)
        self.declare_parameter("description_timeout", 20.0)
        self.declare_parameter("response_timeout", 20.0)

        world_name = self.get_parameter("world_name").value
        description_topic = self.get_parameter(
            "robot_description_topic"
        ).value
        self._description = None
        description_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._description_subscription = self.create_subscription(
            String,
            description_topic,
            self._on_description,
            description_qos,
        )
        self._service_name = f"/world/{world_name}/create"
        self._client = self.create_client(SpawnEntity, self._service_name)

    def _on_description(self, message):
        if message.data.strip():
            self._description = message.data

    def _wait_for_description(self, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and self._description is None:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))
        return self._description is not None

    def _wait_for_service(self, timeout):
        """Poll in short intervals so ROS shutdown cannot leave a traceback."""
        deadline = time.monotonic() + timeout
        while rclpy.ok():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            try:
                if self._client.wait_for_service(
                    timeout_sec=min(0.1, remaining)
                ):
                    return True
            except Exception:
                if not rclpy.ok():
                    return False
                raise
        return False

    @staticmethod
    def _remaining_timeout(deadline, phase_timeout):
        """Return this phase's share of the single startup time budget."""
        return max(
            0.0,
            min(phase_timeout, deadline - time.monotonic()),
        )

    def spawn(self):
        """Perform the bounded spawn transaction and return a process code."""
        startup_timeout = float(
            self.get_parameter("startup_timeout").value
        )
        service_timeout = float(
            self.get_parameter("service_timeout").value
        )
        description_timeout = float(
            self.get_parameter("description_timeout").value
        )
        response_timeout = float(
            self.get_parameter("response_timeout").value
        )
        for name, value in (
            ("startup_timeout", startup_timeout),
            ("service_timeout", service_timeout),
            ("description_timeout", description_timeout),
            ("response_timeout", response_timeout),
        ):
            if not math.isfinite(value) or value <= 0.0:
                self.get_logger().error(
                    f"{name} must be finite and greater than zero"
                )
                return 2

        spec = SpawnSpec(
            name=str(self.get_parameter("robot_name").value).strip(),
            allow_renaming=bool(
                self.get_parameter("allow_renaming").value
            ),
            x=float(self.get_parameter("x").value),
            y=float(self.get_parameter("y").value),
            z=float(self.get_parameter("z").value),
            yaw=float(self.get_parameter("yaw").value),
        )
        if not spec.name:
            self.get_logger().error("robot_name cannot be empty")
            return 2
        for name, value in (
            ("x", spec.x),
            ("y", spec.y),
            ("z", spec.z),
            ("yaw", spec.yaw),
        ):
            if not math.isfinite(value):
                self.get_logger().error(f"{name} must be finite")
                return 2

        # Every phase consumes the same monotonic budget.  This prevents the
        # nominal per-phase timeouts from accumulating into a multi-minute
        # launch failure.
        startup_deadline = time.monotonic() + startup_timeout
        service_budget = self._remaining_timeout(
            startup_deadline, service_timeout
        )

        self.get_logger().info(
            f"Waiting up to {service_budget:.1f}s for {self._service_name}"
        )
        if not self._wait_for_service(service_budget):
            if not rclpy.ok():
                return 130
            self.get_logger().error(
                f"Gazebo spawn service unavailable: {self._service_name}"
            )
            return 1

        description_budget = self._remaining_timeout(
            startup_deadline, description_timeout
        )
        self.get_logger().info(
            "Waiting up to "
            f"{description_budget:.1f}s for the robot description"
        )
        if not self._wait_for_description(description_budget):
            if not rclpy.ok():
                return 130
            self.get_logger().error("Robot description was not received")
            return 1

        request = make_spawn_request(spec, self._description)
        self.get_logger().info(
            f"Requesting entity '{spec.name}' in {self._service_name}"
        )
        future = self._client.call_async(request)
        response_budget = self._remaining_timeout(
            startup_deadline, response_timeout
        )
        deadline = time.monotonic() + response_budget
        while rclpy.ok() and not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                future.cancel()
                self.get_logger().error(
                    "Gazebo did not answer the spawn request within "
                    f"{response_budget:.1f}s"
                )
                return 1
            rclpy.spin_once(self, timeout_sec=min(0.1, remaining))

        if not future.done():
            return 130
        try:
            response = future.result()
        except Exception as error:  # noqa: B902 - ROS future exceptions vary
            self.get_logger().error(f"Spawn service call failed: {error}")
            return 1
        if not response.success:
            self.get_logger().error(
                f"Gazebo rejected entity '{spec.name}'"
            )
            return 1
        self.get_logger().info(f"Spawned entity '{spec.name}' successfully")
        return 0


def main():
    """Run the one-shot spawner."""
    node = None
    try:
        rclpy.init()
        node = EntitySpawner()
        return node.spawn()
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    except Exception:
        if not rclpy.ok():
            return 130
        raise
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.try_shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
