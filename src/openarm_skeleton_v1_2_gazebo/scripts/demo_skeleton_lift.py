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

"""Guarded semantic demo for the provisional five-joint skeleton lift."""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


SKELETON_JOINTS = tuple(
    f"skeleton_joint_{index}" for index in range(1, 6)
)
COMMAND_TOPICS = tuple(
    f"/skeleton_lift/joint_{index}/command" for index in range(1, 6)
)
POSES = {
    "low": (0.0, 0.0, 0.0, 0.0, 0.0),
    "high": (
        -math.pi / 6.0,
        math.pi / 3.0,
        -math.pi / 3.0,
        math.pi / 3.0,
        -math.pi / 6.0,
    ),
}
DEMO_LIMITS = (
    (-0.58, 0.05),
    (-0.05, 1.15),
    (-1.15, 0.05),
    (-0.05, 1.15),
    (-0.58, 0.05),
)
TOP_PLATFORM_SUM_TOLERANCE = 1.0e-6
START_STATE_SUM_TOLERANCE = 0.15


def smoothstep(phase):
    """Return a clamped cubic ease-in/ease-out interpolation factor."""
    phase = min(max(float(phase), 0.0), 1.0)
    return phase * phase * (3.0 - 2.0 * phase)


def interpolate_pose(start, target, phase):
    """Interpolate two five-joint poses with zero endpoint velocity."""
    if len(start) != len(SKELETON_JOINTS):
        raise ValueError("start pose must contain five joint positions")
    if len(target) != len(SKELETON_JOINTS):
        raise ValueError("target pose must contain five joint positions")
    factor = smoothstep(phase)
    return tuple(
        start_value + factor * (target_value - start_value)
        for start_value, target_value in zip(start, target)
    )


def validate_target_pose(pose):
    """Reject non-finite, out-of-contract, or tilted target profiles."""
    if len(pose) != len(SKELETON_JOINTS):
        raise ValueError("target pose must contain five joint positions")
    for index, (position, limits) in enumerate(
        zip(pose, DEMO_LIMITS), start=1
    ):
        lower, upper = limits
        if not math.isfinite(position):
            raise ValueError(f"skeleton joint {index} is not finite")
        if not lower <= position <= upper:
            raise ValueError(
                f"skeleton joint {index} target {position:.4f} rad "
                f"is outside provisional demo range [{lower}, {upper}]"
            )
    if abs(sum(pose)) > TOP_PLATFORM_SUM_TOLERANCE:
        raise ValueError(
            "target profile tilts the top platform; joint sum must be zero"
        )
    return tuple(float(position) for position in pose)


def estimated_camera_height(pose):
    """Estimate camera-link height from the official planar CAD chain."""
    if len(pose) != len(SKELETON_JOINTS):
        raise ValueError("pose must contain five joint positions")

    height = 0.12
    angle = -pose[0]
    for offset_z, next_joint in zip(
        (-0.47, 0.47, -0.47, 0.47), pose[1:]
    ):
        height += -math.sin(angle) * offset_z
        angle -= next_joint

    camera_offset_y = 0.175
    camera_offset_z = -0.046
    height += (
        math.cos(angle) * camera_offset_y
        - math.sin(angle) * camera_offset_z
    )
    return height


class SkeletonLiftDemo(Node):
    """Publish synchronized lift targets and verify simulator feedback."""

    def __init__(self):
        super().__init__("skeleton_lift_demo")
        self._publishers = [
            self.create_publisher(Float64, topic, 10)
            for topic in COMMAND_TOPICS
        ]
        self._positions = {}
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            20,
        )

    def _on_joint_state(self, message):
        for name, position in zip(message.name, message.position):
            if name in SKELETON_JOINTS and math.isfinite(position):
                self._positions[name] = float(position)

    def measured_pose(self):
        """Return ordered simulator feedback, or ``None`` until complete."""
        if not all(name in self._positions for name in SKELETON_JOINTS):
            return None
        return tuple(self._positions[name] for name in SKELETON_JOINTS)

    def publish_pose(self, pose):
        """Publish one synchronized five-joint command sample."""
        for publisher, position in zip(self._publishers, pose):
            message = Float64()
            message.data = float(position)
            publisher.publish(message)


def _spin_until(node, predicate, timeout):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def _check_start_pose(pose):
    for index, (position, limits) in enumerate(
        zip(pose, DEMO_LIMITS), start=1
    ):
        lower, upper = limits
        if not lower - 0.02 <= position <= upper + 0.02:
            raise RuntimeError(
                f"measured skeleton joint {index} is outside the "
                "provisional demo envelope"
            )
    if abs(sum(pose)) > START_STATE_SUM_TOLERANCE:
        raise RuntimeError(
            "measured skeleton pose tilts the top platform too far for "
            "this guarded demo"
        )


def _run_segment(node, start, target, duration, rate):
    period = 1.0 / rate
    started = time.monotonic()
    last_command = start
    while rclpy.ok():
        elapsed = time.monotonic() - started
        phase = min(elapsed / duration, 1.0)
        last_command = interpolate_pose(start, target, phase)
        node.publish_pose(last_command)
        rclpy.spin_once(node, timeout_sec=0.0)
        if phase >= 1.0:
            break
        time.sleep(period)
    return last_command


def _wait_for_target(node, target, timeout, tolerance, rate):
    period = 1.0 / rate
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        node.publish_pose(target)
        rclpy.spin_once(node, timeout_sec=min(period, 0.05))
        measured = node.measured_pose()
        if measured is not None:
            error = max(
                abs(actual - desired)
                for actual, desired in zip(measured, target)
            )
            if error <= tolerance:
                return measured, error
    measured = node.measured_pose()
    if measured is None:
        return None, math.inf
    error = max(
        abs(actual - desired)
        for actual, desired in zip(measured, target)
    )
    return measured, error


def _parse_arguments(argv):
    parser = argparse.ArgumentParser(
        description=(
            "Move the official OpenArm skeleton between guarded semantic "
            "LOW and HIGH simulation profiles."
        )
    )
    parser.add_argument(
        "--target",
        choices=("high", "low", "cycle"),
        default="high",
    )
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--dwell", type=float, default=2.0)
    parser.add_argument("--startup-timeout", type=float, default=15.0)
    parser.add_argument("--settle-timeout", type=float, default=5.0)
    parser.add_argument("--tolerance", type=float, default=0.04)
    parser.add_argument("--rate", type=float, default=30.0)
    args = parser.parse_args(remove_ros_args(args=argv)[1:])
    if args.duration <= 0.0:
        parser.error("--duration must be positive")
    if args.dwell < 0.0:
        parser.error("--dwell cannot be negative")
    if args.startup_timeout <= 0.0 or args.settle_timeout <= 0.0:
        parser.error("timeouts must be positive")
    if args.tolerance <= 0.0 or args.rate <= 0.0:
        parser.error("--tolerance and --rate must be positive")
    return args


def main(argv=None):
    argv = sys.argv if argv is None else argv
    args = _parse_arguments(argv)
    for pose in POSES.values():
        validate_target_pose(pose)

    rclpy.init(args=argv)
    node = SkeletonLiftDemo()
    last_command = None
    try:
        node.get_logger().info(
            "Waiting for all five skeleton joints on /joint_states..."
        )
        ready = _spin_until(
            node,
            lambda: node.measured_pose() is not None,
            args.startup_timeout,
        )
        if not ready:
            raise RuntimeError(
                "no complete skeleton feedback; launch "
                "skeleton_demo.launch.py first"
            )

        current = node.measured_pose()
        _check_start_pose(current)
        targets = (
            (POSES["high"], POSES["low"])
            if args.target == "cycle"
            else (POSES[args.target],)
        )
        for segment_index, target in enumerate(targets, start=1):
            target_name = (
                "HIGH" if target == POSES["high"] else "LOW"
            )
            node.get_logger().info(
                f"Moving to {target_name} over {args.duration:.1f}s; "
                f"estimated camera z={estimated_camera_height(target):.3f}m"
            )
            last_command = _run_segment(
                node,
                current,
                target,
                args.duration,
                args.rate,
            )
            measured, error = _wait_for_target(
                node,
                target,
                args.settle_timeout,
                args.tolerance,
                args.rate,
            )
            if measured is None or error > args.tolerance:
                raise RuntimeError(
                    f"{target_name} did not converge: max joint "
                    f"error={error:.4f}rad; verify that "
                    "enable_skeleton_lift:=true"
                )
            node.get_logger().info(
                f"Reached {target_name}: max error={error:.4f}rad, "
                "measured=["
                + ", ".join(f"{value:.3f}" for value in measured)
                + "]"
            )
            current = measured
            if segment_index < len(targets) and args.dwell > 0.0:
                dwell_deadline = time.monotonic() + args.dwell
                while rclpy.ok() and time.monotonic() < dwell_deadline:
                    node.publish_pose(target)
                    rclpy.spin_once(node, timeout_sec=0.05)

        node.get_logger().info("Skeleton lift demo PASS")
        return 0
    except (RuntimeError, ValueError) as error:
        node.get_logger().error(f"Skeleton lift demo FAIL: {error}")
        return 1
    finally:
        if last_command is not None and rclpy.ok():
            for _ in range(3):
                node.publish_pose(last_command)
                rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
