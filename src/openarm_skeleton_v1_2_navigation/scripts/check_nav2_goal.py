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

"""Send a bounded Nav2 goal and verify motion and cross-track error."""

import argparse
import math
import sys
import time

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args


class Nav2GoalCheck(Node):
    """Observe odometry while acting as a NavigateToPose client."""

    def __init__(self):
        super().__init__("check_openarm_nav2_goal")
        self.odom = None
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def _on_odom(self, message):
        self.odom = message


def _spin_until(node, predicate, timeout):
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


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


def _arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal-distance", type=float, default=0.60)
    parser.add_argument("--min-motion", type=float, default=0.25)
    parser.add_argument("--max-cross-track", type=float, default=0.05)
    parser.add_argument("--startup-timeout", type=float, default=20.0)
    parser.add_argument("--goal-timeout", type=float, default=90.0)
    return parser.parse_args(remove_ros_args(sys.argv)[1:])


def main():
    args = _arguments()
    values = (
        args.goal_distance,
        args.min_motion,
        args.max_cross_track,
        args.startup_timeout,
        args.goal_timeout,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in values):
        print("FAIL: distances and timeouts must be finite and positive")
        return 2
    if args.min_motion >= args.goal_distance:
        print("FAIL: --min-motion must be smaller than --goal-distance")
        return 2

    rclpy.init()
    node = Nav2GoalCheck()
    try:
        ready = _spin_until(
            node,
            lambda: node.odom is not None and node.client.server_is_ready(),
            args.startup_timeout,
        )
        if not ready:
            print(
                "FAIL: /odom and /navigate_to_pose were not ready within "
                f"{args.startup_timeout:g}s"
            )
            return 1

        start = node.odom.pose.pose
        heading = _yaw(start.orientation)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "odom"
        goal.pose.pose.position.x = (
            start.position.x + args.goal_distance * math.cos(heading)
        )
        goal.pose.pose.position.y = (
            start.position.y + args.goal_distance * math.sin(heading)
        )
        goal.pose.pose.orientation = start.orientation

        send_future = node.client.send_goal_async(goal)
        if not _spin_until(node, send_future.done, args.startup_timeout):
            print("FAIL: timed out while sending the Nav2 goal")
            return 1
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print("FAIL: Nav2 rejected the goal")
            return 1

        result_future = goal_handle.get_result_async()
        if not _spin_until(node, result_future.done, args.goal_timeout):
            cancel_future = goal_handle.cancel_goal_async()
            _spin_until(node, cancel_future.done, 5.0)
            print(f"FAIL: Nav2 goal exceeded {args.goal_timeout:g}s")
            return 1

        result = result_future.result()
        end = node.odom.pose.pose.position
        delta_x = end.x - start.position.x
        delta_y = end.y - start.position.y
        motion = math.hypot(
            delta_x,
            delta_y,
        )
        along_track = delta_x * math.cos(heading) + delta_y * math.sin(
            heading
        )
        cross_track = -delta_x * math.sin(heading) + delta_y * math.cos(
            heading
        )
        passed = (
            result.status == GoalStatus.STATUS_SUCCEEDED
            and motion >= args.min_motion
            and abs(cross_track) <= args.max_cross_track
        )
        label = "PASS" if passed else "FAIL"
        print(
            f"{label}: action_status={result.status}, "
            f"goal_distance={args.goal_distance:.3f} m, "
            f"odom_motion={motion:.3f} m, "
            f"along_track={along_track:.3f} m, "
            f"cross_track={cross_track:.3f} m, "
            f"max_cross_track={args.max_cross_track:.3f} m"
        )
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
