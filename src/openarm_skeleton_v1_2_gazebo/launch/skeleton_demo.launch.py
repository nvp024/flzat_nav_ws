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

"""Launch the opt-in skeleton simulator and optionally run one demo."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(
        get_package_share_directory("openarm_skeleton_v1_2_gazebo")
    )
    auto_run = LaunchConfiguration("auto_run")
    headless = LaunchConfiguration("headless")
    demo_target = LaunchConfiguration("demo_target")
    demo_duration = LaunchConfiguration("demo_duration")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / "launch" / "simulation.launch.py")
        ),
        launch_arguments={
            "enable_skeleton_lift": "true",
            "headless": headless,
        }.items(),
    )
    demo = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="openarm_skeleton_v1_2_gazebo",
                executable="demo_skeleton_lift.py",
                name="automatic_skeleton_lift_demo",
                output="screen",
                condition=IfCondition(auto_run),
                arguments=[
                    "--target",
                    demo_target,
                    "--duration",
                    demo_duration,
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "auto_run",
                default_value="true",
                description="Automatically run the semantic lift demo.",
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo server without its GUI.",
            ),
            DeclareLaunchArgument(
                "demo_target",
                default_value="high",
                choices=("high", "low", "cycle"),
            ),
            DeclareLaunchArgument(
                "demo_duration",
                default_value="8.0",
                description="Seconds per smooth motion segment.",
            ),
            LogInfo(
                msg=(
                    "PROVISIONAL SIMULATION ONLY: skeleton limits and "
                    "controllers are not validated for hardware."
                )
            ),
            simulation,
            demo,
        ]
    )
