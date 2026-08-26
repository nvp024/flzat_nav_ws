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

"""Launch the hotel lobby, official robot, and verified lobby route."""

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
    world = package_share / "worlds" / "hotel_lobby_demo.sdf"
    auto_run = LaunchConfiguration("auto_run")
    headless = LaunchConfiguration("headless")
    route_scale = LaunchConfiguration("route_scale")
    linear_speed = LaunchConfiguration("linear_speed")
    use_sim_time = LaunchConfiguration("use_sim_time")
    transport_partition = LaunchConfiguration("transport_partition")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(package_share / "launch" / "simulation.launch.py")
        ),
        launch_arguments={
            "world": str(world),
            "world_name": "hotel_lobby",
            "robot_name": "openarm_skeleton_hotel",
            "enable_skeleton_lift": "false",
            "headless": headless,
            "use_sim_time": use_sim_time,
            "transport_partition": transport_partition,
            "gui_config": str(
                package_share / "config" / "hotel_nav_gui.config"
            ),
            "x": "-5.0",
            "y": "-3.5",
            "z": "0.0",
            "yaw": "0.0",
        }.items(),
    )
    route_demo = TimerAction(
        period=7.0,
        actions=[
            Node(
                package="openarm_skeleton_v1_2_gazebo",
                executable="demo_indoor_route.py",
                name="automatic_hotel_lobby_route_demo",
                output="screen",
                condition=IfCondition(auto_run),
                arguments=[
                    "--route-scale",
                    route_scale,
                    "--linear-speed",
                    linear_speed,
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "auto_run",
                default_value="true",
                description=(
                    "Automatically drive the hotel lobby route."
                ),
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo server without its GUI.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "transport_partition",
                default_value="",
                description=(
                    "Gazebo transport partition; empty creates a unique one."
                ),
            ),
            DeclareLaunchArgument(
                "route_scale",
                default_value="1.0",
                description=(
                    "Scale straight segments in [0.25, 1.0]."
                ),
            ),
            DeclareLaunchArgument(
                "linear_speed",
                default_value="0.25",
                description="Route speed in m/s, maximum 0.30.",
            ),
            LogInfo(
                msg=(
                    "Starting hotel lobby + official robot + bounded "
                    "automatic route. Skeleton and arms remain locked."
                )
            ),
            simulation,
            route_demo,
        ]
    )
