"""Single public entry point for the complete OpenArm simulation stack."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    navigation_share = Path(
        get_package_share_directory("openarm_skeleton_v1_2_navigation")
    )

    complete_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(navigation_share / "launch" / "nav2_sim.launch.py")
        ),
        launch_arguments={
            "slam": LaunchConfiguration("slam"),
            "map": LaunchConfiguration("map"),
            "headless": LaunchConfiguration("headless"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart": LaunchConfiguration("autostart"),
            "params_file": LaunchConfiguration("params_file"),
            "transport_partition": LaunchConfiguration("transport_partition"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "slam",
                default_value="true",
                description=(
                    "Use SLAM Toolbox. Set false to localize in a saved map."
                ),
            ),
            DeclareLaunchArgument(
                "map",
                default_value="",
                description=(
                    "Absolute map YAML path required when slam:=false."
                ),
            ),
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo without its GUI.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz with the OpenArm Nav2 layout.",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(
                    navigation_share / "config" / "nav2_params.yaml"
                ),
                description="Absolute Nav2 and SLAM parameter file.",
            ),
            DeclareLaunchArgument(
                "transport_partition",
                default_value="",
                description=(
                    "Gazebo transport partition; empty creates a unique one."
                ),
            ),
            LogInfo(
                msg=(
                    "OpenArm all-in-one: hotel Gazebo world, robot, lidar, "
                    "SLAM/localization, Nav2, safety watchdog and RViz."
                )
            ),
            complete_stack,
        ]
    )
