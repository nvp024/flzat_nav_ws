"""Launch the hotel simulation with Nav2 SLAM or map localization."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


NAVIGATION_START_DELAY_SECONDS = 7.0
RVIZ_START_DELAY_SECONDS = 9.0


def _validate_mode(context):
    slam = LaunchConfiguration("slam").perform(context).strip().lower()
    if slam in {"1", "true", "yes", "on"}:
        return []
    map_path = LaunchConfiguration("map").perform(context).strip()
    if not map_path or not Path(map_path).is_file():
        raise RuntimeError(
            "Localization mode requires map:=/absolute/path/to/map.yaml"
        )
    return []


def generate_launch_description():
    navigation_share = Path(
        get_package_share_directory("openarm_skeleton_v1_2_navigation")
    )
    gazebo_share = Path(
        get_package_share_directory("openarm_skeleton_v1_2_gazebo")
    )
    nav2_share = Path(get_package_share_directory("nav2_bringup"))

    slam = LaunchConfiguration("slam")
    map_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    params_file = LaunchConfiguration("params_file")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_share / "launch" / "hotel_demo.launch.py")
        ),
        launch_arguments={
            "auto_run": "false",
            "headless": LaunchConfiguration("headless"),
            "use_sim_time": use_sim_time,
            "transport_partition": LaunchConfiguration(
                "transport_partition"
            ),
        }.items(),
    )
    slam_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(nav2_share / "launch" / "slam_launch.py")
        ),
        condition=IfCondition(slam),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": params_file,
            "use_respawn": "false",
        }.items(),
    )
    map_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(nav2_share / "launch" / "localization_launch.py")
        ),
        condition=UnlessCondition(slam),
        launch_arguments={
            "map": map_file,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": params_file,
            "use_composition": "false",
            "use_respawn": "false",
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(navigation_share / "launch" / "navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "params_file": params_file,
        }.items(),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="nav2_rviz",
        output="screen",
        arguments=[
            "-d",
            str(
                navigation_share
                / "config"
                / "nav2_openarm_view.rviz"
            ),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("slam", default_value="true"),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "transport_partition",
                default_value="",
                description=(
                    "Gazebo transport partition; empty creates a unique one."
                ),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(
                    navigation_share / "config" / "nav2_params.yaml"
                ),
            ),
            OpaqueFunction(function=_validate_mode),
            simulation,
            LogInfo(
                msg=(
                    "Loading the Gazebo robot visual before starting Nav2 "
                    f"at {NAVIGATION_START_DELAY_SECONDS:.0f}s and RViz "
                    f"at {RVIZ_START_DELAY_SECONDS:.0f}s."
                )
            ),
            TimerAction(
                period=NAVIGATION_START_DELAY_SECONDS,
                actions=[slam_localization, map_localization, navigation],
            ),
            TimerAction(period=RVIZ_START_DELAY_SECONDS, actions=[rviz]),
        ]
    )
