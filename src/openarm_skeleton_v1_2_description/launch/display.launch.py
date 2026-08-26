"""Display the full official OpenArm Skeleton v1.2 model in RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(
        get_package_share_directory("openarm_skeleton_v1_2_description")
    )
    urdf = package_share / "urdf" / "openarm_skeleton_v1_2.urdf"
    rviz_config = package_share / "rviz" / "openarm_skeleton_v1_2.rviz"

    robot_description = urdf.read_text(encoding="utf-8")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                arguments=["-d", str(rviz_config)],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
        ]
    )
