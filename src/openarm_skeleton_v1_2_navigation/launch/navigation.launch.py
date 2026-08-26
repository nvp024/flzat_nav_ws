"""Launch the bounded Nav2 core while preserving the public cmd_vel relay."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


NAVIGATION_NODES = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
]


def generate_launch_description():
    package_share = Path(
        get_package_share_directory("openarm_skeleton_v1_2_navigation")
    )
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    common_parameters = [params_file, {"use_sim_time": use_sim_time}]

    smoothed_nav_to_pose_bt = str(
        package_share
        / "behavior_trees"
        / "navigate_to_pose_with_smoothing.xml"
    )

    def nav_node(
        package, executable, name, remappings=None, extra_parameters=None
    ):
        parameters = list(common_parameters)
        if extra_parameters:
            parameters.append(extra_parameters)
        return Node(
            package=package,
            executable=executable,
            name=name,
            output="screen",
            parameters=parameters,
            remappings=remappings or [],
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(package_share / "config" / "nav2_params.yaml"),
            ),
            nav_node(
                "nav2_controller",
                "controller_server",
                "controller_server",
                [("cmd_vel", "cmd_vel_nav")],
            ),
            nav_node(
                "nav2_smoother", "smoother_server", "smoother_server"
            ),
            nav_node("nav2_planner", "planner_server", "planner_server"),
            nav_node(
                "nav2_behaviors",
                "behavior_server",
                "behavior_server",
                [("cmd_vel", "cmd_vel_nav")],
            ),
            nav_node(
                "nav2_bt_navigator",
                "bt_navigator",
                "bt_navigator",
                extra_parameters={
                    "default_nav_to_pose_bt_xml": smoothed_nav_to_pose_bt,
                },
            ),
            nav_node(
                "nav2_waypoint_follower",
                "waypoint_follower",
                "waypoint_follower",
            ),
            nav_node(
                "nav2_velocity_smoother",
                "velocity_smoother",
                "velocity_smoother",
                [
                    ("cmd_vel", "cmd_vel_nav"),
                    ("cmd_vel_smoothed", "cmd_vel"),
                ],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[
                    {
                        "autostart": autostart,
                        "node_names": NAVIGATION_NODES,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
        ]
    )
