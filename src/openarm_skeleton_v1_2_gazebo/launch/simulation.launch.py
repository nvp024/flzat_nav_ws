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

"""Spawn the OpenArm Skeleton in Gazebo Harmonic with a safe ``/cmd_vel``."""

from pathlib import Path
import os
import re

from ament_index_python.packages import (
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


STARTUP_DEADLINE_SECONDS = 28.0


def _is_true(value):
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _gazebo_command(verbosity, paused, headless, world, gui_config=""):
    """Build a shell-free Gazebo Harmonic command."""
    command = ["gz", "sim", "-v", verbosity]
    if not _is_true(paused):
        command.append("-r")
    if _is_true(headless):
        command.extend(["-s", "--headless-rendering"])
    elif gui_config:
        command.extend(["--gui-config", gui_config])
    command.append(world)
    return command


def _resolve_transport_partition(requested, launch_pid=None):
    """Return a unique Gazebo Transport partition."""
    requested = requested.strip()
    if requested:
        partition = requested
    else:
        pid = os.getpid() if launch_pid is None else int(launch_pid)
        partition = f"openarm_sim_{pid}"
    if (
        len(partition) > 128
        or re.fullmatch(r"[A-Za-z0-9_.-]+", partition) is None
    ):
        raise RuntimeError(
            "transport_partition must contain only letters, numbers, "
            "dot, underscore or dash and be at most 128 characters"
        )
    return partition


def _gazebo_environment(description_share, transport_partition):
    """Expose ROS-installed plugins and official CAD meshes to Gazebo."""
    library_path = os.environ.get("LD_LIBRARY_PATH", "")
    resource_root = str(description_share.parent)

    def _extend(name, extra):
        return os.pathsep.join(
            value
            for value in (os.environ.get(name, ""), extra)
            if value
        )

    return {
        "GZ_PARTITION": transport_partition,
        "GZ_SIM_SYSTEM_PLUGIN_PATH": _extend(
            "GZ_SIM_SYSTEM_PLUGIN_PATH", library_path
        ),
        "GZ_SIM_RESOURCE_PATH": _extend(
            "GZ_SIM_RESOURCE_PATH", resource_root
        ),
    }


def _shutdown_on_unexpected_exit(action, label):
    """Shut down once if a critical process exits while launch is running."""

    def _on_exit(event, context):
        if context.is_shutdown:
            return []
        reason = f"{label} exited with code {event.returncode}"
        return [EmitEvent(event=Shutdown(reason=reason))]

    return RegisterEventHandler(
        OnProcessExit(target_action=action, on_exit=_on_exit)
    )


def _launch_setup(context):
    package_share = Path(
        get_package_share_directory("openarm_skeleton_v1_2_gazebo")
    )
    description_share = Path(
        get_package_share_directory("openarm_skeleton_v1_2_description")
    )
    world = LaunchConfiguration("world").perform(context).strip()
    if not Path(world).is_file():
        raise RuntimeError(f"Gazebo world file not found: {world}")
    gui_config = LaunchConfiguration("gui_config").perform(context).strip()
    if gui_config and not Path(gui_config).is_file():
        raise RuntimeError(f"Gazebo GUI config file not found: {gui_config}")
    world_name = LaunchConfiguration("world_name").perform(context).strip()
    robot_name = LaunchConfiguration("robot_name").perform(context).strip()
    if not world_name or not robot_name:
        raise RuntimeError("world_name and robot_name cannot be empty")
    transport_partition = _resolve_transport_partition(
        LaunchConfiguration("transport_partition").perform(context)
    )
    gazebo_environment = _gazebo_environment(
        description_share,
        transport_partition,
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    description_topic = LaunchConfiguration("robot_description_topic")
    gazebo = ExecuteProcess(
        cmd=_gazebo_command(
            LaunchConfiguration("verbosity").perform(context),
            LaunchConfiguration("paused").perform(context),
            LaunchConfiguration("headless").perform(context),
            world,
            gui_config,
        ),
        additional_env=gazebo_environment,
        output="screen",
        shell=False,
    )

    enable_skeleton_lift = _is_true(
        LaunchConfiguration("enable_skeleton_lift").perform(context)
    )
    profile = (
        "openarm_skeleton_v1_2_lift_demo.urdf"
        if enable_skeleton_lift
        else "openarm_skeleton_v1_2_base_demo.urdf"
    )
    urdf_path = description_share / "urdf" / profile
    if not urdf_path.is_file():
        raise RuntimeError(f"Robot profile not found: {urdf_path}")
    robot_description = urdf_path.read_text(encoding="utf-8")
    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="official_robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    create_service = (
        f"/world/{world_name}/create"
        "@ros_gz_interfaces/srv/SpawnEntity"
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="official_gazebo_bridge",
        output="screen",
        arguments=[create_service],
        parameters=[
            {
                "config_file": str(package_share / "config" / "bridge.yaml"),
                "use_sim_time": use_sim_time,
            }
        ],
        additional_env=gazebo_environment,
    )
    watchdog = Node(
        package="openarm_skeleton_v1_2_gazebo",
        executable="cmd_vel_watchdog.py",
        name="official_cmd_vel_watchdog",
        output="screen",
        parameters=[
            str(package_share / "config" / "cmd_vel_watchdog.yaml")
        ],
    )
    spawner = Node(
        package="openarm_skeleton_v1_2_gazebo",
        executable="spawn_entity.py",
        name="spawn_official_openarm",
        output="screen",
        parameters=[
            {
                "world_name": world_name,
                "robot_name": robot_name,
                "robot_description_topic": description_topic,
                "allow_renaming": False,
                "x": ParameterValue(
                    LaunchConfiguration("x"), value_type=float
                ),
                "y": ParameterValue(
                    LaunchConfiguration("y"), value_type=float
                ),
                "z": ParameterValue(
                    LaunchConfiguration("z"), value_type=float
                ),
                "yaw": ParameterValue(
                    LaunchConfiguration("yaw"), value_type=float
                ),
                "startup_timeout": 20.0,
                "service_timeout": 20.0,
                "description_timeout": 20.0,
                "response_timeout": 20.0,
            }
        ],
    )

    startup_watchdog = TimerAction(
        period=STARTUP_DEADLINE_SECONDS,
        actions=[
            LogInfo(
                msg=(
                    "Official robot did not spawn within "
                    f"{STARTUP_DEADLINE_SECONDS:.0f}s"
                )
            ),
            EmitEvent(
                event=Shutdown(reason="Official robot startup timeout")
            ),
        ],
    )

    def _after_spawn(event, _context):
        if event.returncode == 0:
            startup_watchdog.cancel()
            return [
                LogInfo(
                    msg=(
                        "Official OpenArm spawned. Publish geometry_msgs/Twist "
                        "to /cmd_vel or run check_cmd_vel_motion.py."
                    )
                )
            ]
        reason = f"official robot spawn failed with code {event.returncode}"
        return [
            LogInfo(msg=reason),
            EmitEvent(event=Shutdown(reason=reason)),
        ]

    delayed_robot = TimerAction(
        period=2.0,
        actions=[state_publisher, spawner],
    )
    return [
        LogInfo(
            msg=(
                "Isolated Gazebo transport partition: "
                f"{transport_partition}"
            )
        ),
        gazebo,
        bridge,
        watchdog,
        _shutdown_on_unexpected_exit(gazebo, "Gazebo"),
        _shutdown_on_unexpected_exit(bridge, "Gazebo bridge"),
        _shutdown_on_unexpected_exit(watchdog, "Base command watchdog"),
        startup_watchdog,
        RegisterEventHandler(
            OnProcessExit(target_action=spawner, on_exit=_after_spawn)
        ),
        delayed_robot,
    ]


def generate_launch_description():
    package_share = Path(
        get_package_share_directory("openarm_skeleton_v1_2_gazebo")
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=str(
                    package_share / "worlds" / "official_robot_demo.sdf"
                ),
                description="Absolute path to the SDF world.",
            ),
            DeclareLaunchArgument(
                "world_name", default_value="openarm_official"
            ),
            DeclareLaunchArgument(
                "robot_name", default_value="openarm_skeleton"
            ),
            DeclareLaunchArgument(
                "robot_description_topic",
                default_value="/robot_description",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "enable_skeleton_lift",
                default_value="false",
                description=(
                    "Enable provisional five-joint skeleton lift demo."
                ),
            ),
            DeclareLaunchArgument(
                "transport_partition",
                default_value="",
                description=(
                    "Gazebo transport partition. Empty creates a "
                    "unique partition for this launch."
                ),
            ),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument(
                "gui_config",
                default_value="",
                description=(
                    "Optional absolute Gazebo GUI config; ignored headless."
                ),
            ),
            DeclareLaunchArgument("paused", default_value="false"),
            DeclareLaunchArgument("verbosity", default_value="2"),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
