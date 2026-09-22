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

"""Run OpenArm, a selectable light scene, and ROS 2 bridges in Isaac Sim 5."""

import argparse
from pathlib import Path
import shutil
import tempfile
import traceback

from isaac_scenes import SCENE_NAMES, get_scene_objects
from prepare_isaac_urdf import prepare_urdf


WHEEL_JOINTS = ["drive_joint_1", "drive_joint_2"]
PASSIVE_CASTER_JOINTS = {
    "caster_joint_1",
    "caster_wheel_joint_1",
    "caster_joint_2",
    "caster_wheel_joint_2",
}
WHEEL_RADIUS_METERS = 0.03810
WHEEL_DISTANCE_METERS = 0.45
DRIVE_DAMPING = 1.0e5
DRIVE_MAX_FORCE = 30.0
LIDAR_HEIGHT_METERS = 0.3802
COMMAND_TOPIC = "isaac_cmd_vel"
JOINT_STATE_TOPIC = "joint_states"
ODOMETRY_TOPIC = "isaac_odom_raw"
SCAN_TOPIC = "scan"
ODOM_FRAME = "odom"
BASE_FRAME = "base_footprint"
SCAN_FRAME = "base_scan"


def _arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-urdf", required=True)
    parser.add_argument("--mesh-dir", required=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    parser.add_argument("--scene", choices=SCENE_NAMES, default="hotel")
    parser.add_argument("--lidar-config", default="Example_Rotary_2D")
    parser.add_argument("--renderer", default="RayTracedLighting")
    args, _isaac_arguments = parser.parse_known_args()
    if args.max_frames < 0:
        parser.error("--max-frames must be zero or greater")
    return args


def _import_result_path(result):
    """Return a possible prim path from importer command variants."""
    values = result if isinstance(result, tuple) else (result,)
    for value in reversed(values):
        if isinstance(value, str) and value.startswith("/"):
            return value
        get_path = getattr(value, "GetPath", None)
        if callable(get_path):
            return str(get_path())
    return ""


def _find_articulation_root(stage, preferred_path, physics_schema):
    if preferred_path:
        preferred = stage.GetPrimAtPath(preferred_path)
        if preferred and preferred.IsValid():
            if preferred.HasAPI(physics_schema.ArticulationRootAPI):
                return str(preferred.GetPath())
            for prim in stage.Traverse():
                if str(prim.GetPath()).startswith(preferred_path + "/"):
                    if prim.HasAPI(physics_schema.ArticulationRootAPI):
                        return str(prim.GetPath())
    roots = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.HasAPI(physics_schema.ArticulationRootAPI)
    ]
    if len(roots) != 1:
        raise RuntimeError(f"expected one robot articulation root, found {roots}")
    return roots[0]


def _set_import_option(config, name, value):
    if hasattr(config, name):
        setattr(config, name, value)


def _import_robot(urdf_path, stage, kit_commands, urdf_module, physics_schema):
    config = urdf_module.ImportConfig()
    for name, value in {
        "merge_fixed_joints": True,
        "fix_base": False,
        "make_default_prim": False,
        "create_physics_scene": False,
        "collision_from_visuals": False,
        "self_collision": False,
        "replace_cylinders_with_capsules": False,
    }.items():
        _set_import_option(config, name, value)
    if hasattr(config, "default_drive_type"):
        config.default_drive_type = (
            urdf_module.UrdfJointTargetType.JOINT_DRIVE_VELOCITY
        )

    result = kit_commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(urdf_path),
        import_config=config,
        dest_path="",
    )
    imported_path = _import_result_path(result)
    return _find_articulation_root(
        stage, imported_path, physics_schema
    )


def _configure_joint_drives(stage, physics_schema):
    """Release passive casters and give both drive wheels bounded authority."""
    caster_found = set()
    drive_found = set()
    for prim in stage.Traverse():
        name = prim.GetName()
        if name not in PASSIVE_CASTER_JOINTS and name not in WHEEL_JOINTS:
            continue
        if not prim.HasAPI(physics_schema.DriveAPI, "angular"):
            raise RuntimeError(f"joint has no angular drive: {prim.GetPath()}")
        drive = physics_schema.DriveAPI(prim, "angular")
        if name in PASSIVE_CASTER_JOINTS:
            caster_found.add(name)
            drive.GetStiffnessAttr().Set(0.0)
            drive.GetDampingAttr().Set(0.0)
        else:
            drive_found.add(name)
            drive.GetStiffnessAttr().Set(0.0)
            # USD angular-drive damping is authored per degree. A value near
            # the URDF importer's wheeled-robot defaults is needed for the
            # controller's rad/s targets to have useful authority. The old
            # value of 10 let both wheels stall around 0.2 rad/s while Nav2
            # requested opposite velocities for a turn.
            drive.GetDampingAttr().Set(DRIVE_DAMPING)
            drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
    if caster_found != PASSIVE_CASTER_JOINTS:
        raise RuntimeError(
            "could not find all passive caster joints after URDF import: "
            f"{sorted(PASSIVE_CASTER_JOINTS - caster_found)}"
        )
    if drive_found != set(WHEEL_JOINTS):
        raise RuntimeError(
            "could not find both drive joints after URDF import: "
            f"{sorted(set(WHEEL_JOINTS) - drive_found)}"
        )


def _create_scene(world, fixed_cuboid, np, scene_name):
    world.scene.add_default_ground_plane()
    for item in get_scene_objects(scene_name):
        name = item["name"]
        world.scene.add(
            fixed_cuboid(
                prim_path=f"/World/openarm_scene/{scene_name}/{name}",
                name=f"openarm_{scene_name}_{name}",
                position=np.array(item["position"]),
                scale=np.array(item["scale"]),
                color=np.array(item["color"]),
            )
        )


def _create_lidar(
    robot_path, config, commands, gf, stage
):
    requested_path = f"{robot_path}/openarm_lidar"
    result = commands.execute(
        "IsaacSensorCreateRtxLidar",
        translation=gf.Vec3d(0.0, 0.0, LIDAR_HEIGHT_METERS),
        orientation=gf.Quatd(1.0, 0.0, 0.0, 0.0),
        path=requested_path,
        parent=None,
        config=config,
        variant=None,
        force_camera_prim=False,
        **{"omni:sensor:Core:skipDroppingInvalidPoints": True},
    )
    sensor_path = _import_result_path(result) or requested_path
    sensor_prim = stage.GetPrimAtPath(sensor_path)
    if not sensor_prim or not sensor_prim.IsValid():
        raise RuntimeError(f"RTX lidar was not created at {sensor_path}")
    return sensor_path


def _create_clock_graph(og):
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/World/OpenArmGraphs/clock", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.SET_VALUES: [
                ("Context.inputs:useDomainIDEnvVar", True),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
            ],
        },
    )


def _create_scan_graph(og, sdf, lidar_path):
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/World/OpenArmGraphs/scan", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("RunOneFrame", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                ("CreateRenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("LidarHelper", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
            ],
            keys.SET_VALUES: [
                ("Context.inputs:useDomainIDEnvVar", True),
                ("CreateRenderProduct.inputs:cameraPrim", [sdf.Path(lidar_path)]),
                ("LidarHelper.inputs:topicName", SCAN_TOPIC),
                ("LidarHelper.inputs:type", "laser_scan"),
                ("LidarHelper.inputs:frameId", SCAN_FRAME),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RunOneFrame.inputs:execIn"),
                ("RunOneFrame.outputs:step", "CreateRenderProduct.inputs:execIn"),
                ("CreateRenderProduct.outputs:execOut", "LidarHelper.inputs:execIn"),
                ("CreateRenderProduct.outputs:renderProductPath", "LidarHelper.inputs:renderProductPath"),
                ("Context.outputs:context", "LidarHelper.inputs:context"),
            ],
        },
    )


def _create_state_graph(og, sdf, robot_path):
    keys = og.Controller.Keys
    target = [sdf.Path(robot_path)]
    og.Controller.edit(
        {"graph_path": "/World/OpenArmGraphs/state", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("PublishRawTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ],
            keys.SET_VALUES: [
                ("Context.inputs:useDomainIDEnvVar", True),
                ("PublishJointState.inputs:targetPrim", target),
                ("PublishJointState.inputs:topicName", JOINT_STATE_TOPIC),
                ("ComputeOdom.inputs:chassisPrim", target),
                ("PublishOdom.inputs:topicName", ODOMETRY_TOPIC),
                ("PublishOdom.inputs:odomFrameId", ODOM_FRAME),
                ("PublishOdom.inputs:chassisFrameId", BASE_FRAME),
                ("PublishRawTF.inputs:childFrameId", BASE_FRAME),
                ("PublishRawTF.inputs:parentFrameId", ODOM_FRAME),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("OnPlaybackTick.outputs:tick", "ComputeOdom.inputs:execIn"),
                ("ComputeOdom.outputs:execOut", "PublishOdom.inputs:execIn"),
                ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
                ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
                ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
                ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
                ("OnPlaybackTick.outputs:tick", "PublishRawTF.inputs:execIn"),
                ("ComputeOdom.outputs:orientation", "PublishRawTF.inputs:rotation"),
                ("ComputeOdom.outputs:position", "PublishRawTF.inputs:translation"),
                ("ReadSimTime.outputs:simulationTime", "PublishRawTF.inputs:timeStamp"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "PublishOdom.inputs:context"),
                ("Context.outputs:context", "PublishRawTF.inputs:context"),
            ],
        },
    )


def _create_drive_graph(og, sdf, robot_path):
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": "/World/OpenArmGraphs/drive", "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("SubscribeTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("BreakLinVel", "omni.graph.nodes.BreakVector3"),
                ("BreakAngVel", "omni.graph.nodes.BreakVector3"),
                ("DiffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("ArtController", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            keys.SET_VALUES: [
                ("Context.inputs:useDomainIDEnvVar", True),
                ("SubscribeTwist.inputs:topicName", COMMAND_TOPIC),
                ("DiffController.inputs:maxAngularSpeed", 1.0),
                ("DiffController.inputs:maxLinearSpeed", 0.6),
                ("DiffController.inputs:wheelDistance", WHEEL_DISTANCE_METERS),
                ("DiffController.inputs:wheelRadius", WHEEL_RADIUS_METERS),
                ("ArtController.inputs:jointNames", WHEEL_JOINTS),
                ("ArtController.inputs:targetPrim", [sdf.Path(robot_path)]),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "SubscribeTwist.inputs:execIn"),
                ("OnPlaybackTick.outputs:deltaSeconds", "DiffController.inputs:dt"),
                ("SubscribeTwist.outputs:execOut", "DiffController.inputs:execIn"),
                ("SubscribeTwist.outputs:linearVelocity", "BreakLinVel.inputs:tuple"),
                ("BreakLinVel.outputs:x", "DiffController.inputs:linearVelocity"),
                ("SubscribeTwist.outputs:angularVelocity", "BreakAngVel.inputs:tuple"),
                ("BreakAngVel.outputs:z", "DiffController.inputs:angularVelocity"),
                ("DiffController.outputs:velocityCommand", "ArtController.inputs:velocityCommand"),
                ("OnPlaybackTick.outputs:tick", "ArtController.inputs:execIn"),
                ("Context.outputs:context", "SubscribeTwist.inputs:context"),
            ],
        },
    )


def main():
    args = _arguments()
    runtime_directory = Path(tempfile.mkdtemp(prefix="openarm_isaac_"))
    prepared_urdf = runtime_directory / "openarm_skeleton_v1_2_isaac.urdf"
    prepare_urdf(
        args.source_urdf,
        args.mesh_dir,
        prepared_urdf,
    )

    # SimulationApp must be created before importing any other Isaac module.
    from isaacsim import SimulationApp

    simulation_app = SimulationApp(
        {
            "headless": args.headless,
            "renderer": args.renderer,
            "width": 1280,
            "height": 720,
        }
    )
    exit_code = 0
    try:
        from isaacsim.core.utils.extensions import enable_extension

        for extension in (
            "isaacsim.asset.importer.urdf",
            "isaacsim.robot.wheeled_robots",
            "isaacsim.ros2.bridge",
            "isaacsim.sensors.rtx",
        ):
            enable_extension(extension)
        for _ in range(4):
            simulation_app.update()

        import numpy as np
        import omni.graph.core as og
        import omni.kit.commands
        import omni.usd
        from isaacsim.asset.importer.urdf import _urdf
        from isaacsim.core.api import World
        from isaacsim.core.api.objects import FixedCuboid
        from pxr import Gf, Sdf, UsdLux, UsdPhysics

        world = World(
            stage_units_in_meters=1.0,
            physics_dt=1.0 / 60.0,
            rendering_dt=1.0 / 30.0,
        )
        _create_scene(world, FixedCuboid, np, args.scene)
        stage = omni.usd.get_context().get_stage()
        light = UsdLux.DomeLight.Define(
            stage, f"/World/openarm_scene/{args.scene}/light"
        )
        light.CreateIntensityAttr(1000.0)
        robot_path = _import_robot(
            prepared_urdf,
            stage,
            omni.kit.commands,
            _urdf,
            UsdPhysics,
        )
        _configure_joint_drives(stage, UsdPhysics)
        lidar_path = _create_lidar(
            robot_path,
            args.lidar_config,
            omni.kit.commands,
            Gf,
            stage,
        )
        stage.DefinePrim("/World/OpenArmGraphs", "Scope")
        _create_clock_graph(og)
        _create_scan_graph(og, Sdf, lidar_path)
        _create_state_graph(og, Sdf, robot_path)
        _create_drive_graph(og, Sdf, robot_path)

        world.reset()
        world.play()
        print("OPENARM ISAAC READY", flush=True)
        print(f"  scene: {args.scene}", flush=True)
        print(f"  articulation: {robot_path}", flush=True)
        print(f"  lidar: {lidar_path}", flush=True)
        print(
            "  publishes: /clock /scan /odom /joint_states /tf",
            flush=True,
        )
        print("  safe command input: /cmd_vel_safe", flush=True)

        frame = 0
        while simulation_app.is_running():
            # RTX lidar requires rendered frames even in headless mode.
            world.step(render=True)
            frame += 1
            if args.max_frames and frame >= args.max_frames:
                break
    except Exception as error:  # Isaac logs the detailed extension context.
        exit_code = 1
        print(f"OPENARM ISAAC ERROR: {error}", flush=True)
        traceback.print_exc()
    finally:
        try:
            simulation_app.close()
        finally:
            shutil.rmtree(runtime_directory, ignore_errors=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
