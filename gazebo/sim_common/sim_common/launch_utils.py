"""Shared launch utilities for simulation packages"""

import math
import os
import shutil
import subprocess
import sys
from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# highest ZED frame rate (Hz) at which depth rays are simulated at full resolution
ZED_FULL_RES_RATE = 15.0

# gz topic -> ROS type mappings for the chassis' simulated ZED (ground truth - the
# zed_camera node turns these into what the real wrapper publishes)
ZED_BRIDGES = [
    "/chassis/zed/image@sensor_msgs/msg/Image[gz.msgs.Image",
    "/chassis/zed/depth_rays/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
    "/chassis/zed/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
    "/chassis/zed/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
    "/chassis/ground_truth/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
]


def log(msg):
    """Green info log for launch files"""
    print("\033[0;32m", "[INFO] [launch]: ", msg, "\x1b[0m", sep="")


def err(msg):
    """Red error log, exits with code 1"""
    print("\033[0;31m", "[ERROR] [launch]: ", msg, "\x1b[0m", sep="")
    sys.exit(1)


def get_asset(package, *parts):
    """
    Resolve a file path inside a ROS2 package share directory.
    Exits with an error if the resolved path does not exist.
    """
    pkg_dir = get_package_share_directory(package)
    path = os.path.join(pkg_dir, *parts)
    if not Path(path).exists():
        err(f"File {path} does not exist!")
    return path


def gz_supports_sim_command():
    """Return True when `gz` exists and exposes the `sim` subcommand"""
    if shutil.which("gz") is None:
        return False

    try:
        result = subprocess.run(
            ["gz", "--commands"],
            check=False,
            capture_output=True,
            text=True,
        )
    except OSError:
        return False

    commands = result.stdout.splitlines()
    return any(line.strip() == "sim" for line in commands)


def process_robot_description(urdf_file, controller_config, extra_mappings=None):
    """
    Run xacro over `urdf_file`, wiring in the controller config and platform plugin
    extension, plus any `extra_mappings` for robot-specific xacro args
    """
    return xacro.process_file(
        urdf_file,
        mappings={
            "controller_config": controller_config,
            "plugin_extension": ".dylib" if sys.platform == "darwin" else ".so",
            **(extra_mappings or {}),
        },
    ).toxml()


def gazebo_launch_actions(
    world_file, gz_gui_config, gui_launch_arg="gui", gui_delay=2.0, combined_gui=False
):
    """
    Bring up the gz sim server, plus its GUI if the installed `gz` supports launching
    them as separate processes (falls back to the server's built-in GUI otherwise).
    `gui_delay` gives the server a head start before the GUI client connects -
    worlds with heavier meshes/plugins to load may need more than the default.
    `combined_gui` forces server+GUI into a single process even when split mode
    is available: worlds with rendering-dependent sensors (e.g. IMU/cameras) need
    this, since a server running separately from the GUI has no render context of
    its own for its Sensors system to use, and never produces any sensor data.
    Returns a list of launch actions to splice into a LaunchDescription.
    """
    use_split_gui = gz_supports_sim_command() and not combined_gui
    gz_server_args = (
        ["-r", "-s", world_file]
        if use_split_gui
        else ["-r", world_file, "--gui-config", gz_gui_config]
    )

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": " ".join(gz_server_args)}.items(),
    )

    if not use_split_gui:
        return [gz_server]

    gz_gui = ExecuteProcess(
        cmd=["gz", "sim", "--force-version", "8", "-g", "--gui-config", gz_gui_config],
        output="screen",
        condition=IfCondition(LaunchConfiguration(gui_launch_arg)),
    )
    return [gz_server, TimerAction(period=gui_delay, actions=[gz_gui])]


def spawn_robot_node(robot_name, robot_desc, z=0.1):
    """
    Node that spawns `robot_desc` into the running gz sim world as `robot_name`.
    `z` is the spawn height above the world origin - bump it for worlds whose
    ground isn't a flat plane at z=0 (e.g. a heightmap terrain), so the robot
    drops onto the surface instead of spawning underneath it.
    """
    return Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", robot_name, "-string", robot_desc, "-x", "0", "-y", "0", "-z", str(z)],
        output="screen",
    )


def robot_state_publisher_node(robot_desc):
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_desc}],
    )


def clock_bridge_node():
    """ros_gz_bridge node that bridges the gz sim clock onto /clock"""
    return Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )


def zed_launch_arguments():
    """Launch arguments for the simulated ZED 2i, named after the wrapper's parameters"""
    return [
        DeclareLaunchArgument(
            "zed_frame_rate",
            default_value="15",
            description="Simulated ZED image/depth rate in Hz (the real camera grabs at 60).",
        ),
        DeclareLaunchArgument(
            "zed_point_cloud_freq",
            default_value="5",
            description="Simulated ZED point cloud rate in Hz (wrapper depth.point_cloud_freq).",
        ),
        DeclareLaunchArgument(
            "zed_point_cloud_res",
            default_value="REDUCED",
            description="Simulated ZED point cloud resolution: COMPACT (full) or REDUCED "
            "(half), like the wrapper's depth.point_cloud_res.",
        ),
    ]


def zed_xacro_mappings(context):
    """
    xacro args for chassis_zed.urdf.xacro, to pass to `process_robot_description`.
    Needs the launch `context` (so call it from an OpaqueFunction) because the
    camera's render rate comes from the `zed_frame_rate` launch argument.
    """
    zed_frame_rate = float(LaunchConfiguration("zed_frame_rate").perform(context))
    return {
        "zed_frame_rate": str(zed_frame_rate),
        # zed_camera.py keeps up with full-resolution depth rays up to 15 Hz;
        # above that, thin the rays so their total data rate stays the same
        "zed_depth_ray_scale": str(min(1.0, math.sqrt(ZED_FULL_RES_RATE / zed_frame_rate))),
    }


def zed_nodes():
    """
    The simulated StereoLabs ZED 2i: a bridge for its gz ground truth, and the
    zed_camera node that republishes it on /zed/zed_node/* like zed-ros2-wrapper
    """
    sensor_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="zed_bridge",
        arguments=ZED_BRIDGES,
        output="screen",
    )
    zed_camera = Node(
        package="sim_common",
        executable="zed_camera",
        namespace="zed",
        output="screen",
        parameters=[
            {
                "pub_frame_rate": ParameterValue(
                    LaunchConfiguration("zed_frame_rate"), value_type=float
                ),
                "point_cloud_freq": ParameterValue(
                    LaunchConfiguration("zed_point_cloud_freq"), value_type=float
                ),
                "point_cloud_res": LaunchConfiguration("zed_point_cloud_res"),
                "use_sim_time": True,
            },
        ],
    )
    return [sensor_bridge, zed_camera]


def rviz_node(rviz_config, rviz_launch_arg="rviz"):
    return Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration(rviz_launch_arg)),
    )


def joint_state_broadcaster_spawner():
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )


def controller_spawner_node(controller_name, controller_config=None):
    arguments = [controller_name]
    if controller_config is not None:
        arguments += ["--param-file", controller_config]
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=arguments,
    )


def chain_controller_spawners(spawn_robot, *spawners):
    """
    Start `spawners` one after another, each once the previous one exits, starting
    once `spawn_robot` exits. Mirrors how controller_manager spawners need the
    robot (and each other) to already be loaded before they can activate.
    Returns the list of RegisterEventHandler actions to splice into a LaunchDescription.
    """
    handlers = []
    previous_action = spawn_robot
    for spawner in spawners:
        handlers.append(
            RegisterEventHandler(
                event_handler=OnProcessExit(target_action=previous_action, on_exit=[spawner])
            )
        )
        previous_action = spawner
    return handlers
