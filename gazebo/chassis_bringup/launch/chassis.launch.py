"""
Launch script for the Gazebo chassis simulation
"""

import math
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from sim_common.launch_utils import (
    chain_controller_spawners,
    clock_bridge_node,
    controller_spawner_node,
    gazebo_launch_actions,
    get_asset,
    joint_state_broadcaster_spawner,
    process_robot_description,
    robot_state_publisher_node,
    rviz_node,
    spawn_robot_node,
)

ROBOT_NAME = "chassis"

# highest ZED frame rate (Hz) at which depth rays are simulated at full resolution
ZED_FULL_RES_RATE = 15.0

# gz topic -> ROS type mappings for the chassis' simulated sensors (ground truth -
# the sim_common driver nodes turn these into what the real drivers publish)
SENSOR_BRIDGES = [
    "/chassis/gps/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
    "/chassis/zed/image@sensor_msgs/msg/Image[gz.msgs.Image",
    "/chassis/zed/depth_rays/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
    "/chassis/zed/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
    "/chassis/zed/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
    "/chassis/ground_truth/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
]


def robot_actions(context, controller_config):
    """
    Spawn, state publisher and controllers - deferred until launch args resolve
    because xacro needs `zed_frame_rate` to set the camera's render rate
    """
    urdf_file = get_asset("chassis_description", "urdf", "chassis.urdf")
    zed_frame_rate = float(LaunchConfiguration("zed_frame_rate").perform(context))
    robot_desc = process_robot_description(
        urdf_file,
        controller_config,
        {
            "zed_frame_rate": str(zed_frame_rate),
            # zed_camera.py keeps up with full-resolution depth rays up to 15 Hz;
            # above that, thin the rays so their total data rate stays the same
            "zed_depth_ray_scale": str(min(1.0, math.sqrt(ZED_FULL_RES_RATE / zed_frame_rate))),
        },
    )

    spawn_robot = spawn_robot_node(ROBOT_NAME, robot_desc)
    joint_state_broadcaster = joint_state_broadcaster_spawner()
    forward_velocity_controller = controller_spawner_node(
        "forward_velocity_controller", controller_config
    )

    return [
        TimerAction(period=5.0, actions=[spawn_robot]),
        robot_state_publisher_node(robot_desc),
        *chain_controller_spawners(
            spawn_robot, joint_state_broadcaster, forward_velocity_controller
        ),
    ]


def generate_launch_description():
    """ROS launch method, must have this name"""

    controller_config = get_asset("chassis_bringup", "config", "chassis.controller.yaml")
    world_file = get_asset("sim_worlds", "worlds", "rover.world.sdf")
    rviz_config = get_asset("chassis_bringup", "config", "chassis.rviz")
    gz_gui_config = get_asset("sim_worlds", "gui", "gui.config")

    # ----------------------------------------------
    # Drivebase bridge
    # ----------------------------------------------

    drivebase_bridge = Node(
        package="sim_common",
        executable="drivebase",
        output="screen",
    )

    # ----------------------------------------------
    # Sensors
    # ----------------------------------------------

    sensor_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=SENSOR_BRIDGES,
        output="screen",
    )

    # GPS (u-blox ZED-F9P): /ublox_gps_node/fix and fix_velocity like the real driver
    ublox_gps = Node(
        package="sim_common",
        executable="ublox_gps",
        output="screen",
        parameters=[
            {"fix_mode": LaunchConfiguration("gps_fix_mode"), "use_sim_time": True},
        ],
    )

    # Camera (StereoLabs ZED 2i): /zed/zed_node/* like zed-ros2-wrapper
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

    # ----------------------------------------------
    # Optional props
    # ----------------------------------------------

    # TTT faces +X in its own frame; yaw 180 deg turns it to face the rover.
    # Its mesh resolves from sim_worlds/models via GZ_SIM_RESOURCE_PATH
    spawn_ttt = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "TTT",
            "-file",
            get_asset("sim_worlds", "models", "TTT", "model.sdf"),
            "-x",
            "6.5",
            "-Y",
            str(math.pi),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("ttt")),
    )

    # ----------------------------------------------
    # Rosbridge
    # ----------------------------------------------

    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        output="screen",
        parameters=[
            {"port": ParameterValue(LaunchConfiguration("rosbridge_port"), value_type=int)}
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true", description="Open RViz."),
            DeclareLaunchArgument("gui", default_value="true", description="Open Gazebo GUI."),
            DeclareLaunchArgument(
                "rosbridge_port",
                default_value=os.environ.get("ROSBRIDGE_PORT", "9090"),
                description="Port for the rosbridge WebSocket server.",
            ),
            DeclareLaunchArgument(
                "gps_fix_mode",
                default_value="rtk_fixed",
                description="Simulated GPS fix quality: rtk_fixed or standalone.",
            ),
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
            DeclareLaunchArgument(
                "ttt",
                default_value="false",
                description="Spawn the TTT model 6.5 m ahead of the rover.",
            ),
            # the ZED is a rendering sensor, which needs the server's render context.
            # macOS gz can't run server+GUI in one process (gz-sim#44), but there the
            # split-mode server renders offscreen on its own
            *gazebo_launch_actions(
                world_file, gz_gui_config, combined_gui=sys.platform != "darwin"
            ),
            OpaqueFunction(function=robot_actions, args=[controller_config]),
            # same head start as the robot spawn, so the world is up first
            TimerAction(period=5.0, actions=[spawn_ttt]),
            clock_bridge_node(),
            rviz_node(rviz_config),
            drivebase_bridge,
            sensor_bridge,
            ublox_gps,
            zed_camera,
            rosbridge,
        ]
    )
