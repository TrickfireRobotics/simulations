"""Launch the complete Viator rover in Gazebo."""

import math
import os
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from sim_common.launch_utils import (
    chain_controller_spawners,
    clock_bridge_node,
    controller_spawner_node,
    gazebo_launch_actions,
    get_asset,
    joint_gui_node,
    joint_state_broadcaster_spawner,
    process_robot_description,
    robot_state_publisher_node,
    rviz_node,
    spawn_robot_node,
)

ROBOT_NAME = "viator"
ZED_FULL_RES_RATE = 15.0

SENSOR_BRIDGES = [
    "/viator/gps/navsat@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
    "/viator/zed/image@sensor_msgs/msg/Image[gz.msgs.Image",
    "/viator/zed/depth_rays/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
    "/viator/zed/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
    "/viator/zed/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
    "/viator/ground_truth/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
]


def robot_actions(context, controller_config, urdf_file):
    """Resolve sensor arguments before processing the Viator xacro."""
    zed_frame_rate = float(LaunchConfiguration("zed_frame_rate").perform(context))
    robot_desc = process_robot_description(
        urdf_file,
        controller_config,
        {
            "zed_frame_rate": str(zed_frame_rate),
            "zed_depth_ray_scale": str(min(1.0, math.sqrt(ZED_FULL_RES_RATE / zed_frame_rate))),
        },
    )

    spawn_robot = spawn_robot_node(ROBOT_NAME, robot_desc, z=1.5)
    joint_state_broadcaster = joint_state_broadcaster_spawner()
    forward_velocity_controller = controller_spawner_node(
        "forward_velocity_controller", controller_config
    )
    joint_trajectory_controller = controller_spawner_node(
        "joint_trajectory_controller", controller_config
    )

    follow_camera = ExecuteProcess(
        cmd=[
            "gz",
            "topic",
            "-t",
            "/gui/track",
            "-m",
            "gz.msgs.CameraTrack",
            "-p",
            (
                'track_mode: FOLLOW follow_target: {name: "viator"} '
                "follow_offset: {x: -4, y: 0, z: 1.7} "
                "follow_pgain: 0.02 track_pgain: 0.02"
            ),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("follow_rover")),
    )

    return [
        TimerAction(period=5.0, actions=[spawn_robot]),
        robot_state_publisher_node(robot_desc),
        *chain_controller_spawners(
            spawn_robot,
            joint_state_broadcaster,
            forward_velocity_controller,
            joint_trajectory_controller,
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_robot,
                on_exit=[TimerAction(period=3.0, actions=[follow_camera])],
            ),
            condition=IfCondition(LaunchConfiguration("gui")),
        ),
    ]


def generate_launch_description():
    controller_config = get_asset("viator_sim", "config", "viator.controller.yaml")
    world_file = get_asset("sim_worlds", "worlds", "mars.world.sdf")
    rviz_config = get_asset("viator_sim", "config", "viator.rviz")
    urdf_file = get_asset("viator_description", "urdf", "viator.urdf.xacro")
    gz_gui_config = get_asset("sim_worlds", "gui", "mars_gui.config")

    drivebase_bridge = Node(package="sim_common", executable="drivebase", output="screen")
    arm_bridge = Node(package="sim_common", executable="arm", output="screen")
    sensor_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="viator_sensor_bridge",
        arguments=SENSOR_BRIDGES,
        output="screen",
    )
    ublox_gps = Node(
        package="viator_sim",
        executable="ublox_gps",
        output="screen",
        parameters=[{"fix_mode": LaunchConfiguration("gps_fix_mode"), "use_sim_time": True}],
    )
    zed_camera = Node(
        package="viator_sim",
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
            }
        ],
    )
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
                description="Simulated ZED image and depth rate in Hz.",
            ),
            DeclareLaunchArgument(
                "zed_point_cloud_freq",
                default_value="5",
                description="Simulated ZED point cloud rate in Hz.",
            ),
            DeclareLaunchArgument(
                "zed_point_cloud_res",
                default_value="REDUCED",
                description="Simulated ZED point cloud resolution: COMPACT or REDUCED.",
            ),
            DeclareLaunchArgument(
                "ttt",
                default_value="false",
                description="Spawn the TTT task model ahead of Viator.",
            ),
            DeclareLaunchArgument(
                "follow_rover",
                default_value="true",
                description="Make the Gazebo camera follow Viator.",
            ),
            DeclareLaunchArgument(
                "joint_gui",
                default_value="false",
                description="Open the arm joint control GUI.",
            ),
            *gazebo_launch_actions(
                world_file,
                gz_gui_config,
                combined_gui=sys.platform != "darwin",
            ),
            OpaqueFunction(function=robot_actions, args=[controller_config, urdf_file]),
            TimerAction(period=5.0, actions=[spawn_ttt]),
            clock_bridge_node(),
            rviz_node(rviz_config),
            joint_gui_node(urdf_file),
            drivebase_bridge,
            arm_bridge,
            sensor_bridge,
            ublox_gps,
            zed_camera,
            rosbridge,
        ]
    )
