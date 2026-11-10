"""
Launch script for the Gazebo chassis simulation
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
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
    zed_launch_arguments,
    zed_nodes,
    zed_xacro_mappings,
)

ROBOT_NAME = "chassis"


def robot_actions(context, controller_config):
    """
    Spawn, state publisher and controllers - deferred until launch args resolve
    because xacro needs `zed_frame_rate` to set the camera's render rate
    """
    urdf_file = get_asset("chassis_description", "urdf", "chassis.urdf")
    robot_desc = process_robot_description(
        urdf_file, controller_config, zed_xacro_mappings(context)
    )

    # the marsyard terrain is ~0.4m tall at the origin (vs. the 0.1m default meant
    # for a flat ground plane) - spawn above it so the chassis drops onto the
    # surface instead of into it
    # TEMP DEBUG: raised from 0.7 - Viator's wheel-bottom is ~0.40m below base_link,
    # which at z=0.7 only cleared the ~0.4m terrain by about 0.1m. Testing whether
    # spawning higher (so the chassis free-falls onto the terrain instead of
    # possibly starting embedded in it) stops the rover from launching off-screen.
    spawn_robot = spawn_robot_node(ROBOT_NAME, robot_desc, z=1.5)
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
    world_file = get_asset("sim_worlds", "worlds", "mars.world.sdf")
    rviz_config = get_asset("chassis_bringup", "config", "chassis.rviz")
    gz_gui_config = get_asset("sim_worlds", "gui", "mars_gui.config")

    # ----------------------------------------------
    # Drivebase bridge
    # ----------------------------------------------

    drivebase_bridge = Node(
        package="sim_common",
        executable="drivebase",
        output="screen",
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
            *zed_launch_arguments(),
            # the ZED is a rendering sensor, which needs the server's render context.
            # macOS gz can't run server+GUI in one process (gz-sim#44), but there the
            # split-mode server renders offscreen on its own
            *gazebo_launch_actions(
                world_file, gz_gui_config, combined_gui=sys.platform != "darwin"
            ),
            OpaqueFunction(function=robot_actions, args=[controller_config]),
            clock_bridge_node(),
            rviz_node(rviz_config),
            drivebase_bridge,
            *zed_nodes(),
            rosbridge,
        ]
    )
