"""
Launch script for the Gazebo chassis simulation
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
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


def generate_launch_description():
    """ROS launch method, must have this name"""

    controller_config = get_asset("chassis_bringup", "config", "chassis.controller.yaml")
    world_file = get_asset("sim_worlds", "worlds", "mars.world.sdf")
    rviz_config = get_asset("chassis_bringup", "config", "chassis.rviz")
    urdf_file = get_asset("chassis_description", "urdf", "chassis.urdf")
    gz_gui_config = get_asset("sim_worlds", "gui", "mars_gui.config")

    robot_desc = process_robot_description(urdf_file, controller_config)

    # the marsyard terrain is ~1.6m tall at the origin (vs. the 0.1m default meant
    # for a flat ground plane) - spawn above it so the chassis drops onto the
    # surface instead of into it
    spawn_robot = spawn_robot_node(ROBOT_NAME, robot_desc, z=1.9)
    joint_state_broadcaster = joint_state_broadcaster_spawner()
    forward_velocity_controller = controller_spawner_node(
        "forward_velocity_controller", controller_config
    )

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
            *gazebo_launch_actions(world_file, gz_gui_config),
            TimerAction(period=5.0, actions=[spawn_robot]),
            robot_state_publisher_node(robot_desc),
            DeclareLaunchArgument("rviz", default_value="true", description="Open RViz."),
            DeclareLaunchArgument("gui", default_value="true", description="Open Gazebo GUI."),
            DeclareLaunchArgument(
                "rosbridge_port",
                default_value=os.environ.get("ROSBRIDGE_PORT", "9090"),
                description="Port for the rosbridge WebSocket server.",
            ),
            *chain_controller_spawners(
                spawn_robot, joint_state_broadcaster, forward_velocity_controller
            ),
            clock_bridge_node(),
            rviz_node(rviz_config),
            drivebase_bridge,
            rosbridge,
        ]
    )
