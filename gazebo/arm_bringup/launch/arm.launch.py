"""
Launch script for the Gazebo arm simulation
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
    heartbeat_responder_node,
    joint_gui_node,
    joint_state_broadcaster_spawner,
    process_robot_description,
    robot_state_publisher_node,
    rviz_node,
    spawn_robot_node,
)

ROBOT_NAME = "arm"


def generate_launch_description():
    """ROS launch method, must have this name"""

    controller_config = get_asset("arm_bringup", "config", "arm.controller.yaml")
    world_file = get_asset("sim_worlds", "worlds", "arm.world.sdf")
    rviz_config = get_asset("arm_bringup", "config", "arm.rviz")
    urdf_file = get_asset("arm_description", "urdf", "arm.urdf")
    gz_gui_config = get_asset("sim_worlds", "gui", "gui.config")

    robot_desc = process_robot_description(urdf_file, controller_config)

    spawn_robot = spawn_robot_node(ROBOT_NAME, robot_desc)
    joint_state_broadcaster = joint_state_broadcaster_spawner()
    joint_trajectory_controller = controller_spawner_node(
        "joint_trajectory_controller", controller_config
    )

    # ----------------------------------------------
    # Arm bridge
    # ----------------------------------------------

    arm_bridge = Node(
        package="sim_common",
        executable="arm",
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
            {
                "port": ParameterValue(LaunchConfiguration("rosbridge_port"), value_type=int),
                "websocket_ping_interval": 5.0,
                "websocket_ping_timeout": 10.0,
            }
        ],
    )

    return LaunchDescription(
        [
            *gazebo_launch_actions(
                world_file,
                gz_gui_config,
                physics_engine="gz-physics-bullet-featherstone-plugin",
            ),
            TimerAction(period=5.0, actions=[spawn_robot]),
            robot_state_publisher_node(robot_desc),
            DeclareLaunchArgument("rviz", default_value="true", description="Open RViz."),
            DeclareLaunchArgument("gui", default_value="true", description="Open Gazebo GUI."),
            DeclareLaunchArgument(
                "joint_gui",
                default_value="false",
                description="Open the arm joint control GUI.",
            ),
            DeclareLaunchArgument(
                "rosbridge_port",
                default_value=os.environ.get("ROSBRIDGE_PORT", "9090"),
                description="Port for the rosbridge WebSocket server.",
            ),
            *chain_controller_spawners(
                spawn_robot, joint_state_broadcaster, joint_trajectory_controller
            ),
            clock_bridge_node(),
            rviz_node(rviz_config),
            joint_gui_node(urdf_file),
            arm_bridge,
            heartbeat_responder_node(),
            rosbridge,
        ]
    )
