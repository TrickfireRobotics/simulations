"""
Launch script for the Gazebo rover simulation (chassis + arm)
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
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
    joint_state_broadcaster_spawner,
    process_robot_description,
    robot_state_publisher_node,
    rviz_node,
    spawn_robot_node,
)

ROBOT_NAME = "rover"


def generate_launch_description():
    """ROS launch method, must have this name"""

    controller_config = get_asset("rover_bringup", "config", "rover.controller.yaml")
    world_file = get_asset("sim_worlds", "worlds", "mars.world.sdf")
    rviz_config = get_asset("rover_bringup", "config", "rover.rviz")
    urdf_file = get_asset("rover_description", "urdf", "rover.urdf.xacro")
    gz_gui_config = get_asset("sim_worlds", "gui", "mars_gui.config")

    robot_desc = process_robot_description(urdf_file, controller_config)

    # see chassis.launch.py: spawn above the marsyard terrain so the rover
    # drops onto the surface instead of into it
    spawn_robot = spawn_robot_node(ROBOT_NAME, robot_desc, z=1.5)
    joint_state_broadcaster = joint_state_broadcaster_spawner()
    forward_velocity_controller = controller_spawner_node(
        "forward_velocity_controller", controller_config
    )
    joint_trajectory_controller = controller_spawner_node(
        "joint_trajectory_controller", controller_config
    )

    # start after `create` confirms the rover exists, avoiding a race with model and terrain loading
    follow_rover_camera = ExecuteProcess(
        cmd=[
            "gz",
            "topic",
            "-t",
            "/gui/track",
            "-m",
            "gz.msgs.CameraTrack",
            "-p",
            (
                'track_mode: FOLLOW follow_target: {name: "rover"} '
                "follow_offset: {x: -4, y: 0, z: 1.7} "
                "follow_pgain: 0.02 track_pgain: 0.02"
            ),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("gui")),
    )
    follow_camera_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[TimerAction(period=3.0, actions=[follow_rover_camera])],
        )
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

    # ----------------------------------------------
    # Arm joint GUI
    # ----------------------------------------------
    # see arm.launch.py - points at the raw xacro file since joint_gui.py
    # processes it itself, and only shows revolute joints (the arm's), since
    # the chassis wheel joints are type="continuous"

    joint_gui = Node(
        package="sim_common",
        executable="joint_gui",
        arguments=[urdf_file],
        condition=IfCondition(LaunchConfiguration("arm_gui")),
        output="screen",
    )

    return LaunchDescription(
        [
            *gazebo_launch_actions(world_file, gz_gui_config),
            TimerAction(period=5.0, actions=[spawn_robot]),
            robot_state_publisher_node(robot_desc),
            DeclareLaunchArgument("rviz", default_value="true", description="Open RViz."),
            DeclareLaunchArgument("gui", default_value="true", description="Open Gazebo GUI."),
            DeclareLaunchArgument(
                "arm_gui", default_value="true", description="Open arm Joint GUI."
            ),
            DeclareLaunchArgument(
                "rosbridge_port",
                default_value=os.environ.get("ROSBRIDGE_PORT", "9090"),
                description="Port for the rosbridge WebSocket server.",
            ),
            *chain_controller_spawners(
                spawn_robot,
                joint_state_broadcaster,
                forward_velocity_controller,
                joint_trajectory_controller,
            ),
            follow_camera_after_spawn,
            clock_bridge_node(),
            rviz_node(rviz_config),
            drivebase_bridge,
            rosbridge,
            joint_gui,
        ]
    )
