"""
Launch script for the Gazebo arm simulation
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
    # Joint GUI
    # ----------------------------------------------

    joint_gui = Node(
        package="sim_common",
        executable="joint_gui",
        arguments=[urdf_file],
        condition=IfCondition(LaunchConfiguration("gui")),
        output="screen",
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
            DeclareLaunchArgument("gui", default_value="true", description="Open Joint GUI."),
            *chain_controller_spawners(
                spawn_robot, joint_state_broadcaster, joint_trajectory_controller
            ),
            clock_bridge_node(),
            rviz_node(rviz_config),
            joint_gui,
        ]
    )
