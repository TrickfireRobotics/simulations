"""
Launch script for the ArduPilot SITL + Gazebo drone simulation.

Unlike the URDF/ros2_control robots (arm, chassis), the quadcopter is an SDF
model embedded directly in the world file (see drone_description) and is
flown by ArduCopter SITL, not ROS 2 controllers - there's no robot_state_publisher,
controller spawners, or RViz here.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from sim_common.launch_utils import err, gazebo_launch_actions, get_asset

# UW campus, arbitrary - override by editing --home below for a real flying field.
SITL_HOME = "47.6521,-122.3037,0.0,0"


def generate_launch_description():
    """ROS launch method, must have this name"""

    world_file = get_asset("drone_description", "worlds", "drone.world.sdf")
    gz_gui_config = get_asset("sim_worlds", "gui", "gui.config")

    arducopter_bin = os.environ.get("ARDUCOPTER_BIN")
    if not arducopter_bin or not os.path.isfile(arducopter_bin):
        err(
            "ARDUCOPTER_BIN not set or missing - ArduPilot SITL is not built.\n"
            "        Run: sim gazebo setup-drone"
        )

    # ------------------------------------------------
    # ArduCopter SITL
    # ------------------------------------------------
    # --model JSON talks to the ardupilot_gazebo plugin's JSON backend on the
    # model's configured fdm ports. ArduPilot's serial-device parser has no
    # listening/server UDP mode - only udpclient: (see AP_HAL_SITL/UARTDriver.cpp) -
    # so SITL sends telemetry out to the GCS's address instead of waiting for
    # one. QGroundControl runs in this same container, so that's just loopback.
    sitl = ExecuteProcess(
        cmd=[
            arducopter_bin,
            "--model",
            "JSON",
            "--home",
            SITL_HOME,
            "--speedup",
            "1",
            "-I0",
            "--serial0=udpclient:127.0.0.1:14550",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            *gazebo_launch_actions(world_file, gz_gui_config, combined_gui=True),
            DeclareLaunchArgument("gui", default_value="true", description="Open Gazebo GUI."),
            # SITL's JSON backend retries aggressively (and noisily) until the Gazebo
            # plugin is up, which competes for CPU with the GUI's mesh import right
            # when it needs it most. Give Gazebo (server + GUI + meshes) a real head
            # start before SITL starts hammering it.
            TimerAction(period=15.0, actions=[sitl]),
        ]
    )
