"""Paths to various files and directories used by the CLI"""

from pathlib import Path


def _find_repo_root() -> Path:
    for candidate in [Path.cwd(), *Path.cwd().parents]:
        if (candidate / ".git").exists():
            return candidate
    raise RuntimeError(
        "Could not locate repo root (no .git directory found). "
        "Run 'sim' from within the repository."
    )


REPO_DIR = _find_repo_root()

DOCKER_DIR = REPO_DIR / "docker"

GAZEBO_DIR = REPO_DIR / "gazebo"
GAZEBO_WORKSPACE_DIR = GAZEBO_DIR
GAZEBO_DOCKER_DIR = DOCKER_DIR

CHRONO_TERRAIN_DIR = REPO_DIR / "chrono"

# Alias used by gazebo launcher modules
WORKSPACE_DIR = GAZEBO_WORKSPACE_DIR

# Native / pixi build paths
NATIVE_BUILD_DIR = REPO_DIR / ".native"
NATIVE_BIN = NATIVE_BUILD_DIR / "bin"
NATIVE_ENVS = NATIVE_BUILD_DIR / "envs"
NATIVE_ENV_PREFIX = NATIVE_ENVS / "ros_env"
NATIVE_CONTROL_WS = NATIVE_BUILD_DIR / "gz_ros2_control_ws"

# ArduPilot SITL + ardupilot_gazebo plugin
ARDUPILOT_DIR = Path("/opt/ardupilot")
ARDUPILOT_GAZEBO_DIR = Path("/opt/ardupilot_gazebo")
ARDUCOPTER_BIN = ARDUPILOT_DIR / "build" / "sitl" / "bin" / "arducopter"
ARDUPILOT_GAZEBO_BUILD_DIR = ARDUPILOT_GAZEBO_DIR / "build"
ARDUPILOT_GAZEBO_PLUGIN = ARDUPILOT_GAZEBO_BUILD_DIR / "libArduPilotPlugin.so"

MACOS_SOURCE_FILES = GAZEBO_WORKSPACE_DIR / "sim_common" / "macos"
MACOS_MAMBA_ROOT = NATIVE_BUILD_DIR / "mamba"
MACOS_ROS_BASE = NATIVE_BUILD_DIR / "ros_base"
MACOS_ROS_GZ_WS = NATIVE_BUILD_DIR / "ros_gz_ws"
MACOS_CONTROL_WS = NATIVE_CONTROL_WS
MACOS_ENV_LOCK = MACOS_SOURCE_FILES / "env.lock.yml"
MACOS_VERSIONS_FILE = MACOS_SOURCE_FILES / "versions.env"
