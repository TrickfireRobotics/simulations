#!/usr/bin/env zsh

if [ -n "${ZSH_VERSION:-}" ]; then
    tf_shell=zsh
elif [ -n "${BASH_VERSION:-}" ]; then
    tf_shell=bash
else
    echo "source.sh supports Bash and Zsh" >&2
    return 1
fi

# source ROS
tf_ros_setup="/opt/ros/jazzy/setup.$tf_shell"
if [ -f "$tf_ros_setup" ]; then
    . "$tf_ros_setup" || echo "sourcing ROS failed"
fi

# source gazebo
tf_gazebo_setup="${TF_GAZEBO_DIR:-}/.artifacts/install/setup.$tf_shell"
if [ -n "${TF_GAZEBO_DIR:-}" ] && [ -f "$tf_gazebo_setup" ]; then
    . "$tf_gazebo_setup"

    sim_worlds_share="$TF_GAZEBO_DIR/.artifacts/install/sim_worlds/share/sim_worlds"
    if [[ -d "$sim_worlds_share/worlds" ]]; then
        export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:+$GZ_SIM_RESOURCE_PATH:}$sim_worlds_share:$sim_worlds_share/models"
    fi
fi

unset tf_shell tf_ros_setup tf_gazebo_setup sim_worlds_share
