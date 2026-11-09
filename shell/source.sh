#!/usr/bin/env zsh

# source ROS
if [[ -f /opt/ros/jazzy/setup.zsh ]]; then
    source /opt/ros/jazzy/setup.zsh || echo "sourcing ROS failed"
fi

# source gazebo
if [[ -n "${TF_GAZEBO_DIR:-}" && -f "$TF_GAZEBO_DIR/install/setup.zsh" ]]; then
    source "$TF_GAZEBO_DIR/install/setup.zsh"

    sim_worlds_share="$TF_GAZEBO_DIR/install/sim_worlds/share/sim_worlds"
    if [[ -d "$sim_worlds_share/worlds" ]]; then
        export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:+$GZ_SIM_RESOURCE_PATH:}$sim_worlds_share"
    fi
fi
