case $- in
*i*) ;;
*) return ;;
esac

case ":${PATH:-}:" in
*":$HOME/.local/bin:"*) ;;
*) export PATH="$HOME/.local/bin${PATH:+:$PATH}" ;;
esac

# project environment
source "$HOME/simulations/shell/source.sh" || echo "sourcing ROS & Gazebo failed"
source "$HOME/simulations/docker/.env" || echo "sourcing docker envs failed"

# display setup
if [[ -n "${FORCE_VNC:-}" ]]; then
    export DISPLAY="${FORCE_VNC_DISPLAY:-:77}"
elif [[ -n ${DISPLAY:-} && ${DISPLAY} =~ '^[0-9]+(\.[0-9]+)?$' ]]; then
    export DISPLAY=":${DISPLAY}"
fi

# aliases
alias c='clear'
alias ll='ls -alF --color=auto'
alias la='ls -A --color=auto'
alias l='ls -CF --color=auto'

alias rs='tf_source_env'
alias rsource='tf_source_env'

alias rtl='ros2 topic list'
alias rte='ros2 topic echo'
alias gtl='gz topic -l'
alias gte='gz topic -e -t'

alias sg='sim gazebo'
alias sc='sim chrono'
