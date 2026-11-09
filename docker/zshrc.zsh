[[ -o interactive ]] || return

typeset -U path PATH
path=("$HOME/.local/bin" $path)

# ---------- shell behavior ----------
HISTFILE="$HOME/.zsh_history"
HISTSIZE=10000
SAVEHIST=20000
setopt append_history hist_expire_dups_first hist_ignore_dups hist_ignore_space
setopt hist_reduce_blanks share_history auto_cd auto_pushd pushd_ignore_dups
setopt correct interactive_comments

# ---------- locate repo ----------
_tf_find_repo() {
    local directory
    for directory in "$PWD" "$HOME/simulations" "/workspaces/simulations" "/workspace/simulations"; do
        if [[ -d "$directory/gazebo" && -f "$directory/pyproject.toml" ]]; then
            print -r -- "$directory"
            return 0
        fi
    done

    directory="$(git rev-parse --show-toplevel 2>/dev/null)"
    if [[ -n "$directory" && -d "$directory/gazebo" && -f "$directory/pyproject.toml" ]]; then
        print -r -- "$directory"
        return 0
    fi

    return 1
}

if TF_REPO_DIR="$(_tf_find_repo)"; then
    export TF_REPO_DIR
    export TF_GAZEBO_DIR="$TF_REPO_DIR/gazebo"
fi

# ---------- venv ----------
if [[ -f "${TF_REPO_DIR}/.venv/bin/activate" ]]; then
    source "${TF_REPO_DIR}/.venv/bin/activate"
fi

# ---------- ROS/Gazebo environment ----------
tf_source_env() {
    if [[ -f /opt/ros/jazzy/setup.zsh ]]; then
        source /opt/ros/jazzy/setup.zsh
    fi

    if [[ -n "${TF_GAZEBO_DIR:-}" && -f "$TF_GAZEBO_DIR/install/setup.zsh" ]]; then
        source "$TF_GAZEBO_DIR/install/setup.zsh"

        local sim_worlds_share="$TF_GAZEBO_DIR/install/sim_worlds/share/sim_worlds"
        if [[ -d "$sim_worlds_share/worlds" ]]; then
            export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:+$GZ_SIM_RESOURCE_PATH:}$sim_worlds_share"
        fi
    fi
}

if [[ -z "${TF_ENV_SOURCED:-}" ]]; then
    tf_source_env
    export TF_ENV_SOURCED=1
fi

# ---------- display ----------
if [[ -n "${FORCE_VNC:-}" ]]; then
    export DISPLAY="${FORCE_VNC_DISPLAY:-:77}"
elif [[ -n ${DISPLAY:-} && ${DISPLAY} =~ '^[0-9]+(\.[0-9]+)?$' ]]; then
    export DISPLAY=":${DISPLAY}"
fi

# ---------- prompt ----------
autoload -Uz add-zsh-hook vcs_info
zstyle ':vcs_info:git:*' formats '%F{75}%b%f'
zstyle ':vcs_info:git:*' actionformats '%F{75}%b|%a%f'

_tf_prompt() {
    vcs_info
    PROMPT="%F{213}%n%f %F{244}at%f %F{39}%~%f ${vcs_info_msg_0_:+%F{244}on%f $vcs_info_msg_0_}\n%(?.%F{70}❯.%F{196}❯)%f "
}
add-zsh-hook precmd _tf_prompt

# ---------- completion, history, and highlighting ----------
autoload -Uz compinit
compinit
zstyle ':completion:*' menu select
zstyle ':completion:*' matcher-list 'm:{a-zA-Z}={A-Za-z}'
zstyle ':completion:*' list-colors "${(s.:.)LS_COLORS}"

source /usr/share/zsh-autosuggestions/zsh-autosuggestions.zsh
ZSH_AUTOSUGGEST_HIGHLIGHT_STYLE='fg=244'
ZSH_AUTOSUGGEST_STRATEGY=(history completion)

bindkey '^[[A' history-beginning-search-backward
bindkey '^[[B' history-beginning-search-forward

source /usr/share/zsh-syntax-highlighting/zsh-syntax-highlighting.zsh

# ---------- navigation ----------
if [[ "${TF_AUTO_CD_REPO:-1}" = 1 && -n "${TF_REPO_DIR:-}" && "$PWD" = "$HOME" ]]; then
    cd "$TF_REPO_DIR" || true
fi

# ---------- aliases ----------
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
