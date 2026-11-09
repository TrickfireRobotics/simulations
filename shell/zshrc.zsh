[[ -o interactive ]] || return

typeset -U path PATH
path=("$HOME/.local/bin" $path)

# shell config
HISTFILE="$HOME/.zsh_history"
HISTSIZE=10000
SAVEHIST=20000
setopt append_history hist_expire_dups_first hist_ignore_dups hist_ignore_space
setopt hist_reduce_blanks share_history auto_cd auto_pushd pushd_ignore_dups
setopt correct interactive_comments

# source
source "$HOME/simulations/shell/source.sh" || echo "sourcing ROS & Gazebo failed"

# display setup
if [[ -n "${FORCE_VNC:-}" ]]; then
    export DISPLAY="${FORCE_VNC_DISPLAY:-:77}"
elif [[ -n ${DISPLAY:-} && ${DISPLAY} =~ '^[0-9]+(\.[0-9]+)?$' ]]; then
    export DISPLAY=":${DISPLAY}"
fi

# shell features
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
source $HOME/simulations/shell/theme.zsh || true

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
