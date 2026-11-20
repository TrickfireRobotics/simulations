[[ $- == *i* ]] || return

# shell config
HISTFILE="$HOME/.bash_history"
HISTSIZE=10000
HISTFILESIZE=20000
HISTCONTROL=ignoreboth:erasedups
shopt -s histappend cmdhist

source "$HOME/simulations/shell/config.sh"

# shell features
if [[ -r /usr/share/bash-completion/bash_completion ]]; then
    source /usr/share/bash-completion/bash_completion
fi

bind '"\e[A": history-search-backward'
bind '"\e[B": history-search-forward'
