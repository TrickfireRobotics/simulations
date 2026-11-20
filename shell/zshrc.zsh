[[ -o interactive ]] || return

# shell config
HISTFILE="$HOME/.zsh_history"
HISTSIZE=10000
SAVEHIST=20000
setopt append_history hist_expire_dups_first hist_ignore_dups hist_ignore_space
setopt hist_reduce_blanks share_history auto_cd auto_pushd pushd_ignore_dups
setopt correct interactive_comments

source "$HOME/simulations/shell/config.sh"

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
