# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# colored GCC warnings and errors
#export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi
# Make sure we are on a serial console (i.e. the device used starts with
# /dev/tty[A-z]), otherwise we confuse e.g. the eclipse launcher which tries do
# use ssh
case $(tty 2>/dev/null) in
        /dev/tty[A-z]*) [ -x /usr/bin/resize_tty ] && /usr/bin/resize_tty >/dev/null;;
esac

source /opt/tros/setup.bash
source /root/dev_ws/install/local_setup.bash
#source /userdata/dev_ws/install/local_setup.bash
#source /opt/tros/setup.bash
echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor
ip=`ifconfig wlan0 | grep -w inet | awk {'print $2'}`
version=2.0.2

# info out
echo -e "IP: \033[32m$ip\033[0m"
echo -e "VERSION: \033[32m$version\033[0m"



alias r3="ros2 launch origincar_base origincar_bringup.launch.py"
alias r4="ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
alias r1="ros2 launch origincar_bringup camera.launch.py"
alias rc="python3 /root/dev_ws/src/controller/controller/controller.py"
alias rf="python3 /root/dev_ws/src/line_follower/line_follower/line_follower_qrcode_nn.py"
alias run_ai="cd /root/dev_ws && ros2 launch dnn_node_example dnn_node_example.launch.py dnn_example_config_file:=./yolov5.config dnn_example_image_width:=1920 dnn_example_image_height:=1080"
alias r2="python3 /root/dev_ws/src/line_follower/line_follower/compress.py"
alias rb="python3 /root/dev_ws/src/line_follower/line_follower/baoli.py"


alias run_ros='function _run_ros() { python3 /root/dev_ws/src/$1/$1/$2; }; _run_ros'


function run() {
    trap 'kill $(jobs -p)' EXIT 
    run_dipan > /dev/null &
    run_ws > /dev/null &
    run_camera > /dev/null &
    run_ros line_follower line_follower_qrcode_nn.py > /dev/null &
    run_ros controller controller.py &
    wait  
}

alias wifi='nmcli device wifi rescan && wifi_connect "ANO_WINWINWIN" "roswinwin"'
