#!/usr/bin/env zsh
#################################################################
### ROS RELATED ###
## VAR ##
export DISPLAY=:0 # make sure it only display on the default screen
export ROS_CATKIN_WS="$HOME/UWARL_catkin_ws"
export UWARL_CONFIGS="$HOME/uwarl-robot_configs"
export UWARL_SUMMIT_CONFIGS="$HOME/uwarl-robot_configs/summit"
ic () {
    echo "[UWARL-ROS-Config] $1"
}

#################################################################
ic " === BEGIN ==="
## ROS Env ##
ic " Sourcing ros ws @ $ROS_CATKIN_WS"
source /opt/ros/$ROS_DISTRO/setup.zsh
source $ROS_CATKIN_WS/devel/setup.zsh
ic " Sourcing robot params @ $UWARL_SUMMIT_CONFIGS"
source $UWARL_SUMMIT_CONFIGS/summitxl_params.env

## ROS Networks ##
#export ROS_MASTER_URI=http://192.168.1.11:11311/
#export ROS_IP=192.168.1.11

ic " ROS MASTER @ $ROS_MASTER_URI"
ic " ROS MASTER @ $ROS_IP"

## Alias Commnads ##
## ROS src:
alias source_ws="source $ROS_CATKIN_WS/$ROS_DISTRO/devel/setup.zsh"

## Custom Shortcuts:
ic " Sourcing shortcuts @ $UWARL_CONFIGS/scripts/shortcuts.sh"
source "$UWARL_CONFIGS/scripts/shortcuts.sh"

ic " Sourcing \`\$ git_status_ws\` command @ $UWARL_CONFIGS/scripts/git_status_ws.sh"
alias git_status_ws="zsh $UWARL_CONFIGS/scripts/git_status_ws.sh"

ic " === COMPLETE ==="