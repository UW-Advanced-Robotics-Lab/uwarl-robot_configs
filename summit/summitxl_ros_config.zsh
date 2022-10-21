#!/usr/bin/env zsh

### ROS RELATED ###
## VAR ##
export ROS_CATKIN_WS="$HOME/UWARL_catkin_ws"
export UWARL_CONFIGS="$HOME/uwarl-robot_configs/summit"

## ROS Env ##
echo "[UWARL-ROS-Config] Sourcing ros ws @ $ROS_CATKIN_WS"
source /opt/ros/melodic/setup.zsh
source $ROS_CATKIN_WS/devel/setup.zsh
source $UWARL_CONFIGS/summitxl_params.env

## ROS Networks ##
#export ROS_MASTER_URI=http://192.168.1.11:11311/
#export ROS_IP=192.168.1.11

echo "[UWARL-ROS-Config] ROS MASTER @ " $ROS_MASTER_URI
echo "[UWARL-ROS-Config] ROS MASTER @ " $ROS_IP

## Alias Commnads ##
## kill processor:
alias kill-ros="ps aux  | grep -e ros | awk '{print $2}' | xargs -i -exec kill -9 {}"
alias source_ws="source $ROS_CATKIN_WS/devel/setup.zsh"
alias git-status-all="find . -maxdepth 1 -type d -execdir sh -c 'cd {}; echo \"[Git {}]\"; git status; git remote -v; echo \"----------\n\"' \;"
alias git-push-all="find . -maxdepth 1 -type d -execdir sh -c 'cd {}; echo \"[Pushing {}]\"; git push; echo \"----------\n\"' \;"
