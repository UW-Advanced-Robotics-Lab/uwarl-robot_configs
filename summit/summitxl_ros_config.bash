#!/usr/bin/env bash

### ROS RELATED ###
## VAR ##
export ROS_CATKIN_WS="$HOME/UWARL_catkin_ws"
export UWARL_CONFIGS="$HOME/uwarl-robot_configs/summit"

## ROS Env ##
echo "[UWARL-ROS-Config] Sourcing ros ws @ $ROS_CATKIN_WS"
source /opt/ros/melodic/setup.bash
source $ROS_CATKIN_WS/devel/setup.bash
source $UWARL_CONFIGS/summitxl_params.env

## ROS Networks ##
#export ROS_MASTER_URI=http://192.168.0.200:11311/
#export ROS_IP=192.168.0.120

echo "[UWARL-ROS-Config] ROS MASTER @ " $ROS_MASTER_URI
echo "[UWARL-ROS-Config] ROS MASTER @ " $ROS_IP

## Alias Commnads ##
## kill processor:
alias kill-ros="ps aux  | grep -e ros | awk '{print $2}' | xargs -i -exec kill -9 {}"
alias source_ws="source $ROS_CATKIN_WS/devel/setup.zsh"


