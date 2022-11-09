#!/usr/bin/env zsh
source "$HOME/uwarl-robot_configs/scripts/common.sh"
source "$HOME/uwarl-robot_configs/scripts/git_functions.sh"

#################################################################
## Config Check ##
ic  " [Auto-Configuration Begin] ..."
ic  " ==================================================="

if [[ -d "$ROS_CATKIN_WS" ]]; then
    ic " x- $ROS_CATKIN_WS Already Configured"
else
    ic_wrn " [x] $ROS_CATKIN_WS has been created"
    create_catkin_ws
fi

#################################################################
## Auto-Install ##
if [[ $USER = "uwarl" ]]; then
    ic_wrn " Adlink MXE211 Summit PC detected!" 
    ic " ==================================================="
    ic_wrn " [Auto-Configuration] Updating Submodules."
    load_submodules "${SUBMODULES_FOR_SUMMIT[@]}"
    ic_wrn " [Auto-Configuration] Loading Env."
    load_robot_env # load robot env if not loaded

elif [[ $USER = "uwarl-orin" ]]; then
    ic_wrn " Jetson Orin WAM PC detected!"
    ic " ==================================================="
    ic_wrn " [Auto-Configuration] Updating Submodules."
    load_submodules "${SUBMODULES_FOR_WAM[@]}"
    # ic_wrn " [Auto-Configuration] Loading Env."
    # load_robot_env # load robot env if not loaded

else
    ic_wrn " NON-Robot PC User detected! Begin local build:"
    ic " ==================================================="
    ic_wrn " [Auto-Configuration] Updating Submodules."
    load_submodules "${SUBMODULES_FOR_PC[@]}"
    ic_wrn " [Auto-Configuration] Loading Env."
    load_robot_env # load robot env if not loaded

fi
ic " [Auto-Configuration End] ..."

#################################################################
## Auto-Source ##
ic  " Source ~/.zshrc"
source $HOME/.zshrc
ic  "<<< EOF"