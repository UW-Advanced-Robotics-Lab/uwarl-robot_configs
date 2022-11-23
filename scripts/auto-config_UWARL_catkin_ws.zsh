#!/usr/bin/env zsh
source "$HOME/uwarl-robot_configs/scripts/common.sh"
source "$HOME/uwarl-robot_configs/scripts/git_functions.sh"

#################################################################
## Pre-req Check ##
ic  " ==================================================="
ic  " [Checking ROS Pre-requisite] ..."
ic  " ==================================================="
ic  " >>> {Checking your Ubuntu version} "
version=`lsb_release -sc`
relesenum=`grep DISTRIB_DESCRIPTION /etc/*-release | awk -F 'Ubuntu ' '{print $2}' | awk -F ' LTS' '{print $1}'`
ic  " >>> {Your Ubuntu version is: [Ubuntu $version $relesenum]}"
if [ -x "$(command -v rosversion)" ] ; then
    rosversion='rosversion -d'
    ic_wrn " ROS exists!! [ROS version: $rosversion] Pre-requisite met! Continue ..."
else
    ic_err " ERROR::ROS does not exist, maybe you have not yet installed ROS?"
    ### Attempt to install ROS:
    case $version in
        "focal" )
            ic_wrn "Attempting to auto-install ROS Noetic for Jetson!"
            install_ros_noetic
            ic "Continue ..."
        ;;
        *)
        ic_err " >>> {ERROR: Auto-installation of ROS for [Ubuntu $version $relesenum] was not implemented, please install ROS manually.}"
        exit 0
    esac
fi

#################################################################
## Config Check ##
ic  " ==================================================="
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
ic  " ==================================================="
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
    ic_wrn " [Auto-Configuration] Loading Env."
    load_robot_env # load robot env if not loaded

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