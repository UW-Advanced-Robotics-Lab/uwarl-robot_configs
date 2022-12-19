#!/usr/bin/env zsh
source "$HOME/uwarl-robot_configs/scripts/common.sh"
source "$HOME/uwarl-robot_configs/scripts/git_functions.sh"

#################################################################
## Pre-req Check ##
ic_title  "Checking ROS Pre-requisite ..."
ic  " >>> {Checking your Ubuntu version} "
version=`lsb_release -sc`
relesenum=`grep DISTRIB_DESCRIPTION /etc/*-release | awk -F 'Ubuntu ' '{print $2}' | awk -F ' LTS' '{print $1}'`
ic  " >>> {Your Ubuntu version is: [Ubuntu $version $relesenum]}"
if [[ -x "$(command -v rosversion)" ]] ; then
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
ic_title  "Auto-Configuration Begin ..."

if [[ -d "$ROS_CATKIN_WS" ]]; then
    ic_err " x- $ROS_CATKIN_WS Already Configured"
else
    create_catkin_ws
    ic_wrn " [x] $ROS_CATKIN_WS has been created"
fi

if [[ -d "$JX_LINUX" ]]; then
    ic_err " x- $JX_LINUX Already Configured"
else
    create_JX_Linux
fi

#################################################################
## Auto-Install ##
if [[ $USER = "uwarl" ]]; then
    ic " - Adlink MXE211 Summit PC detected!" 
    load_submodules "${SUBMODULES_FOR_SUMMIT[@]}"
    load_common 
    # install drivers :
    install_pcan_if_not

elif [[ $USER = "uwarl-orin" ]]; then
    ic " - Jetson Orin WAM PC detected!"
    load_submodules "${SUBMODULES_FOR_WAM[@]}"
    load_common 
    # install drivers :
    install_pcan_if_not NETDEV_SUPPORT
    install_libbarrett_if_not
    install_librealsense_if_not # for Intel Sensors
else
    ic " - NON-Robot PC User detected! Begin local build:"
    load_submodules "${SUBMODULES_FOR_PC[@]}"
    load_common 
fi

#################################################################
## Auto-Source ##
ic_title  "Source ~/.zshrc"
source $HOME/.zshrc
ic  "\n\n<<< EOF"