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

if [[ -d "$ROS_CATKIN_WS/src" ]]; then
    ic_err " x- $ROS_CATKIN_WS/src Already Configured"
else
    create_catkin_ws
    ic_wrn " [x] $ROS_CATKIN_WS/src has been created"
fi

if [[ -d "$JX_LINUX" ]]; then
    ic_err " x- $JX_LINUX Already Configured"
else
    create_JX_Linux
fi

source_all_common_configs

#################################################################
## Auto-Install ##
if [[ $USER = "uwarl" ]] && [[ $LOCAL_PC_IP = "$ROS_SUMMIT_IP" ]]; then
    ic " - Adlink MXE211 Summit PC detected!" 
    ic " > Loading SUMMIT workspace submodules:"
    load_submodules "${SUBMODULES_FOR_SUMMIT[@]}"
    load_common 
    # install drivers :
    install_pcan_if_not

elif [[ $USER = "deck" ]]; then
    ic " - Steam Deck Controller detected!" 
    ic " > Loading Deck workspace submodules:"
    load_submodules "${SUBMODULES_FOR_DECK[@]}"
    load_common 
    ic " > Create a controller app @ the desktop!"
    cp $HOME/uwarl-robot_configs/deck/uwarl_controller.desktop $HOME/Desktop
    cp $HOME/uwarl-robot_configs/deck/uwarl_rviz.desktop $HOME/Desktop

elif [[ $USER = "uwarl-orin" ]] && [[ $LOCAL_PC_IP = "$ROS_WAM_IP" ]]; then
    ic " - Jetson Orin WAM PC detected!"
    ic " > Loading WAM workspace submodules:"
    load_submodules "${SUBMODULES_FOR_WAM[@]}"
    load_common 
    # install drivers :
    install_misc_utilities # misc apt 
    install_pcan_if_not NETDEV_SUPPORT
    install_libbarrett_if_not
    install_librealsense_if_not # for Intel Sensors
    install_dlink_dongle # for dlink dongle
    install_jetson_utilities # for jetson utilities

else
    ic " - NON-Robot PC User detected! Begin local build:"
    if [[ $USER = "parallels" ]] && [[ $LOCAL_PC_IP = "$ROS_JX_PARALLEL_PC_IP" ]]; then
        ic " > Loading parallels workspace submodules:"
        load_submodules "${SUBMODULES_FOR_JX_PARALLEL[@]}"
    elif [[ $USER = "oem" ]] && [[ $LOCAL_PC_IP = "$ROS_JX_OEM_PC_IP" ]]; then
        ic " > Loading parallels workspace submodules:"
        install_misc_utilities # misc apt 
        install_libbarrett_if_not
        install_librealsense_if_not # for Intel Sensors
        load_submodules "${SUBMODULES_FOR_JX_OEM[@]}"
    elif [[ $USER = "uwarl-laptop-4" ]] && [[ $LOCAL_PC_IP = "$ROS_P51_LENOVO_PC_IP" ]]; then
        ic " > Loading parallels workspace submodules:"
        load_submodules "${SUBMODULES_FOR_P51_LENOVO[@]}"
    elif [[ $USER = "uwarl-laptop-3" ]] && [[ $LOCAL_PC_IP = "$ROS_P50s_LENOVO_PC_IP" ]]; then
        ic " > Loading parallels workspace submodules:"
        load_submodules "${SUBMODULES_FOR_P50s_LENOVO[@]}"
    else
        ic " > Loading default workspace submodules:"
        load_submodules "${SUBMODULES_FOR_PC_DEFAULT[@]}"
    fi
    load_common 
fi

#################################################################
## Auto-Source ##
# ic_source $HOME/.zshrc "Auto-Source Zshrc"
ic  "\n\n<<< EOF"
