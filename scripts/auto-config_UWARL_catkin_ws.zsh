#!/usr/bin/env zsh
source "$HOME/uwarl-robot_configs/scripts/common.sh"
source "$HOME/uwarl-robot_configs/scripts/git_functions.sh"

#################################################################
## Pre-req Check ##
ic_title  "Checking ROS Pre-requisite ..."
ic  " >>> {Checking your Ubuntu version} "
ic  " >>> {Your Ubuntu version is: [Ubuntu $LOCAL_DISTRIB_CODENAME $LOCAL_DISTRIB_DESCRIPTION]}"
if [[ -x "$(command -v rosversion)" ]] ; then
    rosversion='rosversion -d'
    ic_wrn " ROS exists!! [ROS version: $rosversion] Pre-requisite met! Continue ..."
else
    ic_err " ERROR::ROS does not exist, maybe you have not yet installed ROS?"
    ### Attempt to install ROS:
    case $LOCAL_DISTRIB_CODENAME in
        "focal" )
            ic_wrn "Attempting to auto-install ROS Noetic for Jetson!"
            install_ros_noetic
            ic "Continue ..."
        ;;
        *)
        ic_err " >>> {ERROR: Auto-installation of ROS for [Ubuntu $LOCAL_DISTRIB_CODENAME $LOCAL_DISTRIB_DESCRIPTION] was not implemented, please install ROS manually.}"
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
ic_title "Auto-Installation Begin ..."
case $UWARL_ROBOT_PC_NAME in
    "ADLINK_MXE211_SUMMIT" )
        ic " - Adlink MXE211 Summit PC detected!" 
        ic " > Loading SUMMIT workspace submodules:"
        load_submodules "${SUBMODULES_FOR_SUMMIT[@]}"
        # install drivers :
        install_pcan_if_not
    ;;
    "JETSON_ORIN_WAM")
        ic " - Jetson Orin WAM PC detected!"
        ic " > Loading WAM workspace submodules:"
        load_submodules "${SUBMODULES_FOR_WAM[@]}"
        # install drivers :
        install_misc_utilities # misc apt 
        install_pcan_if_not NETDEV_SUPPORT
        install_libbarrett_if_not
        install_librealsense_if_not # for Intel Sensors
        # install_dlink_dongle # for dlink dongle # [April 21, 2022] No need, as we already have internal wifi card working
        install_jetson_utilities # for jetson utilities
    ;;
    "STEAM_DECK_CONTROLLER")
        ic " - Steam Deck Controller detected!" 
        ic " > Loading Deck workspace submodules:"
        load_submodules "${SUBMODULES_FOR_DECK[@]}"
        ic " > Create a controller app @ the desktop!"
        cp $HOME/uwarl-robot_configs/deck/uwarl_controller.desktop $HOME/Desktop
        cp $HOME/uwarl-robot_configs/deck/uwarl_rviz.desktop $HOME/Desktop
    ;;
    "PARALLELS_VM_JACK")
        load_submodules "${SUBMODULES_FOR_JX_PARALLEL[@]}"
    ;;
    "ARL_DESKTOP_ARNAB")
        load_submodules "${SUBMODULES_FOR_AJ_DESKTOP[@]}"
        # install drivers :
        install_misc_utilities # misc apt
        install_pcan_if_not NETDEV_SUPPORT
        install_libbarrett_if_not
    ;;
    "JX_DESKTOP_JACK")
        load_submodules "${SUBMODULES_FOR_JX_DESKTOP[@]}"
        install_misc_utilities # misc apt 
        install_libbarrett_if_not
        install_librealsense_if_not # for Intel Sensors
    ;;
    "UWARL_LAPTOP_4_JEONGWOO")
        load_submodules "${SUBMODULES_FOR_P51_LENOVO[@]}"
    ;;
    "UWARL_LAPTOP_3_SIMON")
        load_submodules "${SUBMODULES_FOR_P50s_LENOVO[@]}"
    ;;
    "UWARL_LAPTOP_3_TIM")
        load_submodules "${SUBMODULES_FOR_TIM[@]}"
        install_misc_utilities # misc apt 
    ;;
    # [DEFAULT]:
    *)
        ic_err " >>> ERROR: Auto-installation Is Empty, Unknown PC!"
        ic_wrn "       (1) Please Manually Register your PC in the \"common.sh\" script."
        ic_wrn "       (2) Please Manually Register your PC in the \"auto-config_UWARL_catkin_ws.zsh\" script."
        ic_err " >>> ERROR: Abort Workspace Setup!"
        ic_wrn "       (3) Please reattempt with \"update_ws\" after the configurations are done."
        # END
esac

ic_title "Auto-Installation End ..."
load_common

#################################################################
## Auto-Source ##
# ic_source $HOME/.zshrc "Auto-Source Zshrc"
ic  "\n\n<<< EOF"
