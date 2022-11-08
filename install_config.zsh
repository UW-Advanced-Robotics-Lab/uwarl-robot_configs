#!/usr/bin/env zsh

#################################################################
## USER PARAM: ##
# export UWARL_catkin_ws_branch="waterloo_steel/adlink-mxe211-melodic/dev/v1.0.0"
export UWARL_catkin_ws_branch="waterloo_steel/universal/local-pc/develop/jx"
SUBMODULES_FOR_PC=(
    "multimap_server_msgs"
    "system_monitor"
    "uwarl-barrett-ros-pkg"
    "uwarl-multimap_server"
    "uwarl-robot_localization_utils"
    # "uwarl-robotnik_base_hw" # not needed for simulation
    "uwarl-robotnik_msgs"
    "uwarl-robotnik_sensors"
    "uwarl-summit_xl_common"
    "uwarl-summit_xl_robot"
    "waterloo_steel"
)
SUBMODULES_FOR_SUMMIT=(
    "multimap_server_msgs"
    "system_monitor"
    # "uwarl-barrett-ros-pkg" # not needed for summit?
    "uwarl-multimap_server"
    "uwarl-robot_localization_utils"
    "uwarl-robotnik_base_hw"
    "uwarl-robotnik_msgs"
    "uwarl-robotnik_sensors"
    "uwarl-summit_xl_common"
    "uwarl-summit_xl_robot"
    # "waterloo_steel" not needed for summit?
)

#################################################################
## VAR ##
export ROS_CATKIN_WS="$HOME/UWARL_catkin_ws"
export UWARL_SUMMIT_ROS_CONFIGS="$HOME/uwarl-robot_configs/summit/summitxl_ros_config.zsh"

## FUNC ##
#### TABLE OF COLORS #### #### #### #### #### #### ####
#      Black        0;30     Dark Gray     1;30       #
#      Red          0;31     Light Red     1;31       #
#      Green        0;32     Light Green   1;32       #
#      Brown/Orange 0;33     Yellow        1;33       #
#      Blue         0;34     Light Blue    1;34       #
#      Purple       0;35     Light Purple  1;35       #
#      Cyan         0;36     Light Cyan    1;36       #
#      Light Gray   0;37     White         1;37       #
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color
ic () {
    echo -e "${CYAN}[UWARL-Config-install]${NC} ${YELLOW} $1 ${NC}"
}

load_submodules(){
    # update submodules:
    ic "Loading Submodules Recursively uwarl-robot_configs @ $ROS_CATKIN_WS/src"

    list_of_modules=("$@")
    total=${#list_of_modules[@]}
    i=0
    for module in "${list_of_modules[@]}"; do
        i=$(( i + 1 ))
        ic "    > [$i/$total] - Loading submodule @ $module"
        git submodule update --init $module
    done
    ic " Done loading submodules."
}

create_catkin_ws(){
    # Create catkin workspace
    ic "Creating catkin workspace @ $ROS_CATKIN_WS"
    cd $HOME
    mkdir -p $ROS_CATKIN_WS/src
    # clone --> to the src/
    ic "Cloning uwarl-robot_configs @ $ROS_CATKIN_WS/src"
    git clone git@github.com:UW-Advanced-Robotics-Lab/SUMMIT-catkin_ws.git $ROS_CATKIN_WS/src
    # checkout branch:
    ic "Checking out branch $UWARL_catkin_ws_branch @ $ROS_CATKIN_WS/src"
    cd $ROS_CATKIN_WS/src && git checkout $UWARL_catkin_ws_branch
}

load_robot_env() {
    ic " Sourcing robot params @ $UWARL_SUMMIT_ROS_CONFIGS"
    echo "source $UWARL_SUMMIT_ROS_CONFIGS" >> "$HOME/.zshrc"
}

#################################################################
## Config Check ##
ic  " [Auto-Configuration Begin] ..."
if [[ -d "$ROS_CATKIN_WS" ]]; then
    ic " [Auto-Configuration] $ROS_CATKIN_WS Already Configured"
    ic " [Auto-Configuration End] ..."
else
    ic " [Auto-Configuration] will now begin"
    #################################################################
    ## Auto-Install ##
    if [[ $USER = "uwarl" ]]; then
        ic " Adlink MXE211 Summit PC detected!" 
        ic " ==================================================="
        load_robot_env

    elif [[ $USER = "uwarl-orin" ]]; then
        ic " Jetson Orin WAM PC detected!"
        ic " ==================================================="
        ic " TODO"

    else
        ic " NON-Robot PC User detected! Begin local build:"
        ic " ==================================================="
        create_catkin_ws
        load_submodules "${SUBMODULES_FOR_PC[@]}"
        load_robot_env
    fi
    ic " [Auto-Configuration End] ..."
fi
ic  " Source ~/.zshrc"
source $HOME/.zshrc
ic  "<<< EOF"