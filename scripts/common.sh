#!/usr/bin/env zsh
#################################################################
## USER PARAM: ##
export UWARL_catkin_ws_branch="waterloo_steel/adlink-mxe211-melodic/dev/v1.0.0"
# export UWARL_catkin_ws_branch="waterloo_steel/universal/local-pc/develop/jx"
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
SUBMODULES_FOR_WAM=(
    "uwarl-barrett-ros-pkg"
    # "waterloo_steel" not needed for summit?
)

#################################################################
## VAR ##
export ROS_CATKIN_WS="$HOME/UWARL_catkin_ws"
export UWARL_CONFIGS="$HOME/uwarl-robot_configs"
export UWARL_SUMMIT_ROS_CONFIGS="$HOME/uwarl-robot_configs/summit/summitxl_ros_config.zsh"
export OUTPUT_STATUS_LOG_DIR="$ROS_CATKIN_WS/src/git_status_ws.log"

## CONST ##
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
BLUE='\033[0;34m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

## FUNC ##
function ic () {
    echo -e "${CYAN}[UWARL-Robot_Config]${NC} ${BLUE} $1 ${NC}"
}
function ic_err () {
    echo -e "${CYAN}[UWARL-Robot_Config]${NC} ${RED} $1 ${NC}"
}
function ic_wrn () {
    echo -e "${CYAN}[UWARL-Robot_Config]${NC} ${YELLOW} $1 ${NC}"
}


function cat_summit_env() {
    ic_wrn " [SUMMIT ENV CONFIG]: "
    ic     "    - ROBOT_ID                : $ROBOT_ID"
    ic     "    - ROBOT_XACRO             : $ROBOT_XACRO"
    ic     "    - ROBOT_XACRO_PACKAGE     : $ROBOT_XACRO_PACKAGE"
    ic     "    - ROBOT_GEARBOX           : $ROBOT_GEARBOX"
    ic     "    - ROBOT_KINEMATICS        : $ROBOT_KINEMATICS"
    ic     "    - ROBOT_WHEEL_DIAMETER    : $ROBOT_WHEEL_DIAMETER"
    ic     "    - ROBOT_TRACK_WIDTH       : $ROBOT_TRACK_WIDTH"
    ic     "    - ROBOT_WHEEL_BASE        : $ROBOT_WHEEL_BASE"
    ic     "    - ROBOT_HAS_ENCODER       : $ROBOT_HAS_ENCODER"
    ic     "    - ROBOT_HAS_FRONT_LASER   : $ROBOT_HAS_FRONT_LASER"
    ic     "    - ROBOT_FRONT_LASER_MODEL : $ROBOT_FRONT_LASER_MODEL"
    ic     "    - ROBOT_FRONT_LASER_PORT  : $ROBOT_FRONT_LASER_PORT"
    ic     "    - ROBOT_FRONT_LASER_IP    : $ROBOT_FRONT_LASER_IP"
    ic     "    - ROBOT_PAD_MODEL         : $ROBOT_PAD_MODEL"
}
