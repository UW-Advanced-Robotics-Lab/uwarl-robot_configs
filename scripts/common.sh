#!/usr/bin/env zsh
#################################################################
## USER PARAM: ##
export UWARL_catkin_ws_branch="waterloo_steel/universal/ros1/develop/jx"
# export UWARL_catkin_ws_branch="waterloo_steel/universal/ros1/feature/wam_node"

SUBMODULES_FOR_PC=(
    ## SUMMIT Side:
    "multimap_server_msgs"
    "system_monitor"
    "uwarl-multimap_server"
    "uwarl-robot_localization_utils"
    # "uwarl-robotnik_base_hw" # not needed for simulation !  # [x86_64 only]
    "uwarl-robotnik_msgs"
    "uwarl-robotnik_sensors"
    "uwarl-summit_xl_common"
    "uwarl-summit_xl_robot"
    "waterloo_steel"
    ## WAM Side:
    # "uwarl-barrett_wam_hw"  # : Enabled for local dev.  # [x86_64, aarch64/arm64]
    # "uwarl-barrett_wam_msgs"
    "uwarl-barrett-ros-pkg" # [DEPRECATED]
    # "uwarl-zed_ros_wrapper"
)
SUBMODULES_FOR_SUMMIT=(
    ## SUMMIT Side:
    "multimap_server_msgs"
    "system_monitor"
    "uwarl-multimap_server"
    "uwarl-robot_localization_utils"
    "uwarl-robotnik_base_hw" # [waterloo_steel/adlink-mxe211-melodic/main] # [x86_64 only]
    "uwarl-robotnik_msgs"
    "uwarl-robotnik_sensors"
    "uwarl-summit_xl_common" # [waterloo_steel/adlink-mxe211-melodic/main]
    "uwarl-summit_xl_robot"  # [waterloo_steel/adlink-mxe211-melodic/main]
    "waterloo_steel"         # [universal/ros1/main]
    ## WAM Side:
    # "uwarl-barrett_wam_hw"  # [NOT NEEDED]
    # "uwarl-barrett_wam_msgs"
    # "uwarl-barrett-ros-pkg" # [DEPRECATED]
)
SUBMODULES_FOR_WAM=(
    ## SUMMIT Side:
    # "waterloo_steel" not needed for WAM?
    ## WAM Side:
    # "uwarl-barrett_wam_hw"   # [x86_64, aarch64/arm64]
    # "uwarl-barrett_wam_msgs"
    "uwarl-barrett-ros-pkg" # [DEPRECATED]
    "uwarl-zed_ros_wrapper"
)

#################################################################
## NETWORK PARAM: ##
export ROS_SUMMIT_IP=192.168.1.11
export ROS_SUMMIT_HOSTNAME=192.168.1.11
export ROS_SUMMIT_MASTER_URI=http://localhost:11311/
export ROS_SUMMIT_DISTRO=melodic

export ROS_WAM_IP=192.168.1.10
export ROS_WAM_HOSTNAME=192.168.1.10
export ROS_WAM_MASTER_URI=http://$ROS_SUMMIT_IP:11311/
export ROS_WAM_DISTRO=noetic

export ROS_PC_IP=192.168.1.101
export ROS_PC_HOSTNAME=192.168.1.101
export ROS_PC_MASTER_URI=http://$ROS_SUMMIT_IP:11311/ # <--- change it to localhost for local roscore
export ROS_PC_DISTRO=noetic

#################################################################
## VAR ##
export DISPLAY=:0 # make sure it only display on the default screen

## FILE PATH ##
export COMMON_ROBOT_CONFIGS="$HOME/uwarl-robot_configs/scripts/common.sh"
export OUTPUT_STATUS_LOG_DIR="$ROS_CATKIN_WS/src/git_status_ws.log"

## FILE DIRECTORY ##
export ROS_CATKIN_WS="$HOME/UWARL_catkin_ws"
export UWARL_CONFIGS="$HOME/uwarl-robot_configs"
export JX_LINUX="$HOME/JX_Linux"

export UWARL_SUMMIT_SPECIFIC="$HOME/uwarl-robot_configs/summit"
export UWARL_WAM_SPECIFIC="$HOME/uwarl-robot_configs/wam"

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
function ic_title () {
    echo " "
    ic " ==================================================="
    ic " [ $1 ]"
    ic " ==================================================="
}
function ic_err () {
    echo -e "${CYAN}[UWARL-Robot_Config]${NC} ${RED} $1 ${NC}"
}
function ic_wrn () {
    echo -e "${CYAN}[UWARL-Robot_Config]${NC} ${YELLOW} $1 ${NC}"
}
function ic_log () {
    echo "[UWARL-Robot_Config] $1" >> $OUTPUT_STATUS_LOG_DIR
}
function ic_bind_cmd () {
    echo -e "${CYAN}[UWARL-Robot_Config]   Aliasing \`${NC}${YELLOW}\$ $1${NC}\` command @ $UWARL_CONFIGS/scripts/shortcuts.sh"
    alias $1=$2
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
function cat_ros_env() {
    ic_wrn " [ROS CONFIG]: "
    ic     "    - ROS HOST   : $ROS_HOSTNAME"
    ic     "    - ROS MASTER : $ROS_MASTER_URI"
    ic     "    - ROS IP     : $ROS_IP"
}

function source_ros() {
    ic_title "ROS" "Setting up ROS Master IP:"
    if [[ $USER = "uwarl" ]]; then
        ic " - Adlink MXE211 Summit PC detected!" 
        export ROS_IP=$ROS_SUMMIT_IP
        export ROS_HOSTNAME=$ROS_SUMMIT_HOSTNAME
        export ROS_MASTER_URI=$ROS_SUMMIT_MASTER_URI
        export ROS_DISTRO=$ROS_SUMMIT_DISTRO
    elif [[ $USER = "uwarl-orin" ]]; then
        ic " - Jetson Orin WAM PC detected!"
        export ROS_IP=$ROS_WAM_IP
        export ROS_HOSTNAME=$ROS_WAM_HOSTNAME
        export ROS_MASTER_URI=$ROS_WAM_MASTER_URI
        export ROS_DISTRO=$ROS_WAM_DISTRO
    else
        ic " - NON-Robot PC User detected!"
        export ROS_IP=$ROS_PC_IP
        export ROS_HOSTNAME=$ROS_PC_HOSTNAME
        export ROS_MASTER_URI=$ROS_PC_MASTER_URI
        export ROS_DISTRO=$ROS_PC_DISTRO
    fi

    ic_title "Sourcing $ROS_DISTRO + $ROS_CATKIN_WS:"
    source /opt/ros/$ROS_DISTRO/setup.zsh
    source $ROS_CATKIN_WS/devel/setup.zsh
}

function source_shortcuts() {
    ic_title " Sourcing shortcuts @ $UWARL_CONFIGS/scripts/shortcuts.sh"
    source "$UWARL_CONFIGS/scripts/shortcuts.sh"
}

function source_robot_env() {
    ic_title "Sourcing env @ $UWARL_SUMMIT_SPECIFIC/summitxl_params.env"
    source $UWARL_SUMMIT_SPECIFIC/summitxl_params.env
}

function source_all_common_configs() {
    # action:
    source_robot_env
    source_ros
    source_shortcuts
    # report:
    ic_title "Print Environment Variables: "
    cat_summit_env
    cat_ros_env
}