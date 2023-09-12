#!/usr/bin/env zsh
#################################################################
## USER PARAM: ##
export UWARL_catkin_ws_branch="universal/ros1/arnab/session-jun-2023"

#################################################################
#    ## SUMMIT Side:
#       "multimap_server_msgs"              # TODO: need to deal with mapping
#       "uwarl-multimap_server"             # TODO: need to deal with mapping
#       "uwarl-robot_localization_utils"    # TODO: need to deal with mapping
#       "system_monitor"           # ["robotnik"]
#       "uwarl-robotnik_base_hw"   # [waterloo_steel/adlink-mxe211-melodic/main] # [x86_64 only]
#       "uwarl-robotnik_msgs"
#       "uwarl-robotnik_sensors"
#       "uwarl-summit_xl_common"   # X-[waterloo_steel/universal/main] TODO: need to deal with mapping,
#       "uwarl-summit_xl_robot"    # [waterloo_steel/adlink-mxe211-melodic/main]
#       "waterloo_steel"           # [universal/ros1/main]
#    ## WAM Side:
#       "uwarl-barrett_wam_hw"     # [x86_64, aarch64/arm64]
#       "uwarl-barrett_wam_msgs"
#       "uwarl-realsense_ros"      # [L515 Support]
#       "uwarl-barrett-ros-pkg"    # [DEPRECATED]
#       "uwarl-zed_ros_wrapper"    # [No longer used]
#    ## Vicon Tracker:
#       "uwarl-vicon_bridge"
#################################################################

### SUBMODULES Definitons for Workspace: ###
## NOTE on How to register your PC and Workspace Submodules:
#   (1) Please add your submodules here
#   (2) Please register your PC into `function source_ros()` inside `common.sh``
#   (3) Please modify `auto-config_UWARL_catkin_ws.zsh` to load your submodules

# $USER = "deck":
SUBMODULES_FOR_DECK=(
    ## SUMMIT Side:
    "uwarl-robotnik_msgs"
    "uwarl-robotnik_sensors"
    "waterloo_steel" # pad launcher
    ## WAM Side:
    "uwarl-barrett_wam_msgs"
)
# $USER = "uwarl":
SUBMODULES_FOR_SUMMIT=(
    ## SUMMIT Side:
    "multimap_server_msgs"
    "system_monitor"
    "uwarl-multimap_server"
    "uwarl-robot_localization_utils"
    "uwarl-robotnik_base_hw"
    "uwarl-robotnik_msgs"
    "uwarl-robotnik_sensors"
    "uwarl-summit_xl_common" # TODO: need to deal with mapping
    "uwarl-summit_xl_robot"
    "waterloo_steel"
    ## WAM Side:
    "uwarl-barrett_wam_msgs"  # [NOT USED]
)
# $USER = "uwarl-orin":
SUBMODULES_FOR_WAM=(
    ## SUMMIT Side:
    "uwarl-robotnik_msgs"    # to talk to base
    "uwarl-robotnik_sensors"
    "waterloo_steel"
    ## WAM Side:
    "uwarl-barrett_wam_hw"   # [x86_64, aarch64/arm64]
    "uwarl-barrett_wam_msgs"
    "uwarl-realsense_ros"    # [L515 Support]
    ## Vicon Tracker:
    "uwarl-vicon_bridge"
)
#### USER DEFINED PC: ####
# $USER = "parallels":
SUBMODULES_FOR_JX_PARALLEL=(
    ## SUMMIT Side:
    "multimap_server_msgs"
    "system_monitor"
    "uwarl-multimap_server"
    "uwarl-robot_localization_utils"
    # "uwarl-robotnik_base_hw"  # not needed for simulation !  # [x86_64 only]
    "uwarl-robotnik_msgs"
    "uwarl-robotnik_sensors"
    "uwarl-summit_xl_common"
    "uwarl-summit_xl_robot"
    "waterloo_steel"
    ## WAM Side:
    "uwarl-barrett_wam_hw"      # : Enabled for local dev.  # [x86_64, aarch64/arm64]
    "uwarl-barrett_wam_msgs"
    "uwarl-realsense_ros"       # [L515 Support]
    ## Research:
    # "vins-research-pkg"
    # "uwarl-sensor_calibr"
    ## Simulation:
    "velodyne_simulator"
)
# $USER = "arnab":
SUBMODULES_FOR_AJ_DESKTOP=(
    ## SUMMIT Side:
    "multimap_server_msgs"
    "system_monitor"
    "uwarl-multimap_server"
    "uwarl-robot_localization_utils"
    # "uwarl-robotnik_base_hw"  # not needed for simulation !  # [x86_64 only]
    "uwarl-robotnik_msgs"
    "uwarl-robotnik_sensors"
    "uwarl-summit_xl_common"
    "uwarl-summit_xl_robot"
    "waterloo_steel"
    ## Cart Side:
    "wagon_tf_publisher"
    ## WAM Trajectory controller tutorial (pilz)
    "pilz_tutorial"
    ## WAM Side:
    "uwarl-barrett_wam_hw"      # : Enabled for local dev.  # [x86_64, aarch64/arm64]
    "uwarl-barrett_wam_msgs"
    # "uwarl-realsense_ros"       # [L515 Support]
    ## Research:
    # "vins-research-pkg"
    # "uwarl-sensor_calibr"
    ## Simulation:
    "velodyne_simulator"
)
# $USER = "jx":
SUBMODULES_FOR_JX_DESKTOP=(
    ## SUMMIT Side:
    "multimap_server_msgs"
    "system_monitor"
    "uwarl-multimap_server"
    "uwarl-robot_localization_utils"
    # "uwarl-robotnik_base_hw"  # not needed for simulation !  # [x86_64 only]
    "uwarl-robotnik_msgs"
    "uwarl-robotnik_sensors"
    "uwarl-summit_xl_common"
    "uwarl-summit_xl_robot"
    "waterloo_steel"
    ## WAM Side:
    "uwarl-barrett_wam_hw"      # : Enabled for local dev.  # [x86_64, aarch64/arm64]
    "uwarl-barrett_wam_msgs"
    "uwarl-realsense_ros"       # [L515 Support]
    ## Research:
    "vins-research-pkg"
    "uwarl-sensor_calibr"
)
# $USER = "uwarl-laptop-4"
SUBMODULES_FOR_P51_LENOVO=(
#   ## SUMMIT Side:
    "multimap_server_msgs"              # TODO: need to deal with mapping
    "uwarl-multimap_server"             # TODO: need to deal with mapping
    "uwarl-robot_localization_utils"    # TODO: need to deal with mapping
    "system_monitor"            # ["robotnik"]
#   "uwarl-robotnik_base_hw"   # [waterloo_steel/adlink-mxe211-melodic/main] # [x86_64 only]
    "uwarl-robotnik_msgs"
    "uwarl-robotnik_sensors"
    "uwarl-summit_xl_common"   # X-[waterloo_steel/universal/main] TODO: need to deal with mapping,
    "uwarl-summit_xl_robot"    # [waterloo_steel/adlink-mxe211-melodic/main]
    "waterloo_steel"           # [universal/ros1/main]
#    ## WAM Side:
#       "uwarl-barrett_wam_hw"     # [x86_64, aarch64/arm64]
    "uwarl-barrett_wam_msgs"
#       "uwarl-realsense_ros"      # [L515 Support]
#       "uwarl-barrett-ros-pkg"    # [DEPRECATED]
#       "uwarl-zed_ros_wrapper"    # [No longer used]
#    ## Vicon Tracker:
#       "uwarl-vicon_bridge"
    "velodyne_simulator"
)
# $USER = "uwarl-laptop-3"
SUBMODULES_FOR_P50s_LENOVO=(
#   ## SUMMIT Side:
    "multimap_server_msgs"              # TODO: need to deal with mapping
    "uwarl-multimap_server"             # TODO: need to deal with mapping
    "uwarl-robot_localization_utils"    # TODO: need to deal with mapping
    "system_monitor"            # ["robotnik"]
#   "uwarl-robotnik_base_hw"   # [waterloo_steel/adlink-mxe211-melodic/main] # [x86_64 only]
    "uwarl-robotnik_msgs"
    "uwarl-robotnik_sensors"
    "uwarl-summit_xl_common"   # X-[waterloo_steel/universal/main] TODO: need to deal with mapping,
    "uwarl-summit_xl_robot"    # [waterloo_steel/adlink-mxe211-melodic/main]
    "waterloo_steel"           # [universal/ros1/main]
#    ## WAM Side:
    "uwarl-barrett_wam_hw"     # [x86_64, aarch64/arm64]
    "uwarl-barrett_wam_msgs"
    "uwarl-realsense_ros"      # [L515 Support]
    "uwarl-barrett-ros-pkg"    # [DEPRECATED]
    "uwarl-zed_ros_wrapper"    # [No longer used]
#    ## Vicon Tracker:
    "uwarl-vicon_bridge"
#    ## Research:
    "vins-research-pkg"
    "uwarl-sensor_calibr"
)

#################################################################
## NETWORK PARAM: ##
### User Defined: ###
export ROS_CORE_HOSTER="LOCAL-HOSTS"  # <--- change it to localhost \in ["SUMMIT-PC", "WAM-PC", "REMOTE-PC", "LOCAL-HOSTS"]

### Manually Registered: ###
### [ Robot Network: UWARL-171102A_5G Wired ] ###
export ROS_SUMMIT_IN_NETWORK_IP=192.168.1.11 # MAC Binded
export ROS_WAM_IN_NETWORK_IP=192.168.1.10 # MAC Binded
export ROS_DECK_IN_NETWORK_IP=192.168.1.15 # MAC Binded
### [ Robot Network: UWARL-171102A_5G Wifi ] ###
# DHCP , may change:
export ROS_JX_IN_NETWORK_PARALLEL_PC_IP=192.168.1.100
# export ROS_AJ_IN_NETWORK_DESKTOP_PC_IP=192.168.1.x
# export ROS_P51_IN_NETWORK_LENOVO_PC_IP=192.168.1.x
# export ROS_JX_IN_NETWORK_OEM_PC_IP=192.168.1.x
# export ROS_P50s_IN_NETWORK_LENOVO_PC_IP=192.168.1.x

# [USER:] please change this one if you want to direct it to your own PC to host ROSCORE:
export ROS_EXTERNAL_PC_IN_NETWORK_IP=$ROS_JX_IN_NETWORK_PARALLEL_PC_IP 

#################################################################
## VAR ##
# assign to DISPLAY param:
export DISPLAY_WAM=:0
export DISPLAY_DEFAULT=:0

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

## DEBUGGING PRINT FUNC ##
function ic () {
    echo -e "${CYAN}[UWARL-Robot_Config]${NC} ${BLUE} $1 ${NC}"
}
function ic_title () {
    echo " "
    ic " ==================================================="
    if [[ -z "$2" ]]; then
        ic " [ $1 ] "
    else
        echo -e "${CYAN}[UWARL-Robot_Config]${NC} ${BLUE} [ $1 | ${NC} $2 ${BLUE} ] ${NC}"
    fi
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
    echo -e "${CYAN}[UWARL-Robot_Config]   > Aliasing \`${NC}${YELLOW}\$ $1${NC}\` command := $2"
    alias $1=$2
}
function ic_source () {
    echo -e "${CYAN}[UWARL-Robot_Config]   > Sourcing ${YELLOW} $2 ${BLUE} @ $1 ${NC}"
    source $1
}

#################################################################
### AUTO SYSTEM CONFIG: ###
# ic_title "Auto System Config:"
source /etc/lsb-release # <--- sourcing distribution versions
export LOCAL_DISTRIB_CODENAME=$DISTRIB_CODENAME # <--- fetch local LSB Version from LINUX Env.
export LOCAL_DISTRIB_DESCRIPTION=$DISTRIB_DESCRIPTION # <--- fetch local LSB Version from LINUX Env.
if [[ $LOCAL_DISTRIB_CODENAME = 'Holo' ]]; then
    export LOCAL_PC_IP=`ip -json route get 8.8.8.8 | jq -r '.[].prefsrc'` # <--- fetch local IP from Archlinx Env.
else
    export LOCAL_PC_IP=`hostname -I | cut -d' ' -f1` # <--- fetch local IP from LINUX Env.
fi

#################################################################
## FUNCTIONS ##
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
    ic_wrn " [COMMON SH CONFIG ($USER)]: "
    ic     "    - ROBOT_PC_NAME           : $UWARL_ROBOT_PC_NAME"
    ic     "    - ROS_CORE_HOSTER         : $ROS_CORE_HOSTER"
    ic     "    - IN_ROBOT_NETWORK        : $IN_ROBOT_NETWORK"
    ic_wrn " [ROS CONFIG ($USER)]: "
    ic     "    - ROS ROS_DISTRO          : $ROS_DISTRO"
    ic     "    - ROS HOST                : $ROS_HOSTNAME"
    ic     "    - ROS MASTER              : $ROS_MASTER_URI"
    ic     "    - ROS IP                  : $ROS_IP"
    ic_wrn " [OS REPORT ($USER)]: "
    ic     "    - IP                      : $LOCAL_PC_IP"
    ic     "    - DISPLAY                 : $DISPLAY"
    ic     "    - DISTRIB_CODENAME        : $LOCAL_DISTRIB_CODENAME"
    ic     "    - DISTRIB_DESCRIPTION#    : $LOCAL_DISTRIB_DESCRIPTION"
    ic     "    - PYTHONPATH              : $PYTHONPATH"
    ic     "    - KERNEL                  : $(uname -a)"
}

function cat_sensor_status() {
    if [[ -d "$JX_LINUX/librealsense" ]]; then
        ic_wrn " [SENSOR Driver STATUS]: "
        ic     "  \n$( rs-enumerate-devices -s )"
        # ic     "  \n$(rs-enumerate-devices | grep -E '  Serial Number|  Firmware Version|Usb Type Descriptor|Physical Port' )"
        # rs-enumerate-devices -c # calibration information
        # rs-enumerate-devices -m # stream profiles
        # rs-enumerate-devices -o # device options
        # rs-enumerate-devices -s # summary
    else
        ic     "  Unable to Find Sensor Drivers! [librealsense]"
    fi
}

function ros_core_sync() {
    ## Auto-Assign: ros core synchronization, given core PC name tag ##
    ic_wrn " > ROS CORE is currently hosted by [$1]!"
    case $1 in
    
        "SUMMIT-PC")
            export ROS_MASTER_URI=http://$ROS_SUMMIT_IN_NETWORK_IP:11311/
            ;;
    
        "WAM-PC")
            export ROS_MASTER_URI=http://$ROS_WAM_IN_NETWORK_IP:11311/
            ;;
    
        "REMOTE-PC")
            export ROS_MASTER_URI=http://$ROS_EXTERNAL_PC_IN_NETWORK_IP:11311/
            ;;
    
        "LOCAL-HOSTS")
            export ROS_IP=localhost
            export ROS_HOSTNAME=localhost
            export ROS_MASTER_URI=http://localhost:11311/
            ;;
    
        *)
            exit "Please configure HOSTER from [SUMMIT-PC, WAM-PC, REMOTE-PC, LOCALHOST]!"
            ;;
    esac
}

function sync_ros_core_if_in_robot_network_else_localhost() {
    in_network_ip=$1
    if [[ $LOCAL_PC_IP = "$in_network_ip" ]]; then
        ic_wrn " > We have detected a registered in-network PC, now syncing ros core as $ROS_CORE_HOSTER !"
        ros_core_sync $ROS_CORE_HOSTER
        export ROS_IP=$in_network_ip
        export ROS_HOSTNAME=$in_network_ip
        export IN_ROBOT_NETWORK="YES"
    else # when out-of-network
        ic_wrn " > We have detected a registered out-of-network PC, now forcing local host for ROS_MASTER_URI !"
        ros_core_sync "LOCAL-HOSTS"
        export IN_ROBOT_NETWORK="NOPE, Local Host Only!"
    fi
}

function source_ros() {
    ic_title "ROS" "Setting up ROS environment based on User and PC IP"
    ## Auto-Assign: ##
    ic " > PC Reg.: $USER @ $LOCAL_PC_IP"
    ### Always In Network On-Robot PC ###
    # adlink in-robot-network PC:
    if [[ $USER = "uwarl" ]] && [[ $LOCAL_PC_IP = "$ROS_SUMMIT_IN_NETWORK_IP" ]]; then
        export UWARL_ROBOT_PC_NAME="ADLINK_MXE211_SUMMIT"
        ic_wrn " - Adlink MXE211 Summit PC detected!" 
        # manual config:
        export ROS_DISTRO=melodic
        export DISPLAY=$DISPLAY_DEFAULT
        # export PYTHONPATH_ROS=/usr/bin/python3
        # export PYTHONPATH=$PYTHONPATH_ROS
        # ===> update environment files in .ros:
        sudo cp $UWARL_CONFIGS/summit/user_services/environment $HOME/.ros/environment
        # welcome:
        ic_wrn " - Robot PC User [$UWARL_ROBOT_PC_NAME] detected!"
        # ros core:
        sync_ros_core_if_in_robot_network_else_localhost $ROS_SUMMIT_IN_NETWORK_IP 
        
    # jetson in-robot-network PC:
    elif [[ $USER = "uwarl-orin" ]] && [[ $LOCAL_PC_IP = "$ROS_WAM_IN_NETWORK_IP" ]]; then
        export UWARL_ROBOT_PC_NAME="JETSON_ORIN_WAM"
        ic_wrn " - Jetson Orin WAM PC detected!"
        # manual config:
        export ROS_DISTRO=noetic
        export DISPLAY=$DISPLAY_WAM
        export PYTHONPATH_ROS=/usr/bin/python3
        export PYTHONPATH=$PYTHONPATH_ROS
        # welcome:
        ic_wrn " - Robot PC User [$UWARL_ROBOT_PC_NAME] detected!"
        # ros core:
        sync_ros_core_if_in_robot_network_else_localhost $ROS_WAM_IN_NETWORK_IP 
        
    ### In Network / Out-Network On-Robot PC ###
    # steam deck:
    elif [[ $USER = "deck" ]]; then
        export UWARL_ROBOT_PC_NAME="STEAM_DECK_CONTROLLER"
        ic " - Steam Deck PC detected!"
        # manual config:
        export ROS_DISTRO=noetic
        export DISPLAY=$DISPLAY_DEFAULT
        export PYTHONPATH_ROS=/home/deck/mambaforge/envs/ros_env_3_8/bin/python3
        export PYTHONPATH=$PYTHONPATH_ROS
        # welcome:
        ic_wrn " - Robot PC User [$UWARL_ROBOT_PC_NAME] detected!"
        # ros core:
        sync_ros_core_if_in_robot_network_else_localhost $ROS_DECK_IN_NETWORK_IP 
        
    ### user defined out-of-network PC:
    elif [[ $USER = "parallels" ]]; then
        # manual config:
        export UWARL_ROBOT_PC_NAME="PARALLELS_VM_JACK"
        export ROS_DISTRO=noetic
        export DISPLAY=$DISPLAY_DEFAULT
        export PYTHONPATH_ROS=/usr/bin/python3
        export PYTHONPATH=$PYTHONPATH_ROS
        # welcome:
        ic_wrn " - NON-Robot PC User [$UWARL_ROBOT_PC_NAME] detected!"
        # ros core:
        sync_ros_core_if_in_robot_network_else_localhost $ROS_EXTERNAL_PC_IN_NETWORK_IP 
    
    elif [[ $USER = "arnab" ]]; then
        # manual config:
        export UWARL_ROBOT_PC_NAME="ARL_DESKTOP_ARNAB"
        export ROS_DISTRO=noetic
        export DISPLAY=:1 # <-- if you get an error like 'Invalid MIT-MAGIC-COOKIE-1 key', change the display value.
        export PYTHONPATH_ROS=/usr/bin/python3
        export PYTHONPATH=$PYTHONPATH_ROS
        # welcome:
        ic_wrn " - NON-Robot PC User [$UWARL_ROBOT_PC_NAME] detected!"
        # ros core:
        sync_ros_core_if_in_robot_network_else_localhost $ROS_EXTERNAL_PC_IN_NETWORK_IP 
    
    elif [[ $USER = "jx" ]]; then
        # manual config:
        export UWARL_ROBOT_PC_NAME="JX_DESKTOP_JACK"
        export ROS_DISTRO=noetic
        export DISPLAY=:1
        export PYTHONPATH_ROS=/usr/bin/python3
        export PYTHONPATH=$PYTHONPATH_ROS
        # welcome:
        ic_wrn " - NON-Robot PC User [$UWARL_ROBOT_PC_NAME] detected!"
        # ros core:
        sync_ros_core_if_in_robot_network_else_localhost $ROS_EXTERNAL_PC_IN_NETWORK_IP 
    
    elif [[ $USER = "uwarl-laptop-4" ]]; then
        export UWARL_ROBOT_PC_NAME="UWARL_LAPTOP_4_JEONGWOO"
        # manual config:
        export ROS_DISTRO=melodic
        export DISPLAY=$DISPLAY_DEFAULT
        # export PYTHONPATH_ROS=/usr/bin/python3
        # export PYTHONPATH=$PYTHONPATH_ROS
        # welcome:
        ic_wrn " - NON-Robot PC User [$UWARL_ROBOT_PC_NAME] detected!"
        # ros core:
        sync_ros_core_if_in_robot_network_else_localhost $ROS_EXTERNAL_PC_IN_NETWORK_IP 

    elif [[ $USER = "uwarl" ]]; then
        export UWARL_ROBOT_PC_NAME="UWARL_LAPTOP_3_SIMON"
        # manual config:
        export ROS_DISTRO=noetic
        export DISPLAY=$DISPLAY_DEFAULT
        export PYTHONPATH_ROS=/usr/bin/python3
        export PYTHONPATH=$PYTHONPATH_ROS
        # welcome:
        ic_wrn " - NON-Robot PC User [$UWARL_ROBOT_PC_NAME] detected!"
        # ros core:
        sync_ros_core_if_in_robot_network_else_localhost $ROS_EXTERNAL_PC_IN_NETWORK_IP 

    ### TEMPLATE:
    # elif [[ $USER = "{$USER}" ]]; then
    #     export UWARL_ROBOT_PC_NAME="{ENTER HERE}"
    #     # manual config:
    #     export ROS_DISTRO=noetic
    #     export DISPLAY=$DISPLAY_DEFAULT
    #     export PYTHONPATH_ROS=/usr/bin/python3
    #     export PYTHONPATH=$PYTHONPATH_ROS
    #     # welcome:
    #     ic_wrn " - NON-Robot PC User [$UWARL_ROBOT_PC_NAME] detected!"
    #     # ros core:
    #     sync_ros_core_if_in_robot_network_else_localhost $ROS_EXTERNAL_PC_IN_NETWORK_IP 

    else
        ic_err " - UNREGISTERED PC detected!"
        ic_wrn " > Please add your PC to the ROS config file: $UWARL_CONFIGS/scripts/common.sh"
        export UWARL_ROBOT_PC_NAME="UNKNOWN_PC"
    fi

    ic_source /opt/ros/$ROS_DISTRO/setup.zsh "ROS_DISTRO=$ROS_DISTRO"
    ic_source $ROS_CATKIN_WS/devel/setup.zsh "ROS_CATKIN_WS=$ROS_CATKIN_WS"
}

function print_ascii_title() {
    echo -e "${BLUE} \n██╗    ██╗ █████╗ ████████╗███████╗██████╗ ██╗      ██████╗  ██████╗    \n██║    ██║██╔══██╗╚══██╔══╝██╔════╝██╔══██╗██║     ██╔═══██╗██╔═══██╗   \n██║ █╗ ██║███████║   ██║   █████╗  ██████╔╝██║     ██║   ██║██║   ██║   \n██║███╗██║██╔══██║   ██║   ██╔══╝  ██╔══██╗██║     ██║   ██║██║   ██║   \n╚███╔███╔╝██║  ██║   ██║   ███████╗██║  ██║███████╗╚██████╔╝╚██████╔╝   \n ╚══╝╚══╝ ╚═╝  ╚═╝   ╚═╝   ╚══════╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝    \n                                                                        \n███████╗████████╗███████╗███████╗██╗                   ██╗   ██╗██████╗ \n██╔════╝╚══██╔══╝██╔════╝██╔════╝██║                   ██║   ██║╚════██╗\n███████╗   ██║   █████╗  █████╗  ██║         █████╗    ██║   ██║ █████╔╝\n╚════██║   ██║   ██╔══╝  ██╔══╝  ██║         ╚════╝    ╚██╗ ██╔╝██╔═══╝ \n███████║   ██║   ███████╗███████╗███████╗               ╚████╔╝ ███████╗\n╚══════╝   ╚═╝   ╚══════╝╚══════╝╚══════╝                ╚═══╝  ╚══════╝ ${NC}" 
    echo "\n==="
}

function tmux_custom() {
    if ! command -v tmux &> /dev/null
        tmux source-file $UWARL_CONFIGS/scripts/.tmux.conf
    then
        ic_wrn "<tmux> could not be found! Please install first."
        exit
    fi
}

function source_CUDA() {
    if [[ $USER = "uwarl-orin" ]]; then
        # https://github.com/jetsonhacks/buildLibrealsense2TX/issues/13#issuecomment-573976359
        # Needed to build package like librealsesne for CUDA support!
        ic_wrn "[Detected :: Jetson Orin] Exporting CUDA Path ..."
        export CUDA_HOME=/usr/local/cuda
        export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64:/usr/local/cuda/extras/CUPTI/lib64
        export PATH=$PATH:$CUDA_HOME/bin
    else
        ic_wrn "[HINT] \"source_CUDA\" is not implemented. You may not need it, else define it in common.sh ..."
    fi
}

function source_all_common_configs() {
    print_ascii_title
    # action:
    ic_source $UWARL_SUMMIT_SPECIFIC/summitxl_params.env "Summit Params"
    source_ros
    ic_source "$UWARL_CONFIGS/scripts/shortcuts.sh" "Shortcuts"
    source_CUDA
    # report:
    ic_title "Print Environment Variables: "
    cat_summit_env
    cat_ros_env
    # system status:
    if ! { [[ "$TERM" == *"screen"* ]] && [ -n "$TMUX" ]; } then
        ic_title "Print Driver Status: "
        # Only print sensor when it's not TMUX sessions:
        cat_sensor_status
    fi
}

function tmux_multi_pane () {
    session="UWARL Multi Window"
    tmux start-server
    tmux new-session -d -s $session
    tmux source-file $UWARL_CONFIGS/desktop/tmux.conf
    n_session=${1:-2}
    for i in $(seq 2 $n_session)
    do 
        tmux splitw -t $session -l 1
        tmux send -t $session:0.1 clear C-m
        tmux selectp -t $session:0.0
        tmux selectl -t $session tiled
    done
    tmux a -t $session
}

function tmux_sync () {
    set -e
    if [ $# -lt 2 ]
    then
        ic_wrn "Tmux Sync Usage: $0 [session_name] [command_1]..."
        exit 1
    fi
    
    session=$1
    shift
    tmux start-server
    tmux new -d -s $session
    # tmux source-file $UWARL_CONFIGS/desktop/tmux.conf
    on_error() {
        tmux kill-session -t $session
    }
    trap on_error ERR
    cmd1=$1
    shift
    tmux send -t $session:0 "$cmd1" C-m
    for i in "$@"
    do
        tmux splitw -t $session -l 1
        tmux send -t $session:0.1 "$i" C-m
        tmux selectp -t $session:0.0
        tmux selectl -t $session tiled
    done
    tmux setw synchronize-panes on
    tmux a -t $session
}

function tmux_usync () {
    set -e
    if [ $# -lt 2 ]
    then
        ic_wrn "Tmux Unsync Usage: $0 [session_name] [command_1]..."
        exit 1
    fi
    
    session=$1
    shift
    tmux start-server
    tmux new -d -s $session
    # tmux source-file $UWARL_CONFIGS/desktop/tmux.conf
    on_error() {
        tmux kill-session -t $session
    }
    trap on_error ERR
    cmd1=$1
    shift
    tmux send -t $session:0 "$cmd1" C-m
    for i in "$@"
    do
        tmux splitw -t $session -l 1
        tmux send -t $session:0.1 "$i" C-m
        tmux selectp -t $session:0.0
        tmux selectl -t $session tiled
    done
    tmux a -t $session
}
# <<< EOF
