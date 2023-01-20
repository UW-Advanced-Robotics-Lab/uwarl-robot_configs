#!/usr/bin/env zsh
#################################################################
## USER PARAM: ##
export UWARL_catkin_ws_branch="universal/ros1/jeongwoo/localization-and-mapping"

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
    # "uwarl-summit_xl_common" # TODO: need to deal with mapping
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
# else:
SUBMODULES_FOR_PC_DEFAULT=(
    ## SUMMIT Side:
    "multimap_server_msgs"
    "system_monitor"
    "uwarl-multimap_server"
    "uwarl-robot_localization_utils"
    "uwarl-robotnik_msgs"
    "uwarl-robotnik_sensors"
    "waterloo_steel"
    ## WAM Side:
    "uwarl-barrett_wam_msgs"
    "uwarl-realsense_ros"
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
    "vins-research-pkg"
    "uwarl-sensor_calibr"
)
# $USER = "oem":
SUBMODULES_FOR_JX_OEM=(
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
#       "uwarl-barrett_wam_msgs"
#       "uwarl-realsense_ros"      # [L515 Support]
#       "uwarl-barrett-ros-pkg"    # [DEPRECATED]
#       "uwarl-zed_ros_wrapper"    # [No longer used]
#    ## Vicon Tracker:
#       "uwarl-vicon_bridge"
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
export ROS_CORE_HOSTER="SUMMIT-PC"  # <--- change it to localhost \in ["SUMMIT-PC", "WAM-PC", "REMOTE-PC", "LOCAL-HOSTS"]
### SYSTEM Defined: ###
export LOCAL_PC_IP="$(hostname -I | cut -d' ' -f1)"

### [ Robot Network: UWARL-171102A_5G ] ###
export ROS_SUMMIT_IP=192.168.1.11 # MAC Binded
export ROS_SUMMIT_HOSTNAME=192.168.1.11
export ROS_SUMMIT_DISTRO=melodic

export ROS_WAM_IP=192.168.1.10 # MAC Binded
export ROS_WAM_HOSTNAME=192.168.1.10
export ROS_WAM_DISTRO=noetic

export ROS_DECK_IP=192.168.1.15 # MAC Binded
export ROS_DECK_HOSTNAME=192.168.1.15
export ROS_DECK_DISTRO=noetic

export ROS_PC_IP=192.168.1.100 # DHCP , may change
export ROS_PC_HOSTNAME=192.168.1.100
export ROS_PC_DISTRO=noetic

### [ Other Miscellaneous Networks ] ###
export ROS_JX_PARALLEL_PC_IP=10.211.55.5
export ROS_JX_PARALLEL_PC_HOSTNAME=10.211.55.5
export ROS_JX_PARALLEL_PC_DISTRO=noetic

export ROS_P51_LENOVO_PC_IP=192.168.5.156
export ROS_P51_LENOVO_PC_HOSTNAME=192.168.5.156
export ROS_P51_LENOVO_PC_DISTRO=melodic

export ROS_JX_OEM_PC_IP=10.42.0.1
export ROS_JX_OEM_PC_HOSTNAME=10.42.0.1
export ROS_JX_OEM_PC_DISTRO=noetic

export ROS_P50s_LENOVO_PC_IP=192.168.5.211
export ROS_P50s_LENOVO_PC_HOSTNAME=192.168.5.211
export ROS_P50s_LENOVO_PC_DISTRO=noetic
#################################################################
## VAR ##
# assign to DISPLAY param:
export DISPLAY_WAM=:1
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

## FUNC ##
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
    ic_wrn " [ROS CONFIG ($USER)]: "
    ic     "    - ROS ROS_DISTRO          : $ROS_DISTRO"
    ic     "    - ROS HOST                : $ROS_HOSTNAME"
    ic     "    - ROS MASTER              : $ROS_MASTER_URI"
    ic     "    - ROS IP                  : $ROS_IP"
    ic_wrn " [LINUX ENV CONFIG ($USER)]: "
    ic     "    - IP                      : $LOCAL_PC_IP"
    ic     "    - DISPLAY                 : $DISPLAY"
    ic     "    - KERNEL                  : $(uname -a)"
    ic     "    - PYTHONPATH              : $PYTHONPATH"
}

function cat_sensor_status() {
    ic_wrn " [SENSOR Driver STATUS]: "
    if [[ -d "$JX_LINUX/librealsense" ]]; then
        ic     "  Serials:\n$(rs-enumerate-devices | grep '  Serial Number' )"
        ic     "  Firmware:\n$(rs-enumerate-devices | grep '  Firmware Version' )"
        ic     "  Types:\n$(rs-enumerate-devices | grep 'Usb Type Descriptor' )"
        ic     "  Port:\n$(rs-enumerate-devices | grep 'Physical Port' )"
    fi
}

function ros_core_sync() {
    ## Auto-Assign: ros core synchronization, given core PC name tag ##
    ic_wrn " > ROS CORE is currently hosted by [$1]!"
    case $1 in
    
        "SUMMIT-PC")
            export ROS_SUMMIT_MASTER_URI=http://localhost:11311/
            export ROS_WAM_MASTER_URI=http://$ROS_SUMMIT_IP:11311/
            export ROS_PC_MASTER_URI=http://$ROS_SUMMIT_IP:11311/
            export ROS_DECK_MASTER_URI=http://$ROS_SUMMIT_IP:11311/
            ;;
    
        "WAM-PC")
            export ROS_SUMMIT_MASTER_URI=http://$ROS_WAM_IP:11311/
            export ROS_WAM_MASTER_URI=http://localhost:11311/
            export ROS_PC_MASTER_URI=http://$ROS_WAM_IP:11311/
            export ROS_DECK_MASTER_URI=http://$ROS_WAM_IP:11311/
            ;;
    
        "REMOTE-PC")
            export ROS_SUMMIT_MASTER_URI=http://$ROS_PC_IP:11311/
            export ROS_WAM_MASTER_URI=http://$ROS_PC_IP:11311/
            export ROS_PC_MASTER_URI=http://localhost:11311/
            export ROS_DECK_MASTER_URI=http://$ROS_PC_IP:11311/
            ;;
    
        "LOCAL-HOSTS")
            export ROS_SUMMIT_MASTER_URI=http://localhost:11311/
            export ROS_WAM_MASTER_URI=http://localhost:11311/
            export ROS_PC_MASTER_URI=http://localhost:11311/
            export ROS_DECK_MASTER_URI=http://localhost:11311/
            ;;
    
        *)
            exit "Please configure HOSTER from [SUMMIT-PC, WAM-PC, REMOTE-PC, LOCALHOST]!"
            ;;
    esac
}

function source_ros() {
    ic_title "ROS" "Setting up ROS environment based on User and PC IP"
    ## Auto-Assign: ##
    ic " > PC Reg.: $USER @ $LOCAL_PC_IP"
    # adlink in-robot-network PC:
    if [[ $USER = "uwarl" ]] && [[ $LOCAL_PC_IP = "$ROS_SUMMIT_IP" ]]; then
        ic_wrn " - Adlink MXE211 Summit PC detected!" 
        ic_wrn " > We have detected a registered in-network PC, now applying configs from common.sh !"
        ros_core_sync $ROS_CORE_HOSTER
        export ROS_IP=$ROS_SUMMIT_IP
        export ROS_HOSTNAME=$ROS_SUMMIT_HOSTNAME
        export ROS_MASTER_URI=$ROS_SUMMIT_MASTER_URI
        export ROS_DISTRO=$ROS_SUMMIT_DISTRO
        export DISPLAY=$DISPLAY_DEFAULT
        
    # jetson in-robot-network PC:
    elif [[ $USER = "uwarl-orin" ]] && [[ $LOCAL_PC_IP = "$ROS_WAM_IP" ]]; then
        ic_wrn " - Jetson Orin WAM PC detected!"
        ic_wrn " > We have detected a registered in-network PC, now applying configs from common.sh !"
        ros_core_sync $ROS_CORE_HOSTER
        export ROS_IP=$ROS_WAM_IP
        export ROS_HOSTNAME=$ROS_WAM_HOSTNAME
        export ROS_MASTER_URI=$ROS_WAM_MASTER_URI
        export ROS_DISTRO=$ROS_WAM_DISTRO
        export DISPLAY=$DISPLAY_WAM
        export PYTHONPATH_ROS=/usr/bin/python3
        export PYTHONPATH=$PYTHONPATH_ROS
        
    # steam deck in-robot-network PC:
    elif [[ $USER = "deck" ]]; then
        ic " - Steam Deck PC detected!"
        ic_wrn " > We have detected a registered in-network PC, now applying configs from common.sh !"
        ros_core_sync $ROS_CORE_HOSTER
        export ROS_IP=$ROS_DECK_IP
        export ROS_HOSTNAME=$ROS_DECK_HOSTNAME
        export ROS_MASTER_URI=$ROS_DECK_MASTER_URI
        export ROS_DISTRO=$ROS_DECK_DISTRO
        export DISPLAY=$DISPLAY_DEFAULT
        export PYTHONPATH_ROS=/home/deck/mambaforge/envs/ros_env_3_8/bin/python3
        export PYTHONPATH=$PYTHONPATH_ROS
    
    # default in-robot-network PC:
    elif [[ $LOCAL_PC_IP = "$ROS_PC_IP" ]]; then
        ic_wrn " - NON-Robot PC User detected!"
        ic_wrn " > We have detected a registered in-network PC, now applying configs from common.sh !"
        ros_core_sync $ROS_CORE_HOSTER
        export ROS_IP=$ROS_PC_IP
        export ROS_HOSTNAME=$ROS_PC_HOSTNAME
        export ROS_MASTER_URI=$ROS_PC_MASTER_URI
        export ROS_DISTRO=$ROS_PC_DISTRO
        export DISPLAY=$DISPLAY_DEFAULT
        export PYTHONPATH_ROS=/usr/bin/python3
        export PYTHONPATH=$PYTHONPATH_ROS
        
    ### user defined out-of-network PC:
    elif [[ $USER = "parallels" ]] && [[ $LOCAL_PC_IP = "$ROS_JX_PARALLEL_PC_IP" ]]; then
        ic_wrn " - NON-Robot PC User [Jack's Parallel VM] detected!"
        ic_wrn " > We have detected a registered out-of-network PC, now forcing local host for ROS_MASTER_URI !"
        ros_core_sync "LOCAL-HOSTS"
        export ROS_IP=$ROS_JX_PARALLEL_PC_IP
        export ROS_HOSTNAME=$ROS_JX_PARALLEL_PC_HOSTNAME
        export ROS_MASTER_URI=http://localhost:11311/
        export ROS_DISTRO=$ROS_JX_PARALLEL_PC_DISTRO
        export DISPLAY=$DISPLAY_DEFAULT
        export PYTHONPATH_ROS=/usr/bin/python3
        export PYTHONPATH=$PYTHONPATH_ROS
    
    elif [[ $USER = "oem" ]] && [[ $LOCAL_PC_IP = "$ROS_JX_OEM_PC_IP" ]]; then
        ic_wrn " - NON-Robot PC User [Jack's Parallel VM] detected!"
        ic_wrn " > We have detected a registered out-of-network PC, now forcing local host for ROS_MASTER_URI !"
        ros_core_sync "LOCAL-HOSTS"
        export ROS_IP=$ROS_JX_OEM_PC_IP
        export ROS_HOSTNAME=$ROS_JX_OEM_PC_HOSTNAME
        export ROS_MASTER_URI=http://localhost:11311/
        export ROS_DISTRO=$ROS_JX_OEM_PC_DISTRO
        export DISPLAY=$DISPLAY_DEFAULT
        export PYTHONPATH_ROS=/usr/bin/python3
        export PYTHONPATH=$PYTHONPATH_ROS
    
    elif [[ $USER = "uwarl-laptop-4" ]]; then
        ic_wrn " - NON-Robot PC User [UWARL Laptop 4] detected!"
        ic_wrn " > We have detected a registered out-of-network PC, now forcing local host for ROS_MASTER_URI !"
        ros_core_sync "LOCAL-HOSTS"
        export ROS_IP=$ROS_P51_LENOVO_PC_IP
        export ROS_HOSTNAME=$ROS_P51_LENOVO_PC_HOSTNAME
        export ROS_MASTER_URI=http://localhost:11311/
        export ROS_DISTRO=$ROS_P51_LENOVO_PC_DISTRO
        export DISPLAY=$DISPLAY_DEFAULT

    elif [[ $USER = "uwarl" ]] && [[ $LOCAL_PC_IP = "$ROS_P50s_LENOVO_PC_IP" ]]; then
        ic_wrn " - NON-Robot PC User [UWARL Laptop 4] detected!"
        ic_wrn " > We have detected a registered out-of-network PC, now forcing local host for ROS_MASTER_URI !"
        ros_core_sync "LOCAL-HOSTS"
        export ROS_IP=$ROS_P50s_LENOVO_PC_IP
        export ROS_HOSTNAME=$ROS_P50s_LENOVO_PC_HOSTNAME
        export ROS_MASTER_URI=http://localhost:11311/
        export ROS_DISTRO=$ROS_P50s_LENOVO_PC_DISTRO
        export DISPLAY=$DISPLAY_DEFAULT
        export PYTHONPATH_ROS=/usr/bin/python3
        export PYTHONPATH=$PYTHONPATH_ROS


    
    ### TEMPLATE:
    # elif [[ $USER = "{define-here}" ]] && [[ $LOCAL_PC_IP = "${define-here}" ]]; then
    #     ic_wrn " - NON-Robot PC User [Jack's Parallel VM] detected!"
    #     ic_wrn " > We have detected a registered out-of-network PC, now forcing local host for ROS_MASTER_URI !"
    
    else
        ic_err " - UNREGISTERED Out-of-network/In-network PC detected!"
        ic_wrn " > Please add your PC to the ROS config file: $UWARL_CONFIGS/scripts/robot_env.sh"
        export PYTHONPATH_ROS=/usr/bin/python3
        export PYTHONPATH=$PYTHONPATH_ROS
        export ROS_DISTRO=noetic # by default
    fi

    ic_source /opt/ros/$ROS_DISTRO/setup.zsh "ROS_DISTRO=$ROS_DISTRO"
    ic_source $ROS_CATKIN_WS/devel/setup.zsh "ROS_CATKIN_WS=$ROS_CATKIN_WS"
}

function print_ascii_title() {
    echo -e "${BLUE} \n██╗    ██╗ █████╗ ████████╗███████╗██████╗ ██╗      ██████╗  ██████╗    \n██║    ██║██╔══██╗╚══██╔══╝██╔════╝██╔══██╗██║     ██╔═══██╗██╔═══██╗   \n██║ █╗ ██║███████║   ██║   █████╗  ██████╔╝██║     ██║   ██║██║   ██║   \n██║███╗██║██╔══██║   ██║   ██╔══╝  ██╔══██╗██║     ██║   ██║██║   ██║   \n╚███╔███╔╝██║  ██║   ██║   ███████╗██║  ██║███████╗╚██████╔╝╚██████╔╝   \n ╚══╝╚══╝ ╚═╝  ╚═╝   ╚═╝   ╚══════╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝    \n                                                                        \n███████╗████████╗███████╗███████╗██╗                   ██╗   ██╗██████╗ \n██╔════╝╚══██╔══╝██╔════╝██╔════╝██║                   ██║   ██║╚════██╗\n███████╗   ██║   █████╗  █████╗  ██║         █████╗    ██║   ██║ █████╔╝\n╚════██║   ██║   ██╔══╝  ██╔══╝  ██║         ╚════╝    ╚██╗ ██╔╝██╔═══╝ \n███████║   ██║   ███████╗███████╗███████╗               ╚████╔╝ ███████╗\n╚══════╝   ╚═╝   ╚══════╝╚══════╝╚══════╝                ╚═══╝  ╚══════╝ ${NC}" 
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
    tmux source-file $UWARL_CONFIGS/desktop/tmux.conf
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
