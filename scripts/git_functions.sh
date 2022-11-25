#!/usr/bin/env zsh
source "$HOME/uwarl-robot_configs/scripts/common.sh"

function load_submodules(){
    # update submodules:
    ic_wrn ">-- Loading Submodules Recursively uwarl-robot_configs @ $ROS_CATKIN_WS/src"

    list_of_modules=("$@")
    total=${#list_of_modules[@]}
    i=0
    for module in "${list_of_modules[@]}"; do
        i=$(( i + 1 ))
        ic_wrn "    > [$i/$total] - Loading submodule @ $module"
        git submodule update --init $module
    done
    ic "x--- Done loading submodules."
}

function create_catkin_ws(){
    # Create catkin workspace
    ic_wrn ">-- Creating catkin workspace @ $ROS_CATKIN_WS"
    cd $HOME
    mkdir -p $ROS_CATKIN_WS/src
    # clone --> to the src/
    ic_wrn ">-- Cloning uwarl-robot_configs @ $ROS_CATKIN_WS/src"
    git clone git@github.com:UW-Advanced-Robotics-Lab/UWARL_catkin_ws.git $ROS_CATKIN_WS/src
    # checkout branch:
    ic_wrn ">-- Checking out branch $UWARL_catkin_ws_branch @ $ROS_CATKIN_WS/src"
    cd $ROS_CATKIN_WS/src && git checkout $UWARL_catkin_ws_branch
    
    ic "x--- Done creating catkin workspace."
}

function load_robot_env() {
    ic ">-- Loading the sourcing robot params @ $UWARL_SUMMIT_ROS_CONFIGS ---> $HOME/.zshrc"
    if grep -Fxq "source $UWARL_SUMMIT_ROS_CONFIGS" "$HOME/.zshrc"; then
        ic_err "[!] $UWARL_SUMMIT_ROS_CONFIGS Already loaded in $HOME/.zshrc"
    else
        # scripts did not exist in the file, then add it to the end of the file
        echo "source $UWARL_SUMMIT_ROS_CONFIGS" >> "$HOME/.zshrc"
        ic_wrn "[x] $UWARL_SUMMIT_ROS_CONFIGS is now being loaded in $HOME/.zshrc"
    fi
    ic "x--- Done loading robot params."
}

function check_submodule_status(){
    # start a new log:
    echo "[Git Status Lastly Updated on $(date)]" > $OUTPUT_STATUS_LOG_DIR
    # update submodules:
    ic "Indexing Submodules Recursively uwarl-robot_configs @ $ROS_CATKIN_WS/src >>--log-->> $OUTPUT_STATUS_LOG_DIR"
    echo "------------------------------------------------------------------------------------------------" >> $OUTPUT_STATUS_LOG_DIR
    echo "------------------------------------------------------------------------------------------------"
    
    i=0
    cd "$ROS_CATKIN_WS/src"
    for dir in */ ; do
        i=$(( i + 1 ))
        if [ "$(ls -A $dir)" ]; then
            ic "[$i] $dir is loaded: "
            ic_log "[$i] $dir is loaded: "
            cd "$ROS_CATKIN_WS/src/$dir"
            # now check changes
            git_stats=$(git status --porcelain)
            git_remote=$(git remote -v)

            ic "   > $dir remote version: \n$git_remote"
            ic_log "   > $dir remote version: \n$git_remote"
            if [[ $(git status --porcelain | wc -l) -gt 0 ]]; then 
                ic_err "   > [!] $dir has changes: \n $git_stats"
                ic_log "   > [!] $dir has changes: \n $git_stats"
            else   
                ic "   > [OK] $dir is up-to-date"
                ic_log "   > [OK] $dir is up-to-date"
            fi 

            # git status
            #
            cd "$ROS_CATKIN_WS/src"
        else
            ic_wrn "[$i] $dir submodule is not loaded! "
            ic_log "[$i] $dir submodule is not loaded! "
        fi
        echo "------------------------------------------------------------------------------------------------" >> $OUTPUT_STATUS_LOG_DIR
        echo "------------------------------------------------------------------------------------------------"
    done
    
    ic "x- Done indexing submodules."
}


function install_ros_noetic(){
    ic_wrn "[Installing ROS Noetic] ..."

    ic "> Appending the ROS Noetic package list to sources.list"
    sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${version} main\" > /etc/apt/sources.list.d/ros-latest.list"

    #Checking file added or not
    if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
        ic_err ">>> {Error: Unable to add sources.list, exiting}"
        exit 0
    else
        ic_wrn ">>> ROS Noetic Package is now in the list!"
    fi

    ic "> Install Curl"
    sudo apt install curl

    ic "> Curling ROS Keys ..."
    ret=$(curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -)

    case $ret in
        "OK" )
            echo ">>> [x] Done!"
        ;;
        *)
            echo ">>> {ERROR: Unable to add ROS keys}"
            exit 0
    esac

    ic "> Updating ..."
    sudo apt update
    ic "> Installing ROS ... (might take a while)"
    sudo apt install -y ros-noetic-desktop-full
    source /opt/ros/noetic/setup.zsh
    ic_wrn "x- Done ROS installation."

    ic "> install Catkin Build tools ... "
    sudo apt-get install python3-pip
    sudo pip3 install -U catkin_tools
    catkin config --extend /opt/ros/noetic
    ic_wrn "x- Done Catkin Tools"
}