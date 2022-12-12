#!/usr/bin/env zsh
source "$HOME/uwarl-robot_configs/scripts/common.sh"

function load_submodules(){
    ic_title "Loading Submodules ..."
    # update submodules:
    ic_wrn ">-- Loading Submodules Recursively uwarl-robot_configs @ $ROS_CATKIN_WS/src"

    list_of_modules=("$@")
    total=${#list_of_modules[@]}
    i=0
    for module in "${list_of_modules[@]}"; do
        i=$(( i + 1 ))
        ic_wrn "    > [$i/$total] - Loading submodule @ $module"
        cd $ROS_CATKIN_WS/src
        git submodule update --init --recursive $module
    done

    # install dependencies:
    if [[ $ROS_DISTRO == "noetic" ]]; then
        ic  "ROS: [Noetic]"
        ic_err "[ERR] Missing Ros Dep Tooling"
        ic "> install python3-rosdep"
        sudo apt install python3-rosdep
        sudo rosdep init
        ic_wrn "x- Done python3 rosdep"
    else
        if [ "$(dpkg -l | awk '/rosdep/ {print }'|wc -l)" -ge 1 ]; then
            ic "rosdep exists!"
        else
            ic  "ROS: [Melodic]"
            ic_err "[ERR] Missing Ros Dep Tooling"
            ic "> install python-rosdep"
            sudo apt install python-rosdep
            sudo rosdep init
            ic_wrn "x- Done python rosdep"
        fi
    fi
    ic_wrn ">-- Install ros dependencies @ $ROS_CATKIN_WS"
    cd $ROS_CATKIN_WS && rosdep update
    cd $ROS_CATKIN_WS && rosdep install --from-paths src --ignore-src -r -y
    ic "x--- Done loading submodules."
}

function create_catkin_ws(){
    ic_title "Creating Catkin Workspace ..."
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

function create_JX_Linux(){
    ic_title "Creating `JX_Linux` Third Party Pkg Directory ..."
    # Create catkin workspace
    ic_wrn ">-- Creating `JX_Linux` @ $JX_LINUX"
    cd $HOME
    mkdir -p $JX_LINUX
    ic "x--- Done creating `JX_Linux` Third Party Pkg Dir."
}

function install_pcan(){
    ic_title "Installing `pcan_linux_driver` into $JX_LINUX ..."
    ic_wrn ">-- Please note that this installation may not support custom kernel-header"
    
    ic_wrn ">-- Download `pcan_linux_driver`"
    cd $JX_LINUX
    # download driver:
    wget https://www.peak-system.com/fileadmin/media/linux/files/peak-linux-driver-8.15.2.tar.gz
    # unzip:
    tar -xzf peak-linux-driver-8.15.2.tar.gz
    cd peak-linux-driver-8.15.2

    if [[ $1 = "NETDEV_SUPPORT" ]]; then
        ic_wrn ">-- Make PCAN with NETDEV:"
        sudo make -C driver NET=NETDEV_SUPPORT 
    else
        ic_wrn ">-- Make PCAN without ~NETDEV:"
        sudo make -C driver 
    fi

    ic_wrn ">-- Install `pcan_linux_driver`"
    sudo make install
    ic_wrn ">-- Probing `pcan_linux_driver`"
    sudo modprobe pcan
    ic_wrn ">-- Checking `pcan_linux_driver`"
    sudo dmesg | grep pcan

    ic "x--- Done installling `pcan_linux_driver`! "
}


function load_common() {
    ic_title "Loading Common Environment Parameters ..."
    ic ">-- Loading the sourcing robot params @ $COMMON_ROBOT_CONFIGS ---> $HOME/.zshrc"
    if grep -Fxq "source $COMMON_ROBOT_CONFIGS" "$HOME/.zshrc"; then
        ic_err "   [!] $COMMON_ROBOT_CONFIGS Already loaded in $HOME/.zshrc"
    else
        # scripts did not exist in the file, then add it to the end of the file
        echo "source $COMMON_ROBOT_CONFIGS" >> "$HOME/.zshrc"
        echo "source_all_common_configs # sourcing all configs" >> "$HOME/.zshrc"
        ic_wrn "   [x] $COMMON_ROBOT_CONFIGS is now being loaded in $HOME/.zshrc"
    fi
    ic "x--- Done loading common params."
}

function check_submodule_status(){
    ic_title "Checking Submodule Status ..."
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
            git_head=$(git rev-parse --abbrev-ref HEAD)
            git_head_ver=$(git rev-parse --short HEAD)
            ic "   > $dir on branch @ [$git_head_ver] $git_head"
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
    ic_title "Installing ROS Noetic ..."

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

    ic "> install python3-rosdep2"
    sudo apt install python3-rosdep2
    ic_wrn "x- Done python3 rosdep2"
}