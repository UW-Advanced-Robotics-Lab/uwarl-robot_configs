#!/usr/bin/env zsh
source "$HOME/uwarl-robot_configs/scripts/common.sh"
local_change_counter=0

# checks if branch has something pending
function parse_git_dirty() {
    git diff --quiet --ignore-submodules HEAD 2>/dev/null; [ $? -eq 1 ] && echo "*"
}

# gets the current git branch
function parse_git_branch() {
    git branch --no-color 2> /dev/null | sed -e '/^[^*]/d' -e "s/* \(.*\)/\1$(parse_git_dirty)/"
}

# get last commit hash prepended with @ (i.e. @8a323d0)
function parse_git_hash() {
    git rev-parse --short HEAD 2> /dev/null | sed "s/\(.*\)/@\1/"
}

function commit_ws() {
    check_submodule_status
    
    ic_title "Package Workspace and Commit to Git"
    cd $ROS_CATKIN_WS/src
    if [[ $local_change_counter == 0 ]]; then
        ic_wrn " - Looking good! Committing Workspace ..."
        git add .
        git commit 
        ic_wrn " - All done! You may now push the changes to remote!"
    else
        ic_err " - There are ${local_change_counter} submodule changes to commit before saving the workspace!"
        ic_wrn " - Please cd into submodule && commit submodule changes first!"
        ic_wrn " - Abort!"
    fi
}

function apt_install(){
    if dpkg --get-selections | grep -q "^$1[[:space:]]*install$" >/dev/null; then
        ic_wrn "> Already installed [$1] "
    else
        ic_err "> Missing [$1] "
        ic "    > install [$1] "
        sudo apt install $1
        ic "    x- Installation Complete!"
    fi
}

function load_submodules(){
    ic_title "Loading Submodules ..."
    # check current branch on git configs:
    cd $UWARL_CONFIGS 
    local config_tooling_branch=$(git rev-parse --symbolic-full-name --abbrev-ref HEAD)
    local config_tooling_existed_in_remote=$(git ls-remote --heads origin ${config_tooling_branch})
    cd $ROS_CATKIN_WS/src
    git fetch
    local catkin_ws_existed_in_remote=$(git ls-remote --heads origin ${UWARL_catkin_ws_branch})
    ## Check if both branches are the same: 
    if [[ $config_tooling_branch = $UWARL_catkin_ws_branch ]]; then
        ic_wrn ">-- Both Workspace and Configs are targeting the same branch name [$UWARL_catkin_ws_branch]"
    else
        ic_err ">-- Workspace and Configs do not share the same branch name [$UWARL_catkin_ws_branch != $config_tooling_branch]"
        ic_wrn ">-- Continuing checking out workspace with [$UWARL_catkin_ws_branch], please make sure this is intentional! "
    fi
    ## Check if both branches exist on remote:
    if [[ -z ${config_tooling_existed_in_remote} ]]; then
        ic_wrn ">-- Did not find [$config_tooling_branch] on remote, please check if this is intentional! "
    fi
    if [[ -z ${catkin_ws_existed_in_remote} ]]; then
        # if not, exit: (This function is supposed to be used for updating the workspace from remote.)
        ic_err ">-- Did not find [$UWARL_catkin_ws_branch] on remote, please check if this is intentional!"
        ic_err ">-- Aborting Updating Workspace!"
        ic_wrn ">-- List of available remote branches:"
        git ls-remote --heads
        exit 0 
    else
        # checkout branch:
        ic_wrn ">-- Checking out $ROS_CATKIN_WS/src @ branch [$UWARL_catkin_ws_branch]"
        git pull origin $UWARL_catkin_ws_branch
        git checkout $UWARL_catkin_ws_branch
    fi

    # update submodules:
    ic_wrn ">-- Loading Submodules Recursively uwarl-robot_configs @ $ROS_CATKIN_WS/src"

    ## Indexing every modules given in common based on the system you have, and load them one by one:
    ##      1. If there are any missing modules, it will initialize them and pull from remote origin. 
    ##      2. If there are any modules that are not in the list, it will be not synced. 
    ##              - You have to manually delete them from the workspace, before pushing to remote.
    ##      3. If there are any modules that are not in the remote, it will be not synced.
    ##      4. If there are any modules are outdated, it will be pulled from origin.
    ##      5. If there are any modules have local changes, it will be not synced.
    
    local list_of_modules=("$@")
    local total=${#list_of_modules[@]}
    local i=0
    for module in "${list_of_modules[@]}"; do
        i=$(( i + 1 ))
        ic "    > [$i/$total] - Loading submodule @ $module"
        cd $ROS_CATKIN_WS/src
        if [ -d "ls -A $dir" ]; then
            ic_wrn "    > Directory [$module] is empty, initialize submodules! "
            # checkout submodules
            git submodule update --init --recursive $module
        else
            # update submodules
            ic_wrn "       > Directory [$module] is not empty, entering submodules! "
            cd $module
            local git_submodule_branch_name=$(parse_git_branch)$(parse_git_hash)
            local submodule_branch_name=$(git rev-parse --symbolic-full-name --abbrev-ref HEAD)
            if [[ $submodule_branch_name =~ "HEAD" ]]; then
                ic_wrn "       > [$module] - Submodule @ HEAD already up to date!"
                ic_wrn "       > [$module] - Skip pulling from remote origin"
            else
                ic_wrn "       > [$module] - Submodule @ $submodule_branch_name "
                local existed_in_remote=$(git ls-remote --heads origin ${submodule_branch_name})
                if [[ -z ${existed_in_remote} ]]; then
                    ic_err "       > [$module] - Did not found on remote, please check if this is intentional! "
                    ic_wrn "                   > Skip pulling from remote origin"
                else
                    local local_ahead_count=$(git rev-list --count @{u}..HEAD)
                    if [[ $local_ahead_count == 0 ]]; then
                        ic_wrn "       > [$module] - Found on remote, attempt to pull from remote origin"
                        git pull origin $submodule_branch_name
                    else
                        ic_err "       > [$module] - Found on remote, but local changes is ahead of remote, please check if this is intentional! "
                        ic_wrn "                   > Skip pulling from remote origin"
                    fi
                fi
            fi
            cd ..
        fi
    done

    # install dependencies:
    if [[ $ROS_DISTRO == "noetic" ]]; then
        ic  "ROS: [Noetic]"
        apt_install python3-rosdep
    else
        if [ "$(dpkg -l | awk '/rosdep/ {print }'|wc -l)" -ge 1 ]; then
            ic "rosdep exists!"
        else
            ic  "ROS: [Melodic]"
            apt_install python-rosdep
        fi
    fi
    ic_wrn ">-- Install ros dependencies @ $ROS_CATKIN_WS"
    local default_rosdep_file="/etc/ros/rosdep/sources.list.d/20-default.list"
    if test -f $default_rosdep_file; then
        ic_wrn "> $default_rosdep_file already exists!"
    else
        in_wrn "> Now, initing rosdep!"
        sudo rosdep init
        ic_wrn "> Updating rosdep ..."
        cd $ROS_CATKIN_WS && rosdep update
    fi
    ic_wrn "> Instaling rosdep fron $ROS_CATKIN_WS/src ..."
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

function install_pcan_if_not(){
    ic_title "Installing `pcan_linux_driver` into $JX_LINUX ..."
    ic_wrn ">-- Please note that this installation may not support custom kernel-header"
    if [[ -d "$JX_LINUX/peak-linux-driver-8.15.2" ]]; then
        ic_err " [!] Peak Linux Driver Areadly Installed!"
    else
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
    fi
}

function install_libbarrett_if_not(){
    ic_title "Installing libbarrett into $JX_LINUX ..."
    if [[ -d "$JX_LINUX/uwarl-libbarrett" ]]; then
        ic_err " [!] Libbarrett Areadly Installed!"
    else
        ic_wrn ">-- Cloning uwarl-libbarrett"
        cd $JX_LINUX
        git clone git@github.com:UW-Advanced-Robotics-Lab/uwarl-libbarrett.git
        
        ic_wrn ">-- Installing dependencies from uwarl-libbarrett"
        cd $JX_LINUX/uwarl-libbarrett/scripts
        bash install_dependencies.sh # <-- this script is local reference only

        ic_wrn ">-- Build uwarl-libbarrett"
        cd $JX_LINUX/uwarl-libbarrett
        export CC=/usr/bin/clang
        export CXX=/usr/bin/clang++
        cd $JX_LINUX/uwarl-libbarrett && cmake .
        make -j$(nproc)

        ic_wrn ">-- Install uwarl-libbarrett"
        sudo make install

        ic_wrn ">-- Build uwarl-libbarrett/examples"
        cd $JX_LINUX/uwarl-libbarrett/examples && cmake .
        make -j$(nproc)
        ic "x--- Done installling libbarrett! "
        ic_err "[Reboot Required] Please reboot !"
    fi
}

function install_librealsense_if_not(){
    ic_title "Installing librealsense into $JX_LINUX ..."
    local candidate_path="$JX_LINUX/librealsense"
    if [[ -d "$candidate_path" ]]; then
        ic_err " [!] librealsense Areadly Installed!"
    else
        ic_wrn ">-- Pre-req:"
        sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
        sudo apt-get install -y git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev

        ic_wrn ">-- Cloning librealsense"
        cd $JX_LINUX
        git clone https://github.com/IntelRealSense/librealsense.git
        cd $candidate_path

        ic_wrn ">-- Setup Udev:"
        ./scripts/setup_udev_rules.sh  
        
        ic_wrn ">-- Install libuvc:"
        ./scripts/libuvc_installation.sh

        ic_wrn ">-- Prepare librealsense cmake files:"
        mkdir $candidate_path/build && cd $candidate_path/build
        cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=true -DFORCE_RSUSB_BACKEND=true -DBUILD_PYTHON_BINDINGS=true  -DBUILD_GRAPHICAL_EXAMPLES=true  -DBUILD_WITH_CUDA=false  -DPYTHON_EXECUTABLE=/usr/bin/python3
        
        ic_wrn ">-- Build librealsense"
        make -j$(($(nproc)-1)) 

        ic_wrn ">-- Install librealsense"
        sudo make install

        ic "x--- Done installling librealsense! "
        ic_err "[Reboot Required] Please reboot !"
    fi
}
function install_dlink_dongle(){
    ic_title "Installing DLink Wifi Dongle into $JX_LINUX ..."
    local candidate_path="$JX_LINUX/rtl88x2bu"
    if [[ -d "$candidate_path" ]]; then
        ic_err " [!] rtl88x2bu Areadly Installed!"
    else
        ic_wrn ">-- Cloning rtl88x2bu"
        cd $JX_LINUX
        git clone https://github.com/cilynx/rtl88x2bu.git
        cd $candidate_path
        
        ic_wrn ">-- Build librealsense"
        make -j$(($(nproc)-1)) ARCH=arm64

        ic_wrn ">-- Install librealsense"
        sudo make install

        ic "x--- Done installling librealsense! "
        ic_err "[Reboot Required] Please reboot !"
    fi
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
    ic_wrn "[Git Status Lastly Updated on $(date) by $USER]"
    echo "[Git Status Lastly Updated on $(date) by $USER]" > $OUTPUT_STATUS_LOG_DIR
    # update submodules:
    ic "Indexing Submodules Recursively uwarl-robot_configs @ $ROS_CATKIN_WS/src >>--log-->> $OUTPUT_STATUS_LOG_DIR"
    echo "------------------------------------------------------------------------------------------------" >> $OUTPUT_STATUS_LOG_DIR
    echo "------------------------------------------------------------------------------------------------"
    
    cd "$ROS_CATKIN_WS/src"
    local i=0
    local_change_counter=0
    for dir in */ ; do
        i=$(( i + 1 ))
        if [ "$(ls -A $dir)" ]; then
            ic "[$i] $dir is loaded: "
            ic_log "[$i] $dir is loaded: "
            cd "$ROS_CATKIN_WS/src/$dir"
            # now check changes
            local git_stats=$(git status --porcelain)
            local git_remote=$(git remote -v)
            local git_head=$(git rev-parse --abbrev-ref HEAD)
            local git_head_ver=$(git rev-parse --short HEAD)
            ic "   > $dir on branch @ [$git_head_ver] $git_head"
            ic "   > $dir remote version: \n$git_remote"
            ic_log "   > $dir remote version: \n$git_remote"
            if [[ $(git status --porcelain | wc -l) -gt 0 ]]; then 
                ic_err "   > [!] $dir has changes: \n $git_stats"
                ic_log "   > [!] $dir has changes: \n $git_stats"
                local_change_counter=$(( local_change_counter + 1 ))
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

    apt_install curl

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
    apt_install -y ros-noetic-desktop-full
    source /opt/ros/noetic/setup.zsh
    ic_wrn "x- Done ROS installation."

    ic "> install Catkin Build tools ... "
    apt_install python3-pip
    sudo pip3 install -U catkin_tools
    catkin config --extend /opt/ros/noetic
    ic_wrn "x- Done Catkin Tools"

    # additional toolsets:
    apt_install ros-noetic-rosbash
    apt_install ros-noetic-rviz
}