source "$HOME/uwarl-robot_configs/scripts/common.sh"

ic " > Path of ROS Catkin Workspace: $ROS_CATKIN_WS"
echo "" # empty line
## kill processor:
ic_bind_cmd kill-ros            "ps aux  | grep -e ros | awk '{print $2}' | xargs -i -exec kill -9 {}"

## Catkin Workspace:
### Git:
ic_bind_cmd commit_ws           "source $UWARL_CONFIGS/scripts/git_functions.sh && commit_ws"
ic_bind_cmd check_ws_status     "source $UWARL_CONFIGS/scripts/git_functions.sh && check_submodule_status $ROS_CATKIN_WS/src"
ic_bind_cmd check_config_status "source $UWARL_CONFIGS/scripts/git_functions.sh && check_submodule_status $UWARL_CONFIGS"
ic_bind_cmd check_status        "source $UWARL_CONFIGS/scripts/git_functions.sh && check_submodule_status $(pwd)"
ic_bind_cmd update_ws           "zsh $UWARL_CONFIGS/scripts/auto-config_UWARL_catkin_ws.zsh"
ic_bind_cmd git_log             "git log --graph --pretty=format:'%Cred%h%Creset -%C(yellow)%d%Creset %s %Cgreen(%cr) %C(bold blue)<%an>%Creset' --abbrev-commit"
ic_bind_cmd sync_latest         "source $UWARL_CONFIGS/scripts/git_functions.sh && sync_latest"
ic_bind_cmd jack                "zsh $UWARL_CONFIGS/scripts/auto-config_UWARL_catkin_ws.zsh && ic_wrn \" Jack has checked for you, no problem :) X_O \" "

### catkin build from anywhere:
if [[ -v PYTHONPATH_ROS ]]; then
    ic_bind_cmd build_ws "cd $ROS_CATKIN_WS && catkin build -DPYTHON_EXECUTABLE=$PYTHONPATH_ROS"
else
    ic_bind_cmd build_ws "cd $ROS_CATKIN_WS && catkin build"
fi

## ROS src:
ic_bind_cmd source_ws    "source $ROS_CATKIN_WS/devel/setup.zsh"
ic_bind_cmd source_zsh   "source $HOME/.zshrc"
ic_bind_cmd source_all   "source $HOME/.zshrc && source $ROS_CATKIN_WS/devel/setup.zsh"
ic_bind_cmd cd_ws        "cd $ROS_CATKIN_WS/src"
ic_bind_cmd cd_config    "cd $UWARL_CONFIGS"
ic_bind_cmd clean_ws     "cd $ROS_CATKIN_WS/src && catkin clean"
## Linux:
ic_bind_cmd cd_linux     "cd $JX_LINUX" # to access libraries installed automatically or manually
## Ubuntu Specific:
if [[ "$LOCAL_DISTRIB_DESCRIPTION" == *"Ubuntu"* ]]; then
    ic_bind_cmd open         xdg-open
fi

## ROS launch in tmux:
ic_bind_cmd tmux_sync    tmux_sync # [session name] [cmd_1] **...
ic_bind_cmd tmux_usync   tmux_usync # [session name] [cmd_1] **...
ic_bind_cmd tmux_multi   tmux_multi_pane # [**number of panes]
ic_bind_cmd tmux_kill    "tmux kill-session" #: kill all sessions
ic_bind_cmd tmux_src     "tmux source-file $UWARL_CONFIGS/desktop/tmux.conf" #: apply custom settings to current tmux session

## Markdown script:
echo "" # empty line
ic_source "$UWARL_CONFIGS/scripts/markdown-toc.sh" "Markdown Toc Generator"
ic_bind_cmd md_toc       "markdown_toc"
ic_bind_cmd md_toc_dir   "markdown_toc_directory"

## Host VNC: `$ host_vnc {display_id:0,1,2}` [ARCHIVED: no longer used, as vnc requires physical monitor, and slow/insecure]
# ic_bind_cmd host_vnc     "/usr/bin/x11vnc -forever -bg -usepw -httpdir /usr/share/vnc-java/ -httpport 5901 -display :$1"

## shortcut of shortcuts sequence
ic_bind_cmd update_all   "cd_config && git pull &&"

## shortcuts of rosrun debug:
# 1. make sure build with debug: `$ build_ws --cmake-args -DCMAKE_BUILD_TYPE=Debug`
ic_bind_cmd build_ws_debug  "build_ws -DCMAKE_BUILD_TYPE=Debug"
# 2.a) rosrun_gdb: program crash with context
ic_bind_cmd rosrun_gdb      "rosrun --prefix 'gdb -ex=run -ex=bt -ex=quit --args' " # [ros_pkg] [ros_node] [args]
# 2.b) rosrun_valgrind: memory-related
ic_bind_cmd rosrun_valgrind "rosrun --prefix 'valgrind --log-file="$HOME/valgrind_log.txt"' " # [ros_pkg] [ros_node] [args]
# 2.c) rosrun_valgrind: memory-related with memory leak check
ic_bind_cmd rosrun_valgrind_leaky "rosrun --prefix 'valgrind --log-file="$HOME/valgrind_log.txt" --leak-check=full' " # [ros_pkg] [ros_node] [args]
# 3. console out the report:
ic_bind_cmd valgrind_report "cat $HOME/valgrind_log.txt" 

## Hardware specific shortcuts:
ic_title "Hardware Specific Shortcuts: "
case $UWARL_ROBOT_PC_NAME in
    "ADLINK_MXE211_SUMMIT" )
        ## [Summit] ROS Core Systemctl:
        #     - Pre-req:    `$ source ~/uwarl-robot_configs/scripts/git_functions.sh && install_roscore_systemctl_service`
        #     - Template:   ic_bind_cmd [xx]_systemctl "source ~/uwarl-robot_configs/scripts/git_functions.sh && ros_systemctl [type] [ros_pkg] [ros_launch_file]"
        ic_bind_cmd summit_systemctl "source ~/uwarl-robot_configs/scripts/git_functions.sh && ros_systemctl roscorelaunch waterloo_steel_summit_bringup waterloo_steel_summit"
    ;;
    "JETSON_ORIN_WAM")
        ## Only in Jetson:
        if [[ -d "$JX_LINUX/jetsonUtilities" ]]; then
            ic_bind_cmd jetson_info "cd $JX_LINUX/jetsonUtilities && ./jetsonInfo.py && cd -" # Jetson Info
        fi
        ## shortcuts for jetson:
        # need to call udev to reload usb dev rules for librealsense cameras 
        # - needed everytime relaunching the camera with ros node !!
        # - FIXME: ideally they should have kernel patch later 2023 for jetson orin
        ic_bind_cmd restart_udev "sudo service systemd-udevd --full-restart"
    ;;
    # [DEFAULT]:
    *)
    ;;
esac
