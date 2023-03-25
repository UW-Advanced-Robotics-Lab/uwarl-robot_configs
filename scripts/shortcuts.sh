source "$HOME/uwarl-robot_configs/scripts/common.sh"

ic " > Path of ROS Catkin Workspace: $ROS_CATKIN_WS"
echo "" # empty line
## kill processor:
ic_bind_cmd kill-ros "ps aux  | grep -e ros | awk '{print $2}' | xargs -i -exec kill -9 {}"

## Catkin Workspace:
### Git:
ic_bind_cmd commit_ws "source $UWARL_CONFIGS/scripts/git_functions.sh && commit_ws"
ic_bind_cmd check_ws_status "source $UWARL_CONFIGS/scripts/git_functions.sh && check_submodule_status"
ic_bind_cmd update_ws "zsh $UWARL_CONFIGS/scripts/auto-config_UWARL_catkin_ws.zsh"
ic_bind_cmd git_log "git log --graph --pretty=format:'%Cred%h%Creset -%C(yellow)%d%Creset %s %Cgreen(%cr) %C(bold blue)<%an>%Creset' --abbrev-commit"

### catkin build from anywhere:
if [ $ROS_DISTRO = "melodic" ]; then
    ic_bind_cmd build_ws "cd $ROS_CATKIN_WS && catkin build"
else
    ic_bind_cmd build_ws "cd $ROS_CATKIN_WS && catkin build -DPYTHON_EXECUTABLE=$PYTHONPATH_ROS"
fi

## ROS src:
ic_bind_cmd source_ws    "source $ROS_CATKIN_WS/devel/setup.zsh"
ic_bind_cmd source_zsh   "source $HOME/.zshrc"
ic_bind_cmd source_all   "source $HOME/.zshrc && source $ROS_CATKIN_WS/devel/setup.zsh"
ic_bind_cmd cd_ws        "cd $ROS_CATKIN_WS/src"
ic_bind_cmd cd_config    "cd $UWARL_CONFIGS"
ic_bind_cmd clean_ws     "catkin clean"
## Linux:
ic_bind_cmd cd_linux     "cd $JX_LINUX" # to access libraries installed automatically or manually

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

## Only in Jetson:
if [[ $USER = "uwarl-orin" ]] && [[ $LOCAL_PC_IP = "$ROS_WAM_IP" ]]; then
    if [[ -d "$JX_LINUX/jetsonUtilities" ]]; then
        ic_bind_cmd jetson_info "./$JX_LINUX/jetsonUtilities/jetsonInfo.py" # Jetson Info
    fi
fi
