source "$HOME/uwarl-robot_configs/scripts/common.sh"

ic " > Path of ROS Catkin Workspace: $ROS_CATKIN_WS"
## kill processor:
ic_bind_cmd kill-ros "ps aux  | grep -e ros | awk '{print $2}' | xargs -i -exec kill -9 {}"

## Catkin Workspace:
### Git:
ic_bind_cmd commit_ws "source $UWARL_CONFIGS/scripts/git_functions.sh && commit_ws"
ic_bind_cmd check_ws_status "source $UWARL_CONFIGS/scripts/git_functions.sh && check_submodule_status"
ic_bind_cmd update_ws "zsh $UWARL_CONFIGS/scripts/auto-config_UWARL_catkin_ws.zsh"

### catkin build from anywhere:
if [ $ROS_DISTRO = "melodic" ]; then
    ic_bind_cmd build_ws "cd $ROS_CATKIN_WS && catkin build"
else
    ic_bind_cmd build_ws "cd $ROS_CATKIN_WS && catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3" # for noetic on u20
fi

## ROS src:
ic_bind_cmd source_ws    "source $ROS_CATKIN_WS/devel/setup.zsh"
ic_bind_cmd source_zsh   "source $HOME/.zshrc"
ic_bind_cmd source_all   "source $HOME/.zshrc && source $ROS_CATKIN_WS/devel/setup.zsh"
ic_bind_cmd cd_ws        "cd $ROS_CATKIN_WS/src"
ic_bind_cmd cd_config    "cd $UWARL_CONFIGS"

## Host VNC: `$ host_vnc {display_id:0,1,2}`
ic_bind_cmd host_vnc     "/usr/bin/x11vnc -forever -bg -usepw -httpdir /usr/share/vnc-java/ -httpport 5901 -display :$1"