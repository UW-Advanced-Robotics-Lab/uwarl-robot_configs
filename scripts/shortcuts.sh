source "$HOME/uwarl-robot_configs/scripts/common.sh"

## kill processor:
ic " Aliasing \`\$ kill-ros\` command @ $UWARL_CONFIGS/scripts/shortcuts.sh"
alias kill-ros="ps aux  | grep -e ros | awk '{print $2}' | xargs -i -exec kill -9 {}"

## Catkin Workspace:
### Git:
ic " Aliasing \`\$ check_ws_status\` command from $UWARL_CONFIGS/scripts/git_functions.sh"
alias check_ws_status="source $UWARL_CONFIGS/scripts/git_functions.sh && check_submodule_status"

ic " Aliasing \`\$ update_ws\` command by $UWARL_CONFIGS/scripts/auto-config_UWARL_catkin_ws.sh"
alias update_ws="./$UWARL_CONFIGS/scripts/auto-config_UWARL_catkin_ws.zsh"

### catkin build from anywhere:
ic " Aliasing \`\$ build_ws\` command @ $UWARL_CONFIGS/scripts/shortcuts.sh"
if [ $ROS_DISTRO = "melodic" ]; then
    alias build_ws="cd $ROS_CATKIN_WS && catkin build"
else
    alias build_ws="cd $ROS_CATKIN_WS && catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3" # for noetic on u20
fi

## ROS src:
alias source_ws="source $ROS_CATKIN_WS/devel/setup.zsh"
alias source_zsh="source $HOME/.zshrc"
alias cd_ws="cd $ROS_CATKIN_WS/src"
alias cd_config="cd $UWARL_CONFIGS"

