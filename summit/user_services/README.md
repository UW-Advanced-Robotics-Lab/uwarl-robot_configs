# Setting up ros systemctl at bootup
- this service package will help you to define services that can launch services at the boot up

## Instruction:
```bash
$ cd ~
# setup ros environment to `~/.ros/`
$ sudo cp $HOME/uwarl-robot_configs/summit/user_services/environment $HOME/.ros/environment

# load system services:
#[OPTIONAL] roscore only:
$ sudo cp $HOME/uwarl-robot_configs/summit/user_services/roscore.service /usr/lib/systemd/user
#[this one] roscore and roslaunch:
$ sudo cp $HOME/uwarl-robot_configs/summit/user_services/roscorelaunch@.service /usr/lib/systemd/user
#[OPTIONAL] depends on remote roscore:
$ sudo cp $HOME/uwarl-robot_configs/summit/user_services/roslaunch@.service /usr/lib/systemd/user 

# compile catkin workspace:
$ build_ws

# create launch for summit:
$ systemctl --user daemon-reload
$ systemctl --user enable roscorelaunch@waterloo_steel_bringup:waterloo_steel_summit.launch

# uninstall:
$ systemctl --user disable roscorelaunch@waterloo_steel_bringup:waterloo_steel_summit.launch

# Start at bootup instead of graphical login
sudo loginctl enable-linger $USER
```