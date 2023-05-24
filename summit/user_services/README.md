<toc>

# Table of Contents
[*Last generated: Wed May 24 16:04:42 EDT 2023*]
- [**Setting up ros systemctl at bootup**](#Setting-up-ros-systemctl-at-bootup)
  - [Automated Instruction [May 24, 2023]:](#Automated-Instruction-May-24-2023)
  - [Instruction [DEPRECATED]:](#Instruction-DEPRECATED)

---
</toc>
# Setting up ros systemctl at bootup
- this service package will help you to define services that can launch services at the boot up

## Automated Instruction [May 24, 2023]:
```bash
# modify summitxl_params.env, and update env with:
$ source_all 

### How to build and install summit workspace:
# 1. build:
$ build_ws
# 2. stop system:
$ summit_systemctl stop
$ summit_systemctl status
# 3. source
$ source_all
# 4. reinstall service
$ summit_systemctl reinstall
# 5. start new system:
$ summit_systemctl start
$ summit_systemctl status
# check status:
$ summit_systemctl status
# now, it will autolaunch the summit in the backend.

### For any other workspace, please use similar idea based on the given scripts for summit workspace.
```

## Instruction [DEPRECATED]:
x- Note: this is deprecated, and it is entirely replaced by automation scripts written by Jack.
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
<eof>

---
[*> Back To Top <*](#Table-of-Contents)
</eof>