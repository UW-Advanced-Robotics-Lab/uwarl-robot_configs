# uwarl-robot_configs
This repo will keep track of the configuration files

## WAM:
[TODO]

## SUMMIT:
### ⭐ Local Build (Off-robot PCs):

1. Clone configurations: 
    ```bash
    $ cd ~ && git clone git@github.com:UW-Advanced-Robotics-Lab/uwarl-robot_configs.git
    ```

2. add to `~/zshrc/bashrc` for terminals:
    ```bash
    ### >>> Custom entries:
    source /home/{user}/uwarl-robot_configs/summit/summitxl_ros_config.zsh
    ### <<<< EOF .
    ```

### [⭐ A better version] RC Config for ROS and devices

- Lets use system boot as a method to auto boot services, as they can be restarted easily with status logs without hosting virtual terminals

- User system Permissions:

  ```
  sudo usermod -a -G dialout $USER 
  sudo usermod -a -G root $USER
  ```

- Just modify the file from `uwarl-robot_configs` repository as needed

- Configuration:

  1. Clone configurations: 

     ```bash
     $ cd ~ && git clone git@github.com:UW-Advanced-Robotics-Lab/uwarl-robot_configs.git
     ```

  2. Make sure directories and ip are right in `~/uwarl-robot_configs/summit/user_services/environment`

  3. Add to `~/.zshrc`:

     ```bash
      sudo cp ~/uwarl-robot_configs/summit/user_services/environment ~/.ros/
     ```

- Create auto roslaunch:

  ```bash
  $ cd ~
  # load system services:
  #[OPTIONAL] roscore only:
  $ sudo cp uwarl-robot_configs/summit/user_services/roscore.service /usr/lib/systemd/user
  #[this one] roscore and roslaunch:
  $ sudo cp uwarl-robot_configs/summit/user_services/roscorelaunch@.service /usr/lib/systemd/user
  #[OPTIONAL] depends on remote roscore:
  $ sudo cp uwarl-robot_configs/summit/user_services/roslaunch@.service /usr/lib/systemd/user 
  
  # create launch for summit:
  $ systemctl --user daemon-reload
  $ systemctl --user enable roscorelaunch@summit_xl_bringup:summit_xl_complete.launch
  
  # Start at bootup instead of graphical login
  sudo loginctl enable-linger $USER
  ```

- Check:

  ```bash
  # check system:
  $ systemctl --user status roscorelaunch@summit_xl_bringup:summit_xl_complete.launch.service
  
  # restart:
  $ systemctl --user restart roscorelaunch@summit_xl_bringup:summit_xl_complete.launch.service
  
  # stop:
  $ systemctl --user stop roscorelaunch@summit_xl_bringup:summit_xl_complete.launch.service
  
  # check log:
  $ journalctl --user --user-unit=roscorelaunch@summit_xl_bringup:summit_xl_complete.launch.service
  # live:
  $ journalctl --follow --user --user-unit=roscorelaunch@summit_xl_bringup:summit_xl_complete.launch.service
  ```

- add to `~/zshrc/bashrc` for terminals:

  ```bash
  ### >>> Custom entries:
  source /home/uwarl/uwarl-robot_configs/summit/summitxl_ros_config.zsh
  # source /home/uwarl/uwarl-robot_configs/summit/summitxl_ros_config.bash # bash
  ### <<<< EOF .
  ```

  

#### summitxl_params.env

Robot Configuration Description:
      - ROBOT_ID indicates the name of the robot. This is the name of the namespace under all the nodes will be working. This is also used as the prefix of all the subcomponents.(*summit_xl*)

      - ROBOT_XACRO indicates the path where the xacro file is. (inside the robot folder in robot_description)(*summit_xl.urdf.xacro*)

      - ROBOT_FRONT_LASER_MODEL indicates the model of the laser that the robot is using. The model is the name of the launch file.(*sick_tim561/hokuyo_ug01/hokuyo_ust*)

      - ROBOT_REAR_LASER_MODEL indicates the model of the laser that the robot is using. The model is the name of the launch file.(*sick_tim561/hokuyo_ug01/hokuyo_ust*)

      - ROBOT_HAS_FRONT_LASER indicates if the robot has a laser in front. (*true/false*)

      - ROBOT_HAS_REAR_LASER indicates if the robot has a laser in rear. (*true/false*)

      - ROBOT_HAS_FRONT_PTZ_CAMERA indicates if the robot has the ptz camera in front. (*true/false*)

      - ROBOT_HAS_REAR_PTZ_CAMERA indicates if the robot has the ptz camera in front. (*true/false*)

      - ROBOT_HAS_GPS indicates if the robot has gps. (*true/false*)

      - ROBOT_HAS_FRONT_RGBD_CAMERA indicates if the robot has a front rgbd camera. (*true/false*)

      - ROBOT_FRONT_RGBD_CAMERA_ID camera id to identify in the bus

      - ROBOT_HAS_REAR_RGBD_CAMERA indicates if the robot has a front rgbd camera. (*true/false*)

      - ROBOT_REAR_RGBD_CAMERA_ID camera id to identify in the bus

      - ROBOT_PAD_MODEL pad model used. (*ps4/ps3/logitechf710/xbox360*)

      - ROBOT_GEARBOX establishes the motor gearbox value. (*24V: 12.52 | 48V: 9.56*)

      - ROBOT_HAS_ENCODER indicates if the robot has encoders. (*true/false*)

      - ROBOT_KINEMATICS kinematic configuration of the robot. (*skid/omni/steel_skid/steel_omni*)

      - ROBOT_HAS_ARM indicates if the robot has an arm (*true/false*
