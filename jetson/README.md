# Jetson Only Packages:
[Created on Nov 16, 2023]

## OpenCV :
- Default to 4.6.0
```bash
$ cd ~
$ /$HOME/uwarl-robot_configs/jetson/install_opencv.sh
```
> You may specify different version by adding the argument of the version number after the script 

### Discussions:
There are several dependencies on OpenCV:
- When you install based on `sudo apt install ros-noetic-cv-bridge`, it defaults to pre-built opencv 4.2.0, and not CUDA version
- We need to use `install_opencv.sh` to install hardware accelerated CUDA version
- `intel_realsense` ros wrapper has a library dependency on OpenCV, and `librealsense` which is auto-installed by `install_librealsense_if_not` at the config tools
- `VINS` package will have a dependency on OpenCV
- we need to `build_ws` with the hardware accelerated `OpenCV` installed 

