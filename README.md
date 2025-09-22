# Sinthlab kuka stack
## Steps on Kuka Arm
- Use the "Quick Start guide" to cable up the Arm, Cabinet and the SmartPad
- For the Safety on X11 interface in the Cabinet:
  - We first create 6 Jumper cables witht he provided Pins.
  - Based on the PIN diagram as elaborated in pg-53 of Spez_KUKA_Sunrise_Cabinet_en.pdf (section 6.6.2), we jump pins 1/2, 10/11 (external E-Stop), 3/4, 12/13 (Operator Safety) and 5/6, 14/15 (Safety stop 1).

## Setting up the Stack
*Note*: These steps are meant to be run on Ubuntu 22.04. If using Windows box, please setup Ubuntu 22.04 in your wsl (Note the version is improtant. please do not install anything default), and make sure you are on wsl version 2 (running wsl -l -v should show wsl version and ubuntu name installed).
- Install ROS 2 development tools. Refer this [link]( https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html#setup-sources) to understand details of steps.
  
  ```
  sudo apt install software-properties-common
  sudo add-apt-repository universe
  sudo apt update && sudo apt install curl -y
  export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  sudo apt update
  sudo apt upgrade
  sudo apt install ros-dev-tools
  ```
  
- Create a workspace, clone, and install dependencies
  ```
  source /opt/ros/humble/setup.bash
  mkdir -p lbr-stack/src && cd lbr-stack
  vcs import src --input https://raw.githubusercontent.com/sinthlab/sinthlab-kuka-stack/main/sinthlab_lbr_stack.repos
  rosdep install --from-paths src -i -r -y
  ```
- Build
  ```
  colcon build --symlink-install
  ```

## Visualizing mock setup on rviz
- In terminal 1, launch a mock setup via
```
source install/setup.bash
ros2 launch lbr_bringup mock.launch.py model:=iiwa7
```

- In terminal 2, visualize setup via
```
source install/setup.bash
ros2 launch lbr_bringup rviz.launch.py \
    rviz_cfg_pkg:=lbr_bringup \
    rviz_cfg:=config/mock.rviz
```
**Troubleshoot**: 
- if your rviz windows launhes but is not displaying anything, It might be because the graphics library might be trying to use the graphics card which might not be set. use command ` export LIBGL_ALWAYS_SOFTWARE=1` to first set the library to use CPU, and then run the second step again.

## Acknowledgement
Please note that this work has been built on top if Huber et al [^1] and hence all the original work credit goes to that team.

[^1]: LBR-Stack: ROS 2 and Python Integration of KUKA FRI for Med and IIWA Robots, Journal of Open Source Software [doi](https://doi.org/10.21105/joss.06138)
