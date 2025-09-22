# Sinthlab kuka stack
## Steps on Kuka Arm
- Use the "Quick Start guide" to cable up the Arm, Cabinet and the SmartPad
- For the Safety on X11 interface in the Cabinet:
  - We first create 6 Jumper cables witht he provided Pins.
  - Based on the PIN diagram as elaborated in pg-53 of Spez_KUKA_Sunrise_Cabinet_en.pdf (section 6.6.2), we jump pins 1/2, 10/11 (external E-Stop), 3/4, 12/13 (Operator Safety) and 5/6, 14/15 (Safety stop 1).

## Setting up the Stack
*Note*: These steps are meant for Ubuntu 22.04. Please use either native Ubuntu or if using windows, you can use wsl but make sure you install Ubuntu version 22.04 on wsl. Note the terminal mentioned below refers to Ubuntu terminal (not Powershell or anything else). 
The steps are NOT valid for any other OS (like MAC).

- Install ROS 2 development tools
  ```
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

## Acknowledgement
Please note that this work has been built on top if Huber et al [^1] and hence all the original work credit goes to that team.

[^1]: LBR-Stack: ROS 2 and Python Integration of KUKA FRI for Med and IIWA Robots, Journal of Open Source Software [doi](https://doi.org/10.21105/joss.06138)
