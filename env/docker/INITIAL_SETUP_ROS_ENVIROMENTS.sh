#!/bin/bash

sudo apt update
sudo apt upgrade -y
source /opt/ros/$ROS_DISTRO/setup.bash

sudo apt install python3-pip -y
python3 -m pip install -r src/powder_grinding_ros2/requirements.txt

vcs import src < src/powder_grinding_ros2/third_party.repos
sudo rosdep fix-permissions
rosdep update
rosdep install -r -y -i --from-paths src --rosdistro $ROS_DISTRO

# Install pytracik
cd src/pytracik
chmod +x *.sh
./install.sh
cd ~/ros2_ws


# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --cmake-clean-cache

# Update environmental variables
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
