#!/bin/bash

sudo apt update
sudo apt upgrade -y
source /opt/ros/$ROS_DISTRO/setup.bash

sudo apt install python3-pip -y
python3 -m pip install -r src/powder_grinding/requirements.txt

vcs import src < src/powder_grinding/third_party.repos
sudo rosdep fix-permissions
rosdep update
rosdep install -r -y -i --from-paths src --rosdistro $ROS_DISTRO

# Install pytracik
cd src/powder_grinding/pytracik
pip install -r requirements.txt
pip install -e .
cd /home/ubuntu/user/grinding_ws


# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --cmake-clean-cache

# Update environmental variables
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/ubuntu/user/grinding_ws/install/setup.bash
