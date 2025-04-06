#!/bin/bash

sudo apt update
sudo apt upgrade -y
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/ubuntu/user/grinding_ws/venv/bin/activate

sudo apt install python3-pip -y
pip3 install -r src/powder_grinding/requirements.txt

vcs import src < src/powder_grinding/third_party.repos
sudo rosdep fix-permissions
rosdep update
rosdep install -r -y -i --from-paths src --rosdistro $ROS_DISTRO

# Install pytracik
cd src/pytracik
pip install -r requirements.txt
pip install -e .
cd ../..


# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --cmake-clean-cache

# Update environmental variables
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/ubuntu/user/grinding_ws/install/setup.bash
