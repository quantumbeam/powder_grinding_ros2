#!/bin/bash

sudo apt update
sudo apt upgrade -y
source /opt/ros/$ROS_DISTRO/setup.bash

sudo pip install --upgrade pip

# pip3 install -r src/requirements.txt

vcs import src < src/third_party.repos
vcs import src/third_party/Universal_Robots < src/third_party/Universal_Robots/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.humble.repos
vcs pull src

rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y

# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --cmake-clean-cache

# Update enviromental veriables
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/ubuntu/onolab/onolab_ws/install/setup.bash
