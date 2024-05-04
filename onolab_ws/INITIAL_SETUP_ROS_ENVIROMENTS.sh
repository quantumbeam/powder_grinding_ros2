#!/bin/bash

sudo apt update
sudo apt upgrade -y
source /opt/ros/$ROS_DISTRO/setup.bash

sudo pip install --upgrade pip

# pip3 install -r src/requirements.txt

rosdep update
# vcs import src < src/third_party.repos
# vcs import src/third_party/Universal_Robots < src/third_party/Universal_Robots/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.humble.repos
# vcs import src/third_party/MoveIt2 < src/third_party/MoveIt2/moveit2_tutorials/moveit2_tutorials.repos

rosdep update
rosdep install -r -y -i --from-paths src --rosdistro $ROS_DISTRO

# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --cmake-clean-cache

# Update enviromental veriables
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/ubuntu/onolab/onolab_ws/install/setup.bash
