#!/bin/bash

sudo apt update
sudo apt upgrade -y
source /opt/ros/$ROS_DISTRO/setup.bash

sudo pip install --upgrade pip

# pip3 install -r src/requirements.txt

. BUILD_ROS_WORKSPACE.sh
