#!/bin/bash

# ROS_DISTRO 環境変数が設定されていない場合は "humble" をデフォルトとする
# (スクリプトを直接実行する場合に必要)
: "${ROS_DISTRO:=humble}"

echo "### Installing common apt packages ###"
sudo apt-get update && sudo apt-get install -y \
    git \
    vim \
    nano \
    wget \
    tmux \
    curl \
    terminator \
    iputils-ping \
    net-tools \
    python3-pip \
    python3-venv

echo "### Installing ROS-specific apt packages for ${ROS_DISTRO} ###"
sudo apt-get update -q && sudo apt-get install -y \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-rqt-controller-manager \
    ros-${ROS_DISTRO}-rqt-joint-trajectory-controller \
    ros-${ROS_DISTRO}-rviz-visual-tools \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-ur \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-tf2-ros-py

echo "### Cleaning up apt cache ###"
sudo apt-get clean
sudo rm -rf /var/lib/apt/lists/*

echo "### All specified apt packages installed successfully ###"