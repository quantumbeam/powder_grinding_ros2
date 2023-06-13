### Ros Workspace ###
# Set up the workspace
source /opt/ros/$ROS_DISTRO/setup.bash

sudo apt update

rosdep update

# Updating ROSDEP and installing dependencies
cd ~/onolab/onolab_ws && vcs import src < src/underlay.rosinstall
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y

# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --cmake-clean-cache
source /home/ubuntu/onolab/onolab_ws/install/setup.bash
