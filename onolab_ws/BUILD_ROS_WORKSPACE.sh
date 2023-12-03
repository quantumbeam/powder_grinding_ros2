### Ros Workspace ###
# Set up the workspace

sudo apt update

rosdep update

# Updating ROSDEP and installing dependencies
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y

# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Update enviromental veriables
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/ubuntu/onolab/onolab_ws/install/setup.bash
