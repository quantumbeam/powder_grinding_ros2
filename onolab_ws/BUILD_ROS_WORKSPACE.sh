### Ros Workspace ###
# Set up the workspace

sudo apt update

# Downloading packages
vcs import src < src/third_party.repos
vcs import src/third_party/Universal_Robots < src/third_party/Universal_Robots/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.humble.repos
vcs pull src

# Updating rosdep and installing dependencies
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro=$ROS_DISTRO -y

# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Update enviromental veriables
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/ubuntu/onolab/onolab_ws/install/setup.bash
