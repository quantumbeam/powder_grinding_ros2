### Ros Workspace ###
# Set up the workspace

sudo apt update

# Downloading packages
# vcs import src < src/third_party.repos
# vcs import src/third_party/Universal_Robots < src/third_party/Universal_Robots/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.humble.repos
# vcs import src/third_party/MoveIt2 < src/third_party/MoveIt2/moveit2_tutorials/moveit2_tutorials.repos
vcs pull src

# Updating rosdep and installing dependencies
rosdep update
rosdep install -r -y -i --from-paths src --rosdistro $ROS_DISTRO

# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Update enviromental veriables
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/ubuntu/onolab/onolab_ws/install/setup.bash
