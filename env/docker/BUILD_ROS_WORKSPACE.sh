### Ros Workspace ###
# Set up the workspace

sudo apt update

# Activate virtual environment
python3 -m pip install -r src/powder_grinding/requirements.txt

# Downloading packages
vcs import src < src/powder_grinding/third_party.repos
vcs pull src

# Updating rosdep and installing dependencies
rosdep update
rosdep install -r -y -i --from-paths src --rosdistro $ROS_DISTRO

# Build
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --cmake-clean-cache

# Update environmental variables
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/ubuntu/user/ros2_ws/install/setup.bash
