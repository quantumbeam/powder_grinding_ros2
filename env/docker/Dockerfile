ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop-full
LABEL maintainer="YusakuNakajima<yusaku_nakajima@ap.eng.osaka-u.ac.jp>"

# Ignore keybourd layout seting
ENV DEBIAN_FRONTEND=noninteractive

# set timezone
RUN ln -sf /usr/share/zoneinfo/Asia/Tokyo /etc/localtime
RUN apt-get update \
    && apt-get install -y git tzdata
RUN date

# Install apt packages
RUN apt-get update && apt-get install -y \
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
    python3.12-venv \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install ROS packages
RUN apt-get update -q && \
    apt-get install -y \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-rqt-controller-manager \
    ros-${ROS_DISTRO}-rqt-joint-trajectory-controller \
    ros-${ROS_DISTRO}-rviz-visual-tools \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-ur \
    ros-${ROS_DISTRO}-moveit-visual-tools \
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-trac-ik \
    && apt-get clean \ 
    && rm -rf /var/lib/apt/lists/*

# Create python virtual environment for PEP668
RUN python3 -m venv /home/ubuntu/user/grinding_ws/venv
ENV VIRTUAL_ENV="/home/ubuntu/user/grinding_ws/venv"
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

# Setup workspace
RUN mkdir -p /home/ubuntu/user/grinding_ws/src
COPY BUILD_ROS_WORKSPACE.sh /home/ubuntu/user/grinding_ws
RUN chmod +x /home/ubuntu/user/grinding_ws/BUILD_ROS_WORKSPACE.sh
COPY INITIAL_SETUP_ROS_ENVIROMENTS.sh /home/ubuntu/user/grinding_ws
RUN chmod +x /home/ubuntu/user/grinding_ws/INITIAL_SETUP_ROS_ENVIROMENTS.sh
RUN . /home/ubuntu/user/grinding_ws/venv/bin/activate

# Setup ROS enviroments
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source /home/ubuntu/user/grinding_ws/install/setup.bash" >> /root/.bashrc
# Argcomplete for colcon
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc
# Add alias for easy build
RUN echo "alias b='. /home/ubuntu/user/grinding_ws/venv/bin/activate && cd /home/ubuntu/user/grinding_ws; colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install; source /home/ubuntu/user/grinding_ws/install/setup.bash'" >> /root/.bashrc
