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
    && apt-get clean \ 
    && rm -rf /var/lib/apt/lists/*

# Setup ROS enviroments
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc
RUN echo "source /home/ubuntu/onolab/onolab_ws/install/setup.bash" >> /root/.bashrc
# Argcomplete for colcon
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc
# Add alias for easy build
RUN echo "alias b='cd /home/ubuntu/onolab/onolab_ws; colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install; source /home/ubuntu/onolab/onolab_ws/install/setup.bash'" >> /root/.bashrc
