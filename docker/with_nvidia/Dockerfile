# Parent image is nvidia/cuda
# Add OpenGL and ROS install below
FROM nvidia/cuda:12.6.3-cudnn-devel-ubuntu24.04
LABEL maintainer="YusakuNakajima<yusaku_nakajima@ap.eng.osaka-u.ac.jp>"
ARG ROS_DISTRO=jazzy

# Ignore keyboard layout setting
ENV DEBIAN_FRONTEND=noninteractive
ENV __NV_PRIME_RENDER_OFFLOAD=1
ENV __GLX_VENDOR_LIBRARY_NAME=nvidia

# Install software-properties-common for add-apt-repository
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Add universe repository
RUN add-apt-repository universe

# Set timezone and locale
RUN ln -sf /usr/share/zoneinfo/Asia/Tokyo /etc/localtime
RUN apt-get update && apt-get install -y --no-install-recommends \
    tzdata \
    locales-all \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
RUN date
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US.UTF-8

# Install apt packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales-all \
    git \
    vim \
    nano \
    wget \
    tmux \
    curl \
    terminator \
    iputils-ping \
    net-tools \
    libglib2.0-0 \
    gnupg \
    lsb-release \
    sudo \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Add OpenGL
RUN apt-get update && apt-get install -y --no-install-recommends \
    pkg-config \
    libglvnd-dev \
    libgl1-mesa-dev \
    libegl1-mesa-dev \
    libgles2-mesa-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Add ROS repository and install ROS
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list'

RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-desktop \
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

# Install vcstools
RUN apt-get update && apt-get install -y \
    python3-vcstool \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Install rosdep
RUN apt-get update && apt-get install -y \
    python3-rosdep \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Install colcon
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


# Setup ROS environments
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc
RUN echo "source /home/ubuntu/onolab/onolab_ws/install/setup.bash" >> /root/.bashrc
# Argcomplete for colcon
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /root/.bashrc
# Add alias for easy build
RUN echo "alias b='cd /home/ubuntu/onolab/onolab_ws; colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install; source /home/ubuntu/onolab/onolab_ws/install/setup.bash'" >> /root/.bashrc

