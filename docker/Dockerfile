FROM osrf/ros:humble-desktop
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
    ros-humble-rmw-cyclonedds-cpp \
    && apt-get clean \ 
    && rm -rf /var/lib/apt/lists/*

RUN gosu ubuntu rosdep update && \
    grep -F "source /opt/ros/humble/setup.bash" /home/ubuntu/.bashrc || echo "source /opt/ros/humble/setup.bash" >> /home/ubuntu/.bashrc && \
    grep -F "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" /home/ubuntu/.bashrc || echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/ubuntu/.bashrc && \
    sudo chown ubuntu:ubuntu /home/ubuntu/.bashrc

# Setup ROS enviroments
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /home/ubuntu/onolab/onolab_ws/install/setup.bash" >> /root/.bashrc

# Setup ROS2 DDS
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> /root/.bashrc

# Set ROS2 network ID
RUN echo "export ROS_DOMAIN_ID=1225" >> /root/.bashrc
