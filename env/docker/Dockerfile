ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop-full

LABEL maintainer="YusakuNakajima<yusaku_nakajima@ap.eng.osaka-u.ac.jp>"

# Ignore keyboard layout setting
ENV DEBIAN_FRONTEND=noninteractive

# ROS GPGキーを更新し、リポジトリ設定を適用
RUN \
  rm -rf /etc/apt/sources.list.d/ros2-latest.list && \
  rm -rf /etc/apt/sources.list.d/ros2.list && \
  apt-get update -q && \
  apt-get install -yq --no-install-recommends curl && \
  curl -sS https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $( . /etc/os-release && echo $UBUNTU_CODENAME ) main" > /etc/apt/sources.list.d/ros2.list && \
  rm -rf /var/lib/apt/lists/*

# Set timezone
RUN ln -sf /usr/share/zoneinfo/Asia/Tokyo /etc/localtime \
    && apt-get update \
    && apt-get install -y git tzdata \
    && date

# Install apt packages
COPY install_packages.sh /tmp/install_packages.sh
RUN chmod +x /tmp/install_packages.sh \
    && /tmp/install_packages.sh

# Create a non-root user (ros) and set up passwordless sudo
RUN apt-get update && apt-get install -y sudo \
    && useradd -m -s /bin/bash ros \
    && echo "ros ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Set up ROS environments for ros user
USER ros

# Create python virtual environment for PEP668
RUN python3 -m venv /home/ros/ros2_ws/venv
ENV VIRTUAL_ENV="/home/ros/ros2_ws/venv"
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

# Add venv activation to .bashrc for interactive shells
RUN echo '' >> /home/ros/.bashrc && \
    echo '# Python Virtual Environment for ROS 2' >> /home/ros/.bashrc && \
    echo "source \"$VIRTUAL_ENV/bin/activate\"" >> /home/ros/.bashrc && \
    echo "export PATH=\"$VIRTUAL_ENV/bin:\$PATH\"" >> /home/ros/.bashrc

# Prevent colcon from searching the venv directory for packages
RUN touch /home/ros/ros2_ws/venv/COLCON_IGNORE

# Setup workspace
RUN mkdir -p /home/ros/ros2_ws/src
COPY BUILD_ROS_WORKSPACE.sh /home/ros/ros2_ws
COPY INITIAL_SETUP_ROS_ENVIROMENTS.sh /home/ros/ros2_ws

# Setup ROS environments
RUN echo '' >> /home/ros/.bashrc && \
    echo '# Environment for ROS 2' >> /home/ros/.bashrc && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/ros/.bashrc && \
    echo "export ROS_DISTRO=${ROS_DISTRO}" >> /home/ros/.bashrc && \
    echo "source /home/ros/ros2_ws/install/setup.bash" >> /home/ros/.bashrc

# Configure colcon_cd
RUN echo '' >> /home/ros/.bashrc && \
    echo '# colcon_cd configuration' >> /home/ros/.bashrc && \
    echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> /home/ros/.bashrc && \
    echo "export _colcon_cd_root=~/" >> /home/ros/.bashrc

# Argcomplete for colcon
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/ros/.bashrc

# Add alias for easy build
RUN echo "alias b='cd /home/ros/ros2_ws; colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release; source install/setup.bash'" >> /home/ros/.bashrc

# Add lib path
RUN echo "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH" >> /home/ros/.bashrc && \
    echo '' >> /home/ros/.bashrc

WORKDIR /home/ros/ros2_ws
ENTRYPOINT [ "/bin/bash", "-c", ". venv/bin/activate && bash" ]