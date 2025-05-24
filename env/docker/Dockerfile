ARG ROS_DISTRO=humble
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
COPY install_packages.sh /tmp/install_packages.sh
RUN chmod +x /tmp/install_packages.sh
RUN /tmp/install_packages.sh

# Create a non-root user (ros) and set up passwordless sudo
RUN apt-get update && apt-get install -y sudo \
    && useradd -m -s /bin/bash ros \
    && echo "ros ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Set up ROS environments for ros user
USER ros

# # # Create python virtual environment for PEP668
RUN python3 -m venv /home/ros/ros2_ws/venv
ENV VIRTUAL_ENV="/home/ros/ros2_ws/venv"
ENV PATH="$VIRTUAL_ENV/bin:$PATH"
RUN echo "source venv/bin/activate" >> /home/ros/.bashrc
# Prevent colcon from searching the venv directory for packages
RUN touch /home/ros/ros2_ws/venv/COLCON_IGNORE 

# Setup workspace
RUN mkdir -p /home/ros/ros2_ws/src
COPY BUILD_ROS_WORKSPACE.sh /home/ros/ros2_ws
# RUN chmod +x /home/ros/ros2_ws/BUILD_ROS_WORKSPACE.sh
COPY INITIAL_SETUP_ROS_ENVIROMENTS.sh /home/ros/ros2_ws
# RUN chmod +x /home/ros/ros2_ws/INITIAL_SETUP_ROS_ENVIROMENTS.sh

# Setup ROS enviroments
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/ros/.bashrc
RUN echo "source /home/ros/ros2_ws/install/setup.bash" >> /home/ros/.bashrc
# Argcomplete for colcon
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/ros/.bashrc
# Add alias for easy build
RUN echo "alias b='cd /home/ros/ros2_ws; colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install; source /home/ros/ros2_ws/install/setup.bash'" >> /home/ros/.bashrc
# Add lib path
RUN echo "export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH" >> /home/ros/.bashrc



WORKDIR /home/ros/ros2_ws
ENTRYPOINT [ "/bin/bash", "-c", ". venv/bin/activate && bash" ]
