#!/bin/bash

################################################################################
# Install autostart entries which open three terminals at login and start
#   1. grinding_robot_bringup/cobotta_with_pestle_bringup.launch.py (real robot)
#   2. grinding_motion_routines/cobotta_grinding_demo.launch.py
#   3. grinding_robot_bringup/rqt_joint_trajectory_controller.launch.py (joint jogging)
# in the powder_grinding_ros2 Docker container.
#
# The old ROS1 entries (onolab_ros1) are disabled by renaming them to *.disabled.
# Usage: ./INSTALL-COBOTTA-AUTOSTART.sh [--uninstall]
################################################################################

ENV_DIR=$(cd "$(dirname "$0")" && pwd)
AUTOSTART_DIR=~/.config/autostart
ROS1_ENTRIES="cobotta-brinngup.desktop cobotta-grinding-demo.desktop"
ROS2_ENTRIES="cobotta-bringup-ros2.desktop cobotta-grinding-demo-ros2.desktop cobotta-rqt-jtc-ros2.desktop"

mkdir -p ${AUTOSTART_DIR}

if [[ "$1" == "--uninstall" ]]; then
  for f in ${ROS2_ENTRIES}; do
    rm -fv ${AUTOSTART_DIR}/${f}
  done
  for f in ${ROS1_ENTRIES}; do
    if [ -f ${AUTOSTART_DIR}/${f}.disabled ]; then
      mv -v ${AUTOSTART_DIR}/${f}.disabled ${AUTOSTART_DIR}/${f}
    fi
  done
  exit 0
fi

# Disable ROS1 entries
for f in ${ROS1_ENTRIES}; do
  if [ -f ${AUTOSTART_DIR}/${f} ]; then
    mv -v ${AUTOSTART_DIR}/${f} ${AUTOSTART_DIR}/${f}.disabled
  fi
done

# Install ROS2 entries
for f in ${ROS2_ENTRIES}; do
  sed "s|@ENV_DIR@|${ENV_DIR}|g" ${ENV_DIR}/autostart/${f} > ${AUTOSTART_DIR}/${f}
  echo "Installed ${AUTOSTART_DIR}/${f}"
done
