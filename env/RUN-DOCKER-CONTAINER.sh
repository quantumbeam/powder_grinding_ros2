#!/bin/bash

# Re-execute with bash when called via 'sh' (e.g. from the terminator config)
if [ -z "$BASH_VERSION" ]; then
  exec bash "$0" "$@"
fi

################################################################################
# Usage: ./RUN-DOCKER-CONTAINER.sh [nvidia] [PACKAGE_NAME LAUNCH_FILE [LAUNCH_ARGS...]]
#   - Without PACKAGE_NAME/LAUNCH_FILE, enter the container with a Bash shell.
#   - With them, ask and run 'ros2 launch PACKAGE_NAME LAUNCH_FILE [LAUNCH_ARGS...]' repeatedly.
# e.g. ./RUN-DOCKER-CONTAINER.sh grinding_robot_bringup cobotta_with_pestle_bringup.launch.py use_mock_hardware:=false

################################################################################

# Set the Docker container name from a project name.
# The current user name is used as the project name.
PROJECT=${USER}
NVIDIA_OPTION=""
if [[ $1 == *"nvidia"* ]]; then
  NVIDIA_OPTION=$1
  shift
fi
PACKAGE_NAME=$1
LAUNCH_FILE=$2
if [[ $# -ge 2 ]]; then
  shift 2
  LAUNCH_ARGS="$*"
fi

if [[ $NVIDIA_OPTION == *"nvidia"* ]]; then
  CONTAINER="${PROJECT}-powder-grinding-ros2-nvidia-1"
else
  CONTAINER="${PROJECT}-powder-grinding-ros2-1"
fi
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
echo "$0: PROJECT=${PROJECT}"
echo "$0: CONTAINER=${CONTAINER}"
echo "${SCRIPT_DIR}/docker/compose.yaml"
echo "$0: NVIDIA_OPTION=${NVIDIA_OPTION}"
echo "$0: PACKAGE_NAME=${PACKAGE_NAME}"
echo "$0: LAUNCH_FILE=${LAUNCH_FILE}"
echo "$0: LAUNCH_ARGS=${LAUNCH_ARGS}"


# Run the Docker container in the background.
# Any changes made to './docker/compose.yaml' will recreate and overwrite the container.
if [[ $NVIDIA_OPTION == *"nvidia"* ]]; then
    docker compose -p ${PROJECT} -f ${SCRIPT_DIR}/docker/with_nvidia/compose.yaml up -d
else
    docker compose -p ${PROJECT} -f ${SCRIPT_DIR}/docker/compose.yaml up -d
fi


################################################################################

# Display GUI through X Server by granting full access to any external client.
xhost +local:

################################################################################

if [[ -z "${PACKAGE_NAME}" ]]; then
  # Enter the Docker container with a Bash shell
  docker exec -it -w /home/ros/ros2_ws ${CONTAINER} bash -i
elif [[ -z "${LAUNCH_FILE}" ]]; then
  echo "Error: PACKAGE_NAME is set but LAUNCH_FILE is missing."
  exit 1
else
  docker exec -it -w /home/ros/ros2_ws ${CONTAINER} bash -i -c \
    "/home/ros/ros2_ws/src/powder_grinding_ros2/env/terminator/run-ros2-launch-repeatedly.sh ${PACKAGE_NAME} ${LAUNCH_FILE} ${LAUNCH_ARGS}"
fi
