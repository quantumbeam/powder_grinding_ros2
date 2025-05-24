#!/bin/bash


################################################################################
# arg1: project name

################################################################################

# Set the Docker container name from a project name (first argument).
# If no argument is given, use the current user name as the project name.
PROJECT=${USER}
NVIDIA_OPTION=$1
if [[ $NVIDIA_OPTION == *"nvidia"* ]]; then
  CONTAINER="${PROJECT}-ros2-humble-nvidia-1"
else
  CONTAINER="${PROJECT}-ros2-humble-1"
fi
echo "$0: PROJECT=${PROJECT}"
echo "$0: CONTAINER=${CONTAINER}"
echo $(dirname "$0")/docker/compose.yaml
echo "$1: NVIDIA_OPTION=${NVIDIA_OPTION}"


# Run the Docker container in the background.
# Any changes made to './docker/compose.yaml' will recreate and overwrite the container.
if [[ $NVIDIA_OPTION == *"nvidia"* ]]; then
    docker compose -p ${PROJECT} -f $(dirname "$0")/docker/with_nvidia/compose.yaml up -d
else
    docker compose -p ${PROJECT} -f $(dirname "$0")/docker/compose.yaml up -d
fi


################################################################################

# Display GUI through X Server by granting full access to any external client.
xhost +local:

################################################################################

# Enter the Docker container with a Bash shell
docker exec -it  -w /home/ubuntu/user/ros2_ws ${CONTAINER} bash -i