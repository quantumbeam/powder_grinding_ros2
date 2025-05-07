#!/bin/bash

# Use the current user name as docker project name.
DOCKER_PROJECT=${USER}

NVIDIA_OPTION=$1
echo "$1: NVIDIA_OPTION=${NVIDIA_OPTION}"
echo "$0: USER=${DOCKER_PROJECT}"
if [[ $NVIDIA_OPTION == *"nvidia"* ]]; then
  DOCKER_CONTAINER="${DOCKER_PROJECT}_powder_grinding_ros2_humble_nvidia_1"
  echo "$0: DOCKER_CONTAINER=${DOCKER_CONTAINER}"
else
  DOCKER_CONTAINER="${DOCKER_PROJECT}_powder_grinding_ros2_humble_1"
  echo "$0: DOCKER_CONTAINER=${DOCKER_CONTAINER}"
fi

# Stop and remove the Docker container.
EXISTING_DOCKER_CONTAINER_ID=`docker ps -aq -f name=${DOCKER_CONTAINER}`
if [ ! -z "${EXISTING_DOCKER_CONTAINER_ID}" ]; then
  echo "Stop the container ${DOCKER_CONTAINER} with ID: ${EXISTING_DOCKER_CONTAINER_ID}."
  docker stop ${EXISTING_DOCKER_CONTAINER_ID}
  echo "Remove the container ${DOCKER_CONTAINER} with ID: ${EXISTING_DOCKER_CONTAINER_ID}."
  docker rm ${EXISTING_DOCKER_CONTAINER_ID}
fi

# Build docker image
if [[ $NVIDIA_OPTION == *"nvidia"* ]]; then
    docker compose -p ${DOCKER_PROJECT} -f docker/with_nvidia/compose.yaml build
else
    docker compose -p ${DOCKER_PROJECT} -f docker/compose.yaml build
fi

# Initialize environments in the container
echo "Finished building docker image."
echo "You should run RUN-DOCKER-CONTAINER.sh at home directory and INITIAL_SETUP_ROS_ENVIROMENTS.sh in docker container at first."
echo "After that, you can run ROS commands and use packages in docker container."