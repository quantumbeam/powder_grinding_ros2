#!/bin/bash

################################################################################
# Ask the user and run 'ros2 launch PACKAGE_NAME LAUNCH_FILE [LAUNCH_ARGS...]' repeatedly.
# ROS2 version of onolab_ros1/terminator/run-roslaunch-repeatedly.sh
################################################################################

PACKAGE_NAME=$1
LAUNCH_FILE=$2
shift 2
LAUNCH_ARGS="$*"

if [ -z "${LAUNCH_FILE}" ]; then
  echo "No launch file is specified."
  echo "Usage: $0 PACKAGE_NAME LAUNCH_FILE [LAUNCH_ARGS...]"
  exit 1
fi

PROMPT_START=$'\e[4m\e[1m'
PROMPT_END=$'\e[0m'
KEY_START=$'\e[7m'
KEY_END=$'\e[0m'${PROMPT_START}
PROMPT="${PROMPT_START}Run 'ros2 launch ${PACKAGE_NAME} ${LAUNCH_FILE} ${LAUNCH_ARGS}'? Press:"$'\n'"'${KEY_START}r${KEY_END}' to run with the robot, "$'\n'"'${KEY_START}c${KEY_END}' to enter a child shell,"$'\n'"'${KEY_START}q${KEY_END}' to quit.${PROMPT_END}"$'\n'

while true; do
  read -n 1 -s -p "${PROMPT}" input;
  if [ "${input}" = "r" ]; then
    echo "";
  elif [ "${input}" = "q" ]; then
    break;
  elif [ "${input}" = "c" ]; then
    cat <<EOF

Starting a new shell process.
You will return to the above prompt when you exit from this shell.

EOF
    bash -i
    continue;
  else
    continue;
  fi;
  echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}";
  ros2 launch ${PACKAGE_NAME} ${LAUNCH_FILE} ${LAUNCH_ARGS};
  echo "" # Display an empty line.
done

cat <<EOF

Starting a new shell process.

EOF

exec bash -i
