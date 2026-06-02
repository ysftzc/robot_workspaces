#!/usr/bin/env bash
set -eo pipefail

WORKSPACE=/home/yusuf/robot_workspaces/combined_ws

source /opt/ros/jazzy/setup.bash
source /home/yusuf/robot_workspaces/husarion_ws/install/setup.bash
source /home/yusuf/robot_workspaces/franka_ros2_ws/install/setup.bash
source "${WORKSPACE}/install/setup.bash"

cd "${WORKSPACE}"

exec ros2 launch combined_robot sera_survey.launch.py
