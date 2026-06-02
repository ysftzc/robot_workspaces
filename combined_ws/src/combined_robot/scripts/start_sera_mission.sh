#!/usr/bin/env bash
set -eo pipefail

WORKSPACE=/home/yusuf/robot_workspaces/combined_ws

source /opt/ros/jazzy/setup.bash
source /home/yusuf/robot_workspaces/husarion_ws/install/setup.bash
source /home/yusuf/robot_workspaces/franka_ros2_ws/install/setup.bash
source "${WORKSPACE}/install/setup.bash"

cd "${WORKSPACE}"

exec ros2 launch combined_robot sera_mission.launch.py \
  route_name:=full_survey_then_harvest_scan \
  mission_mode:=survey_harvest \
  arm_motion_mode:=moveit \
  world_file:=tomato_farm_sera.sdf \
  harvest_pick_enabled:=true \
  harvest_pick_allowed_classes:=fully_ripened,ripe,rotten,disease,diseased \
  harvest_pick_reject_classes:=green,unripe \
  harvest_pick_good_classes:=fully_ripened,ripe \
  harvest_pick_bad_classes:=rotten,disease,diseased,bad,green,unripe \
  harvest_pick_max_attempts:=0 \
  harvest_pick_max_per_waypoint:=0 \
  harvest_pick_max_candidates_per_target:=6 \
  mission_autostart:=false \
  loop_route:=false \
  stop_on_failure:=false \
  gz_gui:=false \
  run_rviz:=false \
  run_tomato_map_panel:=false \
  tomato_mapper_model_filter_use_live_gazebo_pose:=false
