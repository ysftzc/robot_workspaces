#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/husarion_ws/install/setup.bash
source ~/franka_ros2_ws/install/setup.bash
source ~/combined_ws/install/setup.bash

ros2 param set /controller_manager fr3_arm_controller.type joint_trajectory_controller/JointTrajectoryController
ros2 run controller_manager spawner fr3_arm_controller --controller-manager /controller_manager --param-file ~/combined_ws/install/combined_robot/share/combined_robot/config/fr3_controllers.yaml
