# Greenhouse Mapping, Nav2, and Mission Manager

This package runs the Gazebo greenhouse simulation, SLAM mapping, Nav2 localization/navigation, keepout masks, and a simple waypoint mission manager for the Husarion Panther + Franka FR3 robot.

Workspace:

```bash
/home/yusuf/robot_workspaces/combined_ws
```

Main package:

```bash
/home/yusuf/robot_workspaces/combined_ws/src/combined_robot
```

Main world:

```bash
worlds/tomato_farm_sera.sdf
```

Saved map and keepout mask:

```bash
maps/my_map.yaml
maps/my_map.pgm
maps/keepout_mask.yaml
maps/keepout_mask.pgm
```

## Build

After a reboot, every terminal must source all three workspaces. If only
`/opt/ros/jazzy` is sourced, Gazebo can partially start but the robot may be
missing or inconsistent in the GUI/entity tree.

Use this once after code, launch, config, or map changes:

```bash
cd /home/yusuf/robot_workspaces/combined_ws
source /opt/ros/jazzy/setup.bash
source /home/yusuf/robot_workspaces/husarion_ws/install/setup.bash
source /home/yusuf/robot_workspaces/franka_ros2_ws/install/setup.bash
colcon build --packages-select combined_robot --symlink-install
source install/setup.bash
```

Use this source block in every new terminal:

```bash
cd /home/yusuf/robot_workspaces/combined_ws
source /opt/ros/jazzy/setup.bash
source /home/yusuf/robot_workspaces/husarion_ws/install/setup.bash
source /home/yusuf/robot_workspaces/franka_ros2_ws/install/setup.bash
source install/setup.bash
```

## Quick Start After Reboot

Terminal 1, start the full Gazebo + Nav2 + mission manager stack:

```bash
cd /home/yusuf/robot_workspaces/combined_ws/src/combined_robot
bash scripts/start_sera_mission.sh
```

Equivalent manual command:

```bash
cd /home/yusuf/robot_workspaces/combined_ws
source /opt/ros/jazzy/setup.bash
source /home/yusuf/robot_workspaces/husarion_ws/install/setup.bash
source /home/yusuf/robot_workspaces/franka_ros2_ws/install/setup.bash
source install/setup.bash
ros2 launch combined_robot sera_mission.launch.py route_name:=center_corridor_patrol mission_autostart:=false loop_route:=false
```

Terminal 2, open RViz:

```bash
cd /home/yusuf/robot_workspaces/combined_ws
source /opt/ros/jazzy/setup.bash
source /home/yusuf/robot_workspaces/husarion_ws/install/setup.bash
source /home/yusuf/robot_workspaces/franka_ros2_ws/install/setup.bash
source install/setup.bash
rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz --ros-args -p use_sim_time:=true
```

In RViz, set the robot pose with `2D Pose Estimate`, then send goals with
`Nav2 Goal`. To start the waypoint mission, use Terminal 3:

```bash
cd /home/yusuf/robot_workspaces/combined_ws
source /opt/ros/jazzy/setup.bash
source /home/yusuf/robot_workspaces/husarion_ws/install/setup.bash
source /home/yusuf/robot_workspaces/franka_ros2_ws/install/setup.bash
source install/setup.bash
ros2 service call /mission_manager/start std_srvs/srv/Trigger
```

## Autonomous Survey

To make the robot survey the full greenhouse without manual goal clicking, use
the dedicated survey launch. This starts Gazebo, Nav2, and mission manager with
the `full_survey` route and `mission_autostart=true`.

Terminal 1:

```bash
cd /home/yusuf/robot_workspaces/combined_ws/src/combined_robot
bash scripts/start_sera_survey.sh
```

Equivalent manual command:

```bash
cd /home/yusuf/robot_workspaces/combined_ws
source /opt/ros/jazzy/setup.bash
source /home/yusuf/robot_workspaces/husarion_ws/install/setup.bash
source /home/yusuf/robot_workspaces/franka_ros2_ws/install/setup.bash
source install/setup.bash
ros2 launch combined_robot sera_survey.launch.py
```

Terminal 2:

```bash
cd /home/yusuf/robot_workspaces/combined_ws
source /opt/ros/jazzy/setup.bash
source /home/yusuf/robot_workspaces/husarion_ws/install/setup.bash
source /home/yusuf/robot_workspaces/franka_ros2_ws/install/setup.bash
source install/setup.bash
rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz --ros-args -p use_sim_time:=true
```

After RViz opens, only give `2D Pose Estimate`. The survey route starts by
itself and covers:

```text
center corridor -> upper corridor -> lower corridor -> return_home
```

## Mapping

Terminal 1, start Gazebo + SLAM:

```bash
cd /home/yusuf/robot_workspaces/combined_ws
source /opt/ros/jazzy/setup.bash
source /home/yusuf/robot_workspaces/husarion_ws/install/setup.bash
source /home/yusuf/robot_workspaces/franka_ros2_ws/install/setup.bash
source install/setup.bash
ros2 launch combined_robot sera_mapping.launch.py
```

Terminal 2, open RViz:

```bash
cd /home/yusuf/robot_workspaces/combined_ws
source /opt/ros/jazzy/setup.bash
source /home/yusuf/robot_workspaces/husarion_ws/install/setup.bash
source /home/yusuf/robot_workspaces/franka_ros2_ws/install/setup.bash
source install/setup.bash
rviz2 -d src/combined_robot/rviz/sera_mapping.rviz --ros-args -p use_sim_time:=true
```

Terminal 3, drive slowly while mapping:

```bash
cd /home/yusuf/robot_workspaces/combined_ws
source /opt/ros/jazzy/setup.bash
source /home/yusuf/robot_workspaces/husarion_ws/install/setup.bash
source /home/yusuf/robot_workspaces/franka_ros2_ws/install/setup.bash
source install/setup.bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}, twist: {linear: {x: 0.15}, angular: {z: 0.0}}}"
```

Turn slowly when needed:

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/TwistStamped "{header: {frame_id: base_link}, twist: {linear: {x: 0.05}, angular: {z: 0.25}}}"
```

Stop the publisher with `Ctrl+C`.

Save the map:

```bash
cd /home/yusuf/robot_workspaces/combined_ws/src/combined_robot/maps
source /opt/ros/jazzy/setup.bash
source /home/yusuf/robot_workspaces/husarion_ws/install/setup.bash
source /home/yusuf/robot_workspaces/franka_ros2_ws/install/setup.bash
source /home/yusuf/robot_workspaces/combined_ws/install/setup.bash
ros2 run nav2_map_server map_saver_cli -f my_map
```

Expected output:

```bash
maps/my_map.yaml
maps/my_map.pgm
```

## Nav2 Manual Test

Terminal 1, start Gazebo + Nav2:

```bash
cd /home/yusuf/robot_workspaces/combined_ws
source /opt/ros/jazzy/setup.bash
source /home/yusuf/robot_workspaces/husarion_ws/install/setup.bash
source /home/yusuf/robot_workspaces/franka_ros2_ws/install/setup.bash
source install/setup.bash
ros2 launch combined_robot sera_nav2.launch.py
```

Terminal 2, open RViz:

```bash
cd /home/yusuf/robot_workspaces/combined_ws
source /opt/ros/jazzy/setup.bash
source /home/yusuf/robot_workspaces/husarion_ws/install/setup.bash
source /home/yusuf/robot_workspaces/franka_ros2_ws/install/setup.bash
source install/setup.bash
rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz --ros-args -p use_sim_time:=true
```

In RViz:

1. Fixed frame should be `map`.
2. Use `2D Pose Estimate` before sending a goal.
3. Put the pose on the robot's real starting location.
4. Use `Nav2 Goal` for a short test inside a corridor.

The tested center corridor is:

```text
y ~= 0.0
```

The side corridors are:

```text
upper corridor: y ~= 1.6
lower corridor: y ~= -1.6
```

## Mission Manager Run

This is the tested route workflow.

Terminal 1, start Gazebo + Nav2 + mission manager without autostart:

```bash
cd /home/yusuf/robot_workspaces/combined_ws
source /opt/ros/jazzy/setup.bash
source /home/yusuf/robot_workspaces/husarion_ws/install/setup.bash
source /home/yusuf/robot_workspaces/franka_ros2_ws/install/setup.bash
source install/setup.bash
ros2 launch combined_robot sera_mission.launch.py route_name:=center_corridor_patrol mission_autostart:=false loop_route:=false
```

Terminal 2, open RViz:

```bash
cd /home/yusuf/robot_workspaces/combined_ws
source /opt/ros/jazzy/setup.bash
source /home/yusuf/robot_workspaces/husarion_ws/install/setup.bash
source /home/yusuf/robot_workspaces/franka_ros2_ws/install/setup.bash
source install/setup.bash
rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz --ros-args -p use_sim_time:=true
```

In RViz, click `2D Pose Estimate` and set the robot pose. Wait for the launch terminal to show:

```text
initialPoseReceived
Managed nodes are active
```

Terminal 3, start the mission:

```bash
cd /home/yusuf/robot_workspaces/combined_ws
source /opt/ros/jazzy/setup.bash
source /home/yusuf/robot_workspaces/husarion_ws/install/setup.bash
source /home/yusuf/robot_workspaces/franka_ros2_ws/install/setup.bash
source install/setup.bash
ros2 service call /mission_manager/start std_srvs/srv/Trigger
```

Expected mission log:

```text
Mission started.
Sending waypoint 1/4: center_1 (3.00, 0.00, yaw 0.00)
Begin navigating from current location (...) to (3.00, 0.00)
Passing new path to controller.
```

Check mission state:

```bash
ros2 topic echo /mission_manager/status --once
```

Expected:

```text
data: running=True; route=center_corridor_patrol; index=0/4; current=center_1
```

Stop, reset, and restart the mission:

```bash
ros2 service call /mission_manager/stop std_srvs/srv/Trigger
ros2 service call /mission_manager/reset std_srvs/srv/Trigger
ros2 service call /mission_manager/start std_srvs/srv/Trigger
```

Available routes are defined in:

```bash
config/sera_waypoints.yaml
```

Current routes:

```text
center_corridor_patrol
upper_corridor_patrol
lower_corridor_patrol
```

## Important Topics

```text
/lidar/scan              LaserScan used by SLAM, AMCL, and costmaps
/odometry/wheels         wheel odometry from diff_drive_controller
/cmd_vel                 TwistStamped command consumed by drive_controller
/map                     occupancy grid map
/filter_mask             Nav2 keepout mask map
/costmap_filter_info     Nav2 keepout filter metadata
/mission_manager/status  current mission manager state
/tf                      map -> odom -> base_link TF chain
```

The Gazebo lidar bridge remaps Gazebo `/scan` to ROS `/lidar/scan`.

The drive controller expects:

```text
geometry_msgs/msg/TwistStamped on /cmd_vel
```

## Quick Checks

Check topic connections:

```bash
ros2 topic info /lidar/scan -v
ros2 topic info /odometry/wheels -v
ros2 topic info /cmd_vel -v
ros2 topic info /mission_manager/status -v
```

Check controllers:

```bash
ros2 control list_controllers
```

Expected active controllers:

```text
drive_controller
joint_state_broadcaster
fr3_arm_controller
fr3_gripper_controller
```

Check Nav2 lifecycle:

```bash
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /filter_mask_server
ros2 lifecycle get /costmap_filter_info_server
```

Expected:

```text
active [3]
```

Check velocity command:

```bash
ros2 topic echo /cmd_vel --once
```

Expected while navigating:

```text
twist:
  linear:
    x: ...
  angular:
    z: ...
```

Check TF:

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
```

## Costmap And Keepout Debug

Check keepout topics:

```bash
ros2 topic info /filter_mask -v
ros2 topic info /costmap_filter_info -v
```

The launch terminal should show:

```text
KeepoutFilter: Received filter info from /costmap_filter_info topic.
KeepoutFilter: Received filter mask from /filter_mask topic.
```

Clear costmaps:

```bash
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap
```

Temporarily disable keepout filters:

```bash
ros2 param set /global_costmap/global_costmap keepout_filter.enabled false
ros2 param set /local_costmap/local_costmap keepout_filter.enabled false
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap
```

Re-enable keepout filters:

```bash
ros2 param set /global_costmap/global_costmap keepout_filter.enabled true
ros2 param set /local_costmap/local_costmap keepout_filter.enabled true
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap
```

Probe global costmap values at important greenhouse points:

```bash
cd /home/yusuf/robot_workspaces/combined_ws
source /opt/ros/jazzy/setup.bash
source /home/yusuf/robot_workspaces/husarion_ws/install/setup.bash
source /home/yusuf/robot_workspaces/franka_ros2_ws/install/setup.bash
source install/setup.bash
python3 - <<'PY'
import rclpy
from nav_msgs.msg import OccupancyGrid

points = [
    (3.0, 0.0),
    (6.0, 0.0),
    (9.0, 0.0),
    (3.0, 1.0),
    (3.0, -1.0),
    (3.0, 1.6),
    (3.0, -1.6),
]

def cb(msg):
    ox = msg.info.origin.position.x
    oy = msg.info.origin.position.y
    res = msg.info.resolution
    w = msg.info.width
    h = msg.info.height
    for x, y in points:
        px = int((x - ox) / res)
        py = int((y - oy) / res)
        if 0 <= px < w and 0 <= py < h:
            print(f'{x:.1f},{y:.1f}: {msg.data[py * w + px]}')
        else:
            print(f'{x:.1f},{y:.1f}: outside')
    rclpy.shutdown()

rclpy.init()
node = rclpy.create_node('costmap_probe')
node.create_subscription(OccupancyGrid, '/global_costmap/costmap', cb, 10)
rclpy.spin(node)
PY
```

Expected current result:

```text
3.0,0.0: 0
6.0,0.0: 0
9.0,0.0: 0
3.0,1.0: 100
3.0,-1.0: 100
3.0,1.6: 0
3.0,-1.6: 0
```

Meaning:

```text
0   = free
100 = blocked or keepout
99  = inflated/high-cost area
```

## Map Cleanup Note

The center corridor must be free in the saved map because it is a real driving corridor in Gazebo.

During testing, some occupied pixels were found in `maps/my_map.pgm` around `y = 0.0`. Those pixels made Nav2 think the empty center corridor was blocked. The current `maps/my_map.pgm` has the center driving corridor cleaned.

Backup before the cleanup:

```bash
maps/my_map.before_center_corridor_cleanup.pgm
```

If the map is edited again, rebuild before launching because launch files load maps from the installed package share:

```bash
cd /home/yusuf/robot_workspaces/combined_ws
source /opt/ros/jazzy/setup.bash
source /home/yusuf/robot_workspaces/husarion_ws/install/setup.bash
source /home/yusuf/robot_workspaces/franka_ros2_ws/install/setup.bash
colcon build --packages-select combined_robot --symlink-install
source install/setup.bash
```

## Current Nav2 Tuning

The greenhouse setup is tuned for basic corridor navigation:

```text
robot_radius: 0.28
global inflation_radius: 0.45
local inflation_radius: 0.45
planner tolerance: 0.30
xy_goal_tolerance: 0.20
```

Keepout mask behavior:

```text
black in keepout_mask.pgm = forbidden keepout area
white in keepout_mask.pgm = allowed area
```

The keepout mask blocks plant/pot rows while keeping the center and side corridors open.

The row keepout lines intentionally start around `x ~= 3.1m` instead of the
greenhouse entrance. This leaves the entrance side open so the robot can switch
between the center, upper, and lower corridors before the crop rows begin.

Backup before the corridor-entry cleanup:

```bash
maps/keepout_mask.before_corridor_entry_cleanup.pgm
```

## Common Problems

Wrong RViz command:

```bash
rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz --ros-
args -p use_sim_time:=true
```

Correct RViz command:

```bash
rviz2 -d /opt/ros/jazzy/share/nav2_bringup/rviz/nav2_default_view.rviz --ros-args -p use_sim_time:=true
```

If Nav2 says:

```text
AMCL cannot publish a pose or update the transform. Please set the initial pose...
```

Use `2D Pose Estimate` in RViz.

If Gazebo opens but the robot is missing from the scene or Entity Tree after a
reboot, the terminal was probably not fully sourced. Stop the launch and use the
Quick Start command above. A healthy model list contains:

```text
combined_robot
```

If Nav2 says:

```text
Action server is inactive. Rejecting the goal.
```

Wait until the launch terminal shows:

```text
Managed nodes are active
```

If planner says:

```text
Failed to create plan with tolerance of: 0.300000
```

Check:

```bash
ros2 topic echo /mission_manager/status --once
ros2 topic echo /cmd_vel --once
ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap
ros2 service call /local_costmap/clear_entirely_local_costmap nav2_msgs/srv/ClearEntireCostmap
```

Then inspect the global costmap with the probe script above. The center corridor route requires `y = 0.0` to be free.
