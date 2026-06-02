# Graph Report - combined_ws  (2026-05-24)

## Corpus Check
- 111 files · ~12,624,320 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 793 nodes · 1466 edges · 50 communities detected
- Extraction: 97% EXTRACTED · 3% INFERRED · 0% AMBIGUOUS · INFERRED: 41 edges (avg confidence: 0.79)
- Token cost: 0 input · 0 output

## Community Hubs (Navigation)
- [[_COMMUNITY_Community 0|Community 0]]
- [[_COMMUNITY_Community 1|Community 1]]
- [[_COMMUNITY_Community 2|Community 2]]
- [[_COMMUNITY_Community 3|Community 3]]
- [[_COMMUNITY_Community 4|Community 4]]
- [[_COMMUNITY_Community 5|Community 5]]
- [[_COMMUNITY_Community 6|Community 6]]
- [[_COMMUNITY_Community 7|Community 7]]
- [[_COMMUNITY_Community 8|Community 8]]
- [[_COMMUNITY_Community 9|Community 9]]
- [[_COMMUNITY_Community 10|Community 10]]
- [[_COMMUNITY_Community 11|Community 11]]
- [[_COMMUNITY_Community 12|Community 12]]
- [[_COMMUNITY_Community 13|Community 13]]
- [[_COMMUNITY_Community 14|Community 14]]
- [[_COMMUNITY_Community 15|Community 15]]
- [[_COMMUNITY_Community 16|Community 16]]
- [[_COMMUNITY_Community 17|Community 17]]
- [[_COMMUNITY_Community 18|Community 18]]
- [[_COMMUNITY_Community 19|Community 19]]
- [[_COMMUNITY_Community 20|Community 20]]
- [[_COMMUNITY_Community 21|Community 21]]
- [[_COMMUNITY_Community 22|Community 22]]
- [[_COMMUNITY_Community 23|Community 23]]
- [[_COMMUNITY_Community 24|Community 24]]
- [[_COMMUNITY_Community 25|Community 25]]
- [[_COMMUNITY_Community 26|Community 26]]
- [[_COMMUNITY_Community 27|Community 27]]
- [[_COMMUNITY_Community 28|Community 28]]
- [[_COMMUNITY_Community 29|Community 29]]
- [[_COMMUNITY_Community 30|Community 30]]
- [[_COMMUNITY_Community 31|Community 31]]
- [[_COMMUNITY_Community 32|Community 32]]
- [[_COMMUNITY_Community 33|Community 33]]
- [[_COMMUNITY_Community 34|Community 34]]
- [[_COMMUNITY_Community 35|Community 35]]
- [[_COMMUNITY_Community 36|Community 36]]
- [[_COMMUNITY_Community 37|Community 37]]
- [[_COMMUNITY_Community 38|Community 38]]
- [[_COMMUNITY_Community 39|Community 39]]
- [[_COMMUNITY_Community 40|Community 40]]
- [[_COMMUNITY_Community 41|Community 41]]
- [[_COMMUNITY_Community 47|Community 47]]
- [[_COMMUNITY_Community 66|Community 66]]
- [[_COMMUNITY_Community 67|Community 67]]
- [[_COMMUNITY_Community 68|Community 68]]
- [[_COMMUNITY_Community 69|Community 69]]
- [[_COMMUNITY_Community 70|Community 70]]
- [[_COMMUNITY_Community 71|Community 71]]
- [[_COMMUNITY_Community 72|Community 72]]

## God Nodes (most connected - your core abstractions)
1. `MissionManager` - 85 edges
2. `GreenhouseNearestPickPlace` - 66 edges
3. `TomatoDepthMapper` - 46 edges
4. `DetachablePickPlace` - 27 edges
5. `TomatoCollisionSceneManager` - 26 edges
6. `ArmCartesianGui` - 25 edges
7. `TomatoDepthDetector` - 23 edges
8. `GazeboTomatoDetector` - 21 edges
9. `RobotControlGUI` - 15 edges
10. `_look_at_orientation()` - 15 edges

## Surprising Connections (you probably didn't know these)
- `GreenhouseNearestPickPlace` --uses--> `DetachablePickPlace`  [INFERRED]
  src/combined_robot/combined_robot/greenhouse_nearest_pick_place.py → src/combined_robot/combined_robot/pick_place_detachable.py
- `TomatoCandidate` --uses--> `DetachablePickPlace`  [INFERRED]
  src/combined_robot/combined_robot/greenhouse_nearest_pick_place.py → src/combined_robot/combined_robot/pick_place_detachable.py

## Communities (73 total, 21 thin omitted)

### Community 0 - "Community 0"
Cohesion: 0.06
Nodes (10): main(), MissionManager, MissionState, _model_pose_from_sdf(), _normalize_angle(), _parse_sdf_pose(), _resolve_mirrored_arm_poses(), _tomato_record_key() (+2 more)

### Community 1 - "Community 1"
Cohesion: 0.05
Nodes (25): _correct_depth_surface_to_center(), DetectionAdapter, main(), ManualPixelAdapter, _model_name_matches_detection_class(), PixelDetection, RGB-D pixel-to-3D tomato mapper with detector adapters.  The node intentionally, Convert a Gazebo tomato model pose to a ROS frame used by detections. (+17 more)

### Community 2 - "Community 2"
Cohesion: 0.07
Nodes (21): Candidate, _class_from_name(), GazeboTomatoDetector, main(), _model_name_from_message(), _parse_model_location(), _parse_model_set(), Gazebo ground-truth tomato detector.  This is a temporary detector adapter for t (+13 more)

### Community 3 - "Community 3"
Cohesion: 0.05
Nodes (20): _default_config_path(), JointStateWaiter, _load_yaml(), main(), _parse_args(), _save_pose(), InitialPosePublisher, main() (+12 more)

### Community 4 - "Community 4"
Cohesion: 0.13
Nodes (11): DetachablePickPlace, _duration(), _extract_gz_pose_value(), main(), _parse_args(), _parse_gazebo_model_pose(), Get the current TCP position in world frame, or None if TF unavailable., Force the tomato to the current TCP world position, repeated for stability. (+3 more)

### Community 5 - "Community 5"
Cohesion: 0.14
Nodes (7): ArmCartesianGui, _axis_angle_quat(), _clamp(), main(), _quat_multiply(), _quat_normalize(), _quat_to_rpy()

### Community 6 - "Community 6"
Cohesion: 0.11
Nodes (25): _axis_angle_quaternion(), _build_candidates(), _look_at_orientation(), main(), _matrix_to_quaternion(), _model_collision_radius(), _normalize_quaternion(), _normalize_vector() (+17 more)

### Community 7 - "Community 7"
Cohesion: 0.16
Nodes (3): main(), Publish mapped tomato detections as RViz markers and MoveIt collision objects., TomatoCollisionSceneManager

### Community 9 - "Community 9"
Cohesion: 0.15
Nodes (3): main(), Depth-based red tomato target publisher.  Publishes:   /tomato_center  geometry_, TomatoDepthDetector

### Community 10 - "Community 10"
Cohesion: 0.16
Nodes (5): _image_to_rgb(), main(), Live Tkinter viewer for YOLO tomato bounding boxes., YoloBboxViewer, YoloBboxViewerNode

### Community 11 - "Community 11"
Cohesion: 0.26
Nodes (18): add_package_runtime_dependencies(), _append_unique_value(), get_commands(), get_packages(), handle_dsv_types_except_source(), _include_comments(), main(), order_packages() (+10 more)

### Community 12 - "Community 12"
Cohesion: 0.26
Nodes (18): add_package_runtime_dependencies(), _append_unique_value(), get_commands(), get_packages(), handle_dsv_types_except_source(), _include_comments(), main(), order_packages() (+10 more)

### Community 13 - "Community 13"
Cohesion: 0.2
Nodes (12): franka_fk(), GeometricPickAndPlace, main(), Generates waypoints to ensure the arm moves in a perfectly straight line., Exact Forward Kinematics matching FR3 URDF, Numerical Inverse Kinematics with Orientation Control, rot_x(), rot_y() (+4 more)

### Community 14 - "Community 14"
Cohesion: 0.16
Nodes (3): Update position labels at ~5 Hz., Runs at ~10 Hz, publishes cmd_vel based on currently held keys., RobotControlGUI

### Community 15 - "Community 15"
Cohesion: 0.21
Nodes (7): _as_bool(), GreenhousePlanningScene, _inverse_rotate_rpy(), main(), _parse_pose(), _quat_from_rpy(), _rotate_rpy()

### Community 16 - "Community 16"
Cohesion: 0.23
Nodes (9): FR3FK, main(), PickAndPlace, pos(), Analytic FK from fr3_link0 to fr3_hand_tcp.     Each frame: [tx, ty, tz, roll, p, _rpy(), _Rz(), show() (+1 more)

### Community 17 - "Community 17"
Cohesion: 0.12
Nodes (6): Return current arm joint positions or home defaults., Build a RobotState message for the arm joints., Call MoveIt GetCartesianPath for smooth straight-line motion.          Returns (, Execute a RobotTrajectory from GetCartesianPath via the arm controller., Single-point IK move to a Cartesian pose., Smooth Cartesian straight-line move through waypoint poses.

### Community 18 - "Community 18"
Cohesion: 0.23
Nodes (5): _class_filter_param(), _image_to_rgb(), main(), Ultralytics YOLO detector bridge for tomato RGB images.  Publishes generic JSON, YoloTomatoDetector

### Community 20 - "Community 20"
Cohesion: 0.22
Nodes (4): main(), Bridge mapped tomato records into the PoseStamped topic used by pick., _record_label(), TomatoMapPickTarget

### Community 21 - "Community 21"
Cohesion: 0.22
Nodes (6): franka_fk(), main(), Simplified Forward Kinematics to calculate X,Y,Z of the TCP, Custom Numerical Inverse Kinematics Solver, solve_ik(), VisionPickAndPlace

### Community 22 - "Community 22"
Cohesion: 0.36
Nodes (10): generate_launch_description(), load_yaml(), combined_gazebo.launch.py Launches the Panther + Franka FR3 combined robot in Ga, # NOTE: imu_broadcaster is intentionally NOT spawned here., # NOTE: imu_broadcaster is intentionally NOT spawned here., # NOTE: imu_broadcaster is intentionally NOT spawned here., # NOTE: imu_broadcaster is intentionally NOT spawned here., # NOTE: imu_broadcaster is intentionally NOT spawned here. (+2 more)

### Community 23 - "Community 23"
Cohesion: 0.23
Nodes (6): _make_pose(), _quat_conjugate(), Return tomato center offset in the TCP frame at the selected pick pose., Return tomato center offset in the TCP frame at the selected pick pose., Return tomato center offset in the TCP frame at the selected pick pose., _rotate_vector()

### Community 24 - "Community 24"
Cohesion: 0.39
Nodes (8): activate_controller(), check_topic(), get_controller_states(), main(), Returns dict of controller_name -> state., Try to activate a controller., Check if a topic has publishers., run()

### Community 25 - "Community 25"
Cohesion: 0.42
Nodes (8): linked_image_from_socket(), main(), material_base(), optimize(), prepare_source(), reset_scene(), scale_image(), simplified_material()

### Community 27 - "Community 27"
Cohesion: 0.33
Nodes (4): combined_gazebo.launch.py Launches the Panther + Franka FR3 combined robot in Ga, # IMPORTANT: This is delayed (see below) so that drive_controller's, # NOTE: odometry/wheels remap REMOVED — drive_controller already, # NOTE: imu_broadcaster is intentionally NOT spawned here.

### Community 28 - "Community 28"
Cohesion: 0.33
Nodes (5): _forward_approach_orientation(), TCP z-axis points toward +X in fr3_link0 frame (forward approach).      FR3 TCP, TCP z-axis points toward ±Y for side approach., Choose approach orientation based on tomato position., _side_approach_orientation()

### Community 29 - "Community 29"
Cohesion: 0.6
Nodes (3): colcon_append_unique_value(), colcon_package_source_powershell_script(), colcon_prepend_unique_value()

### Community 30 - "Community 30"
Cohesion: 0.7
Nodes (3): generate_launch_description(), load_yaml(), Launch big-greenhouse nearest-tomato pick-place with Panther + Franka FR3.

### Community 31 - "Community 31"
Cohesion: 0.6
Nodes (3): generate_launch_description(), combined_gazebo.launch.py Launches the Panther + Franka FR3 combined robot in Ga, # NOTE: imu_broadcaster is intentionally NOT spawned here.

## Knowledge Gaps
- **80 isolated node(s):** `Analytic FK from fr3_link0 to fr3_hand_tcp.     Each frame: [tx, ty, tz, roll, p`, `Return 4x4 transform from fr3_link0 to hand_tcp.`, `Simplified Forward Kinematics to calculate X,Y,Z of the TCP`, `Custom Numerical Inverse Kinematics Solver`, `Exact Forward Kinematics matching FR3 URDF` (+75 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **21 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `DetachablePickPlace` connect `Community 4` to `Community 8`, `Community 3`, `Community 6`?**
  _High betweenness centrality (0.231) - this node is a cross-community bridge._
- **Why does `GreenhouseNearestPickPlace` connect `Community 8` to `Community 4`, `Community 6`, `Community 17`, `Community 19`, `Community 23`, `Community 26`, `Community 28`?**
  _High betweenness centrality (0.171) - this node is a cross-community bridge._
- **Why does `MissionManager` connect `Community 0` to `Community 2`, `Community 3`?**
  _High betweenness centrality (0.152) - this node is a cross-community bridge._
- **Are the 39 inferred relationships involving `str` (e.g. with `prepare_source()` and `optimize()`) actually correct?**
  _`str` has 39 INFERRED edges - model-reasoned connections that need verification._
- **Are the 2 inferred relationships involving `DetachablePickPlace` (e.g. with `GreenhouseNearestPickPlace` and `TomatoCandidate`) actually correct?**
  _`DetachablePickPlace` has 2 INFERRED edges - model-reasoned connections that need verification._
- **What connects `Analytic FK from fr3_link0 to fr3_hand_tcp.     Each frame: [tx, ty, tz, roll, p`, `Return 4x4 transform from fr3_link0 to hand_tcp.`, `Simplified Forward Kinematics to calculate X,Y,Z of the TCP` to the rest of the system?**
  _80 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Community 0` be split into smaller, more focused modules?**
  _Cohesion score 0.06 - nodes in this community are weakly interconnected._