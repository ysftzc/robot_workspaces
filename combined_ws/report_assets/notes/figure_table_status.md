# Figure and Table Asset Status

This file tracks which report assets can be produced from the current workspace and which ones need a live screenshot or manual experimental data.

## Figures

| No. | Title | Current status | Source / action |
|---:|---|---|---|
| 1 | TF tree coordinate frame hierarchy | Done | `report_assets/figures/figure01_tf_tree.png`; generated from URDF + TF publishers |
| 2 | Overall system architecture (5 layers) | Done | `report_assets/figures/figure02_system_architecture_5_layers.png`; Graphviz source in `.dot` |
| 3 | ROS2 node graph / topic flow diagram | Done | `report_assets/figures/figure03_ros2_node_topic_flow.png`; detailed version also saved |
| 4 | Gazebo greenhouse environment with B and C rows | Done | `report_assets/figures/figure04_gazebo_greenhouse_b_c_rows.png`; annotated Gazebo crop with B/C row inset |
| 5 | Panther + Franka FR3 combined robot model | Needs screenshot | Gazebo or RViz robot view |
| 6 | Robot platform - basket and gripper pad detail | Needs screenshot | Gazebo close-up; URDF confirms pads and baskets |
| 7 | RGB-D camera placement on FR3 hand | Needs screenshot | Gazebo/RViz close-up; URDF source: `panther_camera.xacro` |
| 8 | SLAM Toolbox generated greenhouse map | Done | `report_assets/figures/figure08_slam_toolbox_greenhouse_map.png`; generated from `src/combined_robot/maps/my_map.yaml` and `.pgm` |
| 9 | Keepout mask overlay on map | Done | `report_assets/figures/figure09_keepout_mask_overlay.png`; generated from `my_map.pgm` and `keepout_mask.pgm` |
| 10 | Nav2 global path and waypoints | Done | `report_assets/figures/figure10_nav2_global_path_waypoints.png`; generated from `my_map.yaml`, `keepout_mask.pgm`, `sera_initial_pose.yaml`, and `sera_waypoints.yaml` |
| 11 | AMCL particle cloud on map | Needs live RViz screenshot | Requires running Nav2/AMCL |
| 12 | MissionManager state machine diagram | Can generate | From `mission_manager.py` states and transitions |
| 13 | FR3 scan poses around plant | Can generate + screenshot | `fr3_observation_poses.yaml`; best as RViz/Gazebo montage |
| 14 | YOLO training loss and precision-recall curves | Ready source | `yolo_models/tomato/results.csv`, external run `results.png`, `BoxPR_curve.png` |
| 15 | Sample YOLO training images with annotations | Ready source | `train_batch*.jpg`, `val_batch*_labels.jpg` in YOLO run folders |
| 16 | YOLO bounding box viewer - live detections | Needs screenshot/video frame | Use last demo video or live launch |
| 17 | Tomato map GUI panel | Needs screenshot/video frame | Use last demo video or live launch |
| 18 | RViz tomato collision scene | Needs screenshot/video frame | Use RViz or demo video |
| 19 | Pick-place sequence | Needs video frame montage | Use latest demo screencast |
| 20 | OMPL approach motion in RViz | Needs RViz screenshot/video frame | Live or demo video if visible |
| 21 | Pilz LIN straight-line pick motion | Needs RViz/Gazebo frame sequence | Use demo video if visible |
| 22 | Tomato attached to gripper in Gazebo | Needs screenshot/video frame | Use demo video |
| 23 | Tomato released into basket in Gazebo | Needs screenshot/video frame | Use demo video |
| 24 | YOLO confusion matrix | Ready source | `yolo_models/tomato/confusion_matrix.png` |
| 25 | YOLO training curves over 50 epochs | Ready source | `yolo_models/tomato/results.csv`; can generate cleaned plot |

## Extended Tables

| No. | Title | Current status | Source / action |
|---:|---|---|---|
| E1 | Extended requirement verification matrix | Can draft now | Requirements in `BITIRME_RAPORU_TASLAK.txt`; verify against launch/topics |
| E2 | All ROS2 nodes with input/output topics | Can generate partially | Launch files and source topic names; live `ros2 node info` improves accuracy |
| E3 | Sensor parameters | Can fill mostly | URDF/Xacro, Nav2/bridge configs; camera 640x480, 30 Hz, FoV 1.211 rad |
| E4 | Nav2 key parameters | Can fill now | `sera_nav2_params.yaml`, `sera_nav2_params_b10_spawn.yaml` |
| E5 | YOLO training parameters | Can fill now | `yolo_models/tomato/args.yaml` |
| E6 | YOLO per-class performance | Partially available | Confusion matrix image exists; per-class metrics need validation output CSV or manual from Ultralytics logs |
| E7 | Scan pose coverage detail | Can fill now | `fr3_observation_poses.yaml`, `sera_waypoints.yaml` |
| E8 | Pick-place trial results | Needs manual/demo log data | Use screencasts and observed attempts; not fully structured yet |
| E9 | Failure cases and solutions | Can draft now | Conversation/debug history plus code fixes: attach lag, service timeout, reachability, Gazebo crash |
