import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    combined_share = get_package_share_directory('combined_robot')

    launch_nav2 = LaunchConfiguration('launch_nav2')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world_file')
    gz_gui = LaunchConfiguration('gz_gui')
    run_rviz = LaunchConfiguration('run_rviz')
    run_planning_scene = LaunchConfiguration('run_planning_scene')
    bridge_depth_points = LaunchConfiguration('bridge_depth_points')
    enable_3d_lidar = LaunchConfiguration('enable_3d_lidar')
    robot_spawn_x = LaunchConfiguration('robot_spawn_x')
    robot_spawn_y = LaunchConfiguration('robot_spawn_y')
    robot_spawn_z = LaunchConfiguration('robot_spawn_z')
    robot_spawn_yaw = LaunchConfiguration('robot_spawn_yaw')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    waypoint_file = LaunchConfiguration('waypoint_file')
    route_name = LaunchConfiguration('route_name')
    mission_mode = LaunchConfiguration('mission_mode')
    arm_pose_file = LaunchConfiguration('arm_pose_file')
    arm_pose_enabled = LaunchConfiguration('arm_pose_enabled')
    arm_motion_mode = LaunchConfiguration('arm_motion_mode')
    arm_controller_action = LaunchConfiguration('arm_controller_action')
    move_action = LaunchConfiguration('move_action')
    arm_avoid_tomatoes = LaunchConfiguration('arm_avoid_tomatoes')
    arm_avoid_tomato_radius_m = LaunchConfiguration('arm_avoid_tomato_radius_m')
    arm_avoid_tomato_z_offset_m = LaunchConfiguration('arm_avoid_tomato_z_offset_m')
    gripper_controller_action = LaunchConfiguration('gripper_controller_action')
    arm_action_server_timeout_sec = LaunchConfiguration('arm_action_server_timeout_sec')
    gripper_action_server_timeout_sec = LaunchConfiguration('gripper_action_server_timeout_sec')
    nav_goal_timeout_sec = LaunchConfiguration('nav_goal_timeout_sec')
    verify_goal_xy_tolerance = LaunchConfiguration('verify_goal_xy_tolerance')
    direct_turn_xy_tolerance = LaunchConfiguration('direct_turn_xy_tolerance')
    mission_autostart = LaunchConfiguration('mission_autostart')
    autostart_delay_sec = LaunchConfiguration('autostart_delay_sec')
    loop_route = LaunchConfiguration('loop_route')
    stop_on_failure = LaunchConfiguration('stop_on_failure')
    publish_initial_pose = LaunchConfiguration('publish_initial_pose')
    initial_pose_file = LaunchConfiguration('initial_pose_file')
    initial_pose_publish_delay_sec = LaunchConfiguration('initial_pose_publish_delay_sec')
    initial_pose_publish_count = LaunchConfiguration('initial_pose_publish_count')
    run_tomato_depth_mapper = LaunchConfiguration('run_tomato_depth_mapper')
    run_tomato_map_panel = LaunchConfiguration('run_tomato_map_panel')
    run_tomato_collision_scene = LaunchConfiguration('run_tomato_collision_scene')
    run_yolo_tomato_detector = LaunchConfiguration('run_yolo_tomato_detector')
    tomato_detection_mode = LaunchConfiguration('tomato_detection_mode')
    tomato_pixel_u = LaunchConfiguration('tomato_pixel_u')
    tomato_pixel_v = LaunchConfiguration('tomato_pixel_v')
    yolo_model_path = LaunchConfiguration('yolo_model_path')
    yolo_site_packages = LaunchConfiguration('yolo_site_packages')
    yolo_image_topic = LaunchConfiguration('yolo_image_topic')
    yolo_detection_topic = LaunchConfiguration('yolo_detection_topic')
    yolo_confidence_threshold = LaunchConfiguration('yolo_confidence_threshold')
    yolo_iou_threshold = LaunchConfiguration('yolo_iou_threshold')
    yolo_publish_rate_hz = LaunchConfiguration('yolo_publish_rate_hz')
    yolo_device = LaunchConfiguration('yolo_device')
    yolo_imgsz = LaunchConfiguration('yolo_imgsz')
    yolo_half = LaunchConfiguration('yolo_half')
    yolo_max_det = LaunchConfiguration('yolo_max_det')
    yolo_class_filter = LaunchConfiguration('yolo_class_filter')
    tomato_mapper_rgb_topic = LaunchConfiguration('tomato_mapper_rgb_topic')
    tomato_mapper_depth_topic = LaunchConfiguration('tomato_mapper_depth_topic')
    tomato_mapper_camera_info_topic = LaunchConfiguration('tomato_mapper_camera_info_topic')
    tomato_mapper_current_waypoint_topic = LaunchConfiguration(
        'tomato_mapper_current_waypoint_topic'
    )
    tomato_mapper_current_waypoint = LaunchConfiguration('tomato_mapper_current_waypoint')
    tomato_mapper_camera_frame = LaunchConfiguration('tomato_mapper_camera_frame')
    tomato_mapper_target_frame = LaunchConfiguration('tomato_mapper_target_frame')
    tomato_mapper_global_frame = LaunchConfiguration('tomato_mapper_global_frame')
    tomato_mapper_fallback_global_frame = LaunchConfiguration(
        'tomato_mapper_fallback_global_frame'
    )
    tomato_mapper_prefer_global_frame = LaunchConfiguration('tomato_mapper_prefer_global_frame')
    tomato_mapper_use_latest_tf = LaunchConfiguration('tomato_mapper_use_latest_tf')
    tomato_mapper_correct_depth_to_center = LaunchConfiguration(
        'tomato_mapper_correct_depth_to_center'
    )
    tomato_mapper_merge_distance_m = LaunchConfiguration('tomato_mapper_merge_distance_m')
    tomato_mapper_min_depth_m = LaunchConfiguration('tomato_mapper_min_depth_m')
    tomato_mapper_max_depth_m = LaunchConfiguration('tomato_mapper_max_depth_m')
    tomato_mapper_publish_rate_hz = LaunchConfiguration('tomato_mapper_publish_rate_hz')
    tomato_mapper_panel_rate_hz = LaunchConfiguration('tomato_mapper_panel_rate_hz')
    tomato_mapper_depth_bbox_prefer_inner_roi = LaunchConfiguration(
        'tomato_mapper_depth_bbox_prefer_inner_roi'
    )
    tomato_mapper_depth_bbox_inner_fraction = LaunchConfiguration(
        'tomato_mapper_depth_bbox_inner_fraction'
    )
    tomato_mapper_depth_bbox_min_valid_samples = LaunchConfiguration(
        'tomato_mapper_depth_bbox_min_valid_samples'
    )
    tomato_mapper_depth_bbox_fallback_enabled = LaunchConfiguration(
        'tomato_mapper_depth_bbox_fallback_enabled'
    )
    tomato_mapper_depth_bbox_percentile = LaunchConfiguration(
        'tomato_mapper_depth_bbox_percentile'
    )
    tomato_mapper_model_filter_enabled = LaunchConfiguration(
        'tomato_mapper_model_filter_enabled'
    )
    tomato_mapper_model_filter_frame = LaunchConfiguration(
        'tomato_mapper_model_filter_frame'
    )
    tomato_mapper_model_filter_max_distance_m = LaunchConfiguration(
        'tomato_mapper_model_filter_max_distance_m'
    )
    tomato_mapper_model_filter_use_live_gazebo_pose = LaunchConfiguration(
        'tomato_mapper_model_filter_use_live_gazebo_pose'
    )
    tomato_mapper_model_filter_snap_to_model_center = LaunchConfiguration(
        'tomato_mapper_model_filter_snap_to_model_center'
    )
    tomato_mapper_model_filter_match_strategy = LaunchConfiguration(
        'tomato_mapper_model_filter_match_strategy'
    )
    tomato_mapper_model_filter_projection_max_center_px = LaunchConfiguration(
        'tomato_mapper_model_filter_projection_max_center_px'
    )
    tomato_mapper_model_filter_projection_bbox_margin = LaunchConfiguration(
        'tomato_mapper_model_filter_projection_bbox_margin'
    )
    tomato_collision_input_topic = LaunchConfiguration('tomato_collision_input_topic')
    tomato_collision_planning_scene_topic = LaunchConfiguration(
        'tomato_collision_planning_scene_topic'
    )
    tomato_collision_marker_topic = LaunchConfiguration('tomato_collision_marker_topic')
    tomato_collision_planning_frame = LaunchConfiguration('tomato_collision_planning_frame')
    tomato_collision_radius_m = LaunchConfiguration('tomato_collision_radius_m')
    tomato_collision_radius_scale = LaunchConfiguration('tomato_collision_radius_scale')
    tomato_collision_min_radius_m = LaunchConfiguration('tomato_collision_min_radius_m')
    tomato_collision_max_radius_m = LaunchConfiguration('tomato_collision_max_radius_m')
    tomato_collision_min_confidence = LaunchConfiguration('tomato_collision_min_confidence')
    tomato_collision_min_updated_count = LaunchConfiguration(
        'tomato_collision_min_updated_count'
    )
    tomato_collision_class_filter = LaunchConfiguration('tomato_collision_class_filter')
    tomato_collision_exclude_classes = LaunchConfiguration('tomato_collision_exclude_classes')
    tomato_collision_exclude_models = LaunchConfiguration('tomato_collision_exclude_models')
    tomato_collision_publish_planning_scene = LaunchConfiguration(
        'tomato_collision_publish_planning_scene'
    )
    tomato_collision_publish_markers = LaunchConfiguration('tomato_collision_publish_markers')
    harvest_pick_enabled = LaunchConfiguration('harvest_pick_enabled')
    harvest_pick_after_waypoint = LaunchConfiguration('harvest_pick_after_waypoint')
    harvest_pick_auto_select = LaunchConfiguration('harvest_pick_auto_select')
    harvest_pick_inventory_topic = LaunchConfiguration('harvest_pick_inventory_topic')
    harvest_pick_target_selection_topic = LaunchConfiguration(
        'harvest_pick_target_selection_topic'
    )
    harvest_pick_allowed_classes = LaunchConfiguration('harvest_pick_allowed_classes')
    harvest_pick_reject_classes = LaunchConfiguration('harvest_pick_reject_classes')
    harvest_pick_good_classes = LaunchConfiguration('harvest_pick_good_classes')
    harvest_pick_bad_classes = LaunchConfiguration('harvest_pick_bad_classes')
    harvest_pick_min_confidence = LaunchConfiguration('harvest_pick_min_confidence')
    harvest_pick_min_updated_count = LaunchConfiguration('harvest_pick_min_updated_count')
    harvest_pick_local_radius_m = LaunchConfiguration('harvest_pick_local_radius_m')
    harvest_pick_max_base_lateral_m = LaunchConfiguration('harvest_pick_max_base_lateral_m')
    harvest_pick_min_z_m = LaunchConfiguration('harvest_pick_min_z_m')
    harvest_pick_max_z_m = LaunchConfiguration('harvest_pick_max_z_m')
    harvest_pick_require_model_name = LaunchConfiguration('harvest_pick_require_model_name')
    harvest_pick_prefer_model_pose = LaunchConfiguration('harvest_pick_prefer_model_pose')
    harvest_pick_fallback_to_configured = LaunchConfiguration(
        'harvest_pick_fallback_to_configured'
    )
    harvest_pick_target_exclusion_settle_sec = LaunchConfiguration(
        'harvest_pick_target_exclusion_settle_sec'
    )
    harvest_pick_detection_settle_sec = LaunchConfiguration(
        'harvest_pick_detection_settle_sec'
    )
    harvest_pick_inventory_max_age_sec = LaunchConfiguration(
        'harvest_pick_inventory_max_age_sec'
    )
    harvest_pick_max_attempts = LaunchConfiguration('harvest_pick_max_attempts')
    harvest_pick_max_per_waypoint = LaunchConfiguration('harvest_pick_max_per_waypoint')
    harvest_return_to_pick_pose_after_attempt = LaunchConfiguration(
        'harvest_return_to_pick_pose_after_attempt'
    )
    harvest_pick_grasp_z_offsets = LaunchConfiguration('harvest_pick_grasp_z_offsets')
    harvest_pick_max_candidates_per_target = LaunchConfiguration(
        'harvest_pick_max_candidates_per_target'
    )
    harvest_pick_tomato_model = LaunchConfiguration('harvest_pick_tomato_model')
    harvest_pick_basket = LaunchConfiguration('harvest_pick_basket')
    harvest_pick_target_topic = LaunchConfiguration('harvest_pick_target_topic')
    harvest_pick_target_radius_topic = LaunchConfiguration('harvest_pick_target_radius_topic')
    harvest_pick_stop_route_after_attempt = LaunchConfiguration(
        'harvest_pick_stop_route_after_attempt'
    )
    harvest_pick_place_in_basket = LaunchConfiguration('harvest_pick_place_in_basket')
    harvest_pick_lock_harvested_to_basket = LaunchConfiguration(
        'harvest_pick_lock_harvested_to_basket'
    )
    harvest_pick_freeze_basket_z_offset = LaunchConfiguration(
        'harvest_pick_freeze_basket_z_offset'
    )
    harvest_pick_harvested_tomato_collision_radius = LaunchConfiguration(
        'harvest_pick_harvested_tomato_collision_radius'
    )
    harvest_pick_timeout_sec = LaunchConfiguration('harvest_pick_timeout_sec')
    harvest_pick_use_live_gazebo_pose = LaunchConfiguration(
        'harvest_pick_use_live_gazebo_pose'
    )
    harvest_pick_robot_model = LaunchConfiguration('harvest_pick_robot_model')
    harvest_pick_prewarm_gripper_attachments = LaunchConfiguration(
        'harvest_pick_prewarm_gripper_attachments'
    )
    harvest_pick_gripper_attach_parent_link = LaunchConfiguration(
        'harvest_pick_gripper_attach_parent_link'
    )
    harvest_pick_base_z_offset = LaunchConfiguration('harvest_pick_base_z_offset')
    harvest_pick_gazebo_pose_timeout_sec = LaunchConfiguration(
        'harvest_pick_gazebo_pose_timeout_sec'
    )
    run_gazebo_tomato_detector = LaunchConfiguration('run_gazebo_tomato_detector')
    gazebo_tomato_class_filter = LaunchConfiguration('gazebo_tomato_class_filter')
    gazebo_tomato_max_forward_m = LaunchConfiguration('gazebo_tomato_max_forward_m')
    gazebo_tomato_max_lateral_m = LaunchConfiguration('gazebo_tomato_max_lateral_m')
    gazebo_tomato_publish_rate_hz = LaunchConfiguration('gazebo_tomato_publish_rate_hz')
    gazebo_tomato_live_pose_query_period_sec = LaunchConfiguration(
        'gazebo_tomato_live_pose_query_period_sec'
    )
    gazebo_tomato_pose_timeout_sec = LaunchConfiguration('gazebo_tomato_pose_timeout_sec')
    gazebo_tomato_pose_source = LaunchConfiguration('gazebo_tomato_pose_source')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('combined_robot'),
                'launch',
                'sera_nav2.launch.py',
            ])
        ),
        condition=IfCondition(launch_nav2),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world_file': world_file,
            'gz_gui': gz_gui,
            'run_rviz': run_rviz,
            'run_moveit': 'true',
            'run_planning_scene': run_planning_scene,
            'bridge_depth_points': bridge_depth_points,
            'enable_3d_lidar': enable_3d_lidar,
            'robot_spawn_x': robot_spawn_x,
            'robot_spawn_y': robot_spawn_y,
            'robot_spawn_z': robot_spawn_z,
            'robot_spawn_yaw': robot_spawn_yaw,
            'params_file': nav2_params_file,
            'publish_initial_pose': 'false',
        }.items(),
    )

    mission_manager = Node(
        package='combined_robot',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'waypoint_file': waypoint_file},
            {'route_name': route_name},
            {'mission_mode': mission_mode},
            {'arm_pose_file': arm_pose_file},
            {'arm_pose_enabled': arm_pose_enabled},
            {'arm_motion_mode': arm_motion_mode},
            {'arm_controller_action': arm_controller_action},
            {'move_action': move_action},
            {'arm_avoid_tomatoes': arm_avoid_tomatoes},
            {'arm_avoid_tomato_radius_m': arm_avoid_tomato_radius_m},
            {'arm_avoid_tomato_z_offset_m': arm_avoid_tomato_z_offset_m},
            {'gripper_controller_action': gripper_controller_action},
            {'arm_action_server_timeout_sec': arm_action_server_timeout_sec},
            {'gripper_action_server_timeout_sec': gripper_action_server_timeout_sec},
            {'nav_goal_timeout_sec': nav_goal_timeout_sec},
            {'verify_goal_xy_tolerance': verify_goal_xy_tolerance},
            {'verify_goal_yaw_tolerance': 0.35},
            {'use_direct_turn_waypoints': True},
            {'direct_turn_xy_tolerance': direct_turn_xy_tolerance},
            {'direct_turn_angular_speed': 0.85},
            {'direct_turn_yaw_gain': 2.0},
            {'autostart': mission_autostart},
            {'autostart_delay_sec': autostart_delay_sec},
            {'loop_route': loop_route},
            {'stop_on_failure': stop_on_failure},
            {'harvest_pick_enabled': harvest_pick_enabled},
            {'harvest_pick_after_waypoint': harvest_pick_after_waypoint},
            {'harvest_pick_auto_select': harvest_pick_auto_select},
            {'harvest_pick_inventory_topic': harvest_pick_inventory_topic},
            {'harvest_pick_target_selection_topic': harvest_pick_target_selection_topic},
            {'harvest_pick_allowed_classes': harvest_pick_allowed_classes},
            {'harvest_pick_reject_classes': harvest_pick_reject_classes},
            {'harvest_pick_good_classes': harvest_pick_good_classes},
            {'harvest_pick_bad_classes': harvest_pick_bad_classes},
            {'harvest_pick_min_confidence': harvest_pick_min_confidence},
            {'harvest_pick_min_updated_count': harvest_pick_min_updated_count},
            {'harvest_pick_local_radius_m': harvest_pick_local_radius_m},
            {'harvest_pick_max_base_lateral_m': harvest_pick_max_base_lateral_m},
            {'harvest_pick_min_z_m': harvest_pick_min_z_m},
            {'harvest_pick_max_z_m': harvest_pick_max_z_m},
            {'harvest_pick_require_model_name': harvest_pick_require_model_name},
            {'harvest_pick_prefer_model_pose': harvest_pick_prefer_model_pose},
            {'harvest_pick_fallback_to_configured': harvest_pick_fallback_to_configured},
            {'harvest_pick_target_exclusion_settle_sec': harvest_pick_target_exclusion_settle_sec},
            {'harvest_pick_detection_settle_sec': harvest_pick_detection_settle_sec},
            {'harvest_pick_inventory_max_age_sec': harvest_pick_inventory_max_age_sec},
            {'harvest_pick_max_attempts': harvest_pick_max_attempts},
            {'harvest_pick_max_per_waypoint': harvest_pick_max_per_waypoint},
            {
                'harvest_return_to_pick_pose_after_attempt':
                    harvest_return_to_pick_pose_after_attempt
            },
            {'harvest_pick_grasp_z_offsets': harvest_pick_grasp_z_offsets},
            {'harvest_pick_max_candidates_per_target': harvest_pick_max_candidates_per_target},
            {
                'harvest_pick_world_file': PathJoinSubstitution([
                    FindPackageShare('combined_robot'),
                    'worlds',
                    world_file,
                ])
            },
            {'harvest_pick_world_name': 'tomato_farm'},
            {'harvest_pick_tomato_model': harvest_pick_tomato_model},
            {'harvest_pick_basket': harvest_pick_basket},
            {'harvest_pick_target_topic': harvest_pick_target_topic},
            {'harvest_pick_target_radius_topic': harvest_pick_target_radius_topic},
            {'harvest_pick_stop_route_after_attempt': harvest_pick_stop_route_after_attempt},
            {'harvest_pick_place_in_basket': harvest_pick_place_in_basket},
            {
                'harvest_pick_lock_harvested_to_basket':
                    harvest_pick_lock_harvested_to_basket
            },
            {'harvest_pick_freeze_basket_z_offset': harvest_pick_freeze_basket_z_offset},
            {
                'harvest_pick_harvested_tomato_collision_radius':
                    harvest_pick_harvested_tomato_collision_radius
            },
            {'harvest_pick_timeout_sec': harvest_pick_timeout_sec},
            {'harvest_pick_use_live_gazebo_pose': harvest_pick_use_live_gazebo_pose},
            {'harvest_pick_robot_model': harvest_pick_robot_model},
            {
                'harvest_pick_prewarm_gripper_attachments':
                    harvest_pick_prewarm_gripper_attachments
            },
            {'harvest_pick_gripper_attach_parent_link': harvest_pick_gripper_attach_parent_link},
            {'harvest_pick_base_z_offset': harvest_pick_base_z_offset},
            {'harvest_pick_gazebo_pose_timeout_sec': harvest_pick_gazebo_pose_timeout_sec},
        ],
    )

    initial_pose_publisher = Node(
        package='combined_robot',
        executable='initial_pose_publisher',
        name='initial_pose_publisher',
        output='screen',
        condition=IfCondition(publish_initial_pose),
        parameters=[
            {'use_sim_time': False},
            {'initial_pose_file': initial_pose_file},
            {'publish_delay_sec': initial_pose_publish_delay_sec},
            {'publish_count': initial_pose_publish_count},
        ],
    )

    tomato_depth_mapper = Node(
        package='combined_robot',
        executable='tomato_depth_mapper',
        name='tomato_depth_mapper',
        output='screen',
        condition=IfCondition(run_tomato_depth_mapper),
        parameters=[
            {'use_sim_time': use_sim_time},
            {'detection_mode': tomato_detection_mode},
            {'pixel_u': tomato_pixel_u},
            {'pixel_v': tomato_pixel_v},
            {'rgb_topic': tomato_mapper_rgb_topic},
            {'depth_topic': tomato_mapper_depth_topic},
            {'camera_info_topic': tomato_mapper_camera_info_topic},
            {'future_detection_topic': yolo_detection_topic},
            {'current_waypoint_topic': tomato_mapper_current_waypoint_topic},
            {'current_waypoint': tomato_mapper_current_waypoint},
            {'camera_frame': tomato_mapper_camera_frame},
            {'target_frame': tomato_mapper_target_frame},
            {'global_frame': tomato_mapper_global_frame},
            {'fallback_global_frame': tomato_mapper_fallback_global_frame},
            {'prefer_global_frame': tomato_mapper_prefer_global_frame},
            {'use_latest_tf': tomato_mapper_use_latest_tf},
            {'correct_depth_to_center': tomato_mapper_correct_depth_to_center},
            {'merge_distance_m': tomato_mapper_merge_distance_m},
            {'min_depth_m': tomato_mapper_min_depth_m},
            {'max_depth_m': tomato_mapper_max_depth_m},
            {'publish_rate_hz': tomato_mapper_publish_rate_hz},
            {'panel_rate_hz': tomato_mapper_panel_rate_hz},
            {'depth_bbox_prefer_inner_roi': tomato_mapper_depth_bbox_prefer_inner_roi},
            {'depth_bbox_inner_fraction': tomato_mapper_depth_bbox_inner_fraction},
            {'depth_bbox_min_valid_samples': tomato_mapper_depth_bbox_min_valid_samples},
            {'depth_bbox_fallback_enabled': tomato_mapper_depth_bbox_fallback_enabled},
            {'depth_bbox_percentile': tomato_mapper_depth_bbox_percentile},
            {'model_filter_enabled': tomato_mapper_model_filter_enabled},
            {
                'model_filter_world_file': PathJoinSubstitution([
                    FindPackageShare('combined_robot'),
                    'worlds',
                    world_file,
                ])
            },
            {'model_filter_frame': tomato_mapper_model_filter_frame},
            {'model_filter_max_distance_m': tomato_mapper_model_filter_max_distance_m},
            {'model_filter_map_x_from_gazebo_y_offset': -4.93},
            {'model_filter_map_y_from_gazebo_x_origin': 35.83},
            {'model_filter_use_live_gazebo_pose': tomato_mapper_model_filter_use_live_gazebo_pose},
            {'model_filter_snap_to_model_center': tomato_mapper_model_filter_snap_to_model_center},
            {'model_filter_match_strategy': tomato_mapper_model_filter_match_strategy},
            {'model_filter_projection_max_center_px': tomato_mapper_model_filter_projection_max_center_px},
            {'model_filter_projection_bbox_margin': tomato_mapper_model_filter_projection_bbox_margin},
            {'model_filter_robot_model': harvest_pick_robot_model},
            {'model_filter_robot_base_z_offset': harvest_pick_base_z_offset},
            {'model_filter_live_pose_timeout_sec': gazebo_tomato_pose_timeout_sec},
            {'model_filter_live_pose_query_period_sec': gazebo_tomato_live_pose_query_period_sec},
            {'output_topic': '/tomato_map/list'},
        ],
    )

    yolo_tomato_detector = Node(
        package='combined_robot',
        executable='yolo_tomato_detector',
        name='yolo_tomato_detector',
        output='screen',
        condition=IfCondition(run_yolo_tomato_detector),
        parameters=[
            {'use_sim_time': use_sim_time},
            {'model_path': yolo_model_path},
            {'yolo_site_packages': yolo_site_packages},
            {'image_topic': yolo_image_topic},
            {'output_topic': yolo_detection_topic},
            {'confidence_threshold': yolo_confidence_threshold},
            {'iou_threshold': yolo_iou_threshold},
            {'publish_rate_hz': yolo_publish_rate_hz},
            {'device': yolo_device},
            {'imgsz': yolo_imgsz},
            {'half': yolo_half},
            {'max_det': yolo_max_det},
            {'class_filter': yolo_class_filter},
        ],
    )

    tomato_map_panel = Node(
        package='combined_robot',
        executable='tomato_map_panel',
        name='tomato_map_panel',
        output='screen',
        condition=IfCondition(run_tomato_map_panel),
        parameters=[
            {'topic': '/tomato_map/list'},
        ],
    )

    tomato_collision_scene_manager = Node(
        package='combined_robot',
        executable='tomato_collision_scene_manager',
        name='tomato_collision_scene_manager',
        output='screen',
        condition=IfCondition(run_tomato_collision_scene),
        parameters=[
            {'use_sim_time': use_sim_time},
            {'input_topic': tomato_collision_input_topic},
            {'picked_topic': '/tomato_harvest/picked'},
            {'target_topic': harvest_pick_target_selection_topic},
            {'planning_scene_topic': tomato_collision_planning_scene_topic},
            {'marker_topic': tomato_collision_marker_topic},
            {'default_frame': tomato_mapper_target_frame},
            {'planning_frame': tomato_collision_planning_frame},
            {'collision_radius_m': tomato_collision_radius_m},
            {'marker_radius_m': tomato_collision_radius_m},
            {'radius_scale': tomato_collision_radius_scale},
            {'min_radius_m': tomato_collision_min_radius_m},
            {'max_radius_m': tomato_collision_max_radius_m},
            {'min_confidence': tomato_collision_min_confidence},
            {'min_updated_count': tomato_collision_min_updated_count},
            {'class_filter': tomato_collision_class_filter},
            {'exclude_classes': tomato_collision_exclude_classes},
            {'exclude_model_names': tomato_collision_exclude_models},
            {'publish_planning_scene': tomato_collision_publish_planning_scene},
            {'publish_markers': tomato_collision_publish_markers},
        ],
    )

    gazebo_tomato_detector = Node(
        package='combined_robot',
        executable='gazebo_tomato_detector',
        name='gazebo_tomato_detector',
        output='screen',
        condition=IfCondition(run_gazebo_tomato_detector),
        parameters=[
            {'use_sim_time': use_sim_time},
            {
                'world_file': PathJoinSubstitution([
                    FindPackageShare('combined_robot'),
                    'worlds',
                    world_file,
                ])
            },
            {'world_name': 'tomato_farm'},
            {'robot_model': 'combined_robot'},
            {'base_frame': 'fr3_link0'},
            {'tf_base_frame': 'fr3_link0'},
            {'map_frame': 'map'},
            {'pose_source': gazebo_tomato_pose_source},
            {'class_filter': gazebo_tomato_class_filter},
            {'output_pose_topic': '/gazebo_tomato_detector/tomato_center'},
            {'output_radius_topic': '/gazebo_tomato_detector/tomato_radius'},
            {'output_selected_topic': '/gazebo_tomato_detector/selected'},
            {'output_list_topic': '/tomato_map/list'},
            {'publish_rate_hz': gazebo_tomato_publish_rate_hz},
            {'min_forward_m': 0.0},
            {'max_forward_m': gazebo_tomato_max_forward_m},
            {'max_lateral_m': gazebo_tomato_max_lateral_m},
            {'min_base_z_m': 0.10},
            {'max_base_z_m': 2.00},
            {'use_live_robot_pose': True},
            {'robot_base_z_offset': harvest_pick_base_z_offset},
            {'map_x_from_gazebo_y_offset': -4.93},
            {'map_y_from_gazebo_x_origin': 35.83},
            {'live_pose_query_period_sec': gazebo_tomato_live_pose_query_period_sec},
            {'gazebo_pose_timeout_sec': gazebo_tomato_pose_timeout_sec},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_nav2',
            default_value='true',
            description='Start Gazebo and Nav2 before the mission manager.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock.',
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='tomato_farm_sera.sdf',
            description='Greenhouse SDF world file. Defaults to the tomato-populated greenhouse.',
        ),
        DeclareLaunchArgument(
            'gz_gui',
            default_value='true',
            description='Start Gazebo GUI. Set false for headless mission runs.',
        ),
        DeclareLaunchArgument(
            'run_rviz',
            default_value='true',
            description='Start RViz with MoveIt interactive markers.',
        ),
        DeclareLaunchArgument(
            'run_planning_scene',
            default_value='true',
            description='Start greenhouse collision objects in the MoveIt planning scene.',
        ),
        DeclareLaunchArgument(
            'bridge_depth_points',
            default_value='false',
            description='Bridge RGB-D PointCloud2 from Gazebo. False keeps the greenhouse mission smoother.',
        ),
        DeclareLaunchArgument(
            'enable_3d_lidar',
            default_value='false',
            description='Spawn temporary 3D GPU LiDAR and bridge /lidar/points.',
        ),
        DeclareLaunchArgument(
            'robot_spawn_x',
            default_value='36.02',
            description='Gazebo X coordinate for the combined robot spawn pose.',
        ),
        DeclareLaunchArgument(
            'robot_spawn_y',
            default_value='5.80',
            description='Gazebo Y coordinate for the combined robot spawn pose.',
        ),
        DeclareLaunchArgument(
            'robot_spawn_z',
            default_value='0.25',
            description='Gazebo Z coordinate for the combined robot spawn pose.',
        ),
        DeclareLaunchArgument(
            'robot_spawn_yaw',
            default_value='1.5858',
            description='Gazebo yaw angle in radians for the combined robot spawn pose.',
        ),
        DeclareLaunchArgument(
            'waypoint_file',
            default_value=os.path.join(combined_share, 'config', 'sera_waypoints.yaml'),
            description='YAML file containing greenhouse waypoint routes.',
        ),
        DeclareLaunchArgument(
            'route_name',
            default_value='center_corridor_patrol',
            description='Route name from the waypoint file.',
        ),
        DeclareLaunchArgument(
            'mission_mode',
            default_value='waypoint',
            description='Mission behavior mode: waypoint, survey_harvest, or stationary_harvest.',
        ),
        DeclareLaunchArgument(
            'arm_pose_file',
            default_value=os.path.join(combined_share, 'config', 'fr3_observation_poses.yaml'),
            description='YAML file containing FR3 camera observation poses.',
        ),
        DeclareLaunchArgument(
            'arm_pose_enabled',
            default_value='true',
            description='Send FR3 camera observation poses during survey_harvest mode.',
        ),
        DeclareLaunchArgument(
            'arm_motion_mode',
            default_value='moveit',
            description='Arm motion mode: direct_joint or moveit.',
        ),
        DeclareLaunchArgument(
            'arm_controller_action',
            default_value='/fr3_arm_controller/follow_joint_trajectory',
            description='FollowJointTrajectory action for the FR3 arm controller.',
        ),
        DeclareLaunchArgument(
            'move_action',
            default_value='/move_action',
            description='MoveIt MoveGroup action for collision-aware arm motion.',
        ),
        DeclareLaunchArgument(
            'arm_avoid_tomatoes',
            default_value='false',
            description='Add tomato collision spheres to FR3 observation MoveIt plans.',
        ),
        DeclareLaunchArgument(
            'arm_avoid_tomato_radius_m',
            default_value='0.085',
            description='Collision sphere radius used around tomato SDF model poses.',
        ),
        DeclareLaunchArgument(
            'arm_avoid_tomato_z_offset_m',
            default_value='0.0',
            description='Optional z offset for tomato collision sphere centers.',
        ),
        DeclareLaunchArgument(
            'gripper_controller_action',
            default_value='/fr3_gripper_controller/follow_joint_trajectory',
            description='FollowJointTrajectory action for the FR3 gripper controller.',
        ),
        DeclareLaunchArgument(
            'arm_action_server_timeout_sec',
            default_value='5.0',
            description='Seconds to wait for the FR3 arm action server before continuing.',
        ),
        DeclareLaunchArgument(
            'gripper_action_server_timeout_sec',
            default_value='5.0',
            description='Seconds to wait for the FR3 gripper action server before continuing.',
        ),
        DeclareLaunchArgument(
            'nav_goal_timeout_sec',
            default_value='120.0',
            description='Seconds allowed for each Nav2 waypoint before canceling it.',
        ),
        DeclareLaunchArgument(
            'verify_goal_xy_tolerance',
            default_value='0.30',
            description='Extra mission-manager XY tolerance after Nav2 reports waypoint success.',
        ),
        DeclareLaunchArgument(
            'direct_turn_xy_tolerance',
            default_value='0.30',
            description='XY tolerance used to handle same-position yaw waypoints without Nav2.',
        ),
        DeclareLaunchArgument(
            'mission_autostart',
            default_value='false',
            description='Start route automatically when mission manager starts.',
        ),
        DeclareLaunchArgument(
            'autostart_delay_sec',
            default_value='18.0',
            description='Seconds to wait after autostart is ready before sending the first mission goal.',
        ),
        DeclareLaunchArgument(
            'loop_route',
            default_value='false',
            description='Restart from the first waypoint after the route completes.',
        ),
        DeclareLaunchArgument(
            'stop_on_failure',
            default_value='true',
            description='Stop the mission when navigation, arm observation, or harvest pick fails.',
        ),
        DeclareLaunchArgument(
            'publish_initial_pose',
            default_value='true',
            description='Publish the saved AMCL initial pose automatically.',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(combined_share, 'config', 'sera_nav2_params.yaml'),
            description='Nav2 parameters file passed to the embedded sera_nav2 launch.',
        ),
        DeclareLaunchArgument(
            'initial_pose_file',
            default_value=os.path.join(combined_share, 'config', 'sera_initial_pose.yaml'),
            description='YAML file containing the saved AMCL initial pose.',
        ),
        DeclareLaunchArgument(
            'initial_pose_publish_delay_sec',
            default_value='8.0',
            description='Seconds to wait before publishing the saved initial pose.',
        ),
        DeclareLaunchArgument(
            'initial_pose_publish_count',
            default_value='45',
            description='Number of times to publish the saved initial pose.',
        ),
        DeclareLaunchArgument(
            'run_tomato_depth_mapper',
            default_value='true',
            description='Start the RGB-D tomato coordinate mapper and live table panel.',
        ),
        DeclareLaunchArgument(
            'run_tomato_map_panel',
            default_value='true',
            description='Start the GUI table panel for /tomato_map/list.',
        ),
        DeclareLaunchArgument(
            'run_tomato_collision_scene',
            default_value='false',
            description='Mirror /tomato_map/list tomatoes into RViz markers and MoveIt collision objects.',
        ),
        DeclareLaunchArgument(
            'run_gazebo_tomato_detector',
            default_value='false',
            description='Temporary Gazebo model tomato detector for survey panel until YOLO is ready.',
        ),
        DeclareLaunchArgument(
            'run_yolo_tomato_detector',
            default_value='false',
            description='Start Ultralytics YOLO tomato detector for camera RGB images.',
        ),
        DeclareLaunchArgument(
            'yolo_model_path',
            default_value='/home/yusuf/robot_workspaces/combined_ws/yolo_models/tomato/best.pt',
            description='Path to the trained YOLO tomato model.',
        ),
        DeclareLaunchArgument(
            'yolo_site_packages',
            default_value='/home/yusuf/yolo_env/lib/python3.12/site-packages',
            description='Python site-packages path containing ultralytics and torch.',
        ),
        DeclareLaunchArgument(
            'yolo_image_topic',
            default_value='/camera/color/image_raw',
            description='RGB image topic consumed by yolo_tomato_detector.',
        ),
        DeclareLaunchArgument(
            'yolo_detection_topic',
            default_value='/yolo/tomato_detections_json',
            description='Generic JSON detection topic consumed by tomato_depth_mapper.',
        ),
        DeclareLaunchArgument(
            'yolo_confidence_threshold',
            default_value='0.35',
            description='Minimum YOLO confidence for detections.',
        ),
        DeclareLaunchArgument(
            'yolo_iou_threshold',
            default_value='0.45',
            description='YOLO NMS IoU threshold.',
        ),
        DeclareLaunchArgument(
            'yolo_publish_rate_hz',
            default_value='4.0',
            description='Maximum YOLO inference rate.',
        ),
        DeclareLaunchArgument(
            'yolo_device',
            default_value='cuda:0',
            description='YOLO inference device, e.g. cuda:0 or cpu.',
        ),
        DeclareLaunchArgument(
            'yolo_imgsz',
            default_value='640',
            description='YOLO inference image size. Lower values reduce VRAM use.',
        ),
        DeclareLaunchArgument(
            'yolo_half',
            default_value='true',
            description='Use half precision for CUDA YOLO inference.',
        ),
        DeclareLaunchArgument(
            'yolo_max_det',
            default_value='80',
            description='Maximum YOLO detections per image.',
        ),
        DeclareLaunchArgument(
            'yolo_class_filter',
            default_value='all',
            description='Comma-separated YOLO class names to publish, or all.',
        ),
        DeclareLaunchArgument(
            'gazebo_tomato_class_filter',
            default_value='all',
            description='Temporary Gazebo detector class filter: empty, ripe, bad, or unripe.',
        ),
        DeclareLaunchArgument(
            'gazebo_tomato_max_forward_m',
            default_value='3.0',
            description='Forward range in fr3_link0 used by the temporary Gazebo tomato detector.',
        ),
        DeclareLaunchArgument(
            'gazebo_tomato_max_lateral_m',
            default_value='1.8',
            description='Lateral range in fr3_link0 used by the temporary Gazebo tomato detector.',
        ),
        DeclareLaunchArgument(
            'gazebo_tomato_publish_rate_hz',
            default_value='0.75',
            description='Temporary Gazebo detector publish rate. Lower values reduce CPU load.',
        ),
        DeclareLaunchArgument(
            'gazebo_tomato_live_pose_query_period_sec',
            default_value='2.0',
            description='Minimum seconds between expensive Gazebo robot pose CLI queries.',
        ),
        DeclareLaunchArgument(
            'gazebo_tomato_pose_timeout_sec',
            default_value='0.8',
            description='Timeout for each temporary Gazebo detector robot pose query.',
        ),
        DeclareLaunchArgument(
            'gazebo_tomato_pose_source',
            default_value='tf',
            description='Robot pose source for temporary Gazebo detector: tf, gazebo_cli, or fallback.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_enabled',
            default_value='true',
            description='Run one pick attempt after a configured harvest scan waypoint.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_after_waypoint',
            default_value='plant_10_b_pick_front',
            description='Waypoint name after which pick starts. Use auto_pick_front for every plant_*_pick_front.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_auto_select',
            default_value='false',
            description='Select the harvest tomato automatically from /tomato_map/list instead of fixed model name.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_inventory_topic',
            default_value='/tomato_map/list',
            description='Tomato inventory JSON topic consumed by mission_manager for auto target selection.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_target_selection_topic',
            default_value='/tomato_harvest/target_selection',
            description='JSON topic announcing the active harvest target so collision scene can exclude it.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_allowed_classes',
            default_value='fully_ripened,ripe,rotten,disease,diseased',
            description='Class/model substrings accepted for automatic harvest target selection.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_reject_classes',
            default_value='green,unripe',
            description='Class/model substrings rejected for automatic harvest target selection.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_good_classes',
            default_value='fully_ripened,ripe',
            description='Class/model substrings routed to the good/ripe basket.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_bad_classes',
            default_value='rotten,disease,diseased,bad,green,unripe',
            description='Class/model substrings routed to the bad/rotten basket.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_min_confidence',
            default_value='0.35',
            description='Minimum YOLO confidence for automatic harvest target selection.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_min_updated_count',
            default_value='2',
            description='Minimum merged observation count for automatic harvest target selection.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_local_radius_m',
            default_value='0.90',
            description='Maximum map XY distance from the active harvest waypoint for selecting a tomato.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_max_base_lateral_m',
            default_value='0.90',
            description='Reject auto-selected harvest tomatoes beyond this fr3_link0 lateral Y distance. Use 0 to disable.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_min_z_m',
            default_value='0.25',
            description='Minimum tomato map Z for automatic harvest target selection.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_max_z_m',
            default_value='1.35',
            description='Maximum tomato map Z for automatic harvest target selection.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_require_model_name',
            default_value='true',
            description='Require model_name in tomato inventory so Gazebo detach can target the exact tomato.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_prefer_model_pose',
            default_value='true',
            description='Use the matched Gazebo tomato model center before the YOLO/depth target for stable demo picking.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_fallback_to_configured',
            default_value='false',
            description='Fall back to harvest_pick_tomato_model when automatic selection finds no target.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_target_exclusion_settle_sec',
            default_value='0.4',
            description='Delay after announcing selected target before starting pick, allowing scene removal.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_detection_settle_sec',
            default_value='3.0',
            description='Wait at a harvest pick-front waypoint for fresh YOLO/depth target updates before selecting a tomato.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_inventory_max_age_sec',
            default_value='4.0',
            description='Maximum age of the last real timestamp update accepted for automatic harvest target selection.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_max_attempts',
            default_value='0',
            description='Maximum automatic pick attempts during the route. Use 0 for unlimited.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_max_per_waypoint',
            default_value='0',
            description='Maximum pick attempts at one pick-front waypoint. Use 0 for unlimited tomatoes at that plant.',
        ),
        DeclareLaunchArgument(
            'harvest_return_to_pick_pose_after_attempt',
            default_value='true',
            description='Return FR3 to the waypoint pick pose after each harvest attempt before selecting another tomato.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_grasp_z_offsets',
            default_value='0.0,0.020,-0.020',
            description='Vertical grasp offsets tried by the harvest pick process.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_max_candidates_per_target',
            default_value='6',
            description='Maximum pick pose candidates tried for one selected tomato. Use 0 for unlimited.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_tomato_model',
            default_value='tomato_ripe2_B_10_0',
            description='Gazebo/SDF tomato model to use when automatic harvest selection is disabled.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_basket',
            default_value='good',
            description='Basket argument passed to greenhouse_nearest_pick_place.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_target_topic',
            default_value='/mission_pick/tomato_center',
            description='Temporary PoseStamped target topic published by mission_manager.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_target_radius_topic',
            default_value='/mission_pick/tomato_radius',
            description='Temporary Float32 radius topic published by mission_manager for radius-aware pick distance.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_stop_route_after_attempt',
            default_value='true',
            description='Stop the mission after a harvest pick attempt.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_place_in_basket',
            default_value='true',
            description='Also run basket place after each harvest pick.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_lock_harvested_to_basket',
            default_value='false',
            description='Lock harvested tomatoes to the basket/base frame instead of leaving them dynamic.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_freeze_basket_z_offset',
            default_value='0.055',
            description='Extra local Z offset used when spawning a harvested tomato into the basket.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_harvested_tomato_collision_radius',
            default_value='0.038',
            description='Sphere collision radius for dynamic harvested tomatoes spawned into the basket.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_timeout_sec',
            default_value='180.0',
            description='Move action timeout passed to the harvest pick process.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_use_live_gazebo_pose',
            default_value='true',
            description='Use live Gazebo model poses for harvest target transforms.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_robot_model',
            default_value='combined_robot',
            description='Gazebo model name of the robot used for live harvest pick pose lookup.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_prewarm_gripper_attachments',
            default_value='false',
            description='Prepare Gazebo gripper attachment plugins before harvest picks.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_gripper_attach_parent_link',
            default_value='fr3_link7',
            description='Robot Gazebo link used as parent for harvested tomato attachment.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_base_z_offset',
            default_value='0.1715',
            description='Z offset from Gazebo robot model origin to the harvest pick base frame.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_gazebo_pose_timeout_sec',
            default_value='8.0',
            description='Timeout for live Gazebo pose queries during harvest pick setup.',
        ),
        DeclareLaunchArgument(
            'tomato_detection_mode',
            default_value='test_center_pixel',
            description='Detection adapter: test_center_pixel, manual_pixel, or yolo_topic_future.',
        ),
        DeclareLaunchArgument(
            'tomato_pixel_u',
            default_value='320.0',
            description='Manual/test pixel U used by the tomato depth mapper.',
        ),
        DeclareLaunchArgument(
            'tomato_pixel_v',
            default_value='240.0',
            description='Manual/test pixel V used by the tomato depth mapper.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_rgb_topic',
            default_value='/camera/color/image_raw',
            description='RGB image topic for tomato_depth_mapper.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_depth_topic',
            default_value='/camera/depth/image',
            description='Depth image topic for tomato_depth_mapper.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_camera_info_topic',
            default_value='/camera/depth/camera_info',
            description='Depth camera info topic for tomato_depth_mapper.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_current_waypoint_topic',
            default_value='/mission_manager/status',
            description='String topic used to read current waypoint/fidan name.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_current_waypoint',
            default_value='unknown',
            description='Fallback waypoint/fidan name when no status topic is available.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_camera_frame',
            default_value='fr3_camera_depth_optical_frame',
            description='Camera optical frame used for depth projection.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_target_frame',
            default_value='base_link',
            description='Robot frame fallback for mapped tomato points.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_global_frame',
            default_value='map',
            description='Preferred global frame for mapped tomato points.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_fallback_global_frame',
            default_value='odom',
            description='Fallback global frame when map TF is not available.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_prefer_global_frame',
            default_value='true',
            description='Prefer global map/odom output before falling back to target_frame.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_use_latest_tf',
            default_value='true',
            description='Use latest available TF for RGB-D detections to avoid stale Gazebo image timestamp extrapolation.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_correct_depth_to_center',
            default_value='true',
            description='Shift YOLO/depth surface samples along the camera ray by tomato radius before publishing pick/collision centers.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_merge_distance_m',
            default_value='0.05',
            description='3D merge distance for repeated tomato observations in one waypoint.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_min_depth_m',
            default_value='0.15',
            description='Minimum accepted depth in meters.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_max_depth_m',
            default_value='3.0',
            description='Maximum accepted depth in meters.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_publish_rate_hz',
            default_value='5.0',
            description='Maximum RGB-D mapping rate.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_panel_rate_hz',
            default_value='0.0',
            description='Terminal panel refresh rate. Use 0 to disable terminal table.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_depth_bbox_fallback_enabled',
            default_value='true',
            description='When the bbox center has invalid depth, sample valid depth inside the YOLO bbox.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_depth_bbox_prefer_inner_roi',
            default_value='true',
            description='When YOLO bbox is available, use an inner bbox ROI depth sample before center-window depth.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_depth_bbox_inner_fraction',
            default_value='0.30',
            description='Fraction of the YOLO bbox used for preferred inner ROI depth sampling.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_depth_bbox_min_valid_samples',
            default_value='5',
            description='Minimum valid depth pixels required inside the bbox ROI.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_depth_bbox_percentile',
            default_value='50.0',
            description='Depth percentile used by bbox ROI sampling; 50 is median.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_model_filter_enabled',
            default_value='true',
            description='Reject mapped detections that are not near tomato_* models in the active SDF world.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_model_filter_frame',
            default_value='map',
            description='Frame used for SDF tomato model centers; use base_link for stationary tests without map TF.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_model_filter_max_distance_m',
            default_value='0.50',
            description='Maximum map-frame distance from a tomato_* model pose for accepting a detection.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_model_filter_use_live_gazebo_pose',
            default_value='true',
            description='Also match detections against tomato model poses using live Gazebo robot pose queries.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_model_filter_snap_to_model_center',
            default_value='true',
            description='Publish the matched Gazebo tomato model center in /tomato_map/list after YOLO confirms visibility.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_model_filter_match_strategy',
            default_value='hybrid',
            description='Tomato model match strategy: hybrid, projection_2d, or spatial_3d.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_model_filter_projection_max_center_px',
            default_value='110.0',
            description='Maximum pixel error accepted when matching YOLO boxes to projected SDF tomato centers.',
        ),
        DeclareLaunchArgument(
            'tomato_mapper_model_filter_projection_bbox_margin',
            default_value='0.60',
            description='Expanded-bbox margin ratio for projection matching under leaf occlusion.',
        ),
        DeclareLaunchArgument(
            'tomato_collision_input_topic',
            default_value='/tomato_map/list',
            description='Tomato JSON list consumed by tomato_collision_scene_manager.',
        ),
        DeclareLaunchArgument(
            'tomato_collision_planning_scene_topic',
            default_value='/planning_scene',
            description='MoveIt planning scene topic for tomato collision objects.',
        ),
        DeclareLaunchArgument(
            'tomato_collision_marker_topic',
            default_value='/tomato_collision_scene/markers',
            description='RViz MarkerArray topic for visible tomato obstacle spheres.',
        ),
        DeclareLaunchArgument(
            'tomato_collision_planning_frame',
            default_value='base_link',
            description='MoveIt frame used for tomato collision objects.',
        ),
        DeclareLaunchArgument(
            'tomato_collision_radius_m',
            default_value='0.032',
            description='Fallback sphere radius for detected tomato obstacles when no per-record radius exists.',
        ),
        DeclareLaunchArgument(
            'tomato_collision_radius_scale',
            default_value='1.00',
            description='Safety scale applied to tomato obstacle radii before publishing MoveIt collision spheres.',
        ),
        DeclareLaunchArgument(
            'tomato_collision_min_radius_m',
            default_value='0.015',
            description='Minimum accepted per-record tomato collision radius.',
        ),
        DeclareLaunchArgument(
            'tomato_collision_max_radius_m',
            default_value='0.08',
            description='Maximum accepted per-record tomato collision radius.',
        ),
        DeclareLaunchArgument(
            'tomato_collision_min_confidence',
            default_value='0.0',
            description='Minimum YOLO confidence for publishing a tomato obstacle when confidence exists.',
        ),
        DeclareLaunchArgument(
            'tomato_collision_min_updated_count',
            default_value='1',
            description='Minimum merged observation count before a tomato becomes an obstacle.',
        ),
        DeclareLaunchArgument(
            'tomato_collision_class_filter',
            default_value='all',
            description='Comma-separated classes to publish as obstacles, or all.',
        ),
        DeclareLaunchArgument(
            'tomato_collision_exclude_classes',
            default_value='',
            description='Comma-separated tomato classes or model-name tokens to keep out of the obstacle scene.',
        ),
        DeclareLaunchArgument(
            'tomato_collision_exclude_models',
            default_value='',
            description='Comma-separated tomato model names to keep out of the obstacle scene.',
        ),
        DeclareLaunchArgument(
            'tomato_collision_publish_planning_scene',
            default_value='false',
            description='Publish tomato obstacles to MoveIt /planning_scene.',
        ),
        DeclareLaunchArgument(
            'tomato_collision_publish_markers',
            default_value='true',
            description='Publish visible RViz markers for tomato obstacles.',
        ),
        nav2_launch,
        initial_pose_publisher,
        mission_manager,
        yolo_tomato_detector,
        tomato_depth_mapper,
        tomato_collision_scene_manager,
        gazebo_tomato_detector,
        tomato_map_panel,
    ])
