import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    combined_share = get_package_share_directory('combined_robot')

    world_file = LaunchConfiguration('world_file')
    mission_autostart = LaunchConfiguration('mission_autostart')
    route_name = LaunchConfiguration('route_name')
    mission_mode = LaunchConfiguration('mission_mode')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    initial_pose_file = LaunchConfiguration('initial_pose_file')
    harvest_pick_max_attempts = LaunchConfiguration('harvest_pick_max_attempts')
    harvest_pick_max_per_waypoint = LaunchConfiguration('harvest_pick_max_per_waypoint')
    run_rviz = LaunchConfiguration('run_rviz')
    run_gazebo_gui_client = LaunchConfiguration('run_gazebo_gui_client')
    run_yolo_bbox_viewer = LaunchConfiguration('run_yolo_bbox_viewer')
    run_tomato_map_panel = LaunchConfiguration('run_tomato_map_panel')
    yolo_model_path = LaunchConfiguration('yolo_model_path')
    yolo_site_packages = LaunchConfiguration('yolo_site_packages')
    yolo_device = LaunchConfiguration('yolo_device')
    robot_spawn_x = LaunchConfiguration('robot_spawn_x')
    robot_spawn_y = LaunchConfiguration('robot_spawn_y')
    robot_spawn_z = LaunchConfiguration('robot_spawn_z')
    robot_spawn_yaw = LaunchConfiguration('robot_spawn_yaw')
    enable_3d_lidar = LaunchConfiguration('enable_3d_lidar')

    mission = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(combined_share, 'launch', 'sera_mission.launch.py')
        ),
        launch_arguments={
            'world_file': world_file,
            'gz_gui': 'false',
            'run_rviz': run_rviz,
            'route_name': route_name,
            'mission_mode': mission_mode,
            'mission_autostart': mission_autostart,
            'loop_route': 'false',
            'stop_on_failure': 'false',
            'nav2_params_file': nav2_params_file,
            'publish_initial_pose': 'true',
            'initial_pose_file': initial_pose_file,
            'robot_spawn_x': robot_spawn_x,
            'robot_spawn_y': robot_spawn_y,
            'robot_spawn_z': robot_spawn_z,
            'robot_spawn_yaw': robot_spawn_yaw,
            'enable_3d_lidar': enable_3d_lidar,
            'run_tomato_map_panel': run_tomato_map_panel,
            'run_yolo_tomato_detector': 'true',
            'yolo_model_path': yolo_model_path,
            'yolo_site_packages': yolo_site_packages,
            'yolo_device': yolo_device,
            'run_tomato_collision_scene': 'true',
            'run_gazebo_tomato_detector': 'false',
            'gazebo_tomato_pose_timeout_sec': '2.0',
            'tomato_detection_mode': 'yolo_topic_future',
            'tomato_mapper_prefer_global_frame': 'true',
            'tomato_mapper_model_filter_use_live_gazebo_pose': 'false',
            'tomato_mapper_model_filter_snap_to_model_center': 'true',
            'tomato_mapper_model_filter_max_distance_m': '1.10',
            'tomato_collision_min_confidence': '0.0',
            'tomato_collision_min_updated_count': '1',
            'tomato_collision_class_filter': 'all',
            'tomato_collision_publish_planning_scene': 'true',
            'tomato_collision_publish_markers': 'true',
            'harvest_pick_enabled': 'true',
            'harvest_pick_after_waypoint': 'auto_pick_front',
            'harvest_pick_auto_select': 'true',
            'harvest_pick_allowed_classes': 'fully_ripened,ripe,rotten,disease,diseased',
            'harvest_pick_reject_classes': 'green,unripe',
            'harvest_pick_good_classes': 'fully_ripened,ripe',
            'harvest_pick_bad_classes': 'rotten,disease,diseased,bad',
            'harvest_pick_min_confidence': '0.0',
            'harvest_pick_min_updated_count': '1',
            'harvest_pick_local_radius_m': '1.40',
            'harvest_pick_max_attempts': harvest_pick_max_attempts,
            'harvest_pick_max_per_waypoint': harvest_pick_max_per_waypoint,
            'harvest_pick_max_candidates_per_target': '6',
            'harvest_pick_stop_route_after_attempt': 'false',
            'harvest_pick_place_in_basket': 'true',
            'harvest_pick_timeout_sec': '180.0',
            'harvest_pick_prewarm_gripper_attachments': 'false',
            'harvest_pick_gripper_attach_parent_link': 'fr3_link7',
        }.items(),
    )

    gazebo_gui_client = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=['gz', 'sim', '-g', '--force-version', '8'],
                output='screen',
                condition=IfCondition(run_gazebo_gui_client),
            )
        ],
    )

    yolo_bbox_viewer = Node(
        package='combined_robot',
        executable='yolo_bbox_viewer',
        name='yolo_bbox_viewer',
        output='screen',
        condition=IfCondition(run_yolo_bbox_viewer),
        parameters=[
            {'image_topic': '/camera/color/image_raw'},
            {'detection_topic': '/yolo/tomato_detections_json'},
            {'min_confidence': 0.0},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_file',
            default_value='tomato_farm_sera.sdf',
            description='Greenhouse SDF world file for the stationary harvest demo.',
        ),
        DeclareLaunchArgument(
            'mission_autostart',
            default_value='false',
            description='Start the survey-to-pick-front harvest demo automatically after launch.',
        ),
        DeclareLaunchArgument(
            'route_name',
            default_value='full_survey_then_pick_front_only',
            description='Demo route to run; default surveys the greenhouse, then uses only B/C pick-front harvest poses.',
        ),
        DeclareLaunchArgument(
            'mission_mode',
            default_value='survey_harvest',
            description='Mission behavior mode: stationary_harvest for no driving, survey_harvest to navigate between plant waypoints.',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=os.path.join(
                combined_share, 'config', 'sera_nav2_params.yaml'
            ),
            description='Nav2 parameter file used by the included Sera mission launch.',
        ),
        DeclareLaunchArgument(
            'initial_pose_file',
            default_value=os.path.join(
                combined_share, 'config', 'sera_initial_pose.yaml'
            ),
            description='Initial AMCL pose file published after launch.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_max_attempts',
            default_value='0',
            description='Maximum pick attempts during the demo route. Use 0 for unlimited.',
        ),
        DeclareLaunchArgument(
            'harvest_pick_max_per_waypoint',
            default_value='0',
            description='Maximum pick attempts at one pick-front waypoint. Use 0 to keep harvesting reachable tomatoes at that plant.',
        ),
        DeclareLaunchArgument(
            'run_rviz',
            default_value='true',
            description='Start RViz with MoveIt and tomato collision visualization.',
        ),
        DeclareLaunchArgument(
            'run_gazebo_gui_client',
            default_value='true',
            description='Open Gazebo GUI as a separate client while the server runs headless.',
        ),
        DeclareLaunchArgument(
            'run_yolo_bbox_viewer',
            default_value='true',
            description='Open the live YOLO bounding-box viewer.',
        ),
        DeclareLaunchArgument(
            'run_tomato_map_panel',
            default_value='false',
            description='Open the tomato map table panel.',
        ),
        DeclareLaunchArgument(
            'yolo_model_path',
            default_value=os.path.expanduser('~/robot_workspaces/combined_ws/yolo_models/tomato/best.pt'),
            description='Path to the trained YOLO tomato model.',
        ),
        DeclareLaunchArgument(
            'yolo_site_packages',
            default_value=os.path.expanduser('~/yolo_env/lib/python3.12/site-packages'),
            description='Python site-packages path containing ultralytics and torch.',
        ),
        DeclareLaunchArgument(
            'yolo_device',
            default_value='cpu',
            description='YOLO inference device for the demo launch.',
        ),
        DeclareLaunchArgument(
            'robot_spawn_x',
            default_value='35.82',
            description='Gazebo X coordinate for the spawned robot.',
        ),
        DeclareLaunchArgument(
            'robot_spawn_y',
            default_value='5.80',
            description='Gazebo Y coordinate for the spawned robot.',
        ),
        DeclareLaunchArgument(
            'robot_spawn_z',
            default_value='0.25',
            description='Gazebo Z coordinate for the spawned robot.',
        ),
        DeclareLaunchArgument(
            'robot_spawn_yaw',
            default_value='1.5858',
            description='Gazebo yaw angle in radians for the spawned robot.',
        ),
        DeclareLaunchArgument(
            'enable_3d_lidar',
            default_value='false',
            description='Spawn temporary 3D GPU LiDAR and bridge /lidar/points.',
        ),
        mission,
        gazebo_gui_client,
        yolo_bbox_viewer,
    ])
