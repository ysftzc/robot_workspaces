import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    combined_share = get_package_share_directory('combined_robot')

    launch_sim = LaunchConfiguration('launch_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world_file')
    gz_gui = LaunchConfiguration('gz_gui')
    run_rviz = LaunchConfiguration('run_rviz')
    run_moveit = LaunchConfiguration('run_moveit')
    run_planning_scene = LaunchConfiguration('run_planning_scene')
    bridge_depth_points = LaunchConfiguration('bridge_depth_points')
    enable_3d_lidar = LaunchConfiguration('enable_3d_lidar')
    robot_spawn_x = LaunchConfiguration('robot_spawn_x')
    robot_spawn_y = LaunchConfiguration('robot_spawn_y')
    robot_spawn_z = LaunchConfiguration('robot_spawn_z')
    robot_spawn_yaw = LaunchConfiguration('robot_spawn_yaw')
    map_file = LaunchConfiguration('map')
    keepout_mask_file = LaunchConfiguration('keepout_mask')
    params_file = LaunchConfiguration('params_file')
    nav2_start_delay_sec = LaunchConfiguration('nav2_start_delay_sec')
    publish_initial_pose = LaunchConfiguration('publish_initial_pose')
    initial_pose_file = LaunchConfiguration('initial_pose_file')
    initial_pose_publish_delay_sec = LaunchConfiguration('initial_pose_publish_delay_sec')
    initial_pose_publish_count = LaunchConfiguration('initial_pose_publish_count')
    nav2_lifecycle_start_delay_sec = LaunchConfiguration('nav2_lifecycle_start_delay_sec')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('combined_robot'),
                'launch',
                'combined_gazebo_sera.launch.py',
            ])
        ),
        condition=IfCondition(launch_sim),
        launch_arguments={
            'world_file': world_file,
            'gz_gui': gz_gui,
            'run_rviz': run_rviz,
            'run_moveit': run_moveit,
            'run_planning_scene': run_planning_scene,
            'bridge_depth_points': bridge_depth_points,
            'enable_3d_lidar': enable_3d_lidar,
            'robot_spawn_x': robot_spawn_x,
            'robot_spawn_y': robot_spawn_y,
            'robot_spawn_z': robot_spawn_z,
            'robot_spawn_yaw': robot_spawn_yaw,
        }.items(),
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'bringup_launch.py',
            ])
        ),
        launch_arguments={
            'slam': 'False',
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'False',
            'use_composition': 'False',
        }.items(),
    )

    delayed_nav2_bringup = TimerAction(
        period=nav2_start_delay_sec,
        actions=[nav2_bringup],
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

    keepout_mask_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': keepout_mask_file},
            {'topic_name': '/filter_mask'},
            {'frame_id': 'map'},
        ],
    )

    costmap_filter_info_server = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'type': 0},
            {'filter_info_topic': '/costmap_filter_info'},
            {'mask_topic': '/filter_mask'},
            {'base': 0.0},
            {'multiplier': 1.0},
        ],
    )

    keepout_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_keepout',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['filter_mask_server', 'costmap_filter_info_server']},
        ],
    )

    localization_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization_combined',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server', 'amcl']},
        ],
    )

    navigation_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation_combined',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {
                'node_names': [
                    'controller_server',
                    'smoother_server',
                    'planner_server',
                    'behavior_server',
                    'velocity_smoother',
                    'collision_monitor',
                    'bt_navigator',
                    'waypoint_follower',
                    'docking_server',
                ]
            },
        ],
    )

    delayed_nav2_lifecycle_managers = TimerAction(
        period=nav2_lifecycle_start_delay_sec,
        actions=[
            localization_lifecycle_manager,
            navigation_lifecycle_manager,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_sim',
            default_value='true',
            description='Start the greenhouse Gazebo simulation together with Nav2.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock.',
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='tomato_farm_sera.sdf',
            description='Greenhouse SDF world file for Gazebo.',
        ),
        DeclareLaunchArgument(
            'gz_gui',
            default_value='true',
            description='Start Gazebo GUI with the greenhouse simulation.',
        ),
        DeclareLaunchArgument(
            'run_rviz',
            default_value='true',
            description='Start RViz from the greenhouse simulation launch.',
        ),
        DeclareLaunchArgument(
            'run_moveit',
            default_value='false',
            description='Start MoveIt move_group from the greenhouse simulation launch.',
        ),
        DeclareLaunchArgument(
            'run_planning_scene',
            default_value='true',
            description='Start greenhouse collision objects in the MoveIt planning scene.',
        ),
        DeclareLaunchArgument(
            'bridge_depth_points',
            default_value='false',
            description='Bridge RGB-D PointCloud2 from Gazebo. False keeps simulation smoother.',
        ),
        DeclareLaunchArgument(
            'enable_3d_lidar',
            default_value='false',
            description='Spawn temporary 3D GPU LiDAR and bridge /lidar/points.',
        ),
        DeclareLaunchArgument(
            'robot_spawn_x',
            default_value='35.82',
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
            'map',
            default_value=os.path.join(combined_share, 'maps', 'my_map.yaml'),
            description='Map yaml file for Nav2 localization.',
        ),
        DeclareLaunchArgument(
            'keepout_mask',
            default_value=os.path.join(combined_share, 'maps', 'keepout_mask.yaml'),
            description='Keepout mask yaml file for greenhouse crop rows.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(combined_share, 'config', 'sera_nav2_params.yaml'),
            description='Nav2 parameters file.',
        ),
        DeclareLaunchArgument(
            'nav2_start_delay_sec',
            default_value='22.0',
            description='Seconds to wait before starting Nav2, so Gazebo controllers and odometry are available.',
        ),
        DeclareLaunchArgument(
            'publish_initial_pose',
            default_value='true',
            description='Publish the saved AMCL initial pose for standalone sera_nav2 bringup.',
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
            'nav2_lifecycle_start_delay_sec',
            default_value='28.0',
            description='Seconds to wait before activating Nav2 nodes after delayed bringup.',
        ),
        sim_launch,
        keepout_mask_server,
        costmap_filter_info_server,
        keepout_lifecycle_manager,
        delayed_nav2_bringup,
        delayed_nav2_lifecycle_managers,
        initial_pose_publisher,
    ])
