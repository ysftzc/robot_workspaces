"""
combined_gazebo.launch.py
Launches the Panther + Franka FR3 combined robot in Gazebo (Harmonic).

Changes vs previous version:
  - GZ_SIM_RESOURCE_PATH set so Gazebo finds meshes from all workspaces
    (without this, robot links appear in entity tree but are invisible)
  - imu_broadcaster removed: EStopSystem hw plugin does NOT expose IMU state
    interfaces through ros2_control in sim. IMU topic still works directly
    via Gazebo sensor → /imu/data_raw (no ros2_control broadcaster needed).
  - open_loop_control deprecation replaced with interpolate_from_desired_state
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def generate_launch_description():
    world_file_arg = LaunchConfiguration('world_file')
    gz_gui = LaunchConfiguration('gz_gui')
    run_moveit = LaunchConfiguration('run_moveit')
    run_planning_scene = LaunchConfiguration('run_planning_scene')
    planning_scene_include_plants = LaunchConfiguration('planning_scene_include_plants')
    run_rviz = LaunchConfiguration('run_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    robot_spawn_x = LaunchConfiguration('robot_spawn_x')
    robot_spawn_y = LaunchConfiguration('robot_spawn_y')
    robot_spawn_z = LaunchConfiguration('robot_spawn_z')
    robot_spawn_yaw = LaunchConfiguration('robot_spawn_yaw')
    bridge_depth_points = LaunchConfiguration('bridge_depth_points')
    enable_3d_lidar = LaunchConfiguration('enable_3d_lidar')

    # ── Resolve package share directories (needed for resource path) ──────────
    husarion_desc_share  = get_package_share_directory('husarion_ugv_description')
    franka_desc_share    = get_package_share_directory('franka_description')
    combined_share_path  = get_package_share_directory('combined_robot')
    try:
        gz_plugins_share_path = get_package_share_directory('combined_robot_gz_plugins')
        gz_plugins_lib_path = os.path.join(
            os.path.dirname(os.path.dirname(gz_plugins_share_path)),
            'lib',
        )
    except Exception:
        gz_plugins_lib_path = ''

    # ── GZ_SIM_RESOURCE_PATH ──────────────────────────────────────────────────
    # Tells Gazebo where to find meshes referenced via package:// URIs.
    # Each entry is the *parent of the package's share dir* so that
    # Gazebo can find it as:  <resource_path>/<pkg_name>/meshes/...
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            # Project-local Gazebo models copied into this package for sharing.
            os.path.join(combined_share_path, 'models'),
            # Gazebo Fuel models cache (tomato, flowerpot, soilbed, structure, lamp …)
            os.path.join(os.path.expanduser('~'), '.gz', 'models'),
            # husarion_ws share tree  → .../install/<pkg>/share → parent
            os.path.dirname(husarion_desc_share),   # ...install/husarion_ugv_description/share
            os.path.join(os.path.dirname(os.path.dirname(husarion_desc_share)), 'husarion_ugv_gazebo', 'share'),
            # franka_ws share tree
            os.path.dirname(franka_desc_share),     # ...install/franka_description/share
            # combined_ws share tree
            os.path.dirname(combined_share_path),
            # System ROS 2 share (for realsense2_description)
            '/opt/ros/jazzy/share',
            # Preserve any existing GZ_SIM_RESOURCE_PATH entries
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ])
    )
    gz_system_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=':'.join([
            gz_plugins_lib_path,
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
        ])
    )

    # ── FindPackageShare substitutions (for use in Command) ───────────────────
    combined_share  = FindPackageShare('combined_robot')
    husarion_desc   = FindPackageShare('husarion_ugv_description')
    husarion_gz     = FindPackageShare('husarion_ugv_gazebo')


    # ── URDF via lazy xacro ───────────────────────────────────────────────────
    xacro_file         = PathJoinSubstitution([combined_share, 'urdf', 'panther_with_franka.urdf.xacro'])
    combined_ctrl_file = PathJoinSubstitution([combined_share, 'config', 'combined_controllers.yaml'])

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        xacro_file,
        ' use_sim:=True',
        ' wheel_config_file:=',
        PathJoinSubstitution([husarion_desc, 'config', 'WH01.yaml']),
        ' controller_config_file:=',
        combined_ctrl_file,
        ' battery_config_file:=',
        PathJoinSubstitution([husarion_gz, 'config', 'battery_plugin.yaml']),
        ' namespace:=',
        ' enable_3d_lidar:=',
        enable_3d_lidar,
        ' components_config_path:=',
        PathJoinSubstitution([husarion_desc, 'config', 'components.yaml']),
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # ── MoveIt planning context for RViz interactive markers ─────────────────
    semantic_xacro_file = PathJoinSubstitution(
        [combined_share, 'urdf', 'panther_with_franka.srdf.xacro']
    )
    robot_description_semantic_content = Command([
        FindExecutable(name='xacro'), ' ',
        semantic_xacro_file,
    ])
    robot_description_semantic = {
        'robot_description_semantic': ParameterValue(
            robot_description_semantic_content,
            value_type=str,
        )
    }
    kinematics_config = {
        'robot_description_kinematics': load_yaml(
            'combined_robot',
            'config/kinematics_override.yaml',
        )
    }
    robot_description_planning = (
        load_yaml('franka_fr3_moveit_config', 'config/fr3_joint_limits.yaml') or {}
    )
    robot_description_planning.update(
        load_yaml('combined_robot', 'config/pilz_cartesian_limits.yaml') or {}
    )
    joint_limits_config = {'robot_description_planning': robot_description_planning}

    ompl_config = load_yaml('franka_fr3_moveit_config', 'config/ompl_planning.yaml') or {}
    if 'panda_arm' in ompl_config and 'fr3_arm' not in ompl_config:
        ompl_config['fr3_arm'] = dict(ompl_config['panda_arm'])
    ompl_config.setdefault('planning_plugins', ['ompl_interface/OMPLPlanner'])
    ompl_config.setdefault(
        'request_adapters',
        [
            'default_planning_request_adapters/ResolveConstraintFrames',
            'default_planning_request_adapters/ValidateWorkspaceBounds',
            'default_planning_request_adapters/CheckStartStateBounds',
            'default_planning_request_adapters/CheckStartStateCollision',
        ],
    )
    ompl_config.setdefault(
        'response_adapters',
        [
            'default_planning_response_adapters/AddTimeOptimalParameterization',
            'default_planning_response_adapters/ValidateSolution',
            'default_planning_response_adapters/DisplayMotionPath',
        ],
    )
    ompl_config.setdefault('start_state_max_bounds_error', 0.1)

    pilz_config = load_yaml(
        'moveit_configs_utils',
        'default_configs/pilz_industrial_motion_planner_planning.yaml',
    ) or {
        'planning_plugins': ['pilz_industrial_motion_planner/CommandPlanner'],
        'default_planner_config': 'PTP',
        'request_adapters': [
            'default_planning_request_adapters/ResolveConstraintFrames',
            'default_planning_request_adapters/ValidateWorkspaceBounds',
            'default_planning_request_adapters/CheckStartStateBounds',
            'default_planning_request_adapters/CheckStartStateCollision',
        ],
        'response_adapters': [
            'default_planning_response_adapters/ValidateSolution',
            'default_planning_response_adapters/DisplayMotionPath',
        ],
    }
    planning_pipelines_config = {
        'planning_pipelines': ['ompl', 'pilz_industrial_motion_planner'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_config,
        'pilz_industrial_motion_planner': pilz_config,
    }
    moveit_simple_controllers = load_yaml(
        'franka_fr3_moveit_config',
        'config/fr3_controllers.yaml',
    ) or {
        'controller_names': ['fr3_arm_controller'],
        'fr3_arm_controller': {
            'action_ns': 'follow_joint_trajectory',
            'type': 'FollowJointTrajectory',
            'default': True,
            'joints': [
                'fr3_joint1',
                'fr3_joint2',
                'fr3_joint3',
                'fr3_joint4',
                'fr3_joint5',
                'fr3_joint6',
                'fr3_joint7',
            ],
        },
    }
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_simple_controllers,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    }
    trajectory_execution = {
        'moveit_manage_controllers': False,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.05,
    }
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            joint_limits_config,
            planning_pipelines_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': True},
        ],
        condition=IfCondition(run_moveit),
    )

    planning_scene_node = Node(
        package='combined_robot',
        executable='greenhouse_planning_scene',
        name='greenhouse_planning_scene',
        output='screen',
        condition=IfCondition(run_planning_scene),
        parameters=[
            {'use_sim_time': True},
            {'world_file': world_file_arg},
            {'planning_scene_topic': '/planning_scene'},
            {'planning_frame': 'fr3_link0'},
            {'frame_mode': 'robot_spawn'},
            {'robot_spawn_x': ParameterValue(robot_spawn_x, value_type=float)},
            {'robot_spawn_y': ParameterValue(robot_spawn_y, value_type=float)},
            {'robot_spawn_z': ParameterValue(robot_spawn_z, value_type=float)},
            {'robot_spawn_yaw': ParameterValue(robot_spawn_yaw, value_type=float)},
            {'base_to_planning_frame_z': 0.1715},
            {'publish_rate_hz': 1.0},
            {'gazebo_map_x_from_gazebo_y_offset': -4.93},
            {'gazebo_map_y_from_gazebo_x_origin': 35.83},
            {'include_tomatoes': False},
            {'include_plants': ParameterValue(planning_scene_include_plants, value_type=bool)},
        ],
    )

    rviz_node = TimerAction(
        period=24.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2_moveit',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    kinematics_config,
                    joint_limits_config,
                    planning_pipelines_config,
                    {'use_sim_time': True},
                ],
                condition=IfCondition(run_rviz),
            )
        ],
    )

    # ── Gazebo simulation ─────────────────────────────────────────────────────
    world_file = PathJoinSubstitution([combined_share, 'worlds', world_file_arg])
    gz_args = [
        world_file,
        PythonExpression([
            "' -r' if '",
            gz_gui,
            "'.lower() in ['true', '1', 'yes', 'on'] else ' -r -s'",
        ]),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': gz_args}.items()
    )

    # ── Robot state publisher ─────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen'
    )

    # -- Spawn combined robot --------------------------------------------------
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'combined_robot',
            '-topic', 'robot_description',
            '-x', robot_spawn_x,
            '-y', robot_spawn_y,
            '-z', robot_spawn_z,
            '-Y', robot_spawn_yaw,
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )




    # ── Clock bridge ──────────────────────────────────────────────────────────
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ── Camera bridge (RealSense D435 on FR3 gripper) ─────────────────────────
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    camera_points_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/depth/image/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(bridge_depth_points),
    )

    # ── Lidar bridge (LDR06 on Panther) ──────────────────────────────────────
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        remappings=[('/scan', '/lidar/scan')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    lidar_3d_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/lidar/points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
        remappings=[('/lidar/points/points', '/lidar/points_cloud')],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(enable_3d_lidar),
    )

    # Gazebo publishes the Panther IMU directly on /imu/data_raw. Feed it to
    # robot_localization so odom->base_link comes from fused wheel + yaw-rate data.
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    ekf_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter',
        parameters=[
            PathJoinSubstitution([combined_share, 'config', 'ekf_sim_config.yaml']),
            {'use_sim_time': True},
        ],
        remappings=[
            ('imu/data', '/imu/data_raw'),
        ],
        output='screen'
    )

    # ── Controller spawners ───────────────────────────────────────────────────
    # NOTE: imu_broadcaster is intentionally NOT spawned here.
    #   Reason: EStopSystem hardware plugin does not expose /imu/* state
    #   interfaces through ros2_control in simulation. The IMU is published
    #   directly by the Gazebo IMU sensor to /imu/data_raw via the Gazebo
    #   plugin — no ros2_control broadcaster is needed for that signal.

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager', '--switch-timeout', '20.0'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['drive_controller', '--controller-manager', '/controller_manager', '--switch-timeout', '20.0'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    fr3_arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fr3_arm_controller', '--controller-manager', '/controller_manager', '--switch-timeout', '20.0'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    fr3_gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fr3_gripper_controller', '--controller-manager', '/controller_manager', '--switch-timeout', '20.0'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Staggered delays — give Gazebo time to initialize hardware plugins
    delayed_jsb   = TimerAction(period=12.0, actions=[joint_state_broadcaster_spawner])
    delayed_drive = TimerAction(period=14.0, actions=[drive_controller_spawner])
    delayed_arm   = TimerAction(period=16.0, actions=[fr3_arm_spawner])
    delayed_ekf   = TimerAction(period=16.0, actions=[ekf_localization])
    delayed_lidar = TimerAction(period=17.0, actions=[lidar_bridge])
    delayed_grip  = TimerAction(period=18.0, actions=[fr3_gripper_spawner])

    return LaunchDescription([
        DeclareLaunchArgument(
            'world_file',
            default_value='tomato_farm_sera.sdf',
            description='Greenhouse SDF world file. Defaults to the tomato-populated greenhouse.',
        ),
        DeclareLaunchArgument(
            'gz_gui',
            default_value='true',
            description='Start Gazebo GUI. Set false to run only the Gazebo server.',
        ),
        DeclareLaunchArgument(
            'run_moveit',
            default_value='true',
            description='Start MoveIt move_group so RViz MotionPlanning can plan and execute.',
        ),
        DeclareLaunchArgument(
            'run_planning_scene',
            default_value='true',
            description='Publish greenhouse collision objects into MoveIt planning scene.',
        ),
        DeclareLaunchArgument(
            'planning_scene_include_plants',
            default_value='false',
            description='Include plant/leaf proxy collision objects in MoveIt planning scene.',
        ),
        DeclareLaunchArgument(
            'run_rviz',
            default_value='true',
            description='Start RViz with MoveIt MotionPlanning interactive markers.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([combined_share, 'rviz', 'sera_moveit.rviz']),
            description='RViz config file.',
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
            'bridge_depth_points',
            default_value='false',
            description='Bridge RGB-D PointCloud2. Leave false for smoother Gazebo unless a node needs point clouds.',
        ),
        DeclareLaunchArgument(
            'enable_3d_lidar',
            default_value='false',
            description='Spawn a temporary 3D GPU LiDAR and bridge /lidar/points for point-cloud visualization.',
        ),
        gz_resource_path,
        gz_system_plugin_path,
        gazebo,
        clock_bridge,
        robot_state_publisher,
        move_group_node,
        planning_scene_node,
        rviz_node,
        spawn,
        camera_bridge,
        camera_points_bridge,
        lidar_3d_bridge,
        imu_bridge,
        delayed_jsb,
        delayed_drive,
        delayed_arm,
        delayed_ekf,
        delayed_lidar,
        delayed_grip,
    ])
