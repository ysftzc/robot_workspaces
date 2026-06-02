import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Resolve package share directories
    husarion_desc_share  = get_package_share_directory('husarion_ugv_description')
    franka_desc_share    = get_package_share_directory('franka_description')
    combined_share_path  = get_package_share_directory('combined_robot')

    # GZ_SIM_RESOURCE_PATH for meshes
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.dirname(husarion_desc_share),
            os.path.join(os.path.dirname(os.path.dirname(husarion_desc_share)), 'husarion_ugv_gazebo', 'share'),
            os.path.dirname(franka_desc_share),
            os.path.join(combined_share_path, 'models'),
            os.path.dirname(combined_share_path),
            '/opt/ros/jazzy/share',
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ])
    )

    combined_share  = FindPackageShare('combined_robot')
    husarion_desc   = FindPackageShare('husarion_ugv_description')
    husarion_gz     = FindPackageShare('husarion_ugv_gazebo')

    # URDF
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
        ' components_config_path:=',
        PathJoinSubstitution([husarion_desc, 'config', 'components.yaml']),
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Gazebo simulation (Greenhouse World)
    world_file = PathJoinSubstitution([combined_share, 'worlds', 'greenhouse_world.sdf'])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': [world_file, ' -r']}.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': True}],
        output='screen'
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'combined_robot', '-topic', 'robot_description', '-x', '0', '-y', '0', '-z', '0.25'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Bridges
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    # Bridge /tf to /tf for Gazebo odometry
    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Controller spawners
    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['drive_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fr3_arm_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fr3_gripper_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    imu_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['imu_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # EKF Localization (Robot Localization) to fix skid-steer rotation
    ekf_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter',
        parameters=[
            PathJoinSubstitution([FindPackageShare('husarion_ugv_localization'), 'config', 'relative_localization.yaml']),
            {'use_sim_time': True}
        ],
        remappings=[
            ('odometry/wheels', '/drive_controller/odom'),
            ('imu/data', '/imu_broadcaster/imu')
        ],
        output='screen'
    )

    delayed_jsb   = TimerAction(period=4.0, actions=[jsb_spawner])
    delayed_imu   = TimerAction(period=5.0, actions=[imu_spawner])
    delayed_drive = TimerAction(period=5.5, actions=[drive_spawner])
    delayed_arm   = TimerAction(period=7.0, actions=[arm_spawner])
    delayed_grip  = TimerAction(period=9.0, actions=[gripper_spawner])
    delayed_ekf   = TimerAction(period=6.0, actions=[ekf_localization])

    return LaunchDescription([
        gz_resource_path,
        gazebo,
        clock_bridge,
        lidar_bridge,
        tf_bridge,
        robot_state_publisher,
        spawn,
        delayed_jsb,
        delayed_imu,
        delayed_drive,
        delayed_arm,
        delayed_grip,
        delayed_ekf
    ])
