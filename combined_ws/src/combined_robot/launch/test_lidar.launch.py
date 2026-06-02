"""
Minimal test: Panther + Franka in empty world.
Only spawns robot + lidar bridge + clock bridge.
Usage: ros2 launch combined_robot test_lidar.launch.py
"""

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

    husarion_desc_share = get_package_share_directory('husarion_ugv_description')
    franka_desc_share   = get_package_share_directory('franka_description')
    combined_share_path = get_package_share_directory('combined_robot')

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.dirname(husarion_desc_share),
            os.path.join(os.path.dirname(os.path.dirname(husarion_desc_share)), 'husarion_ugv_gazebo', 'share'),
            os.path.dirname(franka_desc_share),
            os.path.join(combined_share_path, 'models'),
            os.path.dirname(combined_share_path),
            '/opt/ros/jazzy/share',
            os.path.expanduser('~/.gz/models'),
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ])
    )

    combined_share = FindPackageShare('combined_robot')
    husarion_desc  = FindPackageShare('husarion_ugv_description')
    husarion_gz    = FindPackageShare('husarion_ugv_gazebo')

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

    # Empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
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
        arguments=[
            '-name', 'combined_robot',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '0.0', '-z', '0.5', '-Y', '0.0'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}], output='screen'
    )

    lidar_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='lidar_bridge',
        arguments=['/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        parameters=[{'use_sim_time': True}], output='screen'
    )

    # Controller spawners
    jsb = Node(package='controller_manager', executable='spawner',
               arguments=['joint_state_broadcaster'], parameters=[{'use_sim_time': True}])
    drive = Node(package='controller_manager', executable='spawner',
                 arguments=['drive_controller'], parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        gz_resource_path,
        gazebo,
        robot_state_publisher,
        TimerAction(period=5.0, actions=[spawn]),
        TimerAction(period=8.0, actions=[clock_bridge, lidar_bridge]),
        TimerAction(period=10.0, actions=[jsb]),
        TimerAction(period=12.0, actions=[drive]),
    ])
