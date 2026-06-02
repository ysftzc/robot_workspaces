"""
combined.launch.py
Minimal launch: starts robot_state_publisher only (no Gazebo, no controllers).
Useful for visualizing the robot in RViz2 or testing URDF structure.
"""

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    combined_share = FindPackageShare('combined_robot')
    husarion_desc  = FindPackageShare('husarion_ugv_description')
    husarion_ctrl  = FindPackageShare('husarion_ugv_controller')
    husarion_gz    = FindPackageShare('husarion_ugv_gazebo')

    xacro_file = PathJoinSubstitution([combined_share, 'urdf', 'panther_with_franka.urdf.xacro'])
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

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(robot_description_content, value_type=str),
            'use_sim_time': True,
        }],
        output='screen'
    )

    return LaunchDescription([robot_state_publisher])
