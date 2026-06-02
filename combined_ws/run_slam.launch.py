import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # 1. EKF Node: Provides the missing /tf (odom -> base_link) by fusing wheels and IMU
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter',
            parameters=[
                os.path.join(get_package_share_directory('husarion_ugv_localization'), 'config', 'relative_localization.yaml'),
                {'use_sim_time': True}
            ],
            remappings=[
                ('odometry/wheels', '/drive_controller/odom'),
                ('imu/data', '/imu/data_raw') # Gazebo Harmonic publishes IMU here
            ],
            output='screen'
        ),
        
        # 2. SLAM Toolbox: Properly launched via its official launch file to handle Lifecycle states,
        #    but we pass our custom params file so it listens to /lidar/scan
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_sync_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'slam_params_file': '/home/yusuf/robot_workspaces/combined_ws/custom_slam_params.yaml'
            }.items()
        )
    ])
