import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    combined_share = get_package_share_directory('combined_robot')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('combined_robot'),
                    'launch',
                    'sera_mission.launch.py',
                ])
            ),
            launch_arguments={
                'waypoint_file': os.path.join(combined_share, 'config', 'sera_waypoints.yaml'),
                'route_name': 'full_survey',
                'mission_mode': 'survey_harvest',
                'arm_pose_file': os.path.join(combined_share, 'config', 'fr3_observation_poses.yaml'),
                'arm_pose_enabled': 'true',
                'nav_goal_timeout_sec': '0.0',
                'mission_autostart': 'true',
                'loop_route': 'false',
                'publish_initial_pose': 'true',
                'initial_pose_file': os.path.join(combined_share, 'config', 'sera_initial_pose.yaml'),
                'initial_pose_publish_count': '45',
            }.items(),
        ),
    ])
