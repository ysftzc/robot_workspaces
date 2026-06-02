"""Launch the tomato greenhouse with MoveIt RViz interactive markers."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_file = LaunchConfiguration("world_file")
    gz_gui = LaunchConfiguration("gz_gui")
    rviz_config = LaunchConfiguration("rviz_config")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    spawn_pitch = LaunchConfiguration("spawn_pitch")
    velocity_scaling = LaunchConfiguration("velocity_scaling")
    acceleration_scaling = LaunchConfiguration("acceleration_scaling")

    greenhouse_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("combined_robot"),
                    "launch",
                    "greenhouse_nearest_pick_place.launch.py",
                ]
            )
        ),
        launch_arguments={
            "world_file": world_file,
            "run_demo": "false",
            "run_depth_detector": "false",
            "run_rviz": "true",
            "rviz_config": rviz_config,
            "gz_gui": gz_gui,
            "bridge_point_cloud": "false",
            "spawn_baskets": "false",
            "spawn_arm_controllers": "true",
            "spawn_drive_controller": "false",
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_z": spawn_z,
            "spawn_yaw": spawn_yaw,
            "spawn_pitch": spawn_pitch,
            "velocity_scaling": velocity_scaling,
            "acceleration_scaling": acceleration_scaling,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file", default_value="tomato_farm_sera.sdf"),
            DeclareLaunchArgument("gz_gui", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("combined_robot"), "rviz", "sera_moveit.rviz"]
                ),
            ),
            DeclareLaunchArgument("spawn_x", default_value="37.62"),
            DeclareLaunchArgument("spawn_y", default_value="8.50"),
            DeclareLaunchArgument("spawn_z", default_value="0.20"),
            DeclareLaunchArgument("spawn_yaw", default_value="1.56"),
            DeclareLaunchArgument("spawn_pitch", default_value="0.09"),
            DeclareLaunchArgument("velocity_scaling", default_value="0.10"),
            DeclareLaunchArgument("acceleration_scaling", default_value="0.10"),
            greenhouse_launch,
        ]
    )
