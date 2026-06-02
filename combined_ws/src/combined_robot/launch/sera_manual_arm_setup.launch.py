"""Launch tomato greenhouse, FR3 MoveIt, and a Cartesian arm jog GUI."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_file = LaunchConfiguration("world_file")
    gz_gui = LaunchConfiguration("gz_gui")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    spawn_pitch = LaunchConfiguration("spawn_pitch")
    run_rviz = LaunchConfiguration("run_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    run_planning_scene = LaunchConfiguration("run_planning_scene")
    run_arm_cartesian_gui = LaunchConfiguration("run_arm_cartesian_gui")
    linear_step_m = LaunchConfiguration("linear_step_m")
    angular_step_deg = LaunchConfiguration("angular_step_deg")
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
            "run_rviz": run_rviz,
            "run_planning_scene": run_planning_scene,
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

    arm_gui = TimerAction(
        period=38.0,
        actions=[
            Node(
                package="combined_robot",
                executable="arm_cartesian_gui",
                name="arm_cartesian_gui",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "planning_group": "fr3_arm",
                        "base_frame": "fr3_link0",
                        "ee_link": "fr3_hand_tcp",
                        "pipeline_id": "ompl",
                        "planner_id": "RRTConnectkConfigDefault",
                        "linear_step_m": ParameterValue(linear_step_m, value_type=float),
                        "angular_step_deg": ParameterValue(angular_step_deg, value_type=float),
                        "velocity_scaling": ParameterValue(velocity_scaling, value_type=float),
                        "acceleration_scaling": ParameterValue(acceleration_scaling, value_type=float),
                    }
                ],
                condition=IfCondition(run_arm_cartesian_gui),
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file", default_value="tomato_farm_sera.sdf"),
            DeclareLaunchArgument("gz_gui", default_value="true"),
            DeclareLaunchArgument("spawn_x", default_value="37.62"),
            DeclareLaunchArgument("spawn_y", default_value="8.50"),
            DeclareLaunchArgument("spawn_z", default_value="0.20"),
            DeclareLaunchArgument("spawn_yaw", default_value="1.56"),
            DeclareLaunchArgument("spawn_pitch", default_value="0.09"),
            DeclareLaunchArgument("run_rviz", default_value="true"),
            DeclareLaunchArgument("run_planning_scene", default_value="false"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("combined_robot"), "rviz", "sera_moveit.rviz"]
                ),
            ),
            DeclareLaunchArgument("run_arm_cartesian_gui", default_value="false"),
            DeclareLaunchArgument("linear_step_m", default_value="0.02"),
            DeclareLaunchArgument("angular_step_deg", default_value="5.0"),
            DeclareLaunchArgument("velocity_scaling", default_value="0.10"),
            DeclareLaunchArgument("acceleration_scaling", default_value="0.10"),
            greenhouse_launch,
            arm_gui,
        ]
    )
