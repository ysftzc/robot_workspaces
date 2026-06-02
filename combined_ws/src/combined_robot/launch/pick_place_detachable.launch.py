"""Launch a Gazebo detachable tomato pick-place test with Panther + Franka FR3."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    combined_share_path = get_package_share_directory("combined_robot")
    husarion_desc_share = get_package_share_directory("husarion_ugv_description")
    franka_desc_share = get_package_share_directory("franka_description")

    combined_share = FindPackageShare("combined_robot")
    husarion_desc = FindPackageShare("husarion_ugv_description")
    husarion_gz = FindPackageShare("husarion_ugv_gazebo")

    run_demo = LaunchConfiguration("run_demo")
    basket = LaunchConfiguration("basket")
    world_file_arg = LaunchConfiguration("world_file")
    world_name = LaunchConfiguration("world_name")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    tomato_model = LaunchConfiguration("tomato_model")
    detach_topic = LaunchConfiguration("detach_topic")
    pose_profile = LaunchConfiguration("pose_profile")
    set_pose_world_x = LaunchConfiguration("set_pose_world_x")
    set_pose_world_y = LaunchConfiguration("set_pose_world_y")
    set_pose_world_z = LaunchConfiguration("set_pose_world_z")
    set_pose_world_yaw = LaunchConfiguration("set_pose_world_yaw")
    spawn_baskets = LaunchConfiguration("spawn_baskets")
    good_basket_x = LaunchConfiguration("good_basket_x")
    good_basket_y = LaunchConfiguration("good_basket_y")
    bad_basket_x = LaunchConfiguration("bad_basket_x")
    bad_basket_y = LaunchConfiguration("bad_basket_y")
    basket_z = LaunchConfiguration("basket_z")
    basket_yaw = LaunchConfiguration("basket_yaw")

    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=":".join(
            [
                os.path.join(combined_share_path, "models"),
                os.path.join(os.path.expanduser("~"), ".gz", "models"),
                os.path.dirname(husarion_desc_share),
                os.path.join(
                    os.path.dirname(os.path.dirname(husarion_desc_share)),
                    "husarion_ugv_gazebo",
                    "share",
                ),
                os.path.dirname(franka_desc_share),
                os.path.dirname(combined_share_path),
                "/opt/ros/jazzy/share",
                os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
            ]
        ),
    )

    xacro_file = PathJoinSubstitution(
        [combined_share, "urdf", "panther_with_franka.urdf.xacro"]
    )
    combined_ctrl_file = PathJoinSubstitution(
        [combined_share, "config", "combined_controllers.yaml"]
    )

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            xacro_file,
            " use_sim:=True",
            " wheel_config_file:=",
            PathJoinSubstitution([husarion_desc, "config", "WH01.yaml"]),
            " controller_config_file:=",
            combined_ctrl_file,
            " battery_config_file:=",
            PathJoinSubstitution([husarion_gz, "config", "battery_plugin.yaml"]),
            " namespace:=",
            " components_config_path:=",
            PathJoinSubstitution([husarion_desc, "config", "components.yaml"]),
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    world_file = PathJoinSubstitution([combined_share, "worlds", world_file_arg])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": [world_file, " -r"]}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": True}],
        output="screen",
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "combined_robot",
            "-topic",
            "robot_description",
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            spawn_z,
            "-Y",
            spawn_yaw,
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    good_basket_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "good_pick_basket",
            "-file",
            PathJoinSubstitution([combined_share, "models", "good_pick_basket", "model.sdf"]),
            "-x",
            good_basket_x,
            "-y",
            good_basket_y,
            "-z",
            basket_z,
            "-Y",
            basket_yaw,
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(spawn_baskets),
    )

    bad_basket_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "bad_pick_basket",
            "-file",
            PathJoinSubstitution([combined_share, "models", "bad_pick_basket", "model.sdf"]),
            "-x",
            bad_basket_x,
            "-y",
            bad_basket_y,
            "-z",
            basket_z,
            "-Y",
            basket_yaw,
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(spawn_baskets),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--switch-timeout", "20.0"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["drive_controller", "--controller-manager", "/controller_manager", "--switch-timeout", "20.0"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    fr3_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fr3_arm_controller", "--controller-manager", "/controller_manager", "--switch-timeout", "20.0"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    fr3_gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fr3_gripper_controller", "--controller-manager", "/controller_manager", "--switch-timeout", "20.0"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    demo_runner = TimerAction(
        period=22.0,
        actions=[
            Node(
                package="combined_robot",
                executable="pick_place_detachable",
                arguments=[
                    basket,
                    "--detach-topic",
                    detach_topic,
                    "--world-name",
                    world_name,
                    "--tomato-model",
                    tomato_model,
                    "--profile",
                    pose_profile,
                    "--world-x",
                    set_pose_world_x,
                    "--world-y",
                    set_pose_world_y,
                    "--world-z",
                    set_pose_world_z,
                    "--world-yaw",
                    set_pose_world_yaw,
                ],
                parameters=[{"use_sim_time": True}],
                output="screen",
                condition=IfCondition(run_demo),
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "run_demo",
                default_value="false",
                description="Start the scripted pick-place node after controllers are spawned.",
            ),
            DeclareLaunchArgument(
                "basket",
                default_value="good",
                description="Target basket for the scripted demo: good or bad.",
            ),
            DeclareLaunchArgument(
                "world_file",
                default_value="pick_place_detachable_test.sdf",
                description="World SDF file under combined_robot/worlds.",
            ),
            DeclareLaunchArgument(
                "world_name",
                default_value="pick_place_detachable_test",
                description="Gazebo world name used by set_pose services.",
            ),
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.25"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "tomato_model",
                default_value="tomato_ripe_pick_0",
                description="Gazebo model name of the tomato used by the demo.",
            ),
            DeclareLaunchArgument(
                "detach_topic",
                default_value="/detach/tomato_ripe_pick_0",
                description="Gazebo Empty topic for the target tomato detachable joint.",
            ),
            DeclareLaunchArgument(
                "pose_profile",
                default_value="empty",
                description="Arm pose profile: empty or greenhouse.",
            ),
            DeclareLaunchArgument("set_pose_world_x", default_value="0.0"),
            DeclareLaunchArgument("set_pose_world_y", default_value="0.0"),
            DeclareLaunchArgument("set_pose_world_z", default_value="0.0"),
            DeclareLaunchArgument("set_pose_world_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "spawn_baskets",
                default_value="false",
                description="Spawn standalone good/bad baskets into worlds that do not include them.",
            ),
            DeclareLaunchArgument("good_basket_x", default_value="-0.55"),
            DeclareLaunchArgument("good_basket_y", default_value="0.0"),
            DeclareLaunchArgument("bad_basket_x", default_value="-0.90"),
            DeclareLaunchArgument("bad_basket_y", default_value="0.0"),
            DeclareLaunchArgument("basket_z", default_value="0.0"),
            DeclareLaunchArgument("basket_yaw", default_value="0.0"),
            gz_resource_path,
            gazebo,
            clock_bridge,
            robot_state_publisher,
            spawn,
            TimerAction(period=2.0, actions=[good_basket_spawn, bad_basket_spawn]),
            TimerAction(period=4.0, actions=[joint_state_broadcaster_spawner]),
            TimerAction(period=5.5, actions=[drive_controller_spawner]),
            TimerAction(period=7.0, actions=[fr3_arm_spawner]),
            TimerAction(period=9.0, actions=[fr3_gripper_spawner]),
            demo_runner,
        ]
    )
