"""Launch big-greenhouse nearest-tomato pick-place with Panther + Franka FR3."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except OSError:
        return None


def generate_launch_description():
    combined_share_path = get_package_share_directory("combined_robot")
    husarion_desc_share = get_package_share_directory("husarion_ugv_description")
    franka_desc_share = get_package_share_directory("franka_description")

    combined_share = FindPackageShare("combined_robot")
    husarion_desc = FindPackageShare("husarion_ugv_description")
    husarion_gz = FindPackageShare("husarion_ugv_gazebo")

    run_demo = LaunchConfiguration("run_demo")
    run_depth_detector = LaunchConfiguration("run_depth_detector")
    run_rviz = LaunchConfiguration("run_rviz")
    run_planning_scene = LaunchConfiguration("run_planning_scene")
    gz_gui = LaunchConfiguration("gz_gui")
    bridge_point_cloud = LaunchConfiguration("bridge_point_cloud")
    rviz_config = LaunchConfiguration("rviz_config")
    world_file_arg = LaunchConfiguration("world_file")
    world_name = LaunchConfiguration("world_name")
    pose_profile = LaunchConfiguration("pose_profile")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    spawn_pitch = LaunchConfiguration("spawn_pitch")
    fr3_initial_joint1 = LaunchConfiguration("fr3_initial_joint1")
    fr3_initial_joint2 = LaunchConfiguration("fr3_initial_joint2")
    fr3_initial_joint3 = LaunchConfiguration("fr3_initial_joint3")
    fr3_initial_joint4 = LaunchConfiguration("fr3_initial_joint4")
    fr3_initial_joint5 = LaunchConfiguration("fr3_initial_joint5")
    fr3_initial_joint6 = LaunchConfiguration("fr3_initial_joint6")
    fr3_initial_joint7 = LaunchConfiguration("fr3_initial_joint7")
    basket = LaunchConfiguration("basket")
    target_prefix = LaunchConfiguration("target_prefix")
    target_side = LaunchConfiguration("target_side")
    min_forward = LaunchConfiguration("min_forward")
    max_forward = LaunchConfiguration("max_forward")
    max_lateral = LaunchConfiguration("max_lateral")
    max_picks = LaunchConfiguration("max_picks")
    min_z = LaunchConfiguration("min_z")
    max_z = LaunchConfiguration("max_z")
    planning_group = LaunchConfiguration("planning_group")
    base_frame = LaunchConfiguration("base_frame")
    ee_link = LaunchConfiguration("ee_link")
    approach_distance = LaunchConfiguration("approach_distance")
    pick_distance = LaunchConfiguration("pick_distance")
    grasp_roll = LaunchConfiguration("grasp_roll")
    grasp_lateral_offset = LaunchConfiguration("grasp_lateral_offset")
    gripper_close_width = LaunchConfiguration("gripper_close_width")
    tomato_topic = LaunchConfiguration("tomato_topic")
    tomato_radius_topic = LaunchConfiguration("tomato_radius_topic")
    auto_detach_max_distance = LaunchConfiguration("auto_detach_max_distance")
    min_pick_distance = LaunchConfiguration("min_pick_distance")
    max_pick_distance = LaunchConfiguration("max_pick_distance")
    max_target_distance = LaunchConfiguration("max_target_distance")
    pre_detect_joints = LaunchConfiguration("pre_detect_joints")
    pre_detect_duration = LaunchConfiguration("pre_detect_duration")
    observation_settle = LaunchConfiguration("observation_settle")
    stable_target_samples = LaunchConfiguration("stable_target_samples")
    target_stability_tolerance = LaunchConfiguration("target_stability_tolerance")
    max_approach_base_distance = LaunchConfiguration("max_approach_base_distance")
    max_pick_base_distance = LaunchConfiguration("max_pick_base_distance")
    color_topic = LaunchConfiguration("color_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    tomato_detector_frame = LaunchConfiguration("tomato_detector_frame")
    detector_target_strategy = LaunchConfiguration("detector_target_strategy")
    detector_center_preference_weight = LaunchConfiguration("detector_center_preference_weight")
    detector_center_method = LaunchConfiguration("detector_center_method")
    detector_roi_width_fraction = LaunchConfiguration("detector_roi_width_fraction")
    detector_roi_height_fraction = LaunchConfiguration("detector_roi_height_fraction")
    detector_roi_center_u_fraction = LaunchConfiguration("detector_roi_center_u_fraction")
    detector_roi_center_v_fraction = LaunchConfiguration("detector_roi_center_v_fraction")
    detector_max_abs_x = LaunchConfiguration("detector_max_abs_x")
    detector_max_abs_y = LaunchConfiguration("detector_max_abs_y")
    velocity_scaling = LaunchConfiguration("velocity_scaling")
    acceleration_scaling = LaunchConfiguration("acceleration_scaling")
    grasp_local_x = LaunchConfiguration("grasp_local_x")
    grasp_local_y = LaunchConfiguration("grasp_local_y")
    grasp_local_z = LaunchConfiguration("grasp_local_z")
    grasp_tolerance = LaunchConfiguration("grasp_tolerance")
    moveit_group = LaunchConfiguration("moveit_group")
    ik_frame = LaunchConfiguration("ik_frame")
    ik_link = LaunchConfiguration("ik_link")
    approach_offset = LaunchConfiguration("approach_offset")
    grasp_offset = LaunchConfiguration("grasp_offset")
    grasp_surface_clearance = LaunchConfiguration("grasp_surface_clearance")
    tcp_front_offset = LaunchConfiguration("tcp_front_offset")
    grip_clearance = LaunchConfiguration("grip_clearance")
    pre_grasp_z_offset = LaunchConfiguration("pre_grasp_z_offset")
    lift_offset = LaunchConfiguration("lift_offset")
    spawn_baskets = LaunchConfiguration("spawn_baskets")
    spawn_arm_controllers = LaunchConfiguration("spawn_arm_controllers")
    spawn_drive_controller = LaunchConfiguration("spawn_drive_controller")
    good_basket_x = LaunchConfiguration("good_basket_x")
    good_basket_y = LaunchConfiguration("good_basket_y")
    bad_basket_x = LaunchConfiguration("bad_basket_x")
    bad_basket_y = LaunchConfiguration("bad_basket_y")
    basket_z = LaunchConfiguration("basket_z")
    basket_yaw = LaunchConfiguration("basket_yaw")
    planning_scene_include_plants = LaunchConfiguration("planning_scene_include_plants")
    planning_scene_plant_radius = LaunchConfiguration("planning_scene_plant_radius")
    planning_scene_plant_height = LaunchConfiguration("planning_scene_plant_height")

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
    gl_threaded_optimizations = SetEnvironmentVariable(
        name="__GL_THREADED_OPTIMIZATIONS",
        value="1",
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
            " fr3_initial_joint1:=",
            fr3_initial_joint1,
            " fr3_initial_joint2:=",
            fr3_initial_joint2,
            " fr3_initial_joint3:=",
            fr3_initial_joint3,
            " fr3_initial_joint4:=",
            fr3_initial_joint4,
            " fr3_initial_joint5:=",
            fr3_initial_joint5,
            " fr3_initial_joint6:=",
            fr3_initial_joint6,
            " fr3_initial_joint7:=",
            fr3_initial_joint7,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    semantic_xacro_file = PathJoinSubstitution(
        [combined_share, "urdf", "panther_with_franka.srdf.xacro"]
    )
    robot_description_semantic_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            semantic_xacro_file,
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            robot_description_semantic_content,
            value_type=str,
        )
    }
    kinematics_config = {
        "robot_description_kinematics": load_yaml(
            "combined_robot",
            "config/kinematics_override.yaml",
        )
    }
    robot_description_planning = (
        load_yaml("franka_fr3_moveit_config", "config/fr3_joint_limits.yaml") or {}
    )
    robot_description_planning.update(
        load_yaml("combined_robot", "config/pilz_cartesian_limits.yaml") or {}
    )
    joint_limits_config = {"robot_description_planning": robot_description_planning}

    ompl_config = load_yaml("franka_fr3_moveit_config", "config/ompl_planning.yaml") or {}
    if "panda_arm" in ompl_config and "fr3_arm" not in ompl_config:
        ompl_config["fr3_arm"] = dict(ompl_config["panda_arm"])
    ompl_config.setdefault("planning_plugins", ["ompl_interface/OMPLPlanner"])
    ompl_config.setdefault(
        "request_adapters",
        [
            "default_planning_request_adapters/ResolveConstraintFrames",
            "default_planning_request_adapters/ValidateWorkspaceBounds",
            "default_planning_request_adapters/CheckStartStateBounds",
            "default_planning_request_adapters/CheckStartStateCollision",
        ],
    )
    ompl_config.setdefault(
        "response_adapters",
        [
            "default_planning_response_adapters/AddTimeOptimalParameterization",
            "default_planning_response_adapters/ValidateSolution",
            "default_planning_response_adapters/DisplayMotionPath",
        ],
    )
    ompl_config.setdefault("start_state_max_bounds_error", 0.1)

    pilz_config = load_yaml(
        "moveit_configs_utils",
        "default_configs/pilz_industrial_motion_planner_planning.yaml",
    ) or {
        "planning_plugins": ["pilz_industrial_motion_planner/CommandPlanner"],
        "default_planner_config": "PTP",
        "request_adapters": [
            "default_planning_request_adapters/ResolveConstraintFrames",
            "default_planning_request_adapters/ValidateWorkspaceBounds",
            "default_planning_request_adapters/CheckStartStateBounds",
            "default_planning_request_adapters/CheckStartStateCollision",
        ],
        "response_adapters": [
            "default_planning_response_adapters/ValidateSolution",
            "default_planning_response_adapters/DisplayMotionPath",
        ],
    }
    planning_pipelines_config = {
        "planning_pipelines": ["ompl", "pilz_industrial_motion_planner"],
        "default_planning_pipeline": "ompl",
        "ompl": ompl_config,
        "pilz_industrial_motion_planner": pilz_config,
    }
    moveit_simple_controllers = load_yaml(
        "franka_fr3_moveit_config",
        "config/fr3_controllers.yaml",
    ) or {
        "controller_names": ["fr3_arm_controller"],
        "fr3_arm_controller": {
            "action_ns": "follow_joint_trajectory",
            "type": "FollowJointTrajectory",
            "default": True,
            "joints": [
                "fr3_joint1",
                "fr3_joint2",
                "fr3_joint3",
                "fr3_joint4",
                "fr3_joint5",
                "fr3_joint6",
                "fr3_joint7",
            ],
        },
    }
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.05,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_config,
            joint_limits_config,
            planning_pipelines_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": True},
        ],
    )

    greenhouse_planning_scene = TimerAction(
        period=34.0,
        actions=[
            Node(
                package="combined_robot",
                executable="greenhouse_planning_scene",
                name="greenhouse_planning_scene",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "world_file": world_file_arg,
                        "planning_frame": base_frame,
                        "frame_mode": "robot_spawn",
                        "robot_spawn_x": ParameterValue(spawn_x, value_type=float),
                        "robot_spawn_y": ParameterValue(spawn_y, value_type=float),
                        "robot_spawn_z": ParameterValue(spawn_z, value_type=float),
                        "robot_spawn_pitch": ParameterValue(spawn_pitch, value_type=float),
                        "robot_spawn_yaw": ParameterValue(spawn_yaw, value_type=float),
                        "base_to_planning_frame_z": 0.1715,
                        "include_tomatoes": False,
                        "include_soilbeds": True,
                        "include_pots": True,
                        "include_plants": ParameterValue(
                            planning_scene_include_plants, value_type=bool
                        ),
                        "include_structures": True,
                        "include_lamps": False,
                        "include_baskets": False,
                        "plant_collision_radius_m": ParameterValue(
                            planning_scene_plant_radius, value_type=float
                        ),
                        "plant_collision_height_m": ParameterValue(
                            planning_scene_plant_height, value_type=float
                        ),
                    }
                ],
                condition=IfCondition(run_planning_scene),
            )
        ],
    )

    rviz_node = TimerAction(
        period=36.0,
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_moveit",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    kinematics_config,
                    joint_limits_config,
                    planning_pipelines_config,
                    {"use_sim_time": True},
                ],
                condition=IfCondition(run_rviz),
            )
        ],
    )

    world_file = PathJoinSubstitution([combined_share, "worlds", world_file_arg])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": [world_file, " -r -s"]}.items(),
    )

    gazebo_gui = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=["gz", "sim", "-g"],
                output="screen",
                condition=IfCondition(gz_gui),
            )
        ],
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
            "-P",
            spawn_pitch,
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

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/depth/image@sensor_msgs/msg/Image[gz.msgs.Image",
            "/camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    point_cloud_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/camera/depth/image/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(bridge_point_cloud),
    )

    tomato_depth_detector = Node(
        package="combined_robot",
        executable="tomato_depth_detector",
        name="tomato_depth_detector",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "color_topic": color_topic,
                "depth_topic": depth_topic,
                "camera_info_topic": camera_info_topic,
                "output_pose_topic": tomato_topic,
                "output_radius_topic": tomato_radius_topic,
                "output_frame_id": tomato_detector_frame,
                "target_strategy": detector_target_strategy,
                "min_component_area": 80,
                "max_depth": 4.0,
                "min_center_depth": 0.12,
                "max_center_depth": 0.90,
                "max_abs_x": detector_max_abs_x,
                "max_abs_y": detector_max_abs_y,
                "center_preference_weight": detector_center_preference_weight,
                "center_method": detector_center_method,
                "roi_width_fraction": detector_roi_width_fraction,
                "roi_height_fraction": detector_roi_height_fraction,
                "roi_center_u_fraction": detector_roi_center_u_fraction,
                "roi_center_v_fraction": detector_roi_center_v_fraction,
                "min_radius": 0.025,
                "max_radius": 0.055,
                "mirror_depth_u": True,
                "mirror_depth_projection": False,
                "surface_depth_percentile": 20.0,
                "publish_rate": 5.0,
            }
        ],
        condition=IfCondition(run_depth_detector),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--switch-timeout", "20.0"],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(spawn_arm_controllers),
    )
    drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["drive_controller", "--controller-manager", "/controller_manager", "--switch-timeout", "20.0"],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(spawn_drive_controller),
    )
    fr3_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fr3_arm_controller", "--controller-manager", "/controller_manager", "--switch-timeout", "20.0"],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(spawn_arm_controllers),
    )
    fr3_gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["fr3_gripper_controller", "--controller-manager", "/controller_manager", "--switch-timeout", "20.0"],
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(spawn_arm_controllers),
    )

    demo_runner = TimerAction(
        period=48.0,
        actions=[
            Node(
                package="combined_robot",
                executable="greenhouse_nearest_pick_place",
                arguments=[
                    "--world-file",
                    world_file_arg,
                    "--world-name",
                    world_name,
                    "--basket",
                    basket,
                    "--profile",
                    pose_profile,
                    "--robot-x",
                    spawn_x,
                    "--robot-y",
                    spawn_y,
                    "--robot-z",
                    spawn_z,
                    "--robot-yaw",
                    spawn_yaw,
                    "--robot-pitch",
                    spawn_pitch,
                    "--target-prefix",
                    target_prefix,
                    "--target-side",
                    target_side,
                    "--min-forward",
                    min_forward,
                    "--max-forward",
                    max_forward,
                    "--max-lateral",
                    max_lateral,
                    "--max-picks",
                    max_picks,
                    "--min-z",
                    min_z,
                    "--max-z",
                    max_z,
                    "--grasp-local-x",
                    grasp_local_x,
                    "--grasp-local-y",
                    grasp_local_y,
                    "--grasp-local-z",
                    grasp_local_z,
                    "--grasp-tolerance",
                    grasp_tolerance,
                    "--planning-group",
                    planning_group,
                    "--base-frame",
                    base_frame,
                    "--ee-link",
                    ee_link,
                    "--approach-distance",
                    approach_distance,
                    "--pick-distance",
                    pick_distance,
                    "--grasp-roll",
                    grasp_roll,
                    "--grasp-lateral-offset",
                    grasp_lateral_offset,
                    "--gripper-close-width",
                    gripper_close_width,
                    "--tomato-topic",
                    tomato_topic,
                    "--tomato-radius-topic",
                    tomato_radius_topic,
                    "--auto-detach-max-distance",
                    auto_detach_max_distance,
                    "--use-radius-topic",
                    "--require-radius",
                    "--min-pick-distance",
                    min_pick_distance,
                    "--max-pick-distance",
                    max_pick_distance,
                    "--max-target-distance",
                    max_target_distance,
                    "--pre-detect-joints",
                    pre_detect_joints,
                    "--pre-detect-duration",
                    pre_detect_duration,
                    "--observation-settle",
                    observation_settle,
                    "--stable-target-samples",
                    stable_target_samples,
                    "--target-stability-tolerance",
                    target_stability_tolerance,
                    "--max-approach-base-distance",
                    max_approach_base_distance,
                    "--max-pick-base-distance",
                    max_pick_base_distance,
                    "--velocity-scaling",
                    velocity_scaling,
                    "--acceleration-scaling",
                    acceleration_scaling,
                    "--grasp-offset",
                    grasp_offset,
                    "--grasp-surface-clearance",
                    grasp_surface_clearance,
                    "--tcp-front-offset",
                    tcp_front_offset,
                    "--grip-clearance",
                    grip_clearance,
                    "--pre-grasp-z-offset",
                    pre_grasp_z_offset,
                    "--lift-offset",
                    lift_offset,
                    "--pose-stabilization",
                ],
                parameters=[{"use_sim_time": True}],
                output="screen",
                condition=IfCondition(run_demo),
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("run_demo", default_value="true"),
            DeclareLaunchArgument("run_depth_detector", default_value="true"),
            DeclareLaunchArgument("run_rviz", default_value="false"),
            DeclareLaunchArgument("run_planning_scene", default_value="true"),
            DeclareLaunchArgument("gz_gui", default_value="true"),
            DeclareLaunchArgument("bridge_point_cloud", default_value="false"),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution([combined_share, "rviz", "sera_moveit.rviz"]),
            ),
            DeclareLaunchArgument("world_file", default_value="tomato_farm_sera.sdf"),
            DeclareLaunchArgument("world_name", default_value="tomato_farm"),
            DeclareLaunchArgument("pose_profile", default_value="empty"),
            DeclareLaunchArgument("spawn_x", default_value="37.62"),
            DeclareLaunchArgument("spawn_y", default_value="8.50"),
            DeclareLaunchArgument("spawn_z", default_value="0.20"),
            DeclareLaunchArgument("spawn_yaw", default_value="1.56"),
            DeclareLaunchArgument("spawn_pitch", default_value="0.09"),
            DeclareLaunchArgument("fr3_initial_joint1", default_value="-1.710"),
            DeclareLaunchArgument("fr3_initial_joint2", default_value="-1.267"),
            DeclareLaunchArgument("fr3_initial_joint3", default_value="0.0"),
            DeclareLaunchArgument("fr3_initial_joint4", default_value="-2.792"),
            DeclareLaunchArgument("fr3_initial_joint5", default_value="0.0"),
            DeclareLaunchArgument("fr3_initial_joint6", default_value="2.800"),
            DeclareLaunchArgument("fr3_initial_joint7", default_value="0.0"),
            DeclareLaunchArgument("basket", default_value="good"),
            DeclareLaunchArgument("target_prefix", default_value="tomato_ripe_"),
            DeclareLaunchArgument("target_side", default_value="left"),
            DeclareLaunchArgument("min_forward", default_value="0.25"),
            DeclareLaunchArgument("max_forward", default_value="1.35"),
            DeclareLaunchArgument("max_lateral", default_value="1.25"),
            DeclareLaunchArgument("max_picks", default_value="0"),
            DeclareLaunchArgument("min_z", default_value="0.75"),
            DeclareLaunchArgument("max_z", default_value="1.35"),
            DeclareLaunchArgument("planning_group", default_value="fr3_arm"),
            DeclareLaunchArgument("base_frame", default_value="fr3_link0"),
            DeclareLaunchArgument("ee_link", default_value="fr3_hand_tcp"),
            DeclareLaunchArgument("approach_distance", default_value="0.20"),
            DeclareLaunchArgument("pick_distance", default_value="0.085"),
            DeclareLaunchArgument("grasp_roll", default_value="1.5708"),
            DeclareLaunchArgument("grasp_lateral_offset", default_value="0.0"),
            DeclareLaunchArgument("gripper_close_width", default_value="0.070"),
            DeclareLaunchArgument("tomato_topic", default_value="/tomato_center"),
            DeclareLaunchArgument("tomato_radius_topic", default_value="/tomato_radius"),
            DeclareLaunchArgument("auto_detach_max_distance", default_value="0.30"),
            DeclareLaunchArgument("min_pick_distance", default_value="0.055"),
            DeclareLaunchArgument("max_pick_distance", default_value="0.095"),
            DeclareLaunchArgument("max_target_distance", default_value="1.20"),
            DeclareLaunchArgument("pre_detect_joints", default_value="-1.710,-1.267,0.0,-2.792,0.0,2.800,0.0"),
            DeclareLaunchArgument("pre_detect_duration", default_value="3.0"),
            DeclareLaunchArgument("observation_settle", default_value="1.0"),
            DeclareLaunchArgument("stable_target_samples", default_value="4"),
            DeclareLaunchArgument("target_stability_tolerance", default_value="0.06"),
            DeclareLaunchArgument("max_approach_base_distance", default_value="1.10"),
            DeclareLaunchArgument("max_pick_base_distance", default_value="1.18"),
            DeclareLaunchArgument("color_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image"),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/depth/camera_info"),
            DeclareLaunchArgument("tomato_detector_frame", default_value="fr3_camera_depth_optical_frame"),
            DeclareLaunchArgument("detector_target_strategy", default_value="center_nearest"),
            DeclareLaunchArgument("detector_center_preference_weight", default_value="0.35"),
            DeclareLaunchArgument("detector_center_method", default_value="bbox"),
            DeclareLaunchArgument("detector_roi_width_fraction", default_value="1.0"),
            DeclareLaunchArgument("detector_roi_height_fraction", default_value="1.0"),
            DeclareLaunchArgument("detector_roi_center_u_fraction", default_value="0.5"),
            DeclareLaunchArgument("detector_roi_center_v_fraction", default_value="0.5"),
            DeclareLaunchArgument("detector_max_abs_x", default_value="0.42"),
            DeclareLaunchArgument("detector_max_abs_y", default_value="0.45"),
            DeclareLaunchArgument("velocity_scaling", default_value="0.20"),
            DeclareLaunchArgument("acceleration_scaling", default_value="0.20"),
            DeclareLaunchArgument("grasp_local_x", default_value="0.58"),
            DeclareLaunchArgument("grasp_local_y", default_value="0.0"),
            DeclareLaunchArgument("grasp_local_z", default_value="0.677"),
            DeclareLaunchArgument("grasp_tolerance", default_value="0.0"),
            DeclareLaunchArgument("moveit_group", default_value="fr3_arm"),
            DeclareLaunchArgument("ik_frame", default_value="fr3_link0"),
            DeclareLaunchArgument("ik_link", default_value="fr3_hand_tcp"),
            DeclareLaunchArgument("approach_offset", default_value="0.18"),
            DeclareLaunchArgument("grasp_offset", default_value="0.0"),
            DeclareLaunchArgument("grasp_surface_clearance", default_value="0.030"),
            DeclareLaunchArgument("tcp_front_offset", default_value="0.020"),
            DeclareLaunchArgument("grip_clearance", default_value="0.0"),
            DeclareLaunchArgument("pre_grasp_z_offset", default_value="0.03"),
            DeclareLaunchArgument("lift_offset", default_value="0.18"),
            DeclareLaunchArgument("spawn_baskets", default_value="true"),
            DeclareLaunchArgument("spawn_arm_controllers", default_value="true"),
            DeclareLaunchArgument("spawn_drive_controller", default_value="false"),
            DeclareLaunchArgument("good_basket_x", default_value="37.62"),
            DeclareLaunchArgument("good_basket_y", default_value="7.95"),
            DeclareLaunchArgument("bad_basket_x", default_value="37.62"),
            DeclareLaunchArgument("bad_basket_y", default_value="7.60"),
            DeclareLaunchArgument("basket_z", default_value="0.0"),
            DeclareLaunchArgument("basket_yaw", default_value="1.5708"),
            DeclareLaunchArgument("planning_scene_include_plants", default_value="false"),
            DeclareLaunchArgument("planning_scene_plant_radius", default_value="0.10"),
            DeclareLaunchArgument("planning_scene_plant_height", default_value="1.85"),
            gz_resource_path,
            gl_threaded_optimizations,
            gazebo,
            gazebo_gui,
            clock_bridge,
            robot_state_publisher,
            move_group_node,
            greenhouse_planning_scene,
            rviz_node,
            TimerAction(period=6.0, actions=[spawn]),
            camera_bridge,
            point_cloud_bridge,
            tomato_depth_detector,
            TimerAction(period=2.0, actions=[good_basket_spawn, bad_basket_spawn]),
            TimerAction(period=18.0, actions=[joint_state_broadcaster_spawner]),
            TimerAction(period=24.0, actions=[drive_controller_spawner]),
            TimerAction(period=26.0, actions=[fr3_arm_spawner]),
            TimerAction(period=32.0, actions=[fr3_gripper_spawner]),
            demo_runner,
        ]
    )
