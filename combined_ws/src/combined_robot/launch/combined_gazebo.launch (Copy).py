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
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Resolve package share directories (needed for resource path) ──────────
    husarion_desc_share  = get_package_share_directory('husarion_ugv_description')
    franka_desc_share    = get_package_share_directory('franka_description')
    combined_share_path  = get_package_share_directory('combined_robot')

    # ── GZ_SIM_RESOURCE_PATH ──────────────────────────────────────────────────
    # Tells Gazebo where to find meshes referenced via package:// URIs.
    # Each entry is the *parent of the package's share dir* so that
    # Gazebo can find it as:  <resource_path>/<pkg_name>/meshes/...
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            # husarion_ws share tree  → .../install/<pkg>/share → parent
            os.path.dirname(husarion_desc_share),   # ...install/husarion_ugv_description/share
            os.path.join(os.path.dirname(os.path.dirname(husarion_desc_share)), 'husarion_ugv_gazebo', 'share'),
            # franka_ws share tree
            os.path.dirname(franka_desc_share),     # ...install/franka_description/share
            # combined_ws share tree
            os.path.join(combined_share_path, 'models'),
            os.path.dirname(combined_share_path),
            # System ROS 2 share (for realsense2_description)
            '/opt/ros/jazzy/share',
            # Make sure Gazebo finds the newly downloaded Fonyuy45 models
            os.path.expanduser('~/.gz/models'),
            # Preserve any existing GZ_SIM_RESOURCE_PATH entries
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ])
    )

    # ── FindPackageShare substitutions (for use in Command) ───────────────────
    combined_share  = FindPackageShare('combined_robot')
    husarion_desc   = FindPackageShare('husarion_ugv_description')
    husarion_gz     = FindPackageShare('husarion_ugv_gazebo')
    franka_gz       = FindPackageShare('franka_gazebo_bringup')

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
        ' components_config_path:=',
        PathJoinSubstitution([husarion_desc, 'config', 'components.yaml']),
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # ── Gazebo simulation ─────────────────────────────────────────────────────
    world_file = PathJoinSubstitution([combined_share, 'worlds', 'tomato_farm_px4_complete.sdf'])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': [world_file, ' -r']}.items()
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
            # Lidar artık düzgün yöne bakıyor, yaw=0 ile sera boyunca ileri
            '-x', '10.5', '-y', '0.95', '-z', '0.25', '-Y', '0.0'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Pickup object spawning removed as we are using Fonyuy45's complete greenhouse world.


    # ── Clock bridge ──────────────────────────────────────────────────────────
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ── Camera bridge (RealSense D435 on FR3 gripper) ─────────────────────────
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=[
            '/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth/image/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ── Lidar bridge (Panther LDR06) ──────────────────────────────────────────
    # Panther lidar is on Gazebo /scan → bridge directly to ROS /scan
    # IMPORTANT: This is delayed (see below) so that drive_controller's
    # odom→base_link TF is already available before /scan messages arrive.
    # Without this, slam_toolbox's internal MessageFilter queue fills up
    # with scans that have no matching TF, causing permanent "dropping message".
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ── IMU bridge (Gazebo → ROS) ─────────────────────────────────────────────
    # IMU sensor publishes directly from Gazebo to /imu/data_raw.
    # We bridge it to ROS so the EKF can fuse it with wheel odometry.
    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        arguments=['/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ── EKF Localization ──────────────────────────────────────────────────────
    # Fuses wheel odometry (linear velocity) with IMU (yaw rate) to produce
    # accurate odom→base_link TF. Without this, skid-steer wheel slip causes
    # wildly inaccurate rotation estimates and broken SLAM maps.
    #
    # Uses sim-optimized config (ekf_sim_config.yaml) instead of the default
    # relative_localization.yaml because:
    #   - sensor_timeout: 0.05 → 0.5 (Gazebo sim time jitter tolerance)
    #   - transform_timeout: 0.05 → 0.5
    #   - use_control: false (removes cmd_vel prediction noise)
    ekf_localization = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter',
        parameters=[
            PathJoinSubstitution([
                combined_share,
                'config', 'ekf_sim_config.yaml'
            ]),
            {'use_sim_time': True}
        ],
        remappings=[
            # NOTE: odometry/wheels remap REMOVED — drive_controller already
            # publishes to /odometry/wheels, which matches EKF's odom0 config.
            # The previous remap to /drive_controller/odom was WRONG because
            # that topic has no publisher in Gazebo Harmonic.
            ('imu/data', '/imu/data_raw')
        ],
        output='screen'
    )

    # ── Controller spawners ───────────────────────────────────────────────────
    # NOTE: imu_broadcaster is intentionally NOT spawned here.
    #   Reason: EStopSystem hardware plugin does not expose /imu/* state
    #   interfaces through ros2_control in simulation. The IMU is published
    #   directly by the Gazebo IMU sensor to /imu/data_raw via the Gazebo
    #   plugin — no ros2_control broadcaster is needed for that signal.
    #   Instead, we bridge /imu/data_raw and feed it directly to the EKF.

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['drive_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    fr3_arm_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fr3_arm_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    fr3_gripper_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fr3_gripper_controller', '--controller-manager', '/controller_manager'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Staggered delays — matches working workspace timings
    delayed_jsb   = TimerAction(period=4.0, actions=[joint_state_broadcaster_spawner])
    delayed_drive = TimerAction(period=5.5, actions=[drive_controller_spawner])
    delayed_arm   = TimerAction(period=7.0, actions=[fr3_arm_spawner])
    delayed_grip  = TimerAction(period=9.0, actions=[fr3_gripper_spawner])
    delayed_ekf   = TimerAction(period=10.0, actions=[ekf_localization])

    return LaunchDescription([
        gz_resource_path,
        gazebo,
        clock_bridge,
        robot_state_publisher,
        spawn,
        camera_bridge,
        lidar_bridge,
        imu_bridge,
        delayed_jsb,
        delayed_drive,
        delayed_arm,
        delayed_grip,
        delayed_ekf,
    ])
