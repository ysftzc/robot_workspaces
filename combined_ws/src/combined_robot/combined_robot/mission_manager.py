import json
import math
import re
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from enum import Enum
from pathlib import Path

import rclpy
import yaml
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, TwistStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import CollisionObject, Constraints, JointConstraint, MotionPlanRequest, RobotState
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from combined_robot.pick_place_detachable import (
    BASKET_SETTLE_POSES,
    gazebo_gripper_parent_link,
)


class MissionState(str, Enum):
    BOOT = 'BOOT'
    LOCALIZE = 'LOCALIZE'
    SURVEY_UPPER = 'SURVEY_UPPER'
    SURVEY_CENTER = 'SURVEY_CENTER'
    SURVEY_LOWER = 'SURVEY_LOWER'
    RETURN_TO_NEAREST_START = 'RETURN_TO_NEAREST_START'
    PLAN_HARVEST = 'PLAN_HARVEST'
    HARVESTING = 'HARVESTING'
    RETURN_HOME = 'RETURN_HOME'
    DONE = 'DONE'
    STOPPED = 'STOPPED'
    FAILED = 'FAILED'


class MissionManager(Node):
    _FR3_JOINT_LIMITS = {
        'fr3_joint1': (-2.9007, 2.9007),
        'fr3_joint2': (-1.8361, 1.8361),
        'fr3_joint3': (-2.9007, 2.9007),
        'fr3_joint4': (-3.0770, -0.1169),
        'fr3_joint5': (-2.8763, 2.8763),
        'fr3_joint6': (0.4398, 4.6216),
        'fr3_joint7': (-3.0508, 3.0508),
    }
    _MOVEIT_START_STATE_LIMIT_MARGIN = 1.0e-3

    def __init__(self):
        super().__init__('mission_manager')

        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('route_name', 'center_corridor_patrol')
        self.declare_parameter('mission_mode', 'waypoint')
        self.declare_parameter('autostart', False)
        self.declare_parameter('loop_route', False)
        self.declare_parameter('stop_on_failure', True)
        self.declare_parameter('goal_frame', 'map')
        self.declare_parameter('autostart_requires_localization', True)
        self.declare_parameter('autostart_delay_sec', 18.0)
        self.declare_parameter('arm_pose_file', '')
        self.declare_parameter('arm_pose_enabled', True)
        self.declare_parameter('arm_motion_mode', 'direct_joint')
        self.declare_parameter(
            'arm_controller_action', '/fr3_arm_controller/follow_joint_trajectory'
        )
        self.declare_parameter('move_action', '/move_action')
        self.declare_parameter('moveit_planning_group', 'fr3_arm')
        self.declare_parameter('moveit_pipeline_id', 'ompl')
        self.declare_parameter('moveit_planner_id', 'RRTConnectkConfigDefault')
        self.declare_parameter('moveit_num_planning_attempts', 5)
        self.declare_parameter('moveit_allowed_planning_time_sec', 5.0)
        self.declare_parameter('moveit_velocity_scaling', 0.10)
        self.declare_parameter('moveit_acceleration_scaling', 0.10)
        self.declare_parameter('moveit_joint_tolerance_rad', 0.02)
        self.declare_parameter('arm_avoid_tomatoes', False)
        self.declare_parameter('arm_avoid_tomato_radius_m', 0.085)
        self.declare_parameter('arm_avoid_tomato_z_offset_m', 0.0)
        self.declare_parameter(
            'gripper_controller_action',
            '/fr3_gripper_controller/follow_joint_trajectory',
        )
        self.declare_parameter('arm_motion_duration_sec', 3.0)
        self.declare_parameter('arm_action_server_timeout_sec', 5.0)
        self.declare_parameter('gripper_action_server_timeout_sec', 5.0)
        self.declare_parameter('nav_goal_timeout_sec', 45.0)
        self.declare_parameter('verify_goal_pose', True)
        self.declare_parameter('verify_goal_xy_tolerance', 0.08)
        self.declare_parameter('verify_goal_yaw_tolerance', 0.10)
        self.declare_parameter('use_direct_turn_waypoints', True)
        self.declare_parameter('direct_turn_xy_tolerance', 0.35)
        self.declare_parameter('direct_turn_angular_speed', 0.85)
        self.declare_parameter('direct_turn_yaw_gain', 2.0)
        self.declare_parameter('direct_turn_rate_hz', 20.0)
        self.declare_parameter('harvest_pick_enabled', False)
        self.declare_parameter('harvest_pick_after_waypoint', 'plant_10_b_pick_front')
        self.declare_parameter('harvest_pick_auto_select', False)
        self.declare_parameter('harvest_pick_inventory_topic', '/tomato_map/list')
        self.declare_parameter(
            'harvest_pick_target_selection_topic',
            '/tomato_harvest/target_selection',
        )
        self.declare_parameter(
            'harvest_pick_allowed_classes',
            'fully_ripened,ripe,rotten,disease,diseased',
        )
        self.declare_parameter('harvest_pick_reject_classes', 'green,unripe')
        self.declare_parameter('harvest_pick_good_classes', 'fully_ripened,ripe')
        self.declare_parameter(
            'harvest_pick_bad_classes',
            'rotten,disease,diseased,bad,green,unripe',
        )
        self.declare_parameter('harvest_pick_min_confidence', 0.35)
        self.declare_parameter('harvest_pick_min_updated_count', 2)
        self.declare_parameter('harvest_pick_local_radius_m', 0.90)
        self.declare_parameter('harvest_pick_max_base_lateral_m', 0.90)
        self.declare_parameter('harvest_pick_min_z_m', 0.25)
        self.declare_parameter('harvest_pick_max_z_m', 1.35)
        self.declare_parameter('harvest_pick_require_model_name', True)
        self.declare_parameter('harvest_pick_prefer_model_pose', False)
        self.declare_parameter('harvest_pick_fallback_to_configured', False)
        self.declare_parameter('harvest_pick_target_exclusion_settle_sec', 0.4)
        self.declare_parameter('harvest_pick_detection_settle_sec', 3.0)
        self.declare_parameter('harvest_pick_inventory_max_age_sec', 4.0)
        self.declare_parameter('harvest_pick_max_attempts', 0)
        self.declare_parameter('harvest_pick_max_per_waypoint', 0)
        self.declare_parameter('harvest_return_to_pick_pose_after_attempt', True)
        self.declare_parameter('harvest_pick_grasp_z_offsets', '0.0,0.020,-0.020')
        self.declare_parameter('harvest_pick_max_candidates_per_target', 6)
        self.declare_parameter('harvest_pick_world_file', '')
        self.declare_parameter('harvest_pick_world_name', 'tomato_farm')
        self.declare_parameter('harvest_pick_tomato_model', 'tomato_ripe2_B_10_0')
        self.declare_parameter('harvest_pick_basket', 'good')
        self.declare_parameter('harvest_pick_target_topic', '/mission_pick/tomato_center')
        self.declare_parameter('harvest_pick_target_radius_topic', '/mission_pick/tomato_radius')
        self.declare_parameter('harvest_pick_stop_route_after_attempt', True)
        self.declare_parameter('harvest_pick_place_in_basket', True)
        self.declare_parameter('harvest_pick_lock_harvested_to_basket', False)
        self.declare_parameter('harvest_pick_freeze_basket_z_offset', 0.055)
        self.declare_parameter('harvest_pick_harvested_tomato_collision_radius', 0.038)
        self.declare_parameter('harvest_pick_timeout_sec', 180.0)
        self.declare_parameter('harvest_pick_map_x_from_gazebo_y_offset', -4.93)
        self.declare_parameter('harvest_pick_map_y_from_gazebo_x_origin', 35.83)
        self.declare_parameter('harvest_pick_robot_z', 0.20)
        self.declare_parameter('harvest_pick_base_frame', 'fr3_link0')
        self.declare_parameter('harvest_pick_use_live_gazebo_pose', True)
        self.declare_parameter('harvest_pick_robot_model', 'combined_robot')
        self.declare_parameter('harvest_pick_prewarm_gripper_attachments', False)
        self.declare_parameter('harvest_pick_gripper_attach_parent_link', 'fr3_link7')
        self.declare_parameter('harvest_pick_base_z_offset', 0.1715)
        self.declare_parameter('harvest_pick_gazebo_pose_timeout_sec', 8.0)
        self.declare_parameter('basket_visual_lock_enabled', False)

        self._waypoint_file = self.get_parameter('waypoint_file').value
        self._route_name = self.get_parameter('route_name').value
        self._mission_mode = str(self.get_parameter('mission_mode').value).strip().lower()
        if self._mission_mode not in ('waypoint', 'survey_harvest', 'stationary_harvest'):
            raise RuntimeError(
                f'Unsupported mission_mode "{self._mission_mode}". '
                'Expected "waypoint", "survey_harvest", or "stationary_harvest".'
            )
        self._autostart = self._as_bool(self.get_parameter('autostart').value)
        self._loop_route = self._as_bool(self.get_parameter('loop_route').value)
        self._stop_on_failure = self._as_bool(self.get_parameter('stop_on_failure').value)
        self._goal_frame = self.get_parameter('goal_frame').value
        self._autostart_requires_localization = self._as_bool(
            self.get_parameter('autostart_requires_localization').value
        )
        self._autostart_delay_sec = max(
            0.0,
            float(self.get_parameter('autostart_delay_sec').value),
        )
        if self._mission_mode == 'stationary_harvest':
            self._autostart_requires_localization = False
        self._arm_pose_file = self.get_parameter('arm_pose_file').value
        self._arm_pose_enabled = self._as_bool(self.get_parameter('arm_pose_enabled').value)
        self._arm_motion_mode = str(self.get_parameter('arm_motion_mode').value).strip().lower()
        if self._arm_motion_mode not in ('direct_joint', 'moveit'):
            raise RuntimeError(
                f'Unsupported arm_motion_mode "{self._arm_motion_mode}". '
                'Expected "direct_joint" or "moveit".'
            )
        self._arm_controller_action = self.get_parameter('arm_controller_action').value
        self._move_action = self.get_parameter('move_action').value
        self._moveit_planning_group = self.get_parameter('moveit_planning_group').value
        self._moveit_pipeline_id = self.get_parameter('moveit_pipeline_id').value
        self._moveit_planner_id = self.get_parameter('moveit_planner_id').value
        self._moveit_num_planning_attempts = int(
            self.get_parameter('moveit_num_planning_attempts').value
        )
        self._moveit_allowed_planning_time_sec = float(
            self.get_parameter('moveit_allowed_planning_time_sec').value
        )
        self._moveit_velocity_scaling = float(self.get_parameter('moveit_velocity_scaling').value)
        self._moveit_acceleration_scaling = float(
            self.get_parameter('moveit_acceleration_scaling').value
        )
        self._moveit_joint_tolerance_rad = float(
            self.get_parameter('moveit_joint_tolerance_rad').value
        )
        self._arm_avoid_tomatoes = self._as_bool(
            self.get_parameter('arm_avoid_tomatoes').value
        )
        self._arm_avoid_tomato_radius_m = max(
            0.01, float(self.get_parameter('arm_avoid_tomato_radius_m').value)
        )
        self._arm_avoid_tomato_z_offset_m = float(
            self.get_parameter('arm_avoid_tomato_z_offset_m').value
        )
        self._gripper_controller_action = self.get_parameter(
            'gripper_controller_action'
        ).value
        self._arm_motion_duration_sec = float(
            self.get_parameter('arm_motion_duration_sec').value
        )
        self._arm_action_server_timeout_sec = float(
            self.get_parameter('arm_action_server_timeout_sec').value
        )
        self._gripper_action_server_timeout_sec = float(
            self.get_parameter('gripper_action_server_timeout_sec').value
        )
        self._nav_goal_timeout_sec = float(self.get_parameter('nav_goal_timeout_sec').value)
        self._verify_goal_pose = self._as_bool(self.get_parameter('verify_goal_pose').value)
        self._verify_goal_xy_tolerance = float(
            self.get_parameter('verify_goal_xy_tolerance').value
        )
        self._verify_goal_yaw_tolerance = float(
            self.get_parameter('verify_goal_yaw_tolerance').value
        )
        self._use_direct_turn_waypoints = self._as_bool(
            self.get_parameter('use_direct_turn_waypoints').value
        )
        self._direct_turn_xy_tolerance = float(
            self.get_parameter('direct_turn_xy_tolerance').value
        )
        self._direct_turn_xy_tolerance = max(
            self._direct_turn_xy_tolerance,
            self._verify_goal_xy_tolerance,
        )
        self._direct_turn_angular_speed = float(
            self.get_parameter('direct_turn_angular_speed').value
        )
        self._direct_turn_yaw_gain = float(self.get_parameter('direct_turn_yaw_gain').value)
        self._direct_turn_rate_hz = max(
            1.0, float(self.get_parameter('direct_turn_rate_hz').value)
        )
        self._harvest_pick_enabled = self._as_bool(
            self.get_parameter('harvest_pick_enabled').value
        )
        self._harvest_pick_after_waypoint = str(
            self.get_parameter('harvest_pick_after_waypoint').value
        )
        self._harvest_pick_auto_select = self._as_bool(
            self.get_parameter('harvest_pick_auto_select').value
        )
        self._harvest_pick_inventory_topic = str(
            self.get_parameter('harvest_pick_inventory_topic').value
        )
        self._harvest_pick_target_selection_topic = str(
            self.get_parameter('harvest_pick_target_selection_topic').value
        )
        self._harvest_pick_allowed_classes = self._name_set(
            self.get_parameter('harvest_pick_allowed_classes').value
        )
        self._harvest_pick_reject_classes = self._name_set(
            self.get_parameter('harvest_pick_reject_classes').value
        )
        self._harvest_pick_good_classes = self._name_set(
            self.get_parameter('harvest_pick_good_classes').value
        )
        self._harvest_pick_bad_classes = self._name_set(
            self.get_parameter('harvest_pick_bad_classes').value
        )
        self._harvest_pick_min_confidence = float(
            self.get_parameter('harvest_pick_min_confidence').value
        )
        self._harvest_pick_min_updated_count = int(
            float(self.get_parameter('harvest_pick_min_updated_count').value)
        )
        self._harvest_pick_local_radius_m = float(
            self.get_parameter('harvest_pick_local_radius_m').value
        )
        self._harvest_pick_max_base_lateral_m = max(
            0.0,
            float(self.get_parameter('harvest_pick_max_base_lateral_m').value),
        )
        self._harvest_pick_min_z_m = float(self.get_parameter('harvest_pick_min_z_m').value)
        self._harvest_pick_max_z_m = float(self.get_parameter('harvest_pick_max_z_m').value)
        self._harvest_pick_require_model_name = self._as_bool(
            self.get_parameter('harvest_pick_require_model_name').value
        )
        self._harvest_pick_prefer_model_pose = self._as_bool(
            self.get_parameter('harvest_pick_prefer_model_pose').value
        )
        self._harvest_pick_fallback_to_configured = self._as_bool(
            self.get_parameter('harvest_pick_fallback_to_configured').value
        )
        self._harvest_pick_target_exclusion_settle_sec = max(
            0.0,
            float(self.get_parameter('harvest_pick_target_exclusion_settle_sec').value),
        )
        self._harvest_pick_detection_settle_sec = max(
            0.0,
            float(self.get_parameter('harvest_pick_detection_settle_sec').value),
        )
        self._harvest_pick_inventory_max_age_sec = max(
            0.0,
            float(self.get_parameter('harvest_pick_inventory_max_age_sec').value),
        )
        self._harvest_pick_max_attempts = max(
            0, int(float(self.get_parameter('harvest_pick_max_attempts').value))
        )
        self._harvest_pick_max_per_waypoint = max(
            0, int(float(self.get_parameter('harvest_pick_max_per_waypoint').value))
        )
        self._harvest_return_to_pick_pose_after_attempt = self._as_bool(
            self.get_parameter('harvest_return_to_pick_pose_after_attempt').value
        )
        self._harvest_pick_grasp_z_offsets = str(
            self.get_parameter('harvest_pick_grasp_z_offsets').value
        ).strip() or '0.0'
        self._harvest_pick_max_candidates_per_target = max(
            0,
            int(float(self.get_parameter('harvest_pick_max_candidates_per_target').value)),
        )
        self._harvest_pick_world_file = str(self.get_parameter('harvest_pick_world_file').value)
        self._harvest_pick_world_name = str(self.get_parameter('harvest_pick_world_name').value)
        self._harvest_pick_tomato_model = str(
            self.get_parameter('harvest_pick_tomato_model').value
        )
        self._harvest_pick_configured_tomato_model = self._harvest_pick_tomato_model
        self._harvest_pick_basket = str(self.get_parameter('harvest_pick_basket').value)
        self._harvest_pick_target_topic = str(
            self.get_parameter('harvest_pick_target_topic').value
        )
        self._harvest_pick_target_radius_topic = str(
            self.get_parameter('harvest_pick_target_radius_topic').value
        )
        self._harvest_pick_stop_route_after_attempt = self._as_bool(
            self.get_parameter('harvest_pick_stop_route_after_attempt').value
        )
        self._harvest_pick_place_in_basket = self._as_bool(
            self.get_parameter('harvest_pick_place_in_basket').value
        )
        self._harvest_pick_lock_harvested_to_basket = self._as_bool(
            self.get_parameter('harvest_pick_lock_harvested_to_basket').value
        )
        self._harvest_pick_freeze_basket_z_offset = float(
            self.get_parameter('harvest_pick_freeze_basket_z_offset').value
        )
        self._harvest_pick_harvested_tomato_collision_radius = float(
            self.get_parameter('harvest_pick_harvested_tomato_collision_radius').value
        )
        self._harvest_pick_timeout_sec = float(
            self.get_parameter('harvest_pick_timeout_sec').value
        )
        self._harvest_pick_map_x_offset = float(
            self.get_parameter('harvest_pick_map_x_from_gazebo_y_offset').value
        )
        self._harvest_pick_map_y_origin = float(
            self.get_parameter('harvest_pick_map_y_from_gazebo_x_origin').value
        )
        self._harvest_pick_robot_z = float(self.get_parameter('harvest_pick_robot_z').value)
        self._harvest_pick_base_frame = str(
            self.get_parameter('harvest_pick_base_frame').value
        )
        self._harvest_pick_use_live_gazebo_pose = self._as_bool(
            self.get_parameter('harvest_pick_use_live_gazebo_pose').value
        )
        self._harvest_pick_robot_model = str(
            self.get_parameter('harvest_pick_robot_model').value
        )
        self._harvest_pick_prewarm_gripper_attachments = self._as_bool(
            self.get_parameter('harvest_pick_prewarm_gripper_attachments').value
        )
        self._harvest_pick_gripper_attach_parent_link = gazebo_gripper_parent_link(
            str(self.get_parameter('harvest_pick_gripper_attach_parent_link').value)
        )
        self._harvest_pick_base_z_offset = float(
            self.get_parameter('harvest_pick_base_z_offset').value
        )
        self._harvest_pick_gazebo_pose_timeout_sec = float(
            self.get_parameter('harvest_pick_gazebo_pose_timeout_sec').value
        )

        self._route = self._load_route(self._waypoint_file, self._route_name)
        self._arm_joints, self._arm_poses = self._load_arm_poses(self._arm_pose_file)
        self._goal_index = 0
        self._running = False
        self._goal_handle = None
        self._arm_goal_handle = None
        self._gripper_goal_handle = None
        self._joint_state = None
        self._have_localization = False
        self._latest_pose = None
        self._autostart_armed = self._autostart
        self._autostart_delay_timer = None
        self._retry_timer = None
        self._nav_goal_timer = None
        self._nav_goal_timed_out = False
        self._direct_turn_timer = None
        self._direct_turn_start_time = None
        self._state = (
            MissionState.LOCALIZE
            if self._autostart_requires_localization
            else MissionState.BOOT
        )
        self._harvest_targets = []
        self._active_arm_pose = 'none'
        self._pending_arm_pose = None
        self._pending_arm_pose_index = None
        self._pending_gripper_pose = None
        self._pending_gripper_pose_index = None
        self._pending_gripper_positions = None
        self._active_gripper_positions = None
        self._skipped_arm_poses = set()
        self._harvest_pick_process = None
        self._harvest_pick_timer = None
        self._harvest_pick_settle_timer = None
        self._harvest_pick_settle_waypoint = None
        self._harvest_pick_settle_waypoint_index = None
        self._harvest_pick_target_pose = None
        self._harvest_pick_selected_record = None
        self._harvest_pick_selected_basket = None
        self._harvest_pick_robot_pose = None
        self._harvest_pick_waypoint_index = None
        self._resume_harvest_pick_after_arm_pose = False
        self._harvest_pick_triggered = False
        self._harvest_pick_attempt_count = 0
        self._harvest_pick_attempted_waypoints = {}
        self._tomato_inventory = {}
        self._picked_tomato_models = set()
        self._failed_harvest_pick_models = set()
        self._harvest_pick_model_map_poses = None
        self._harvest_pick_model_world_poses = None
        self._arm_tomato_map_poses = None
        self._harvest_pick_prewarm_thread = None
        self._harvest_pick_prewarm_started = False
        self._logged_arm_tomato_collision_objects = False
        self._stationary_pose_log_time = 0.0
        self._basket_carried_tomatoes = {}
        self._basket_visual_lock_enabled = self._as_bool(
            self.get_parameter('basket_visual_lock_enabled').value
        )
        self._basket_carrier_timer = None
        if self._basket_visual_lock_enabled:
            self._basket_carrier_timer = self.create_timer(
                1.0,
                self._update_basket_tomato_poses,
            )

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        if self._arm_motion_mode == 'moveit':
            self._arm_client = ActionClient(self, MoveGroup, self._move_action)
        else:
            self._arm_client = ActionClient(
                self, FollowJointTrajectory, self._arm_controller_action
            )
        self._gripper_client = ActionClient(
            self, FollowJointTrajectory, self._gripper_controller_action
        )
        self._status_pub = self.create_publisher(String, 'mission_manager/status', 10)
        self._harvest_pick_target_pub = self.create_publisher(
            PoseStamped, self._harvest_pick_target_topic, 10
        )
        self._harvest_pick_target_radius_pub = None
        if self._harvest_pick_target_radius_topic:
            self._harvest_pick_target_radius_pub = self.create_publisher(
                Float32, self._harvest_pick_target_radius_topic, 10
            )
        self._harvest_pick_selection_pub = self.create_publisher(
            String, self._harvest_pick_target_selection_topic, 10
        )
        self._cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel_nav', 10)
        self.create_subscription(JointState, '/joint_states', self._handle_joint_state, 10)
        if self._harvest_pick_inventory_topic:
            self.create_subscription(
                String,
                self._harvest_pick_inventory_topic,
                self._handle_tomato_inventory,
                10,
            )
        self.create_subscription(String, '/tomato_harvest/picked', self._handle_picked_event, 10)
        amcl_pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._handle_amcl_pose, amcl_pose_qos
        )

        self.create_service(Trigger, 'mission_manager/start', self._handle_start)
        self.create_service(Trigger, 'mission_manager/stop', self._handle_stop)
        self.create_service(Trigger, 'mission_manager/reset', self._handle_reset)

        self._status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f'Loaded route "{self._route_name}" with {len(self._route)} waypoints '
            f'in "{self._mission_mode}" mode. Arm motion mode: {self._arm_motion_mode}.'
        )

        if self._autostart:
            if self._autostart_requires_localization:
                self.get_logger().info('Autostart armed. Waiting for localization on /amcl_pose.')
            else:
                self._begin_mission_after_autostart_delay(
                    'Autostart started without localization gate.'
                )

    def _as_bool(self, value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _name_set(self, value):
        raw = '' if value is None else str(value).strip().lower()
        if not raw or raw in ('all', '*', 'none', 'off', 'false'):
            return set()
        return {part.strip() for part in raw.replace(';', ',').split(',') if part.strip()}

    def _set_state(self, state):
        next_state = MissionState(state)
        if self._state == next_state:
            return

        self._state = next_state
        self.get_logger().info(f'Mission state: {self._state.value}')

    def _handle_joint_state(self, msg):
        self._joint_state = msg

    def _handle_tomato_inventory(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Ignoring tomato inventory JSON: {exc}')
            return

        records = payload.get('records', []) if isinstance(payload, dict) else []
        if not isinstance(records, list):
            return

        seen_at = time.monotonic()
        for record in records:
            if not isinstance(record, dict):
                continue
            key = self._tomato_record_key(record)
            if not key:
                continue
            previous = self._tomato_inventory.get(key)
            previous_timestamp = previous.get('timestamp') if isinstance(previous, dict) else None
            stored = dict(record)
            timestamp = stored.get('timestamp')
            if previous is not None and timestamp and timestamp == previous_timestamp:
                stored['_last_seen_monotonic'] = previous.get('_last_seen_monotonic', seen_at)
            else:
                stored['_last_seen_monotonic'] = seen_at
            self._tomato_inventory[key] = stored

    def _handle_picked_event(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        model_name = str(payload.get('tomato_model', '')).strip()
        if not model_name:
            return
        self._picked_tomato_models.add(model_name)
        self._failed_harvest_pick_models.discard(model_name)
        self._tomato_inventory.pop(model_name, None)
        if str(payload.get('status', '')).strip().lower() == 'placed':
            if not self._basket_visual_lock_enabled:
                return
            basket = str(payload.get('basket', self._harvest_pick_basket)).strip()
            if basket not in BASKET_SETTLE_POSES:
                basket = 'good'
            harvested_name = f'harvested_{self._safe_model_name(model_name)}'
            self._basket_carried_tomatoes[harvested_name] = {
                'basket': basket,
                'z_offset': 0.015,
            }
            self.get_logger().info(
                f'Keeping harvested tomato {harvested_name} visually locked in {basket} basket.'
            )

    @staticmethod
    def _safe_model_name(model_name):
        return ''.join(ch if ch.isalnum() or ch == '_' else '_' for ch in model_name)

    def _basket_tomato_world_pose(self, basket, z_offset=0.015):
        try:
            robot_x, robot_y, robot_z, robot_yaw = self._current_robot_gazebo_pose()
        except RuntimeError:
            return None
        local_x, local_y, local_z = BASKET_SETTLE_POSES[basket]
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        return (
            robot_x + cos_yaw * local_x - sin_yaw * local_y,
            robot_y + sin_yaw * local_x + cos_yaw * local_y,
            robot_z + local_z + z_offset,
        )

    def _set_gazebo_model_pose_xyz(self, model_name, xyz):
        x, y, z = xyz
        request = (
            f'name: "{model_name}", '
            f'position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
            'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}'
        )
        timed_out = False
        try:
            result = subprocess.run(
                [
                    'gz',
                    'service',
                    '-s',
                    f'/world/{self._harvest_pick_world_name}/set_pose',
                    '--reqtype',
                    'gz.msgs.Pose',
                    '--reptype',
                    'gz.msgs.Boolean',
                    '--timeout',
                    '500',
                    '--req',
                    request,
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=1.0,
            )
        except (OSError, subprocess.SubprocessError):
            return False
        return result.returncode == 0 and 'data: true' in result.stdout

    def _update_basket_tomato_poses(self):
        if not self._basket_carried_tomatoes:
            return

        stale = []
        for model_name, info in list(self._basket_carried_tomatoes.items()):
            basket = info.get('basket', 'good')
            xyz = self._basket_tomato_world_pose(
                basket,
                float(info.get('z_offset', 0.015)),
            )
            if xyz is None:
                continue
            if not self._set_gazebo_model_pose_xyz(model_name, xyz):
                failures = int(info.get('failures', 0)) + 1
                info['failures'] = failures
                if failures > 25:
                    stale.append(model_name)
            else:
                info['failures'] = 0

        for model_name in stale:
            self._basket_carried_tomatoes.pop(model_name, None)

    @staticmethod
    def _tomato_record_key(record):
        model_name = str(record.get('model_name', '')).strip()
        if model_name:
            return model_name
        tomato_id = str(record.get('tomato_id', '')).strip()
        return tomato_id

    def _load_route(self, waypoint_file, route_name):
        if not waypoint_file:
            raise RuntimeError('waypoint_file parameter is empty')

        path = Path(waypoint_file)
        if not path.exists():
            raise RuntimeError(f'Waypoint file does not exist: {path}')

        with path.open('r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}

        routes = data.get('routes', {})
        if route_name not in routes:
            available = ', '.join(sorted(routes.keys()))
            raise RuntimeError(f'Route "{route_name}" not found. Available routes: {available}')

        waypoints = routes[route_name].get('waypoints', [])
        if not waypoints:
            raise RuntimeError(f'Route "{route_name}" has no waypoints')

        for waypoint in waypoints:
            for key in ('name', 'x', 'y', 'yaw'):
                if key not in waypoint:
                    raise RuntimeError(f'Waypoint is missing "{key}": {waypoint}')
            state = waypoint.get('state')
            if state and state not in MissionState._value2member_map_:
                available = ', '.join(state.value for state in MissionState)
                raise RuntimeError(
                    f'Waypoint "{waypoint["name"]}" has unknown state "{state}". '
                    f'Available states: {available}'
                )

        return waypoints

    def _load_arm_poses(self, arm_pose_file):
        if not self._arm_pose_enabled or not arm_pose_file:
            return [], {}

        path = Path(arm_pose_file)
        if not path.exists():
            raise RuntimeError(f'FR3 observation pose file does not exist: {path}')

        with path.open('r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}

        joints = data.get('joints', [])
        poses = data.get('poses', {})
        if not joints:
            raise RuntimeError(f'FR3 observation pose file has no joints: {path}')

        poses = self._resolve_mirrored_arm_poses(poses, joints)

        for pose_name, pose in poses.items():
            positions = pose.get('positions', [])
            if len(positions) != len(joints):
                raise RuntimeError(
                    f'FR3 observation pose "{pose_name}" has {len(positions)} positions '
                    f'for {len(joints)} joints.'
                )

            gripper_positions = pose.get('gripper_positions')
            if gripper_positions is not None and len(gripper_positions) != 2:
                raise RuntimeError(
                    f'FR3 observation pose "{pose_name}" has {len(gripper_positions)} '
                    'gripper positions for 2 finger joints.'
                )

            duration_sec = float(pose.get('duration_sec', self._arm_motion_duration_sec))
            if duration_sec <= 0.0:
                raise RuntimeError(
                    f'FR3 observation pose "{pose_name}" duration must be positive.'
                )

        return joints, poses

    @staticmethod
    def _resolve_mirrored_arm_poses(poses, joints):
        default_signs = [-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0]
        resolved = {}
        resolving = set()

        def resolve(pose_name):
            if pose_name in resolved:
                return resolved[pose_name]
            if pose_name not in poses:
                raise RuntimeError(f'FR3 observation pose "{pose_name}" is not configured.')
            if pose_name in resolving:
                raise RuntimeError(f'FR3 observation pose mirror cycle includes "{pose_name}".')

            pose = poses[pose_name] or {}
            source_name = pose.get('mirror_from')
            if not source_name:
                resolved[pose_name] = pose
                return pose

            resolving.add(pose_name)
            source_pose = resolve(str(source_name))
            source_positions = source_pose.get('positions', [])
            if len(source_positions) != len(joints):
                raise RuntimeError(
                    f'FR3 observation pose "{source_name}" has {len(source_positions)} '
                    f'positions for {len(joints)} joints.'
                )

            signs = pose.get('mirror_joint_signs', default_signs)
            if len(signs) != len(joints):
                raise RuntimeError(
                    f'FR3 observation pose "{pose_name}" has {len(signs)} mirror signs '
                    f'for {len(joints)} joints.'
                )

            mirrored_pose = dict(source_pose)
            mirrored_pose.update(pose)
            mirrored_pose['positions'] = [
                float(sign) * float(position)
                for sign, position in zip(signs, source_positions)
            ]
            resolved[pose_name] = mirrored_pose
            resolving.remove(pose_name)
            return mirrored_pose

        for pose_name in poses:
            resolve(pose_name)

        return resolved

    def _handle_start(self, _request, response):
        if self._running:
            response.success = True
            response.message = 'Mission is already running.'
            return response

        if self._autostart_requires_localization and not self._have_localization:
            response.success = False
            response.message = 'Localization is not ready yet.'
            self.get_logger().warn('Start requested before localization became ready.')
            return response

        self._autostart_armed = False
        self._cancel_autostart_delay_timer()
        self._begin_mission('Mission started.')
        response.success = True
        response.message = 'Mission started.'
        return response

    def _handle_stop(self, _request, response):
        self._running = False
        self._set_state(MissionState.STOPPED)
        self._cancel_retry_timer()
        self._cancel_autostart_delay_timer()
        self._cancel_nav_goal_timer()
        self._cancel_direct_turn()
        self._cancel_arm_goal()
        self._cancel_gripper_goal()
        self._cancel_harvest_pick()
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self.get_logger().info('Mission stopped.')
        response.success = True
        response.message = 'Mission stopped.'
        return response

    def _handle_reset(self, _request, response):
        self._running = False
        self._goal_index = 0
        self._set_state(
            MissionState.BOOT if self._have_localization else MissionState.LOCALIZE
        )
        self._cancel_retry_timer()
        self._cancel_autostart_delay_timer()
        self._cancel_nav_goal_timer()
        self._cancel_direct_turn()
        self._cancel_arm_goal()
        self._cancel_gripper_goal()
        self._cancel_harvest_pick()
        self._active_arm_pose = 'none'
        self._active_gripper_positions = None
        self._skipped_arm_poses.clear()
        self._harvest_pick_triggered = False
        self._harvest_pick_attempt_count = 0
        self._harvest_pick_attempted_waypoints.clear()
        self._harvest_pick_selected_basket = None
        self._picked_tomato_models.clear()
        self._failed_harvest_pick_models.clear()
        self._basket_carried_tomatoes.clear()
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self.get_logger().info('Mission reset to the first waypoint.')
        response.success = True
        response.message = 'Mission reset to the first waypoint.'
        return response

    def _handle_amcl_pose(self, msg):
        self._latest_pose = msg.pose.pose

        if self._have_localization:
            return

        self._have_localization = True
        self.get_logger().info('Localization is ready from /amcl_pose.')

        if self._autostart_armed and not self._running:
            self._begin_mission_after_autostart_delay(
                'Autostart started after localization became ready.'
            )
        elif self._state == MissionState.LOCALIZE:
            self._set_state(MissionState.BOOT)

    def _begin_mission_after_autostart_delay(self, log_message):
        if self._autostart_delay_sec <= 0.0:
            self._begin_mission(log_message)
            return
        if self._autostart_delay_timer is not None:
            return
        self.get_logger().info(
            f'Autostart will begin in {self._autostart_delay_sec:.1f}s '
            'to let Nav2 and arm controllers become active.'
        )

        def start_once():
            self._cancel_autostart_delay_timer()
            if self._autostart_armed and not self._running:
                self._begin_mission(log_message)

        self._autostart_delay_timer = self.create_timer(
            self._autostart_delay_sec,
            start_once,
        )

    def _begin_mission(self, log_message):
        self._running = True
        self._autostart_armed = False
        self._skipped_arm_poses.clear()
        self.get_logger().info(log_message)
        self._start_harvest_pick_prewarm()
        self._sync_state_from_waypoint()
        self._send_next_goal()

    def _cancel_retry_timer(self):
        if self._retry_timer is not None:
            self._retry_timer.cancel()
            self._retry_timer = None

    def _cancel_autostart_delay_timer(self):
        if self._autostart_delay_timer is not None:
            self._autostart_delay_timer.cancel()
            self._autostart_delay_timer = None

    def _cancel_nav_goal_timer(self):
        if self._nav_goal_timer is not None:
            self._nav_goal_timer.cancel()
            self._nav_goal_timer = None

    def _cancel_direct_turn(self, publish_stop=True):
        if self._direct_turn_timer is not None:
            self._direct_turn_timer.cancel()
            self._direct_turn_timer = None
        self._direct_turn_start_time = None
        if publish_stop:
            self._publish_turn_velocity(0.0)

    def _start_nav_goal_timer(self):
        self._cancel_nav_goal_timer()
        self._nav_goal_timed_out = False
        if self._nav_goal_timeout_sec <= 0.0:
            return

        self._nav_goal_timer = self.create_timer(
            self._nav_goal_timeout_sec, self._handle_nav_goal_timeout
        )

    def _handle_nav_goal_timeout(self):
        self._cancel_nav_goal_timer()
        if not self._running or self._goal_handle is None:
            return

        self._nav_goal_timed_out = True
        waypoint = self._route[self._goal_index]
        self.get_logger().warn(
            f'Waypoint timed out after {self._nav_goal_timeout_sec:.1f}s: '
            f'{waypoint["name"]}'
        )
        self._goal_handle.cancel_goal_async()

    def _cancel_arm_goal(self):
        if self._arm_goal_handle is not None:
            self._arm_goal_handle.cancel_goal_async()
            self._arm_goal_handle = None
        self._pending_arm_pose = None
        self._pending_arm_pose_index = None

    def _cancel_gripper_goal(self):
        if self._gripper_goal_handle is not None:
            self._gripper_goal_handle.cancel_goal_async()
            self._gripper_goal_handle = None
        self._pending_gripper_pose = None
        self._pending_gripper_pose_index = None
        self._pending_gripper_positions = None

    def _cancel_harvest_pick(self):
        if self._harvest_pick_timer is not None:
            self._harvest_pick_timer.cancel()
            self._harvest_pick_timer = None
        if self._harvest_pick_process is not None and self._harvest_pick_process.poll() is None:
            self.get_logger().warn('Stopping active harvest pick process.')
            self._harvest_pick_process.terminate()
        self._harvest_pick_process = None
        self._harvest_pick_target_pose = None
        self._harvest_pick_selected_record = None
        self._harvest_pick_selected_basket = None
        self._harvest_pick_robot_pose = None
        self._harvest_pick_waypoint_index = None
        self._resume_harvest_pick_after_arm_pose = False

    def _schedule_retry(self, delay_sec=2.0):
        self._cancel_retry_timer()
        self._retry_timer = self.create_timer(delay_sec, self._send_next_goal_once)

    def _send_next_goal_once(self):
        self._cancel_retry_timer()
        if self._running and self._goal_handle is None:
            self._send_next_goal()

    def _send_next_goal(self):
        if not self._running:
            return

        if self._goal_index >= len(self._route):
            if not self._loop_route:
                self._complete_route()
                return
            self._goal_index = 0

        self._sync_state_from_waypoint()
        waypoint = self._route[self._goal_index]
        if not self._prepare_arm_pose_for_waypoint(waypoint):
            return

        if self._mission_mode == 'stationary_harvest':
            if not self._refresh_stationary_waypoint_pose(waypoint):
                self._schedule_retry(1.0)
                return
            self.get_logger().info(
                f'Stationary harvest observation reached without Nav2 goal: '
                f'{waypoint["name"]}'
            )
            self._finish_reached_waypoint(waypoint)
            return

        if self._should_use_direct_turn(waypoint):
            self._start_direct_turn(waypoint)
            return

        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Waiting for Nav2 navigate_to_pose action server...')
            self._schedule_retry(2.0)
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._make_pose(waypoint)

        self.get_logger().info(
            f'Sending waypoint {self._goal_index + 1}/{len(self._route)}: '
            f'{waypoint["name"]} ({waypoint["x"]:.2f}, {waypoint["y"]:.2f}, yaw {waypoint["yaw"]:.2f})'
        )

        future = self._nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_goal_response)

    def _should_use_direct_turn(self, waypoint):
        if not self._use_direct_turn_waypoints or self._goal_index <= 0:
            return False
        if self._latest_pose is None:
            return False

        previous = self._route[self._goal_index - 1]
        same_goal_position = (
            math.hypot(
                float(waypoint['x']) - float(previous['x']),
                float(waypoint['y']) - float(previous['y']),
            )
            <= 0.05
        )
        if not same_goal_position:
            return False

        dx = self._latest_pose.position.x - float(waypoint['x'])
        dy = self._latest_pose.position.y - float(waypoint['y'])
        return math.hypot(dx, dy) <= self._direct_turn_xy_tolerance

    def _start_direct_turn(self, waypoint):
        self._cancel_direct_turn(publish_stop=False)
        self._direct_turn_start_time = self.get_clock().now()
        self.get_logger().info(
            f'Direct turn waypoint {self._goal_index + 1}/{len(self._route)}: '
            f'{waypoint["name"]} yaw {waypoint["yaw"]:.2f}'
        )
        self._direct_turn_timer = self.create_timer(
            1.0 / self._direct_turn_rate_hz, self._update_direct_turn
        )

    def _update_direct_turn(self):
        if not self._running or self._goal_index >= len(self._route):
            self._cancel_direct_turn()
            return

        if self._latest_pose is None:
            self._publish_turn_velocity(0.0)
            return

        waypoint = self._route[self._goal_index]
        current_yaw = self._yaw_from_pose(self._latest_pose)
        yaw_error = self._normalize_angle(float(waypoint['yaw']) - current_yaw)

        if abs(yaw_error) <= self._verify_goal_yaw_tolerance:
            self._cancel_direct_turn()
            self._finish_reached_waypoint(waypoint)
            return

        if self._direct_turn_start_time is not None and self._nav_goal_timeout_sec > 0.0:
            elapsed = (
                self.get_clock().now() - self._direct_turn_start_time
            ).nanoseconds / 1.0e9
            if elapsed > self._nav_goal_timeout_sec:
                self._cancel_direct_turn()
                self.get_logger().warn(
                    f'Direct turn timed out after {self._nav_goal_timeout_sec:.1f}s: '
                    f'{waypoint["name"]}'
                )
                if self._stop_on_failure:
                    self._running = False
                    self._set_state(MissionState.FAILED)
                else:
                    self._goal_index += 1
                    self._send_next_goal()
                return

        angular = self._direct_turn_yaw_gain * yaw_error
        angular = max(
            -self._direct_turn_angular_speed,
            min(self._direct_turn_angular_speed, angular),
        )
        self._publish_turn_velocity(angular)

    def _publish_turn_velocity(self, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.angular.z = float(angular_z)
        self._cmd_vel_pub.publish(msg)

    def _finish_reached_waypoint(self, waypoint):
        self.get_logger().info(f'Waypoint reached: {waypoint["name"]}')
        if self._start_harvest_pick_settle_if_needed(waypoint):
            return
        if self._maybe_start_harvest_pick(waypoint):
            return
        self._goal_index += 1
        self._send_next_goal()

    def _start_harvest_pick_settle_if_needed(self, waypoint):
        waypoint_name = str(waypoint.get('name', ''))
        if not self._harvest_pick_enabled or not self._harvest_pick_auto_select:
            return False
        if not self._is_harvest_pick_trigger_waypoint(waypoint_name):
            return False
        if self._harvest_pick_detection_settle_sec <= 0.0:
            return False
        if self._harvest_pick_settle_timer is not None:
            return True

        self._harvest_pick_settle_waypoint = dict(waypoint)
        self._harvest_pick_settle_waypoint_index = self._goal_index
        self.get_logger().info(
            f'Waiting {self._harvest_pick_detection_settle_sec:.1f}s at {waypoint_name} '
            'for fresh YOLO/depth harvest target before pick.'
        )
        self._harvest_pick_settle_timer = self.create_timer(
            self._harvest_pick_detection_settle_sec,
            self._finish_harvest_pick_settle,
        )
        return True

    def _finish_harvest_pick_settle(self):
        if self._harvest_pick_settle_timer is not None:
            self._harvest_pick_settle_timer.cancel()
            self._harvest_pick_settle_timer = None

        waypoint = self._harvest_pick_settle_waypoint
        waypoint_index = self._harvest_pick_settle_waypoint_index
        self._harvest_pick_settle_waypoint = None
        self._harvest_pick_settle_waypoint_index = None

        if waypoint is None or not self._running:
            return
        if waypoint_index != self._goal_index:
            return
        if self._maybe_start_harvest_pick(waypoint):
            return
        self._goal_index += 1
        self._send_next_goal()

    def _maybe_start_harvest_pick(self, waypoint):
        if self._mission_mode == 'stationary_harvest':
            self._refresh_stationary_waypoint_pose(waypoint)

        waypoint_name = str(waypoint.get('name', ''))
        if not self._is_harvest_pick_trigger_waypoint(waypoint_name):
            return False
        if not self._harvest_pick_enabled:
            self.get_logger().warn(
                f'Harvest pick trigger reached at {waypoint_name}, but harvest_pick_enabled=false. '
                'Set harvest_pick_enabled:=true to start the pick pipeline.'
            )
            return False
        if self._harvest_pick_max_attempts and self._harvest_pick_attempt_count >= self._harvest_pick_max_attempts:
            self.get_logger().info(
                f'Harvest pick trigger reached at {waypoint_name}, but max attempts '
                f'({self._harvest_pick_max_attempts}) is already used.'
            )
            return False
        waypoint_attempts = int(self._harvest_pick_attempted_waypoints.get(waypoint_name, 0))
        if (
            self._harvest_pick_max_per_waypoint
            and waypoint_attempts >= self._harvest_pick_max_per_waypoint
        ):
            return False

        selected_record = None
        if self._harvest_pick_auto_select:
            selected_record = self._select_harvest_pick_target(waypoint)
            if selected_record is not None:
                self._harvest_pick_tomato_model = str(selected_record['model_name'])
                self._harvest_pick_selected_basket = self._basket_for_harvest_record(
                    selected_record
                )
            elif self._harvest_pick_fallback_to_configured and self._harvest_pick_configured_tomato_model:
                self._harvest_pick_tomato_model = self._harvest_pick_configured_tomato_model
                self._harvest_pick_selected_basket = self._harvest_pick_basket
                self.get_logger().warn(
                    'No suitable harvest tomato found in inventory for '
                    f'{waypoint_name}; falling back to configured target '
                    f'{self._harvest_pick_tomato_model}.'
                )
            else:
                self.get_logger().warn(
                    f'No suitable harvest tomato found in inventory for {waypoint_name}; '
                    'continuing route without pick.'
                )
                return False
        else:
            self._harvest_pick_selected_basket = self._harvest_pick_basket

        if not self._harvest_pick_tomato_model:
            self.get_logger().error('harvest_pick_tomato_model is empty; cannot run pick.')
            return False
        if self._latest_pose is None and not self._harvest_pick_use_live_gazebo_pose:
            self.get_logger().error('No AMCL pose available; cannot compute robot Gazebo pose for pick.')
            return False

        try:
            target_pose = self._make_harvest_pick_target_pose(selected_record)
        except Exception as exc:
            self.get_logger().error(f'Cannot prepare harvest pick target: {exc}')
            return False

        self._harvest_pick_selected_record = selected_record
        command = self._make_harvest_pick_command()
        self.get_logger().info(
            f'Starting harvest pick after {waypoint["name"]}: '
            f'{self._harvest_pick_tomato_model} -> {self._current_harvest_pick_basket()} basket'
        )
        self.get_logger().info('Harvest pick command: ' + ' '.join(command))

        self._harvest_pick_target_pose = target_pose
        self._harvest_pick_waypoint_index = self._goal_index
        self._harvest_pick_triggered = True
        self._harvest_pick_attempt_count += 1
        self._harvest_pick_attempted_waypoints[waypoint_name] = waypoint_attempts + 1
        self._publish_harvest_pick_selection(waypoint, selected_record, 'selected')
        if self._harvest_pick_target_exclusion_settle_sec > 0.0:
            time.sleep(self._harvest_pick_target_exclusion_settle_sec)
        self._publish_harvest_pick_target()

        try:
            self._harvest_pick_process = subprocess.Popen(command)
        except OSError as exc:
            self.get_logger().error(f'Failed to start harvest pick process: {exc}')
            self._harvest_pick_process = None
            self._harvest_pick_target_pose = None
            return False

        self._harvest_pick_started_at = time.monotonic()
        self._harvest_pick_timer = self.create_timer(0.2, self._update_harvest_pick)
        return True

    def _is_harvest_pick_trigger_waypoint(self, waypoint_name):
        configured = self._harvest_pick_after_waypoint.strip()
        if configured in ('auto_pick_front', '*_pick_front'):
            return bool(re.search(r'_pick_front$', waypoint_name))
        if configured in ('auto_scan_6', '*_scan_6'):
            return bool(re.search(r'_scan_6$', waypoint_name))
        if configured in ('auto_harvest_scan_6', 'harvest_scan_6'):
            return bool(re.search(r'^plant_\d+_[bc]_scan_6$', waypoint_name))
        return waypoint_name == configured

    def _select_harvest_pick_target(self, waypoint):
        candidates = []
        waypoint_name = str(waypoint.get('name', ''))
        plant_number, plant_side = self._harvest_waypoint_tokens(waypoint_name)

        for record in self._tomato_inventory.values():
            score_record = self._stationary_harvest_record_with_nearest_model(
                record,
                plant_number,
                plant_side,
            )
            candidate = self._score_harvest_candidate(
                score_record,
                waypoint,
                plant_number,
                plant_side,
            )
            if candidate is not None:
                candidates.append(candidate)

        if self._mission_mode == 'stationary_harvest' and plant_number and plant_side:
            for record in self._stationary_harvest_model_fallback_records(
                plant_number,
                plant_side,
            ):
                candidate = self._score_harvest_candidate(
                    record,
                    waypoint,
                    plant_number,
                    plant_side,
                )
                if candidate is not None:
                    candidates.append(candidate)

        if not candidates and plant_number and plant_side:
            fallback_records = self._stationary_harvest_model_fallback_records(
                plant_number,
                plant_side,
            )
            for record in fallback_records:
                candidate = self._score_harvest_candidate(
                    record,
                    waypoint,
                    plant_number,
                    plant_side,
                )
                if candidate is not None:
                    candidates.append(candidate)
            if candidates:
                self.get_logger().warn(
                    f'Harvest auto-select is using same-plant model fallback for '
                    f'{waypoint_name}; YOLO/depth inventory had no fresh candidate.'
                )

        if not candidates:
            self.get_logger().warn(
                f'Harvest auto-select found no candidate near {waypoint_name}. '
                f'inventory_records={len(self._tomato_inventory)}'
            )
            return None

        candidates.sort(key=lambda item: item[0], reverse=True)
        robot_pose = None
        for score, distance_xy, record in candidates:
            try:
                if robot_pose is None:
                    robot_pose = self._current_robot_gazebo_pose()
                pose_info = self._harvest_pick_pose_for_candidate_validation(record, robot_pose)
            except Exception as exc:
                self.get_logger().warn(
                    f'Cannot validate harvest candidate {record.get("model_name", "")}: {exc}'
                )
                pose_info = None

            if pose_info is not None:
                target, source = pose_info
                plausible, reason = self._harvest_pick_target_pose_is_plausible(target)
                if not plausible:
                    self.get_logger().warn(
                        'Skipping harvest candidate with implausible target pose: '
                        f'{record.get("model_name", "")} source={source}: {reason}'
                    )
                    continue

            break
        else:
            self.get_logger().warn(
                f'Harvest auto-select found no candidate with a plausible pick pose near '
                f'{waypoint_name}. inventory_records={len(self._tomato_inventory)}'
            )
            return None

        self.get_logger().info(
            'Harvest auto-selected target: '
            f'{record["model_name"]} class={record.get("tomato_class", "unknown")} '
            f'confidence={float(record.get("confidence", 0.0)):.2f} '
            f'distance_xy={distance_xy:.2f}m score={score:.2f} waypoint={waypoint_name}'
        )
        return record

    @staticmethod
    def _harvest_waypoint_tokens(waypoint_name):
        match = re.search(r'^plant_([0-9]+)_([bc])_(?:scan_[0-9]+|pick_front)$', waypoint_name)
        if not match:
            return None, None
        return str(int(match.group(1))), match.group(2).upper()

    def _score_harvest_candidate(self, record, waypoint, plant_number, plant_side):
        model_name = str(record.get('model_name', '')).strip()
        if self._harvest_pick_require_model_name and not model_name:
            return None
        if model_name in self._picked_tomato_models:
            return None
        if model_name in self._failed_harvest_pick_models:
            return None

        class_aliases = self._harvest_record_class_aliases(record)
        if self._harvest_pick_reject_classes and self._harvest_class_matches(
            class_aliases,
            self._harvest_pick_reject_classes,
        ):
            return None
        if self._harvest_pick_allowed_classes and not self._harvest_class_matches(
            class_aliases,
            self._harvest_pick_allowed_classes,
        ):
            return None

        if plant_side and f'_{plant_side}_' not in model_name:
            return None
        exact_plant_match = False
        if plant_number and plant_side:
            exact_plant_match = f'_{plant_side}_{plant_number}_' in model_name
            if not exact_plant_match:
                return None

        try:
            confidence = float(record.get('confidence', 1.0))
        except (TypeError, ValueError):
            confidence = 0.0
        if confidence < self._harvest_pick_min_confidence:
            return None

        try:
            updated_count = int(record.get('updated_count', 1))
        except (TypeError, ValueError):
            updated_count = 1
        if updated_count < self._harvest_pick_min_updated_count:
            return None

        if self._harvest_pick_inventory_max_age_sec > 0.0:
            try:
                age_sec = time.monotonic() - float(record.get('_last_seen_monotonic', 0.0))
            except (TypeError, ValueError):
                age_sec = math.inf
            if age_sec > self._harvest_pick_inventory_max_age_sec:
                return None

        stationary_local_xyz = None
        if self._mission_mode == 'stationary_harvest' and plant_number is None:
            stationary_local_xyz = self._harvest_record_local_xyz(record)

        if stationary_local_xyz is not None:
            x, y, z = stationary_local_xyz
            map_xyz = None
        else:
            map_xyz = self._harvest_record_map_xyz(record)
            if map_xyz is None:
                return None
            x, y, z = map_xyz
        if not all(math.isfinite(value) for value in (x, y, z)):
            return None
        if z < self._harvest_pick_min_z_m or z > self._harvest_pick_max_z_m:
            return None

        if stationary_local_xyz is not None:
            distance_xy = math.hypot(x, y)
        else:
            distance_xy = math.hypot(x - float(waypoint['x']), y - float(waypoint['y']))
        if distance_xy > self._harvest_pick_local_radius_m:
            return None

        score = 10.0 if exact_plant_match else 0.0
        score += min(updated_count, 30) * 0.02
        score += confidence * 2.0
        score -= distance_xy
        return score, distance_xy, record

    def _stationary_harvest_record_with_nearest_model(
        self,
        record,
        plant_number=None,
        plant_side=None,
    ):
        if self._mission_mode != 'stationary_harvest':
            return record

        local_xyz = self._harvest_record_local_xyz(record)
        if local_xyz is None:
            return record

        corrected_model = self._nearest_harvest_model_to_local_record(
            record,
            local_xyz,
            plant_number,
            plant_side,
        )
        if not corrected_model:
            return record

        current_model = str(record.get('model_name', '')).strip()
        if corrected_model == current_model:
            return record

        corrected = dict(record)
        corrected['mapper_model_name'] = current_model
        corrected['model_name'] = corrected_model
        self.get_logger().info(
            f'Stationary harvest corrected mapper model '
            f'{current_model or "none"} -> {corrected_model} from local YOLO target.'
        )
        return corrected

    def _nearest_harvest_model_to_local_record(
        self,
        record,
        local_xyz,
        plant_number=None,
        plant_side=None,
    ):
        try:
            robot_pose = self._current_robot_gazebo_pose()
        except Exception as exc:
            self.get_logger().warn(
                f'Cannot correct stationary harvest model without robot pose: {exc}'
            )
            return None

        robot_x, robot_y, robot_z, robot_yaw = robot_pose
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        target_x, target_y, target_z = local_xyz
        record_aliases = self._harvest_record_class_aliases(record, include_model=False)
        reject_aliases = self._harvest_config_class_aliases(self._harvest_pick_reject_classes)
        allowed_aliases = self._harvest_config_class_aliases(self._harvest_pick_allowed_classes)
        if reject_aliases and record_aliases.intersection(reject_aliases):
            return None

        best_model = None
        best_score = math.inf
        plant_token = ""
        if plant_number and plant_side:
            plant_token = f"_{plant_side}_{plant_number}_"
        for model_name, pose in self._harvest_pick_model_world_pose_items():
            if model_name in self._picked_tomato_models:
                continue
            if model_name in self._failed_harvest_pick_models:
                continue
            if plant_token and plant_token not in model_name:
                continue
            label = model_name.lower()
            model_aliases = self._harvest_model_class_aliases(model_name)
            if reject_aliases and model_aliases.intersection(reject_aliases):
                continue
            if allowed_aliases and not model_aliases.intersection(allowed_aliases):
                continue
            if record_aliases and not model_aliases.intersection(record_aliases):
                continue

            model_x, model_y, model_z = pose[:3]
            dx = model_x - robot_x
            dy = model_y - robot_y
            local_model_x = cos_yaw * dx + sin_yaw * dy
            local_model_y = -sin_yaw * dx + cos_yaw * dy
            local_model_z = model_z - robot_z

            distance_xy = math.hypot(
                local_model_x - target_x,
                local_model_y - target_y,
            )
            if distance_xy > self._harvest_pick_local_radius_m:
                continue
            distance_z = abs(local_model_z - target_z)
            score = distance_xy + 0.25 * distance_z
            if score < best_score:
                best_score = score
                best_model = model_name

        return best_model

    def _stationary_harvest_model_fallback_records(self, plant_number, plant_side):
        records = []
        plant_token = f'_{plant_side}_{plant_number}_'
        now = time.monotonic()
        for model_name, _pose in self._harvest_pick_model_world_pose_items():
            if plant_token not in model_name:
                continue
            if model_name in self._picked_tomato_models:
                continue
            if model_name in self._failed_harvest_pick_models:
                continue
            class_aliases = self._harvest_model_class_aliases(model_name)
            if self._harvest_pick_reject_classes and self._harvest_class_matches(
                class_aliases,
                self._harvest_pick_reject_classes,
            ):
                continue
            if self._harvest_pick_allowed_classes and not self._harvest_class_matches(
                class_aliases,
                self._harvest_pick_allowed_classes,
            ):
                continue
            map_xyz = self._harvest_model_map_xyz(model_name)
            if map_xyz is None:
                continue
            x, y, z = map_xyz
            records.append({
                'tomato_id': f'model:{model_name}',
                'model_name': model_name,
                'tomato_class': self._harvest_model_primary_class(model_name),
                'confidence': 1.0,
                'updated_count': 1,
                'x': x,
                'y': y,
                'z': z,
                'frame_id': self._goal_frame,
                '_last_seen_monotonic': now,
                'target_source': 'same_plant_model_fallback',
            })
        return records

    def _harvest_pick_model_world_pose_items(self):
        if self._harvest_pick_model_world_poses is None:
            self._harvest_pick_model_world_poses = {}
            try:
                world_path = self._resolve_harvest_pick_world_file()
                root = ET.parse(world_path).getroot()
            except Exception as exc:
                self.get_logger().warn(
                    f'Cannot load harvest tomato model world poses: {exc}'
                )
                return []

            world = root.find('world')
            models = world.findall('model') if world is not None else root.findall('.//model')
            for model in models:
                name = model.get('name', '')
                if not name.lower().startswith('tomato'):
                    continue
                pose = self._parse_sdf_pose(model.findtext('pose', default='0 0 0 0 0 0'))
                self._harvest_pick_model_world_poses[name] = pose

        return self._harvest_pick_model_world_poses.items()

    @staticmethod
    def _harvest_record_local_xyz(record):
        target_frame_id = str(record.get('model_center_target_frame_id', '')).strip()
        if target_frame_id and target_frame_id != 'map':
            try:
                xyz = (
                    float(record['model_center_target_x']),
                    float(record['model_center_target_y']),
                    float(record['model_center_target_z']),
                )
            except (KeyError, TypeError, ValueError):
                xyz = None
            if xyz is not None and all(math.isfinite(value) for value in xyz):
                return xyz

        frame_id = str(record.get('frame_id', '')).strip()
        if not frame_id or frame_id == 'map':
            return None

        try:
            xyz = float(record['x']), float(record['y']), float(record['z'])
        except (KeyError, TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in xyz):
            return None
        return xyz

    def _harvest_record_label(self, record):
        class_name = str(record.get('tomato_class', '')).strip().lower()
        model_name = str(record.get('model_name', '')).strip().lower()
        return f'{class_name} {model_name}'

    def _harvest_record_class_aliases(self, record, include_model=True):
        aliases = self._harvest_class_aliases_from_text(
            record.get('tomato_class', '')
        )
        if include_model:
            aliases.update(
                self._harvest_model_class_aliases(
                    str(record.get('model_name', '')).strip()
                )
            )
        return aliases

    def _harvest_model_class_aliases(self, model_name):
        return self._harvest_class_aliases_from_text(model_name)

    def _harvest_config_class_aliases(self, configured_classes):
        aliases = set()
        for token in configured_classes:
            aliases.update(self._harvest_class_aliases_from_text(token))
        return aliases

    def _harvest_class_matches(self, aliases, configured_classes):
        configured_aliases = self._harvest_config_class_aliases(configured_classes)
        return bool(aliases.intersection(configured_aliases))

    @staticmethod
    def _harvest_class_aliases_from_text(value):
        text = str(value).strip().lower()
        if not text:
            return set()

        tokens = {token for token in re.split(r'[^a-z0-9]+', text) if token}
        aliases = set(tokens)

        has_unripe = any(
            token == 'green'
            or token == 'unripe'
            or token.startswith('unripe')
            for token in tokens
        )
        if has_unripe:
            aliases.update({'green', 'unripe'})
            return aliases

        if (
            'fully_ripened' in text
            or {'fully', 'ripened'}.issubset(tokens)
        ):
            aliases.update({'fully_ripened', 'ripe'})

        if any(
            token == 'ripe'
            or token == 'ripened'
            or token.startswith('ripe')
            for token in tokens
        ):
            aliases.add('ripe')

        if any(token.startswith('rotten') for token in tokens):
            aliases.add('rotten')

        if any(token in ('disease', 'diseased') for token in tokens):
            aliases.update({'disease', 'diseased'})

        return aliases

    def _harvest_model_primary_class(self, model_name):
        aliases = self._harvest_model_class_aliases(model_name)
        if aliases.intersection({'green', 'unripe'}):
            return 'green'
        if aliases.intersection({'disease', 'diseased'}):
            return 'diseased'
        if 'rotten' in aliases:
            return 'rotten'
        if 'ripe' in aliases:
            return 'fully_ripened'
        return 'unknown'

    def _basket_for_harvest_record(self, record):
        class_aliases = self._harvest_record_class_aliases(record)
        if self._harvest_pick_bad_classes and self._harvest_class_matches(
            class_aliases,
            self._harvest_pick_bad_classes,
        ):
            return 'bad'
        if self._harvest_pick_good_classes and self._harvest_class_matches(
            class_aliases,
            self._harvest_pick_good_classes,
        ):
            return 'good'
        return self._current_harvest_pick_basket()

    def _current_harvest_pick_basket(self):
        basket = str(self._harvest_pick_selected_basket or self._harvest_pick_basket).strip()
        return basket if basket in ('good', 'bad') else 'good'

    def _harvest_record_map_xyz(self, record):
        model_name = str(record.get('model_name', '')).strip()
        if model_name:
            model_xyz = self._harvest_model_map_xyz(model_name)
            if model_xyz is not None:
                return model_xyz

        frame_id = str(record.get('frame_id', '')).strip()
        if frame_id not in ('map', self._goal_frame):
            return None

        try:
            return float(record['x']), float(record['y']), float(record['z'])
        except (KeyError, TypeError, ValueError):
            return None

    def _harvest_model_map_xyz(self, model_name):
        if self._harvest_pick_model_map_poses is None:
            self._harvest_pick_model_map_poses = {}
            try:
                world_path = self._resolve_harvest_pick_world_file()
                root = ET.parse(world_path).getroot()
            except Exception as exc:
                self.get_logger().warn(
                    f'Cannot load harvest tomato model map poses: {exc}'
                )
                return None

            world = root.find('world')
            models = world.findall('model') if world is not None else root.findall('.//model')
            for model in models:
                name = model.get('name', '')
                if not name.lower().startswith('tomato'):
                    continue
                pose = self._parse_sdf_pose(model.findtext('pose', default='0 0 0 0 0 0'))
                self._harvest_pick_model_map_poses[name] = self._gazebo_pick_pose_to_map_xyz(
                    pose
                )

        return self._harvest_pick_model_map_poses.get(model_name)

    def _publish_harvest_pick_selection(self, waypoint, selected_record, status):
        if not self._harvest_pick_target_selection_topic:
            return

        payload = {
            'status': status,
            'tomato_model': self._harvest_pick_tomato_model,
            'model_name': self._harvest_pick_tomato_model,
            'detach_topic': f'/detach/{self._harvest_pick_tomato_model}',
            'waypoint': waypoint.get('name', ''),
            'timestamp': time.time(),
        }
        if selected_record:
            for key in (
                'tomato_id',
                'model_name',
                'tomato_class',
                'confidence',
                'x',
                'y',
                'z',
                'frame_id',
                'target_source',
                'raw_surface_x',
                'raw_surface_y',
                'raw_surface_z',
                'raw_surface_frame_id',
                'radius_m',
                'estimated_radius_m',
                'updated_count',
            ):
                if key in selected_record:
                    payload[key] = selected_record[key]

        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        for _ in range(3):
            self._harvest_pick_selection_pub.publish(msg)
            time.sleep(0.02)

    def _make_harvest_pick_target_pose(self, selected_record=None):
        robot_x, robot_y, robot_z, robot_yaw = self._current_robot_gazebo_pose()
        self._harvest_pick_robot_pose = (robot_x, robot_y, robot_z, robot_yaw)

        model_name = self._harvest_pick_model_name_from_record(selected_record)
        if self._harvest_pick_prefer_model_pose and model_name:
            try:
                target = self._harvest_pick_target_pose_from_model_name(
                    self._harvest_pick_robot_pose,
                    model_name,
                    'matched_model_pose_to_base',
                )
                plausible, reason = self._harvest_pick_target_pose_is_plausible(target)
                if plausible:
                    return target
                p = target.pose.position
                self.get_logger().warn(
                    f'Implausible model-center harvest target for {model_name}: '
                    f'frame={target.header.frame_id}, pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}); '
                    f'{reason}; falling back to inventory record.'
                )
            except Exception as exc:
                self.get_logger().warn(
                    f'Could not use model-center harvest target for {model_name}: {exc}; '
                    'falling back to inventory record.'
                )

        record_pose = self._harvest_pick_pose_from_record(selected_record, self._harvest_pick_robot_pose)
        if record_pose is not None:
            target, source = record_pose
            p = target.pose.position
            plausible, reason = self._harvest_pick_target_pose_is_plausible(target)
            if not plausible:
                raise RuntimeError(
                    f'Implausible harvest pick target from inventory for '
                    f'{self._harvest_pick_tomato_model}: source={source}, '
                    f'frame={target.header.frame_id}, '
                    f'pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}); {reason}'
                )
            self.get_logger().info(
                f'Harvest pick target from YOLO/depth inventory record: '
                f'model={self._harvest_pick_tomato_model} source={source}, '
                f'frame={target.header.frame_id}, pos=({p.x:.3f}, {p.y:.3f}, {p.z:.3f})'
            )
            return target

        return self._harvest_pick_target_pose_from_model(
            self._harvest_pick_robot_pose,
            'configured_model_pose_to_base',
        )

    def _harvest_pick_target_pose_is_plausible(self, target):
        p = target.pose.position
        values = (p.x, p.y, p.z)
        if not all(math.isfinite(value) for value in values):
            return False, 'target contains non-finite coordinates'

        frame_id = str(target.header.frame_id).strip()
        if frame_id != self._harvest_pick_base_frame:
            return True, ''

        distance_xy = math.hypot(p.x, p.y)
        max_xy = max(1.50, self._harvest_pick_local_radius_m + 0.25)
        if distance_xy > max_xy:
            return False, f'base-frame xy distance {distance_xy:.2f}m exceeds {max_xy:.2f}m'

        if (
            self._harvest_pick_max_base_lateral_m > 0.0
            and abs(p.y) > self._harvest_pick_max_base_lateral_m
        ):
            return (
                False,
                f'base-frame lateral y {p.y:.2f}m exceeds '
                f'+/-{self._harvest_pick_max_base_lateral_m:.2f}m',
            )

        min_z = 0.02
        max_z = max(1.45, self._harvest_pick_max_z_m)
        if p.z < min_z or p.z > max_z:
            return False, f'base-frame z {p.z:.2f}m outside [{min_z:.2f}, {max_z:.2f}]m'

        return True, ''

    def _harvest_pick_target_pose_from_model(self, robot_pose, source):
        return self._harvest_pick_target_pose_from_model_name(
            robot_pose,
            self._harvest_pick_tomato_model,
            source,
        )

    def _harvest_pick_target_pose_from_model_name(self, robot_pose, model_name, source, log=True):
        if not model_name:
            raise RuntimeError('model_name is empty')
        robot_x, robot_y, robot_z, robot_yaw = robot_pose
        world_path = self._resolve_harvest_pick_world_file()
        model_pose = self._model_pose_for_pick(world_path, model_name)
        x, y, z = model_pose[:3]
        dx = x - robot_x
        dy = y - robot_y
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)

        target = PoseStamped()
        target.header.frame_id = self._harvest_pick_base_frame
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.position.x = cos_yaw * dx + sin_yaw * dy
        target.pose.position.y = -sin_yaw * dx + cos_yaw * dy
        target.pose.position.z = z - robot_z
        target.pose.orientation.w = 1.0
        if log:
            self.get_logger().info(
                f'Harvest pick target from model center: model={model_name} '
                f'source={source}, '
                f'world=({x:.3f}, {y:.3f}, {z:.3f}), '
                f'robot_base=({robot_x:.3f}, {robot_y:.3f}, {robot_z:.3f}, yaw {robot_yaw:.3f}), '
                f'{self._harvest_pick_base_frame}=({target.pose.position.x:.3f}, '
                f'{target.pose.position.y:.3f}, {target.pose.position.z:.3f})'
            )
        return target

    def _harvest_pick_pose_for_candidate_validation(self, record, robot_pose):
        model_name = self._harvest_pick_model_name_from_record(record)
        if self._harvest_pick_prefer_model_pose and model_name:
            try:
                target = self._harvest_pick_target_pose_from_model_name(
                    robot_pose,
                    model_name,
                    'candidate_model_pose_to_base',
                    log=False,
                )
                return target, 'candidate_model_pose_to_base'
            except Exception as exc:
                self.get_logger().warn(
                    f'Cannot validate harvest candidate from model center {model_name}: {exc}; '
                    'checking inventory record instead.'
                )
        return self._harvest_pick_pose_from_record(record, robot_pose)

    def _harvest_pick_model_name_from_record(self, record):
        if isinstance(record, dict):
            model_name = str(record.get('model_name', '')).strip()
            if model_name:
                return model_name
        return str(self._harvest_pick_tomato_model or '').strip()

    def _harvest_pick_pose_from_record(self, record, robot_pose):
        if not record:
            return None

        frame_id = str(record.get('frame_id', '')).strip()
        try:
            x = float(record['x'])
            y = float(record['y'])
            z = float(record['z'])
        except (KeyError, TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in (x, y, z)):
            return None

        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.pose.orientation.w = 1.0

        if frame_id in ('map', self._goal_frame):
            robot_x, robot_y, robot_z, robot_yaw = robot_pose
            gazebo_x = self._harvest_pick_map_y_origin - y
            gazebo_y = x - self._harvest_pick_map_x_offset
            dx = gazebo_x - robot_x
            dy = gazebo_y - robot_y
            cos_yaw = math.cos(robot_yaw)
            sin_yaw = math.sin(robot_yaw)
            target.header.frame_id = self._harvest_pick_base_frame
            target.pose.position.x = cos_yaw * dx + sin_yaw * dy
            target.pose.position.y = -sin_yaw * dx + cos_yaw * dy
            target.pose.position.z = z - robot_z
            return target, 'inventory_map_to_base'

        if frame_id:
            target.header.frame_id = frame_id
            target.pose.position.x = x
            target.pose.position.y = y
            target.pose.position.z = z
            return target, 'inventory_record_frame'

        return None

    def _model_pose_for_pick(self, world_path, model_name):
        if self._harvest_pick_use_live_gazebo_pose:
            live_pose = self._live_gazebo_model_pose(model_name)
            if live_pose is not None:
                return live_pose
            self.get_logger().warn(
                f'Could not read live Gazebo pose for {model_name}; falling back to SDF pose.'
            )
        return self._model_pose_from_sdf(world_path, model_name)

    def _resolve_harvest_pick_world_file(self):
        if not self._harvest_pick_world_file:
            raise RuntimeError('harvest_pick_world_file is empty')

        candidate = Path(self._harvest_pick_world_file).expanduser()
        if candidate.exists():
            return candidate

        source_candidate = (
            Path('/home/yusuf/robot_workspaces/combined_ws/src/combined_robot/worlds')
            / self._harvest_pick_world_file
        )
        if source_candidate.exists():
            return source_candidate

        raise RuntimeError(f'world file does not exist: {self._harvest_pick_world_file}')

    @staticmethod
    def _model_pose_from_sdf(world_path, model_name):
        root = ET.parse(world_path).getroot()
        for model in root.findall('.//model'):
            if model.get('name') != model_name:
                continue
            values = [float(value) for value in model.findtext('pose', default='0 0 0 0 0 0').split()]
            while len(values) < 6:
                values.append(0.0)
            return tuple(values[:6])
        raise RuntimeError(f'model "{model_name}" not found in {world_path}')

    def _start_harvest_pick_prewarm(self):
        if not self._harvest_pick_enabled:
            return
        if not self._harvest_pick_prewarm_gripper_attachments:
            return
        if self._harvest_pick_prewarm_started:
            return

        self._harvest_pick_prewarm_started = True
        self._harvest_pick_prewarm_thread = threading.Thread(
            target=self._prewarm_harvest_pick_gripper_attachments,
            name='harvest_pick_gripper_attach_prewarm',
            daemon=True,
        )
        self._harvest_pick_prewarm_thread.start()

    def _prewarm_harvest_pick_gripper_attachments(self):
        try:
            world_path = self._resolve_harvest_pick_world_file()
            tomato_models = self._harvest_pick_prewarm_models(world_path)
        except Exception as exc:
            self.get_logger().warn(f'Could not prepare gripper attachment prewarm: {exc}')
            return

        if not tomato_models:
            self.get_logger().info('Gripper attachment prewarm skipped: no route tomato models found.')
            return

        robot_id = None
        for _attempt in range(8):
            robot_id = self._gazebo_model_id(self._harvest_pick_robot_model)
            if robot_id is not None:
                break
            time.sleep(1.0)
        if robot_id is None:
            self.get_logger().warn(
                f'Gripper attachment prewarm skipped: Gazebo model '
                f'{self._harvest_pick_robot_model} was not found.'
            )
            return

        self.get_logger().info(
            f'Prewarming gripper attachment plugins for {len(tomato_models)} tomato model(s).'
        )
        ready_count = 0
        for tomato_model in tomato_models:
            if self._ensure_gripper_attachment_plugin(robot_id, tomato_model):
                ready_count += 1
            time.sleep(0.05)
        self.get_logger().info(
            f'Gripper attachment prewarm complete: {ready_count}/{len(tomato_models)} topic(s) ready.'
        )

    def _harvest_pick_prewarm_models(self, world_path):
        route_targets = set()
        for waypoint in self._route:
            match = re.search(r'plant_(\d+)_([bc])_', str(waypoint.get('name', '')), re.I)
            if not match:
                continue
            route_targets.add((int(match.group(1)), match.group(2).upper()))

        configured = (self._harvest_pick_configured_tomato_model or '').strip()
        root = ET.parse(world_path).getroot()
        models = []
        seen = set()
        for model in root.findall('.//model'):
            name = model.get('name') or ''
            if not name.startswith('tomato_'):
                continue
            if configured and name == configured:
                pass
            elif route_targets:
                if not any(
                    re.match(rf'tomato_.*_{row}_{plant_id}_\d+$', name)
                    for plant_id, row in route_targets
                ):
                    continue
            else:
                continue

            label = name.lower()
            if self._harvest_pick_reject_classes and any(
                token in label for token in self._harvest_pick_reject_classes
            ):
                continue
            if self._harvest_pick_allowed_classes and not any(
                token in label for token in self._harvest_pick_allowed_classes
            ):
                continue
            if name in seen:
                continue
            seen.add(name)
            models.append(name)
        return models

    def _gazebo_model_id(self, model_name):
        command = [
            'gz',
            'service',
            '-s',
            f'/world/{self._harvest_pick_world_name}/scene/info',
            '--reqtype',
            'gz.msgs.Empty',
            '--reptype',
            'gz.msgs.Scene',
            '--timeout',
            '3000',
            '--req',
            '',
        ]
        try:
            result = subprocess.run(
                command, check=False, capture_output=True, text=True, timeout=5.0
            )
        except (OSError, subprocess.SubprocessError):
            return None
        if result.returncode != 0:
            return None

        pattern = re.compile(
            r'model\s*\{\s*name:\s*"' + re.escape(model_name) + r'"\s*id:\s*(\d+)',
            re.S,
        )
        match = pattern.search(result.stdout)
        if not match:
            return None
        return int(match.group(1))

    def _gripper_attachment_topic_ready(self, tomato_model):
        topic = f'/gripper_attach/{tomato_model}'
        try:
            result = subprocess.run(
                ['gz', 'topic', '-i', '-t', topic],
                check=False,
                capture_output=True,
                text=True,
                timeout=2.0,
            )
        except (OSError, subprocess.SubprocessError):
            return False
        if result.returncode != 0:
            return False
        info = f'{result.stdout}\n{result.stderr}'
        has_subscriber = 'Subscribers' in info and 'No subscribers' not in info
        has_publisher = 'Publishers' in info and 'No publishers' not in info
        return has_subscriber or has_publisher

    def _ensure_gripper_attachment_plugin(self, robot_id, tomato_model):
        if self._gripper_attachment_topic_ready(tomato_model):
            return True

        attach_topic = f'/gripper_attach/{tomato_model}'
        detach_topic = f'/gripper_detach/{tomato_model}'
        state_topic = f'/gripper_attach_state/{tomato_model}'
        innerxml = (
            f'<parent_link>{self._harvest_pick_gripper_attach_parent_link}</parent_link>'
            f'<child_model>{tomato_model}</child_model>'
            '<child_link>link</child_link>'
            f'<detach_topic>{detach_topic}</detach_topic>'
            f'<attach_topic>{attach_topic}</attach_topic>'
            f'<output_topic>{state_topic}</output_topic>'
        )
        request = (
            f'entity {{ id: {robot_id} name: "{self._harvest_pick_robot_model}" type: MODEL }} '
            'plugins { '
            'name: "gz::sim::systems::DetachableJoint" '
            'filename: "gz-sim-detachable-joint-system" '
            f'innerxml: "{innerxml}" '
            '}'
        )
        command = [
            'gz',
            'service',
            '-s',
            f'/world/{self._harvest_pick_world_name}/entity/system/add',
            '--reqtype',
            'gz.msgs.EntityPlugin_V',
            '--reptype',
            'gz.msgs.Boolean',
            '--timeout',
            '5000',
            '--req',
            request,
        ]
        try:
            result = subprocess.run(
                command, check=False, capture_output=True, text=True, timeout=7.0
            )
        except (OSError, subprocess.SubprocessError) as exc:
            self.get_logger().warn(
                f'Gripper attachment prewarm failed for {tomato_model}: {exc}'
            )
            return False

        output = f'{result.stdout}\n{result.stderr}'.strip()
        if result.returncode != 0 or 'data: true' not in result.stdout:
            self.get_logger().warn(
                f'Gripper attachment prewarm was not accepted for {tomato_model}: '
                f'{output or result.returncode}'
            )
            return self._gripper_attachment_topic_ready(tomato_model)

        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if self._gripper_attachment_topic_ready(tomato_model):
                return True
            time.sleep(0.2)
        return False

    def _live_gazebo_model_pose(self, model_name):
        if not model_name:
            return None

        timed_out = False
        try:
            result = subprocess.run(
                [
                    'gz',
                    'model',
                    '-m',
                    model_name,
                    '-p',
                    '--force-version',
                    '8',
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=max(1.0, self._harvest_pick_gazebo_pose_timeout_sec),
            )
            output = f'{result.stdout}\n{result.stderr}'
            returncode = result.returncode
        except subprocess.TimeoutExpired as exc:
            timed_out = True
            stdout = exc.stdout or ''
            stderr = exc.stderr or ''
            if isinstance(stdout, bytes):
                stdout = stdout.decode(errors='replace')
            if isinstance(stderr, bytes):
                stderr = stderr.decode(errors='replace')
            output = f'{stdout}\n{stderr}'
            returncode = 0 if output else 124
        except (OSError, subprocess.SubprocessError) as exc:
            self.get_logger().warn(f'Could not query Gazebo model pose for {model_name}: {exc}')
            return None

        if returncode != 0:
            self.get_logger().warn(
                f'Gazebo model pose query failed for {model_name}: '
                f'{output.strip() or returncode}'
            )
            return None

        match = re.search(
            r'Pose\s*\[ XYZ \(m\) \] \[ RPY \(rad\) \]:\s*'
            r'\[\s*([^\]]+?)\s*\]\s*'
            r'\[\s*([^\]]+?)\s*\]',
            output,
            re.MULTILINE,
        )
        if not match:
            detail = ' before timeout' if timed_out else ''
            self.get_logger().warn(f'Could not parse Gazebo model pose for {model_name}{detail}.')
            return None

        try:
            xyz = [float(value) for value in match.group(1).split()]
            rpy = [float(value) for value in match.group(2).split()]
        except ValueError:
            self.get_logger().warn(f'Invalid Gazebo model pose values for {model_name}.')
            return None

        if len(xyz) != 3 or len(rpy) != 3:
            self.get_logger().warn(f'Incomplete Gazebo model pose for {model_name}.')
            return None
        return tuple(xyz + rpy)

    def _current_robot_gazebo_pose(self):
        if self._harvest_pick_use_live_gazebo_pose:
            live_pose = self._live_gazebo_model_pose(self._harvest_pick_robot_model)
            if live_pose is not None:
                x, y, z, _roll, _pitch, yaw = live_pose
                return x, y, z + self._harvest_pick_base_z_offset, yaw
            self.get_logger().warn(
                f'Could not read live Gazebo pose for {self._harvest_pick_robot_model}; '
                'falling back to AMCL-derived Gazebo pose.'
            )

        if self._latest_pose is None:
            raise RuntimeError('No AMCL pose available for fallback Gazebo pose conversion.')

        map_x = self._latest_pose.position.x
        map_y = self._latest_pose.position.y
        gazebo_x = self._harvest_pick_map_y_origin - map_y
        gazebo_y = map_x - self._harvest_pick_map_x_offset
        gazebo_yaw = self._normalize_angle(self._yaw_from_pose(self._latest_pose) + math.pi / 2.0)
        return gazebo_x, gazebo_y, self._harvest_pick_robot_z, gazebo_yaw

    def _make_harvest_pick_command(self):
        world_path = str(self._resolve_harvest_pick_world_file())
        if self._harvest_pick_robot_pose is not None:
            robot_x, robot_y, robot_z, robot_yaw = self._harvest_pick_robot_pose
        else:
            robot_x, robot_y, robot_z, robot_yaw = self._current_robot_gazebo_pose()
        place_arg = '--place-in-basket' if self._harvest_pick_place_in_basket else '--no-place-in-basket'
        basket_lock_arg = (
            '--lock-harvested-to-basket'
            if self._harvest_pick_lock_harvested_to_basket
            else '--no-lock-harvested-to-basket'
        )
        command = [
            'ros2',
            'run',
            'combined_robot',
            'greenhouse_nearest_pick_place',
            '--world-file',
            world_path,
            '--basket',
            self._current_harvest_pick_basket(),
            '--profile',
            'empty',
            '--world-name',
            self._harvest_pick_world_name,
            '--robot-x',
            f'{robot_x:.4f}',
            '--robot-y',
            f'{robot_y:.4f}',
            '--robot-z',
            f'{robot_z:.4f}',
            '--robot-yaw',
            f'{robot_yaw:.4f}',
            '--tomato-model',
            self._harvest_pick_tomato_model,
            '--detach-topic',
            f'/detach/{self._harvest_pick_tomato_model}',
            '--pose-stabilization',
            '--carry-pose-stabilization',
            '--gripper-attachment',
            place_arg,
            basket_lock_arg,
            '--freeze-basket-z-offset',
            f'{self._harvest_pick_freeze_basket_z_offset:.3f}',
            '--harvested-tomato-collision-radius',
            f'{self._harvest_pick_harvested_tomato_collision_radius:.3f}',
            '--planning-group',
            'fr3_arm',
            '--base-frame',
            self._harvest_pick_base_frame,
            '--ee-link',
            'fr3_hand_tcp',
            '--tomato-topic',
            self._harvest_pick_target_topic,
            '--skip-pre-detect-pose',
            '--auto-grasp-candidates',
            '--approach-direction-source',
            'tcp_to_target',
            '--grasp-angle-offsets-deg',
            '0,-15,15',
            '--grasp-z-offsets',
            self._harvest_pick_grasp_z_offsets,
            '--max-pick-candidates',
            str(self._harvest_pick_max_candidates_per_target),
            '--grasp-lateral-offsets',
            '0.0',
            '--pick-distance',
            '0.082',
            '--pick-distances',
            '0.082,0.078,0.086',
            '--approach-distance',
            '0.20',
            '--gripper-close-width',
            '0.055',
            '--surface-clearance',
            '0.030',
            '--tcp-front-offset',
            '0.020',
            '--min-pick-distance',
            '0.070',
            '--max-pick-distance',
            '0.090',
            '--no-auto-detach-nearest',
            '--max-target-distance',
            '2.50',
            '--max-approach-base-distance',
            '1.50',
            '--max-pick-base-distance',
            '1.50',
            '--position-tolerance',
            '0.030',
            '--orientation-tolerance',
            '0.20',
            '--post-motion-position-tolerance',
            '0.150',
            '--post-motion-wait-timeout',
            '6.0',
            '--allowed-planning-time',
            '8.0',
            '--num-planning-attempts',
            '6',
            '--replan',
            '--replan-attempts',
            '1',
            '--pick-pipeline',
            'pilz_industrial_motion_planner',
            '--pick-planner-id',
            'LIN',
            '--retreat-pipeline',
            'ompl',
            '--retreat-planner-id',
            'RRTConnectkConfigDefault',
            '--grasp-roll',
            '1.5708',
            '--velocity-scaling',
            '0.18',
            '--acceleration-scaling',
            '0.12',
            '--max-cartesian-speed',
            '0.08',
            '--move-action-timeout',
            f'{self._harvest_pick_timeout_sec:.1f}',
            '--tf-timeout',
            '10.0',
            '--stable-target-samples',
            '5',
            '--target-stability-tolerance',
            '0.03',
            '--tomato-timeout',
            '30.0',
        ]
        if self._current_harvest_pick_radius() is not None:
            command.extend(
                [
                    '--tomato-radius-topic',
                    self._harvest_pick_target_radius_topic,
                    '--use-radius-topic',
                ]
            )
        return command

    def _publish_harvest_pick_target(self):
        if self._harvest_pick_target_pose is None:
            return
        msg = PoseStamped()
        msg.header.frame_id = self._harvest_pick_target_pose.header.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose = self._harvest_pick_target_pose.pose
        self._harvest_pick_target_pub.publish(msg)
        self._publish_harvest_pick_radius()

    def _current_harvest_pick_radius(self):
        record = self._harvest_pick_selected_record
        if not record:
            return None
        for key in ('radius_m', 'estimated_radius_m'):
            try:
                radius = float(record[key])
            except (KeyError, TypeError, ValueError):
                continue
            if math.isfinite(radius) and radius > 0.0:
                return radius
        return None

    def _publish_harvest_pick_radius(self):
        if self._harvest_pick_target_radius_pub is None:
            return
        radius = self._current_harvest_pick_radius()
        if radius is None:
            return
        self._harvest_pick_target_radius_pub.publish(Float32(data=radius))

    def _update_harvest_pick(self):
        self._publish_harvest_pick_target()

        if self._harvest_pick_process is None:
            return

        if (
            self._harvest_pick_timeout_sec > 0.0
            and time.monotonic() - self._harvest_pick_started_at
            > self._harvest_pick_timeout_sec + 30.0
        ):
            self.get_logger().error('Harvest pick process timed out; terminating it.')
            self._harvest_pick_process.terminate()
            self._finish_harvest_pick(return_code=124)
            return

        return_code = self._harvest_pick_process.poll()
        if return_code is None:
            return

        self._finish_harvest_pick(return_code)

    def _finish_harvest_pick(self, return_code):
        if self._harvest_pick_timer is not None:
            self._harvest_pick_timer.cancel()
            self._harvest_pick_timer = None

        waypoint_index = self._harvest_pick_waypoint_index
        completed_model = self._harvest_pick_tomato_model
        if return_code != 0 and waypoint_index is not None and waypoint_index < len(self._route):
            self._publish_harvest_pick_selection(
                self._route[waypoint_index],
                None,
                'failed',
            )
        self._harvest_pick_process = None
        self._harvest_pick_target_pose = None
        self._harvest_pick_selected_record = None
        self._harvest_pick_selected_basket = None
        self._harvest_pick_robot_pose = None
        self._harvest_pick_waypoint_index = None
        self._resume_harvest_pick_after_arm_pose = False

        if return_code == 0:
            self.get_logger().info('Harvest pick completed successfully.')
            if completed_model:
                self._picked_tomato_models.add(completed_model)
                self._failed_harvest_pick_models.discard(completed_model)
                self._tomato_inventory.pop(completed_model, None)
        else:
            self.get_logger().error(f'Harvest pick failed with return code {return_code}.')
            if completed_model and self._harvest_pick_auto_select:
                self._failed_harvest_pick_models.add(completed_model)
                self.get_logger().warn(
                    f'Skipping failed harvest target for this mission: {completed_model}'
                )

        if self._harvest_pick_stop_route_after_attempt:
            self._running = False
            self._set_state(MissionState.DONE if return_code == 0 else MissionState.FAILED)
            return

        if return_code != 0 and self._stop_on_failure:
            self._running = False
            self._set_state(MissionState.FAILED)
            return

        if self._running and waypoint_index == self._goal_index:
            if waypoint_index is not None and waypoint_index < len(self._route):
                if self._mission_mode == 'stationary_harvest':
                    self._refresh_stationary_waypoint_pose(self._route[waypoint_index])
                if self._harvest_return_to_pick_pose_after_attempt:
                    if self._return_to_harvest_pick_pose(
                        self._route[waypoint_index],
                        'successful pick' if return_code == 0 else 'failed pick',
                    ):
                        return
                if self._maybe_start_harvest_pick(self._route[waypoint_index]):
                    return
            self._goal_index += 1
            self._send_next_goal()

    def _return_to_harvest_pick_pose(self, waypoint, reason):
        if not self._should_control_arm():
            return False
        pose_name = waypoint.get('arm_pose')
        if not pose_name:
            return False
        self._active_arm_pose = None
        self._skipped_arm_poses.discard(pose_name)
        self.get_logger().info(
            f'Returning FR3 to harvest pick pose "{pose_name}" after {reason}.'
        )
        returning = not self._prepare_arm_pose_for_waypoint(waypoint)
        if returning:
            self._resume_harvest_pick_after_arm_pose = True
        return returning

    def _prepare_arm_pose_for_waypoint(self, waypoint):
        if not self._should_control_arm():
            return True

        pose_name = waypoint.get('arm_pose')
        if not pose_name or pose_name == self._active_arm_pose:
            return True

        if pose_name in self._skipped_arm_poses:
            return True

        pose = self._arm_poses.get(pose_name)
        if pose is None:
            self.get_logger().warn(f'FR3 observation pose "{pose_name}" is not configured.')
            self._skipped_arm_poses.add(pose_name)
            return True

        if (
            self._pending_arm_pose is not None
            or self._arm_goal_handle is not None
            or self._pending_gripper_pose is not None
            or self._gripper_goal_handle is not None
        ):
            return False

        if self._start_gripper_pose_if_needed(pose_name, pose):
            return False

        if not self._arm_client.wait_for_server(
            timeout_sec=self._arm_action_server_timeout_sec
        ):
            self.get_logger().warn(
                f'Waiting for FR3 arm action server: {self._arm_controller_action}'
            )
            self._skipped_arm_poses.add(pose_name)
            return True

        goal_msg = self._make_arm_pose_goal(pose)
        self._pending_arm_pose = pose_name
        self._pending_arm_pose_index = self._goal_index
        self.get_logger().info(f'Sending FR3 observation pose: {pose_name}')

        future = self._arm_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_arm_goal_response)
        return False

    def _start_gripper_pose_if_needed(self, pose_name, pose):
        gripper_positions = pose.get('gripper_positions')
        if gripper_positions is None:
            return False

        target_positions = tuple(float(position) for position in gripper_positions)
        if self._active_gripper_positions == target_positions:
            return False

        if not self._gripper_client.wait_for_server(
            timeout_sec=self._gripper_action_server_timeout_sec
        ):
            self.get_logger().warn(
                f'Waiting for FR3 gripper action server: {self._gripper_controller_action}'
            )
            if self._stop_on_failure:
                self._running = False
                self._set_state(MissionState.FAILED)
                return True
            return False

        goal_msg = self._make_gripper_goal(target_positions, pose)
        self._pending_gripper_pose = pose_name
        self._pending_gripper_pose_index = self._goal_index
        self._pending_gripper_positions = target_positions
        self.get_logger().info(
            f'Sending FR3 gripper pose for {pose_name}: '
            f'[{target_positions[0]:.3f}, {target_positions[1]:.3f}]'
        )

        future = self._gripper_client.send_goal_async(goal_msg)
        future.add_done_callback(self._on_gripper_goal_response)
        return True

    def _should_control_arm(self):
        return (
            self._mission_mode == 'survey_harvest'
            and self._arm_pose_enabled
            and bool(self._arm_joints)
            and bool(self._arm_poses)
        )

    def _make_arm_pose_goal(self, pose):
        if self._arm_motion_mode == 'moveit':
            goal_msg = MoveGroup.Goal()
            goal_msg.request = self._make_moveit_arm_pose_request(pose)
            goal_msg.planning_options.plan_only = False
            goal_msg.planning_options.look_around = False
            goal_msg.planning_options.replan = True
            goal_msg.planning_options.replan_attempts = 2
            goal_msg.planning_options.planning_scene_diff.is_diff = True
            goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True
            self._add_arm_tomato_collision_diff(goal_msg)
            return goal_msg

        duration_sec = float(pose.get('duration_sec', self._arm_motion_duration_sec))

        point = JointTrajectoryPoint()
        point.positions = [float(position) for position in pose['positions']]
        point.time_from_start = Duration(seconds=duration_sec).to_msg()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = list(self._arm_joints)
        goal_msg.trajectory.points = [point]
        return goal_msg

    def _make_moveit_arm_pose_request(self, pose):
        req = MotionPlanRequest()
        req.pipeline_id = self._moveit_pipeline_id
        req.planner_id = self._moveit_planner_id
        req.group_name = self._moveit_planning_group
        req.num_planning_attempts = self._moveit_num_planning_attempts
        req.allowed_planning_time = self._moveit_allowed_planning_time_sec
        req.max_velocity_scaling_factor = self._moveit_velocity_scaling
        req.max_acceleration_scaling_factor = self._moveit_acceleration_scaling
        req.start_state = self._current_robot_state()
        req.goal_constraints.append(self._joint_goal_constraints(pose))
        return req

    def _add_arm_tomato_collision_diff(self, goal_msg):
        tomato_objects = self._arm_tomato_collision_objects_for_moveit()
        if not tomato_objects:
            return

        stamp = self.get_clock().now().to_msg()
        for collision_object in tomato_objects:
            collision_object.header.stamp = stamp
        goal_msg.planning_options.planning_scene_diff.world.collision_objects.extend(
            tomato_objects
        )

        if not self._logged_arm_tomato_collision_objects:
            self.get_logger().info(
                f'Adding {len(tomato_objects)} tomato collision sphere(s) to FR3 '
                'observation MoveIt plans.'
            )
            self._logged_arm_tomato_collision_objects = True

    def _arm_tomato_collision_objects_for_moveit(self):
        if not self._arm_avoid_tomatoes:
            return []

        tomato_map_poses = self._arm_tomato_map_poses_for_moveit()
        if not tomato_map_poses:
            return []
        if self._latest_pose is None:
            self.get_logger().warn(
                'No AMCL pose available; skipping tomato collision spheres for this arm plan.'
            )
            return []

        return [
            self._tomato_collision_object(
                model_name,
                *self._map_xyz_to_base_link(map_x, map_y, map_z),
                frame_id='base_link',
            )
            for model_name, map_x, map_y, map_z in tomato_map_poses
        ]

    def _arm_tomato_map_poses_for_moveit(self):
        if self._arm_tomato_map_poses is not None:
            return self._arm_tomato_map_poses

        try:
            world_path = self._resolve_harvest_pick_world_file()
        except Exception as exc:
            self.get_logger().warn(
                f'Cannot load tomato collision objects for arm planning: {exc}'
            )
            self._arm_tomato_map_poses = []
            return self._arm_tomato_map_poses

        self._arm_tomato_map_poses = self._load_arm_tomato_map_poses(world_path)
        if not self._arm_tomato_map_poses:
            self.get_logger().warn(
                f'No tomato models found for arm collision avoidance in {world_path}'
            )
        return self._arm_tomato_map_poses

    def _load_arm_tomato_map_poses(self, world_path):
        root = ET.parse(world_path).getroot()
        world = root.find('world')
        models = world.findall('model') if world is not None else root.findall('.//model')
        tomato_map_poses = []

        for model in models:
            name = model.get('name', '')
            if not name.lower().startswith('tomato'):
                continue

            pose = self._parse_sdf_pose(model.findtext('pose', default='0 0 0 0 0 0'))
            map_x, map_y, map_z = self._gazebo_pose_to_map_xyz(pose)
            tomato_map_poses.append((name, map_x, map_y, map_z))

        return tomato_map_poses

    @staticmethod
    def _parse_sdf_pose(pose_text):
        values = [float(value) for value in pose_text.split()]
        while len(values) < 6:
            values.append(0.0)
        return tuple(values[:6])

    def _gazebo_pose_to_map_xyz(self, pose):
        x, y, z, _roll, _pitch, _yaw = pose
        return (
            y + self._harvest_pick_map_x_offset,
            self._harvest_pick_map_y_origin - x,
            z + self._arm_avoid_tomato_z_offset_m,
        )

    def _gazebo_pick_pose_to_map_xyz(self, pose):
        x, y, z, _roll, _pitch, _yaw = pose
        return (
            y + self._harvest_pick_map_x_offset,
            self._harvest_pick_map_y_origin - x,
            z,
        )

    def _map_xyz_to_base_link(self, map_x, map_y, map_z):
        robot_x = self._latest_pose.position.x
        robot_y = self._latest_pose.position.y
        robot_yaw = self._yaw_from_pose(self._latest_pose)
        dx = map_x - robot_x
        dy = map_y - robot_y
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        return (
            cos_yaw * dx + sin_yaw * dy,
            -sin_yaw * dx + cos_yaw * dy,
            map_z,
        )

    def _tomato_collision_object(self, model_name, x, y, z, frame_id):
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [self._arm_avoid_tomato_radius_m]

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        collision_object = CollisionObject()
        collision_object.header.frame_id = frame_id
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = model_name
        collision_object.operation = CollisionObject.ADD
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        return collision_object

    def _current_robot_state(self):
        state = RobotState()
        state.is_diff = True
        if self._joint_state is not None:
            state.joint_state.name = list(self._joint_state.name)
            state.joint_state.position = [
                self._sanitized_start_state_position(name, position)
                for name, position in zip(
                    self._joint_state.name,
                    self._joint_state.position,
                )
            ]
            state.joint_state.velocity = list(self._joint_state.velocity)
            state.joint_state.effort = list(self._joint_state.effort)
        return state

    def _sanitized_start_state_position(self, joint_name, position):
        try:
            value = float(position)
        except (TypeError, ValueError):
            return position
        if not math.isfinite(value):
            return position

        limits = self._FR3_JOINT_LIMITS.get(str(joint_name))
        if limits is None:
            return value

        lower, upper = limits
        margin = self._MOVEIT_START_STATE_LIMIT_MARGIN
        safe_lower = lower + margin
        safe_upper = upper - margin
        if value < safe_lower:
            return safe_lower
        if value > safe_upper:
            return safe_upper
        return value

    def _joint_goal_constraints(self, pose):
        constraints = Constraints()
        constraints.name = f'{self._arm_motion_mode}:{pose.get("name", "arm_pose")}'
        for joint_name, position in zip(self._arm_joints, pose['positions']):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(position)
            joint_constraint.tolerance_above = self._moveit_joint_tolerance_rad
            joint_constraint.tolerance_below = self._moveit_joint_tolerance_rad
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        return constraints

    def _make_gripper_goal(self, target_positions, pose):
        duration_sec = float(pose.get('gripper_duration_sec', 1.0))

        point = JointTrajectoryPoint()
        point.positions = list(target_positions)
        point.time_from_start = Duration(seconds=duration_sec).to_msg()

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
            'fr3_finger_joint1',
            'fr3_finger_joint2',
        ]
        goal_msg.trajectory.points = [point]
        return goal_msg

    def _sync_state_from_waypoint(self):
        if (
            self._mission_mode not in ('survey_harvest', 'stationary_harvest')
            or self._goal_index >= len(self._route)
        ):
            return

        state = self._route[self._goal_index].get('state')
        if state:
            self._set_state(state)

    def _complete_route(self):
        if self._mission_mode == 'stationary_harvest':
            self._running = False
            self._set_state(MissionState.DONE)
            self.get_logger().info('Stationary harvest mission complete.')
            return

        if self._mission_mode != 'survey_harvest':
            self._running = False
            self.get_logger().info('Mission complete.')
            return

        self._set_state(MissionState.PLAN_HARVEST)
        self._plan_harvest()

    def _plan_harvest(self):
        if not self._harvest_targets:
            self._running = False
            self._set_state(MissionState.DONE)
            self.get_logger().info(
                'Survey complete. Fruit memory is not connected yet, so there are no harvest targets.'
            )
            return

        self._set_state(MissionState.HARVESTING)
        self.get_logger().warn('Harvest target execution is not implemented yet.')
        self._running = False

    def _make_pose(self, waypoint):
        pose = PoseStamped()
        pose.header.frame_id = self._goal_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(waypoint['x'])
        pose.pose.position.y = float(waypoint['y'])
        pose.pose.position.z = 0.0

        yaw = float(waypoint['yaw'])
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def _is_waypoint_reached(self, waypoint):
        if not self._verify_goal_pose:
            return True

        pose_error = self._get_waypoint_pose_error(waypoint)
        if pose_error is None:
            return False

        distance, yaw_error = pose_error

        if (
            distance <= self._verify_goal_xy_tolerance
            and yaw_error <= self._verify_goal_yaw_tolerance
        ):
            return True

        self.get_logger().warn(
            f'Strict waypoint check failed for {waypoint["name"]}: '
            f'distance={distance:.3f}m/{self._verify_goal_xy_tolerance:.3f}m, '
            f'yaw_error={yaw_error:.3f}rad/{self._verify_goal_yaw_tolerance:.3f}rad.'
        )
        return False

    def _get_waypoint_pose_error(self, waypoint):
        if self._latest_pose is None:
            self.get_logger().warn('No /amcl_pose sample available for strict waypoint check.')
            return None

        dx = self._latest_pose.position.x - float(waypoint['x'])
        dy = self._latest_pose.position.y - float(waypoint['y'])
        distance = math.hypot(dx, dy)

        current_yaw = self._yaw_from_pose(self._latest_pose)
        yaw_error = abs(self._normalize_angle(current_yaw - float(waypoint['yaw'])))
        return distance, yaw_error

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _yaw_from_pose(pose):
        orientation = pose.orientation
        return math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._goal_handle = None
            self.get_logger().warn('Waypoint goal was rejected by Nav2.')
            self._schedule_retry(2.0)
            return

        self._goal_handle = goal_handle
        self._start_nav_goal_timer()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_arm_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warn(f'FR3 observation pose goal failed to send: {exc}')
            self._finish_arm_pose_attempt(False)
            return

        if not goal_handle.accepted:
            self.get_logger().warn(
                f'FR3 observation pose was rejected: {self._pending_arm_pose}'
            )
            self._finish_arm_pose_attempt(False)
            return

        self._arm_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_arm_goal_result)

    def _on_arm_goal_result(self, future):
        result = future.result()
        succeeded = result.status == 4
        if succeeded:
            self.get_logger().info(f'FR3 observation pose reached: {self._pending_arm_pose}')
        else:
            self.get_logger().warn(
                f'FR3 observation pose failed with action status: {result.status}'
            )
        self._finish_arm_pose_attempt(succeeded)

    def _finish_arm_pose_attempt(self, succeeded):
        pose_name = self._pending_arm_pose
        pose_index = self._pending_arm_pose_index
        self._arm_goal_handle = None
        self._pending_arm_pose = None
        self._pending_arm_pose_index = None

        if pose_name:
            if succeeded:
                self._active_arm_pose = pose_name
            else:
                self._skipped_arm_poses.add(pose_name)

        if (
            self._resume_harvest_pick_after_arm_pose
            and self._running
            and self._goal_handle is None
            and pose_index == self._goal_index
        ):
            self._resume_harvest_pick_after_arm_pose = False
            if succeeded and pose_index is not None and pose_index < len(self._route):
                waypoint = self._route[pose_index]
                if self._start_harvest_pick_settle_if_needed(waypoint):
                    return
                if self._maybe_start_harvest_pick(waypoint):
                    return
            self._goal_index += 1
            self._send_next_goal()
            return

        if self._running and self._goal_handle is None and pose_index == self._goal_index:
            self._send_next_goal()

    def _on_gripper_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().warn(f'FR3 gripper pose goal failed to send: {exc}')
            self._finish_gripper_pose_attempt(False)
            return

        if not goal_handle.accepted:
            self.get_logger().warn(
                f'FR3 gripper pose was rejected: {self._pending_gripper_pose}'
            )
            self._finish_gripper_pose_attempt(False)
            return

        self._gripper_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_gripper_goal_result)

    def _on_gripper_goal_result(self, future):
        result = future.result()
        succeeded = result.status == 4
        if succeeded:
            self.get_logger().info(f'FR3 gripper pose reached: {self._pending_gripper_pose}')
        else:
            self.get_logger().warn(
                f'FR3 gripper pose failed with action status: {result.status}'
            )
        self._finish_gripper_pose_attempt(succeeded)

    def _finish_gripper_pose_attempt(self, succeeded):
        pose_index = self._pending_gripper_pose_index
        target_positions = self._pending_gripper_positions
        self._gripper_goal_handle = None
        self._pending_gripper_pose = None
        self._pending_gripper_pose_index = None
        self._pending_gripper_positions = None

        if succeeded and target_positions is not None:
            self._active_gripper_positions = target_positions

        if not succeeded and self._stop_on_failure:
            self._running = False
            self._set_state(MissionState.FAILED)
            return

        if self._running and self._goal_handle is None and pose_index == self._goal_index:
            self._send_next_goal()

    def _on_goal_result(self, future):
        result = future.result()
        self._goal_handle = None
        self._cancel_nav_goal_timer()

        if result.status == 4:
            self._nav_goal_timed_out = False
            waypoint = self._route[self._goal_index]
            if not self._is_waypoint_reached(waypoint):
                pose_error = self._get_waypoint_pose_error(waypoint)
                if pose_error is not None:
                    distance, yaw_error = pose_error
                    if (
                        self._use_direct_turn_waypoints
                        and distance <= self._verify_goal_xy_tolerance
                        and yaw_error > self._verify_goal_yaw_tolerance
                    ):
                        self.get_logger().info(
                            f'Position reached for {waypoint["name"]}; '
                            f'correcting yaw directly ({yaw_error:.3f}rad).'
                        )
                        self._start_direct_turn(waypoint)
                        return

                self.get_logger().warn(
                    f'Nav2 reported success, but pose is outside strict tolerance for '
                    f'{waypoint["name"]}. Retrying same waypoint.'
                )
                self._schedule_retry(0.5)
                return

            self._finish_reached_waypoint(waypoint)
            return

        failure_reason = 'timed out' if self._nav_goal_timed_out else 'failed'
        self._nav_goal_timed_out = False
        self.get_logger().warn(
            f'Waypoint {failure_reason} with action status: {result.status}'
        )
        if self._stop_on_failure:
            self._running = False
            self._set_state(MissionState.FAILED)
        else:
            self._goal_index += 1
            self._send_next_goal()

    def _publish_status(self):
        current = 'none'
        view = 'none'
        target_arm_pose = 'none'
        if self._goal_index < len(self._route):
            waypoint = self._route[self._goal_index]
            current = waypoint['name']
            view = waypoint.get('view', 'none')
            target_arm_pose = waypoint.get('arm_pose', 'none')

        msg = String()
        msg.data = (
            f'running={self._running}; mode={self._mission_mode}; state={self._state.value}; '
            f'route={self._route_name}; view={view}; arm_pose={self._active_arm_pose}; '
            f'target_arm_pose={target_arm_pose}; '
            f'index={self._goal_index}/{len(self._route)}; current={current}'
        )
        self._status_pub.publish(msg)

    def _refresh_stationary_waypoint_pose(self, waypoint):
        try:
            robot_x, robot_y, _robot_z, robot_yaw = self._current_robot_gazebo_pose()
            waypoint['x'] = robot_y + self._harvest_pick_map_x_offset
            waypoint['y'] = self._harvest_pick_map_y_origin - robot_x
            waypoint['yaw'] = self._normalize_angle(robot_yaw - math.pi / 2.0)
            source = 'live_gazebo'
        except Exception as exc:
            if self._latest_pose is None:
                self.get_logger().warn(
                    f'Stationary harvest could not read current robot pose: {exc}'
                )
                return False
            waypoint['x'] = self._latest_pose.position.x
            waypoint['y'] = self._latest_pose.position.y
            waypoint['yaw'] = self._yaw_from_pose(self._latest_pose)
            source = 'amcl'

        now = time.monotonic()
        if now - self._stationary_pose_log_time > 2.0:
            self._stationary_pose_log_time = now
            self.get_logger().info(
                f'Stationary harvest pose source={source}: '
                f'{waypoint["name"]} map=({float(waypoint["x"]):.2f}, '
                f'{float(waypoint["y"]):.2f}, yaw {float(waypoint["yaw"]):.2f})'
            )
        return True


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
