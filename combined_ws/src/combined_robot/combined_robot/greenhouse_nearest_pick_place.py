"""Two-stage tomato pick pipeline for the FR3 greenhouse setup.

Arm sequence:
  1. OMPL plans and executes only to a safe approach pose.
  2. Pilz executes a LIN move from approach pose to pick pose.
  3. The gripper closes.
  4. Pilz executes a LIN retreat back to the approach pose when possible.
     If the tomato is already attached and basket placement is enabled, a
     retreat execution failure does not block the basket placement attempt.
"""

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time
from xml.etree import ElementTree as ET

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    CollisionObject,
    Constraints,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
)
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.time import Time as RclTime
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float32, String
from tf2_ros import TransformException

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

from combined_robot.pick_place_detachable import (
    ARM_JOINT_NAMES,
    BASKET_SETTLE_POSES,
    DEFAULT_WORLD_NAME,
    DetachablePickPlace,
    GRIPPER_OPEN_TOTAL_WIDTH,
    POSE_PROFILES,
)


DEFAULT_PLANNING_GROUP = "fr3_arm"
DEFAULT_BASE_FRAME = "fr3_link0"
DEFAULT_EE_LINK = "fr3_hand_tcp"
DEFAULT_TOMATO_TOPIC = "/tomato_center"
DEFAULT_TOMATO_RADIUS_TOPIC = "/tomato_radius"
DEFAULT_PRE_DETECT_JOINTS = "-1.710,-1.267,0.0,-2.792,0.0,2.800,0.0"
PILZ_PIPELINE = "pilz_industrial_motion_planner"
DEFAULT_HARVESTED_TOMATO_COLLISION_RADIUS = 0.038

BASKET_SLOT_OFFSETS = (
    (-0.10, -0.09, 0.090),
    (-0.02, -0.09, 0.090),
    (0.08, -0.09, 0.090),
    (-0.10, 0.00, 0.090),
    (-0.02, 0.00, 0.090),
    (0.08, 0.00, 0.090),
    (-0.10, 0.09, 0.090),
    (-0.02, 0.09, 0.090),
    (0.08, 0.09, 0.090),
    (-0.06, -0.045, 0.165),
    (0.04, -0.045, 0.165),
    (-0.06, 0.045, 0.165),
    (0.04, 0.045, 0.165),
)


def _q_tuple(q):
    return (q.x, q.y, q.z, q.w)


def _normalize_quaternion(q):
    x, y, z, w = _q_tuple(q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    return Quaternion(x=x / n, y=y / n, z=z / n, w=w / n)


def _quat_multiply(a, b):
    ax, ay, az, aw = _q_tuple(a)
    bx, by, bz, bw = _q_tuple(b)
    return Quaternion(
        x=aw * bx + ax * bw + ay * bz - az * by,
        y=aw * by - ax * bz + ay * bw + az * bx,
        z=aw * bz + ax * by - ay * bx + az * bw,
        w=aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_conjugate(q):
    return Quaternion(x=-q.x, y=-q.y, z=-q.z, w=q.w)


def _axis_angle_quaternion(axis, angle):
    axis = _normalize_vector(axis)
    half = angle * 0.5
    scale = math.sin(half)
    return Quaternion(
        x=axis[0] * scale,
        y=axis[1] * scale,
        z=axis[2] * scale,
        w=math.cos(half),
    )


def _rotate_vector(q, v):
    vq = Quaternion(x=v[0], y=v[1], z=v[2], w=0.0)
    rq = _quat_multiply(_quat_multiply(q, vq), _quat_conjugate(q))
    return (rq.x, rq.y, rq.z)


def _normalize_vector(v):
    n = math.sqrt(sum(component * component for component in v))
    if n < 1e-9:
        raise RuntimeError("zero-length approach direction")
    return tuple(component / n for component in v)


def _matrix_to_quaternion(m):
    m00, m01, m02 = m[0]
    m10, m11, m12 = m[1]
    m20, m21, m22 = m[2]
    trace = m00 + m11 + m22

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = Quaternion(
            w=0.25 * s,
            x=(m21 - m12) / s,
            y=(m02 - m20) / s,
            z=(m10 - m01) / s,
        )
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        q = Quaternion(
            w=(m21 - m12) / s,
            x=0.25 * s,
            y=(m01 + m10) / s,
            z=(m02 + m20) / s,
        )
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        q = Quaternion(
            w=(m02 - m20) / s,
            x=(m01 + m10) / s,
            y=0.25 * s,
            z=(m12 + m21) / s,
        )
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        q = Quaternion(
            w=(m10 - m01) / s,
            x=(m02 + m20) / s,
            y=(m12 + m21) / s,
            z=0.25 * s,
        )
    return _normalize_quaternion(q)


def _look_at_orientation(approach_direction, grasp_roll):
    """Return an EE orientation whose local +Z axis points at the tomato."""
    z_axis = _normalize_vector(approach_direction)
    up = (0.0, 0.0, 1.0)
    x_axis = (
        up[1] * z_axis[2] - up[2] * z_axis[1],
        up[2] * z_axis[0] - up[0] * z_axis[2],
        up[0] * z_axis[1] - up[1] * z_axis[0],
    )
    if math.sqrt(sum(component * component for component in x_axis)) < 1e-6:
        up = (0.0, 1.0, 0.0)
        x_axis = (
            up[1] * z_axis[2] - up[2] * z_axis[1],
            up[2] * z_axis[0] - up[0] * z_axis[2],
            up[0] * z_axis[1] - up[1] * z_axis[0],
        )
    x_axis = _normalize_vector(x_axis)
    y_axis = (
        z_axis[1] * x_axis[2] - z_axis[2] * x_axis[1],
        z_axis[2] * x_axis[0] - z_axis[0] * x_axis[2],
        z_axis[0] * x_axis[1] - z_axis[1] * x_axis[0],
    )

    look_at = _matrix_to_quaternion(
        (
            (x_axis[0], y_axis[0], z_axis[0]),
            (x_axis[1], y_axis[1], z_axis[1]),
            (x_axis[2], y_axis[2], z_axis[2]),
        )
    )
    return _normalize_quaternion(
        _quat_multiply(
            look_at,
            _axis_angle_quaternion((0.0, 0.0, 1.0), grasp_roll),
        )
    )


def _make_pose(x, y, z, orientation):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation = _normalize_quaternion(orientation)
    return pose


def _parse_joint_positions(text):
    value = (text or "").strip()
    if not value or value.lower() in ("none", "off", "false"):
        return None

    parts = [part.strip() for part in value.replace(";", ",").split(",")]
    joints = [float(part) for part in parts if part]
    if len(joints) != len(ARM_JOINT_NAMES):
        raise ValueError(
            f"expected {len(ARM_JOINT_NAMES)} joint values, got {len(joints)}"
        )
    return joints


def _parse_float_list(text, default_values):
    value = (text or "").strip()
    if not value or value.lower() in ("none", "off", "false"):
        return list(default_values)
    parts = [part.strip() for part in value.replace(";", ",").split(",")]
    return [float(part) for part in parts if part]


def _parse_sdf_pose_xyz(text):
    parts = [float(part) for part in (text or "").split()]
    if len(parts) < 3:
        return None
    return (parts[0], parts[1], parts[2])


def _parse_sdf_float(text):
    value = (text or "").strip()
    if not value:
        return None
    return float(value)


class GreenhouseNearestPickPlace(DetachablePickPlace):
    def __init__(self, args):
        detach_topic = args.detach_topic
        if not detach_topic and args.tomato_model:
            detach_topic = f"/detach/{args.tomato_model}"

        super().__init__(
            args.basket,
            detach_topic,
            args.skip_detach,
            args.pose_stabilization,
            args.world_name,
            args.tomato_model or "tomato_center_topic_target",
            args.profile,
            args.robot_x,
            args.robot_y,
            args.robot_z,
            args.robot_yaw,
        )
        self.args = args
        self.robot_base_frame = args.base_frame
        self.robot_tcp_link = args.ee_link
        self.enable_gripper_attachment = args.gripper_attachment
        self._detachable_tomatoes = None
        self._fixed_approach_direction = None
        self._joint_state = None
        self._last_tomato_pose = None
        self._last_tomato_radius = None
        self._last_tomato_seq = 0
        self._last_tomato_log_time = 0.0
        self._last_radius_log_time = 0.0
        self._nearby_tomato_collision_objects = None
        self._logged_nearby_tomato_collisions = False
        self._harvest_source_model = self.tomato_model
        self._visual_source_model = self.tomato_model
        self._carried_proxy_model = None
        self._last_carry_update_time = 0.0

        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)
        self.create_subscription(
            PoseStamped, self.args.tomato_topic, self._tomato_pose_cb, 10
        )
        self.create_subscription(
            Float32, self.args.tomato_radius_topic, self._tomato_radius_cb, 10
        )
        self._picked_pub = self.create_publisher(String, self.args.picked_topic, 10)
        self.move_group_client = ActionClient(self, MoveGroup, self.args.move_action)

    def _joint_state_cb(self, msg):
        self._joint_state = msg

    def _tomato_pose_cb(self, msg):
        self._last_tomato_pose = msg
        self._last_tomato_seq += 1
        now = time.monotonic()
        if now - self._last_tomato_log_time >= 2.0:
            self._last_tomato_log_time = now
            self.get_logger().info(
                "tomato_center received: "
                f"frame={msg.header.frame_id or '<empty>'} "
                f"pos=({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, "
                f"{msg.pose.position.z:.3f})"
            )

    def _tomato_radius_cb(self, msg):
        self._last_tomato_radius = float(msg.data)
        now = time.monotonic()
        if now - self._last_radius_log_time >= 2.0:
            self._last_radius_log_time = now
            self.get_logger().info(f"tomato_radius received: {self._last_tomato_radius:.3f} m")

    def _wait_for_servers(self):
        super()._wait_for_servers()
        self.get_logger().info(f"Waiting for MoveIt action server {self.args.move_action}...")
        if not self.move_group_client.wait_for_server(timeout_sec=60.0):
            raise RuntimeError(f"MoveIt action server is not available: {self.args.move_action}")

    def _wait_for_joint_state(self):
        deadline = time.monotonic() + self.args.joint_state_timeout
        while rclpy.ok() and self._joint_state is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._joint_state is None:
            raise RuntimeError("No /joint_states received; refusing to move")

    def _reset_tomato_observation(self):
        self._last_tomato_pose = None
        self._last_tomato_radius = None

    def _settle_observation_stream(self):
        if self.args.observation_settle <= 0.0:
            return

        self.get_logger().info(
            f"Settling camera stream for {self.args.observation_settle:.1f}s before tomato detection"
        )
        deadline = time.monotonic() + self.args.observation_settle
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _move_to_pre_detect_pose(self):
        if self.args.skip_pre_detect_pose:
            self.get_logger().info("Skipping pre-detect observation pose by request")
            return

        joints = self._pre_detect_joint_positions
        if joints is None:
            self.get_logger().info("No pre-detect observation joints configured")
            return

        self.get_logger().info(
            "Moving arm to pre-detect observation pose so the wrist camera sees tomatoes: "
            + ", ".join(f"{name}={value:.3f}" for name, value in zip(ARM_JOINT_NAMES, joints))
        )
        self._send_trajectory(
            self.arm_client,
            ARM_JOINT_NAMES,
            [joints],
            self.args.pre_detect_duration,
            "pre-detect observation pose",
        )
        self._settle_observation_stream()
        self._reset_tomato_observation()

    def _wait_for_tomato_pose(self):
        self.get_logger().info(
            f"Waiting for tomato pose on {self.args.tomato_topic} "
            f"for {self.args.tomato_timeout:.1f}s"
        )
        deadline = time.monotonic() + self.args.tomato_timeout
        while rclpy.ok() and self._last_tomato_pose is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self._last_tomato_pose is None:
            self.get_logger().error("No tomato_center pose received; no arm motion will run")
            return None
        return self._last_tomato_pose

    def _wait_for_tomato_radius(self):
        if not self.args.use_radius_topic:
            return self.args.pick_distance

        self.get_logger().info(
            f"Waiting for tomato radius on {self.args.tomato_radius_topic} "
            f"for {self.args.tomato_radius_timeout:.1f}s"
        )
        deadline = time.monotonic() + self.args.tomato_radius_timeout
        while rclpy.ok() and self._last_tomato_radius is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self._last_tomato_radius is None:
            if self.args.require_radius:
                self.get_logger().error("No tomato_radius received; no arm motion will run")
                return None
            self.get_logger().warn(
                f"No tomato_radius received; falling back to pick_distance={self.args.pick_distance:.3f} m"
            )
            return self.args.pick_distance

        return self._pick_distance_from_radius(self._last_tomato_radius, log=True)

    def _pick_distance_from_radius(self, radius, log=True):
        raw = radius + self.args.surface_clearance + self.args.tcp_front_offset
        pick_distance = min(max(raw, self.args.min_pick_distance), self.args.max_pick_distance)
        if not log:
            return pick_distance
        if abs(pick_distance - raw) > 1e-6:
            self.get_logger().warn(
                f"Radius-derived pick distance {raw:.3f} m clamped to {pick_distance:.3f} m"
            )
        else:
            self.get_logger().info(
                f"Using radius-derived pick_distance={pick_distance:.3f} m "
                f"(radius={radius:.3f}, surface_clearance={self.args.surface_clearance:.3f}, "
                f"tcp_front_offset={self.args.tcp_front_offset:.3f})"
            )
        return pick_distance

    def _current_pick_distance(self):
        if not self.args.use_radius_topic:
            return self.args.pick_distance
        if self._last_tomato_radius is None:
            return None
        return self._pick_distance_from_radius(self._last_tomato_radius, log=False)

    def _tomato_pose_key(self, msg):
        p = msg.pose.position
        return (
            self._last_tomato_seq,
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            msg.header.frame_id,
            round(p.x, 5),
            round(p.y, 5),
            round(p.z, 5),
        )

    def _stable_target_from_samples(self, samples):
        if len(samples) < self.args.stable_target_samples:
            return None

        recent = samples[-self.args.stable_target_samples :]
        xs = [item[0].pose.position.x for item in recent]
        ys = [item[0].pose.position.y for item in recent]
        zs = [item[0].pose.position.z for item in recent]
        mean_x = sum(xs) / len(xs)
        mean_y = sum(ys) / len(ys)
        mean_z = sum(zs) / len(zs)
        max_error = max(
            math.sqrt(
                (item[0].pose.position.x - mean_x) ** 2
                + (item[0].pose.position.y - mean_y) ** 2
                + (item[0].pose.position.z - mean_z) ** 2
            )
            for item in recent
        )
        if max_error > self.args.target_stability_tolerance:
            return None

        pose = PoseStamped()
        pose.header.frame_id = self.args.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = mean_x
        pose.pose.position.y = mean_y
        pose.pose.position.z = mean_z
        pose.pose.orientation = recent[-1][0].pose.orientation
        pick_distance = sum(item[1] for item in recent) / len(recent)
        return pose, pick_distance, max_error

    def _wait_for_stable_tomato_target(self):
        self.get_logger().info(
            f"Waiting for {self.args.stable_target_samples} stable tomato samples "
            f"on {self.args.tomato_topic} for {self.args.tomato_timeout:.1f}s"
        )
        deadline = time.monotonic() + self.args.tomato_timeout
        last_key = None
        samples = []
        last_spread_log = 0.0

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._last_tomato_pose is None:
                continue
            if self.args.use_radius_topic and self._last_tomato_radius is None:
                continue

            key = self._tomato_pose_key(self._last_tomato_pose)
            if key == last_key:
                continue
            last_key = key

            pick_distance = self._current_pick_distance()
            if pick_distance is None:
                continue

            tomato_base = self._transform_pose_to_base(self._last_tomato_pose)
            if tomato_base is None:
                continue

            samples.append((tomato_base, pick_distance))
            if len(samples) > max(self.args.stable_target_samples * 3, self.args.stable_target_samples):
                samples = samples[-self.args.stable_target_samples * 3 :]

            stable = self._stable_target_from_samples(samples)
            if stable is not None:
                pose, stable_pick_distance, max_error = stable
                self.get_logger().info(
                    "Stable tomato target locked: "
                    f"base=({pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, "
                    f"{pose.pose.position.z:.3f}), pick_distance={stable_pick_distance:.3f}, "
                    f"spread={max_error:.3f} m"
                )
                return pose, stable_pick_distance

            now = time.monotonic()
            if now - last_spread_log >= 2.0 and len(samples) >= 2:
                last_spread_log = now
                latest = samples[-1][0].pose.position
                self.get_logger().info(
                    "Tomato target is not stable yet; latest base sample="
                    f"({latest.x:.3f}, {latest.y:.3f}, {latest.z:.3f})"
                )

        if self._last_tomato_pose is None:
            self.get_logger().error("No tomato_center pose received; no arm motion will run")
        elif self.args.use_radius_topic and self._last_tomato_radius is None and self.args.require_radius:
            self.get_logger().error("No tomato_radius received; no arm motion will run")
        else:
            self.get_logger().error(
                "Tomato target did not become stable; no arm motion will run. "
                "Point the wrist camera at one nearby tomato or narrow detector filters."
            )
        return None

    def _manual_tomato_target(self):
        values = (
            self.args.manual_target_x,
            self.args.manual_target_y,
            self.args.manual_target_z,
        )
        if all(value is None for value in values):
            return None
        if any(value is None for value in values):
            raise RuntimeError(
                "manual target requires --manual-target-x, --manual-target-y, and --manual-target-z"
            )

        pose = PoseStamped()
        pose.header.frame_id = self.args.manual_target_frame or self.args.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.args.manual_target_x
        pose.pose.position.y = self.args.manual_target_y
        pose.pose.position.z = self.args.manual_target_z
        pose.pose.orientation.w = 1.0

        tomato_base = self._transform_pose_to_base(pose)
        if tomato_base is None:
            raise RuntimeError(
                f"manual target transform failed: {pose.header.frame_id} -> {self.args.base_frame}"
            )

        pick_distance = self._current_pick_distance()
        if pick_distance is None:
            pick_distance = self.args.pick_distance
        p = tomato_base.pose.position
        self.get_logger().info(
            "Using manual tomato target: "
            f"input_frame={pose.header.frame_id}, "
            f"base=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}), "
            f"pick_distance={pick_distance:.3f}"
        )
        return tomato_base, pick_distance

    def _lookup_transform(self, target_frame, source_frame, stamp=None, log_error=True):
        lookup_time = stamp or RclTime()
        deadline = time.monotonic() + self.args.tf_timeout
        last_error = None

        while rclpy.ok():
            try:
                return self.tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    lookup_time,
                )
            except TransformException as exc:
                last_error = exc
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                rclpy.spin_once(self, timeout_sec=min(0.05, remaining))

        if log_error:
            self.get_logger().error(
                f"TF lookup failed: {target_frame} <- {source_frame}: {last_error}"
            )
        return None

    def _transform_pose_to_base(self, pose_stamped):
        source_frame = pose_stamped.header.frame_id.strip()
        if not source_frame:
            self.get_logger().warn(
                f"tomato_center has empty frame_id; assuming {self.args.base_frame}"
            )
            source_frame = self.args.base_frame

        if source_frame == self.args.base_frame:
            out = PoseStamped()
            out.header.frame_id = self.args.base_frame
            out.header.stamp = self.get_clock().now().to_msg()
            out.pose = pose_stamped.pose
            out.pose.orientation = _normalize_quaternion(out.pose.orientation)
            return out

        stamp = RclTime()
        if pose_stamped.header.stamp.sec or pose_stamped.header.stamp.nanosec:
            stamp = RclTime.from_msg(pose_stamped.header.stamp)

        transform = self._lookup_transform(
            self.args.base_frame,
            source_frame,
            stamp,
            log_error=not self.args.use_latest_tomato_tf,
        )
        if transform is None and self.args.use_latest_tomato_tf and stamp.nanoseconds:
            self.get_logger().warn(
                "Stamped tomato TF was not available; retrying with latest TF. "
                "This is expected in Gazebo when camera images lag the TF buffer by a few ms."
            )
            transform = self._lookup_transform(self.args.base_frame, source_frame, RclTime())
        if transform is None:
            return None

        tq = _normalize_quaternion(transform.transform.rotation)
        tp = transform.transform.translation
        p = pose_stamped.pose.position
        rx, ry, rz = _rotate_vector(tq, (p.x, p.y, p.z))

        out = PoseStamped()
        out.header.frame_id = self.args.base_frame
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose.position.x = rx + tp.x
        out.pose.position.y = ry + tp.y
        out.pose.position.z = rz + tp.z
        out.pose.orientation = _normalize_quaternion(
            _quat_multiply(tq, _normalize_quaternion(pose_stamped.pose.orientation))
        )
        return out

    def _current_tcp_position(self):
        transform = self._lookup_transform(self.args.base_frame, self.args.ee_link)
        if transform is None:
            return None
        t = transform.transform.translation
        return (t.x, t.y, t.z)

    def _default_world_file(self):
        if self.args.world_file:
            return Path(self.args.world_file)

        if get_package_share_directory is not None:
            try:
                return Path(get_package_share_directory("combined_robot")) / "worlds" / "tomato_farm_sera.sdf"
            except Exception:
                pass

        source = Path(__file__).resolve()
        for parent in source.parents:
            candidate = parent / "worlds" / "tomato_farm_sera.sdf"
            if candidate.exists():
                return candidate
        return Path("src/combined_robot/worlds/tomato_farm_sera.sdf")

    def _load_detachable_tomatoes(self):
        if self._detachable_tomatoes is not None:
            return self._detachable_tomatoes

        world_file = self._default_world_file()
        if not world_file.exists():
            self.get_logger().warn(
                f"Auto detach cannot read world file {world_file}; tomato will not be detached"
            )
            self._detachable_tomatoes = []
            return self._detachable_tomatoes

        try:
            root = ET.parse(world_file).getroot()
        except ET.ParseError as exc:
            self.get_logger().warn(
                f"Auto detach could not parse {world_file}: {exc}; tomato will not be detached"
            )
            self._detachable_tomatoes = []
            return self._detachable_tomatoes

        model_positions = {}
        model_radii = {}
        detach_topics = {}
        for model in root.findall(".//model"):
            name = model.get("name")
            if not name:
                continue
            pose = _parse_sdf_pose_xyz(model.findtext("pose"))
            if pose is not None:
                model_positions[name] = pose
            try:
                radius = _parse_sdf_float(
                    model.findtext("./link/collision/geometry/sphere/radius")
                )
                if radius is None:
                    radius = _parse_sdf_float(
                        model.findtext(".//collision/geometry/sphere/radius")
                    )
            except ValueError:
                radius = None
            if radius is not None and radius > 0.0:
                model_radii[name] = radius
            for plugin in model.findall("plugin"):
                child_model = (plugin.findtext("child_model") or "").strip()
                detach_topic = (plugin.findtext("detach_topic") or "").strip()
                if child_model and detach_topic:
                    detach_topics[child_model] = detach_topic

        tomatoes = []
        for name, detach_topic in detach_topics.items():
            if not name.startswith("tomato_") or name not in model_positions:
                continue
            x, y, z = model_positions[name]
            tomatoes.append(
                {
                    "name": name,
                    "detach_topic": detach_topic,
                    "x": x,
                    "y": y,
                    "z": z,
                    "radius": model_radii.get(name),
                }
            )
        self._detachable_tomatoes = tomatoes
        self.get_logger().info(
            f"Loaded {len(tomatoes)} detachable tomato target(s) from {world_file}"
        )
        return self._detachable_tomatoes

    def _base_point_to_world(self, point):
        cos_yaw = math.cos(self.world_yaw)
        sin_yaw = math.sin(self.world_yaw)
        return (
            self.world_x + cos_yaw * point.x - sin_yaw * point.y,
            self.world_y + sin_yaw * point.x + cos_yaw * point.y,
            self.world_z + point.z,
        )

    def _configure_auto_detach_target(self, tomato_pose):
        if self.detach_topic:
            return
        if self.args.skip_detach:
            self.get_logger().warn(
                "--skip-detach is set; Gazebo tomato will stay attached to the plant"
            )
            return
        if not self.args.auto_detach_nearest:
            return

        world_x, world_y, world_z = self._base_point_to_world(tomato_pose.pose.position)
        candidates = self._load_detachable_tomatoes()
        if not candidates:
            return

        def score(candidate):
            planar = math.hypot(candidate["x"] - world_x, candidate["y"] - world_y)
            vertical = abs(candidate["z"] - world_z)
            return planar + 0.25 * vertical

        best = min(candidates, key=score)
        planar_distance = math.hypot(best["x"] - world_x, best["y"] - world_y)
        vertical_distance = abs(best["z"] - world_z)
        if planar_distance > self.args.auto_detach_max_distance:
            self.get_logger().warn(
                "Nearest detachable tomato is too far from detected target; "
                f"detected_world=({world_x:.3f}, {world_y:.3f}, {world_z:.3f}), "
                f"nearest={best['name']} at ({best['x']:.3f}, {best['y']:.3f}, {best['z']:.3f}), "
                f"planar_distance={planar_distance:.3f} m"
            )
            return

        self.set_target(best["name"], best["detach_topic"])
        self.get_logger().info(
            "Auto detach target selected: "
            f"{best['name']} topic={best['detach_topic']}, "
            f"planar_distance={planar_distance:.3f} m, vertical_delta={vertical_distance:.3f} m"
        )

    def _world_xyz_to_base(self, x, y, z):
        dx = x - self.world_x
        dy = y - self.world_y
        cos_yaw = math.cos(self.world_yaw)
        sin_yaw = math.sin(self.world_yaw)
        return (
            cos_yaw * dx + sin_yaw * dy,
            -sin_yaw * dx + cos_yaw * dy,
            z - self.world_z,
        )

    def _nearby_tomato_collision_objects_for_moveit(self):
        if not self.args.avoid_nearby_tomatoes:
            return []
        if self._nearby_tomato_collision_objects is not None:
            return self._nearby_tomato_collision_objects

        tomatoes = self._load_detachable_tomatoes()
        if not tomatoes:
            self._nearby_tomato_collision_objects = []
            return self._nearby_tomato_collision_objects

        target = next(
            (tomato for tomato in tomatoes if tomato["name"] == self.tomato_model),
            None,
        )
        if target is None and self._last_tomato_pose is not None:
            world_x, world_y, world_z = self._base_point_to_world(
                self._last_tomato_pose.pose.position
            )
            target = {
                "name": self.tomato_model or "tomato_center_topic_target",
                "x": world_x,
                "y": world_y,
                "z": world_z,
                "radius": self.args.nearby_tomato_collision_radius,
            }

        if target is None:
            self.get_logger().warn(
                "Nearby tomato avoidance requested, but the target tomato pose is unavailable; "
                "planning without nearby tomato collision objects."
            )
            self._nearby_tomato_collision_objects = []
            return self._nearby_tomato_collision_objects

        nearby = []
        for tomato in tomatoes:
            if tomato["name"] == target["name"]:
                continue
            dx = tomato["x"] - target["x"]
            dy = tomato["y"] - target["y"]
            dz = tomato["z"] - target["z"]
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            if distance > self.args.nearby_tomato_collision_distance:
                continue
            nearby.append((distance, tomato))

        nearby.sort(key=lambda item: item[0])
        if self.args.nearby_tomato_collision_max_count > 0:
            nearby = nearby[: self.args.nearby_tomato_collision_max_count]

        objects = []
        for _distance, tomato in nearby:
            radius = self.args.nearby_tomato_collision_radius
            if radius <= 0.0:
                radius = tomato.get("radius") or 0.032
            radius = max(0.005, radius + self.args.nearby_tomato_collision_padding)
            base_x, base_y, base_z = self._world_xyz_to_base(
                tomato["x"], tomato["y"], tomato["z"]
            )

            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.SPHERE
            primitive.dimensions = [radius]

            pose = Pose()
            pose.position.x = base_x
            pose.position.y = base_y
            pose.position.z = base_z
            pose.orientation.w = 1.0

            safe_name = "".join(
                ch if ch.isalnum() or ch == "_" else "_" for ch in tomato["name"]
            )
            collision_object = CollisionObject()
            collision_object.header.frame_id = self.args.base_frame
            collision_object.id = f"nearby_tomato_obstacle_{safe_name}"
            collision_object.operation = CollisionObject.ADD
            collision_object.primitives.append(primitive)
            collision_object.primitive_poses.append(pose)
            objects.append(collision_object)

        self._nearby_tomato_collision_objects = objects
        if not self._logged_nearby_tomato_collisions:
            self._logged_nearby_tomato_collisions = True
            self.get_logger().info(
                "Nearby tomato avoidance: "
                f"target={target['name']}, objects={len(objects)}, "
                f"distance={self.args.nearby_tomato_collision_distance:.2f}m, "
                f"padding={self.args.nearby_tomato_collision_padding:.3f}m"
            )
        return self._nearby_tomato_collision_objects

    def _compute_single_pick_candidate(
        self,
        tomato,
        pick_distance,
        direction,
        angle_offset_deg,
        lateral_offset,
        z_offset,
    ):
        direction = _normalize_vector(direction)
        lateral = (-direction[1], direction[0], 0.0)
        orientation = _look_at_orientation(direction, self.args.grasp_roll)
        close_axis = _rotate_vector(orientation, (0.0, 1.0, 0.0))
        target_x = tomato.x + lateral[0] * lateral_offset
        target_y = tomato.y + lateral[1] * lateral_offset
        target_z = tomato.z + z_offset

        approach = _make_pose(
            target_x - direction[0] * self.args.approach_distance,
            target_y - direction[1] * self.args.approach_distance,
            target_z,
            orientation,
        )
        pick = _make_pose(
            target_x - direction[0] * pick_distance,
            target_y - direction[1] * pick_distance,
            target_z,
            orientation,
        )
        retreat = _make_pose(
            approach.position.x,
            approach.position.y,
            approach.position.z,
            orientation,
        )

        approach_base_distance = math.sqrt(
            approach.position.x * approach.position.x
            + approach.position.y * approach.position.y
            + approach.position.z * approach.position.z
        )
        pick_base_distance = math.sqrt(
            pick.position.x * pick.position.x
            + pick.position.y * pick.position.y
            + pick.position.z * pick.position.z
        )
        if approach_base_distance > self.args.max_approach_base_distance:
            return None
        if pick_base_distance > self.args.max_pick_base_distance:
            return None

        return {
            "approach": approach,
            "pick": pick,
            "retreat": retreat,
            "pick_distance": pick_distance,
            "angle_offset_deg": angle_offset_deg,
            "lateral_offset": lateral_offset,
            "z_offset": z_offset,
            "approach_base_distance": approach_base_distance,
            "pick_base_distance": pick_base_distance,
            "close_axis": close_axis,
        }

    def _compute_pick_pose_candidates(self, tomato_pose, pick_distance):
        tomato = tomato_pose.pose.position
        tcp = self._current_tcp_position()
        if tcp is None:
            self.get_logger().error(
                f"Current TCP TF unavailable for ee_link={self.args.ee_link}; "
                "check ee_link/tool0/fr3_hand_tcp/panda_hand selection"
            )
            return []

        dx = tomato.x - tcp[0]
        dy = tomato.y - tcp[1]
        dz = tomato.z - tcp[2]
        target_distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        if target_distance < self.args.min_target_distance:
            self.get_logger().error(
                f"Tomato target is too close to TCP ({target_distance:.3f} m); no arm motion will run"
            )
            return []
        if target_distance > self.args.max_target_distance:
            self.get_logger().error(
                f"Tomato target is too far from TCP ({target_distance:.3f} m > "
                f"{self.args.max_target_distance:.3f} m); no arm motion will run"
            )
            return []
        if self._fixed_approach_direction is not None:
            base_direction = self._fixed_approach_direction
        elif self.args.approach_direction_source == "base_to_target":
            horizontal = math.hypot(tomato.x, tomato.y)
            if horizontal < 1e-4:
                self.get_logger().error("Tomato is too close to base projection; cannot define approach line")
                return []
            base_direction = (tomato.x / horizontal, tomato.y / horizontal, 0.0)
        else:
            horizontal = math.hypot(dx, dy)
            if horizontal < 1e-4:
                self.get_logger().error("Tomato is too close to TCP projection; cannot define approach line")
                return []
            base_direction = (dx / horizontal, dy / horizontal, 0.0)
        angle_offsets = _parse_float_list(
            self.args.grasp_angle_offsets_deg,
            [0.0],
        )
        lateral_offsets = _parse_float_list(
            self.args.grasp_lateral_offsets,
            [self.args.grasp_lateral_offset],
        )
        z_offsets = _parse_float_list(self.args.grasp_z_offsets, [0.0])
        pick_distances = self._pick_distance_candidates or [pick_distance]

        candidates = []
        seen = set()
        for candidate_pick_distance in pick_distances:
            for angle_offset_deg in angle_offsets:
                angle = math.radians(angle_offset_deg)
                cos_angle = math.cos(angle)
                sin_angle = math.sin(angle)
                direction = (
                    base_direction[0] * cos_angle - base_direction[1] * sin_angle,
                    base_direction[0] * sin_angle + base_direction[1] * cos_angle,
                    0.0,
                )
                for z_offset in z_offsets:
                    for lateral_offset in lateral_offsets:
                        key = (
                            round(candidate_pick_distance, 4),
                            round(angle_offset_deg, 4),
                            round(z_offset, 4),
                            round(lateral_offset, 4),
                        )
                        if key in seen:
                            continue
                        seen.add(key)
                        candidate = self._compute_single_pick_candidate(
                            tomato,
                            candidate_pick_distance,
                            direction,
                            angle_offset_deg,
                            lateral_offset,
                            z_offset,
                        )
                        if candidate is not None:
                            candidates.append(candidate)

        if not candidates:
            self.get_logger().error(
                "No pick candidates are inside the conservative FR3 workspace; "
                "move the platform closer or relax max_*_base_distance."
            )
            return []

        candidates.sort(
            key=lambda item: (
                abs(item["pick_distance"] - pick_distance),
                abs(item["angle_offset_deg"]),
                abs(item["z_offset"]),
                abs(item["lateral_offset"] - self.args.grasp_lateral_offset),
                item["pick_base_distance"],
            )
        )
        self.get_logger().info(
            f"Computed {len(candidates)} pick candidate(s) in {self.args.base_frame}: "
            f"tomato=({tomato.x:.3f}, {tomato.y:.3f}, {tomato.z:.3f}), "
            f"target_distance={target_distance:.3f}, preferred_pick_distance={pick_distance:.3f}"
        )
        for index, candidate in enumerate(candidates[: min(len(candidates), 6)], start=1):
            pick = candidate["pick"].position
            approach = candidate["approach"].position
            close_axis = candidate["close_axis"]
            self.get_logger().info(
                f"candidate {index}: pick_distance={candidate['pick_distance']:.3f}, "
                f"angle={candidate['angle_offset_deg']:.1f}deg, "
                f"z_offset={candidate['z_offset']:.3f}, "
                f"lateral_offset={candidate['lateral_offset']:.3f}, "
                f"approach=({approach.x:.3f}, {approach.y:.3f}, {approach.z:.3f}), "
                f"pick=({pick.x:.3f}, {pick.y:.3f}, {pick.z:.3f}), "
                f"close_axis=({close_axis[0]:.3f}, {close_axis[1]:.3f}, {close_axis[2]:.3f})"
            )
        return candidates

    def _compute_pick_poses(self, tomato_pose, pick_distance):
        candidates = self._compute_pick_pose_candidates(tomato_pose, pick_distance)
        if not candidates:
            return None
        first = candidates[0]
        return first["approach"], first["pick"], first["retreat"]

    def _current_robot_state(self):
        state = RobotState()
        state.is_diff = True
        state.joint_state.name = list(self._joint_state.name)
        state.joint_state.position = list(self._joint_state.position)
        state.joint_state.velocity = list(self._joint_state.velocity)
        state.joint_state.effort = list(self._joint_state.effort)
        return state

    def _pose_constraints(self, pose, label):
        bounds_pose = Pose()
        bounds_pose.position = pose.position
        bounds_pose.orientation.w = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.args.position_tolerance]

        volume = BoundingVolume()
        volume.primitives.append(sphere)
        volume.primitive_poses.append(bounds_pose)

        pos = PositionConstraint()
        pos.header.frame_id = self.args.base_frame
        pos.link_name = self.args.ee_link
        pos.constraint_region = volume
        pos.weight = 1.0

        orient = OrientationConstraint()
        orient.header.frame_id = self.args.base_frame
        orient.link_name = self.args.ee_link
        orient.orientation = pose.orientation
        orient.absolute_x_axis_tolerance = self.args.orientation_tolerance
        orient.absolute_y_axis_tolerance = self.args.orientation_tolerance
        orient.absolute_z_axis_tolerance = self.args.orientation_tolerance
        orient.weight = 1.0

        constraints = Constraints()
        constraints.name = label
        constraints.position_constraints.append(pos)
        constraints.orientation_constraints.append(orient)
        return constraints

    def _make_motion_request(self, pose, pipeline_id, planner_id, label):
        req = MotionPlanRequest()
        req.pipeline_id = pipeline_id
        req.planner_id = planner_id
        req.group_name = self.args.planning_group
        req.num_planning_attempts = self.args.num_planning_attempts
        req.allowed_planning_time = self.args.allowed_planning_time
        req.max_velocity_scaling_factor = self.args.velocity_scaling
        req.max_acceleration_scaling_factor = self.args.acceleration_scaling
        req.start_state = self._current_robot_state()
        req.goal_constraints.append(self._pose_constraints(pose, label))
        if pipeline_id == PILZ_PIPELINE:
            req.cartesian_speed_limited_link = self.args.ee_link
            req.max_cartesian_speed = self.args.max_cartesian_speed
        return req

    def _wait_for_future(self, future, timeout_sec, label):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            self._maybe_update_carry_pose()
            if time.monotonic() >= deadline:
                raise TimeoutError(f"{label}: timed out after {timeout_sec:.1f}s")
        if not future.done():
            raise TimeoutError(f"{label}: interrupted before completion")
        return future.result()

    def _maybe_update_carry_pose(self):
        if not getattr(self, "_carry_active", False):
            return
        now = time.monotonic()
        if now - self._last_carry_update_time < 0.20:
            return
        self._last_carry_update_time = now
        self._carry_pose_update()

    def _tcp_position_error(self, pose):
        tcp = self._current_tcp_position()
        if tcp is None:
            return None
        dx = tcp[0] - pose.position.x
        dy = tcp[1] - pose.position.y
        dz = tcp[2] - pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _wait_for_tcp_near_pose(self, pose, label):
        if self.args.post_motion_wait_timeout <= 0.0:
            return

        tolerance = self.args.post_motion_position_tolerance
        deadline = time.monotonic() + self.args.post_motion_wait_timeout
        last_error = None
        while rclpy.ok():
            error = self._tcp_position_error(pose)
            if error is None:
                raise RuntimeError(f"{label}: TCP TF unavailable for post-motion check")
            last_error = error
            if error <= tolerance:
                self.get_logger().info(
                    f"{label}: TCP reached target, position_error={error:.3f} m"
                )
                return
            if time.monotonic() >= deadline:
                break
            rclpy.spin_once(self, timeout_sec=0.05)

        raise RuntimeError(
            f"{label}: TCP did not reach target before timeout "
            f"(error={last_error:.3f} m, tolerance={tolerance:.3f} m)"
        )

    def _move_group_feedback(self, label):
        last_log = {"time": 0.0, "state": ""}

        def _callback(msg):
            now = time.monotonic()
            state = msg.feedback.state
            if state != last_log["state"] or now - last_log["time"] > 2.0:
                self.get_logger().info(f"{label}: MoveGroup feedback: {state}")
                last_log["time"] = now
                last_log["state"] = state

        return _callback

    def _move_group(self, pose, pipeline_id, planner_id, label):
        goal = MoveGroup.Goal()
        goal.request = self._make_motion_request(pose, pipeline_id, planner_id, label)
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = self.args.replan
        goal.planning_options.replan_attempts = self.args.replan_attempts
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        nearby_tomatoes = self._nearby_tomato_collision_objects_for_moveit()
        if nearby_tomatoes:
            stamp = self.get_clock().now().to_msg()
            for collision_object in nearby_tomatoes:
                collision_object.header.stamp = stamp
            goal.planning_options.planning_scene_diff.world.collision_objects.extend(
                nearby_tomatoes
            )

        self.get_logger().info(
            f"{label}: planning with pipeline={pipeline_id}, planner_id={planner_id or '<default>'}, "
            f"group={self.args.planning_group}, ee_link={self.args.ee_link}"
        )
        goal_future = self.move_group_client.send_goal_async(
            goal, feedback_callback=self._move_group_feedback(label)
        )
        goal_handle = self._wait_for_future(goal_future, 10.0, f"{label} goal response")
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"{label}: MoveGroup goal rejected")

        result_future = goal_handle.get_result_async()
        try:
            wrapped = self._wait_for_future(
                result_future, self.args.move_action_timeout, f"{label} result"
            )
        except TimeoutError:
            self.get_logger().error(f"{label}: timeout, canceling MoveGroup goal")
            cancel_future = goal_handle.cancel_goal_async()
            try:
                self._wait_for_future(cancel_future, 5.0, f"{label} cancel")
            except TimeoutError:
                self.get_logger().warn(f"{label}: cancel request timed out")
            raise
        if wrapped is None:
            raise RuntimeError(f"{label}: MoveGroup returned no result")

        result = wrapped.result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(f"{label}: MoveIt error_code={result.error_code.val}")

        self.get_logger().info(
            f"{label}: success, planning_time={result.planning_time:.3f}s"
        )
        rclpy.spin_once(self, timeout_sec=0.2)
        self._wait_for_tcp_near_pose(pose, label)
        return True

    def _execute_two_stage_pick(
        self,
        approach_pose,
        pick_pose,
        retreat_pose,
        carry_tcp_offset=(0.0, 0.0, 0.0),
    ):
        try:
            self._move_group(
                approach_pose,
                self.args.approach_pipeline,
                self.args.approach_planner_id,
                "OMPL/PTP approach_pose",
            )
        except RuntimeError as exc:
            self.get_logger().error(
                f"OMPL/PTP approach failed; gripper will stay open and LIN pick will NOT run: {exc}"
            )
            return False

        self.get_logger().info("Opening gripper after successful approach")
        self._gripper(GRIPPER_OPEN_TOTAL_WIDTH, "open gripper")

        pick_done = False
        try:
            self._move_group(
                pick_pose,
                self.args.pick_pipeline,
                self.args.pick_planner_id,
                "pick_pose",
            )
            pick_done = True
        except RuntimeError as exc:
            if not self.args.pick_fallback_pipeline:
                self.get_logger().error(
                    f"Pick motion failed; gripper will NOT close: {exc}"
                )
                return False
            self.get_logger().warn(
                f"Pick motion failed, trying fallback "
                f"{self.args.pick_fallback_pipeline}/{self.args.pick_fallback_planner_id}: {exc}"
            )
            try:
                self._move_group(
                    pick_pose,
                    self.args.pick_fallback_pipeline,
                    self.args.pick_fallback_planner_id,
                    "fallback pick_pose",
                )
                pick_done = True
            except RuntimeError as fallback_exc:
                self.get_logger().error(
                    f"Fallback pick motion failed; gripper will NOT close: {fallback_exc}"
                )
                return False

        if not pick_done:
            return False

        close_width = self.args.gripper_close_width
        self.get_logger().info(
            f"Closing gripper after successful LIN pick to total width {close_width:.3f} m"
        )
        self._gripper(close_width, "close gripper")

        if self.detach_topic:
            gripper_attached = self._attach_tomato_to_gripper()
            if gripper_attached:
                self.get_logger().info(
                    f"Attached {self.tomato_model} to "
                    f"{self.robot_model_name}::{self.gripper_attach_parent_link} before plant detach"
                )
            self._detach_tomato()
            time.sleep(0.15)
            if not gripper_attached and self.args.carry_pose_stabilization:
                if self._activate_carried_visual_proxy():
                    self.get_logger().info(
                        "Using static carried tomato proxy after plant detach because "
                        "Gazebo gripper attachment is unavailable"
                    )
                self.carry_tcp_offset = carry_tcp_offset
                self.get_logger().info(
                    "Using carry TCP offset "
                    f"({carry_tcp_offset[0]:.3f}, {carry_tcp_offset[1]:.3f}, "
                    f"{carry_tcp_offset[2]:.3f}) m"
                )
                self._carry_active = True
                self._set_tomato_pose_at_tcp(
                    "post-detach tomato carry lock", repeats=2, interval=0.05
                )
            elif not gripper_attached:
                self.get_logger().info(
                    "Plant detach complete; carrying by physical gripper contact "
                    "(TCP set_pose fallback disabled)."
                )

        retreat_error = None
        for attempt in range(2):
            try:
                self._move_group(
                    retreat_pose,
                    self.args.retreat_pipeline,
                    self.args.retreat_planner_id,
                    "retreat_pose",
                )
                retreat_error = None
                break
            except RuntimeError as exc:
                retreat_error = exc
                if attempt == 0:
                    self.get_logger().warn(
                        "retreat_pose failed after tomato attach/detach; "
                        f"refreshing joint state and retrying once: {exc}"
                    )
                    for _ in range(5):
                        rclpy.spin_once(self, timeout_sec=0.05)
                    time.sleep(0.1)

        if retreat_error is not None:
            if not self.args.place_in_basket:
                self.get_logger().error(
                    f"retreat_pose failed and basket placement is disabled: {retreat_error}"
                )
                return False
            if not self._attachment_active and not self.args.carry_pose_stabilization:
                self.get_logger().error(
                    "retreat_pose failed and no tomato carry mechanism is active; "
                    f"not attempting basket placement: {retreat_error}"
                )
                return False
            self.get_logger().warn(
                "retreat_pose still failed, but tomato carry is active and basket "
                f"placement is enabled; continuing to place: {retreat_error}"
            )

        self.get_logger().info("Two-stage pick pipeline complete")
        if not self.args.place_in_basket:
            self._carry_active = False
            self._publish_picked_event("picked")
            return True

        return self._place_in_basket()

    def _event_tomato_model(self):
        return getattr(self, "_harvest_source_model", None) or self.tomato_model

    def _source_visual_model(self):
        return getattr(self, "_visual_source_model", None) or self._event_tomato_model()

    @staticmethod
    def _safe_model_name(model_name):
        return "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in model_name)

    def _publish_picked_event(self, status):
        if not self.args.picked_topic:
            return
        event_model = self._event_tomato_model()
        payload = {
            "tomato_model": event_model,
            "status": status,
            "basket": self.basket,
            "world_name": self.world_name,
            "timestamp": time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        for _ in range(3):
            self._picked_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)
        self.get_logger().info(
            f"Published picked tomato event on {self.args.picked_topic}: {event_model}"
        )

    def _remove_gazebo_model(self, model_name, label, warn=True):
        request = f'name: "{model_name}", type: MODEL'
        command = [
            "gz",
            "service",
            "-s",
            f"/world/{self.world_name}/remove",
            "--reqtype",
            "gz.msgs.Entity",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "2000",
            "--req",
            request,
        ]
        try:
            result = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=3.0,
            )
        except subprocess.TimeoutExpired:
            if warn:
                self.get_logger().warn(f"{label}: remove timed out for {model_name}")
            return False

        ok = result.returncode == 0 and "data: true" in result.stdout
        if not ok and warn:
            stderr = result.stderr.strip()
            stdout = result.stdout.strip()
            self.get_logger().warn(
                f"{label}: remove failed for {model_name}: {stderr or stdout or result.returncode}"
            )
        return ok

    def _static_harvested_tomato_sdf(self, harvested_name, source_model=None, static=True):
        source_model = source_model or self._source_visual_model()
        world_file = self._default_world_file()
        visual_xml = ""
        if world_file.exists():
            try:
                root = ET.parse(world_file).getroot()
                for model in root.findall(".//model"):
                    if model.get("name") != source_model:
                        continue
                    visual = model.find("./link/visual")
                    if visual is not None:
                        visual_xml = ET.tostring(visual, encoding="unicode")
                    break
            except ET.ParseError as exc:
                self.get_logger().warn(
                    f"Could not parse {world_file} for harvested tomato visual: {exc}"
                )

        if not visual_xml:
            visual_xml = (
                '<visual name="visual">'
                "<geometry><sphere><radius>0.045</radius></sphere></geometry>"
                "<material><ambient>0.8 0.05 0.02 1</ambient>"
                "<diffuse>0.8 0.05 0.02 1</diffuse></material>"
                "</visual>"
            )

        inertial_xml = ""
        if not static:
            inertial_xml = (
                '<inertial><mass>0.03</mass>'
                '<inertia><ixx>2.0e-5</ixx><iyy>2.0e-5</iyy><izz>2.0e-5</izz>'
                '<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>'
            )
        collision_radius = max(
            0.005,
            float(getattr(self.args, "harvested_tomato_collision_radius", 0.038)),
        )
        collision_xml = (
            '<collision name="sphere_collision">'
            f"<geometry><sphere><radius>{collision_radius:.4f}</radius></sphere></geometry>"
            "<surface>"
            "<friction><ode><mu>1.8</mu><mu2>1.8</mu2><slip1>0.01</slip1><slip2>0.01</slip2></ode></friction>"
            "<contact><ode><kp>500000</kp><kd>80</kd><max_vel>0.2</max_vel><min_depth>0.001</min_depth></ode></contact>"
            "<bounce><restitution_coefficient>0.03</restitution_coefficient><threshold>0.2</threshold></bounce>"
            "</surface>"
            "</collision>"
        )

        return (
            f'<sdf version="1.9"><model name="{harvested_name}">'
            f"<static>{'true' if static else 'false'}</static>"
            f'<link name="link">{inertial_xml}{collision_xml}{visual_xml}</link>'
            "</model></sdf>"
        )

    def _create_static_harvested_tomato(
        self,
        harvested_name,
        position,
        source_model=None,
        label="harvested tomato freeze",
        static=True,
    ):
        x, y, z = position
        sdf = self._static_harvested_tomato_sdf(
            harvested_name, source_model=source_model, static=static
        )
        request = (
            f'name: "{harvested_name}", '
            "allow_renaming: false, "
            f"sdf: {json.dumps(sdf)}, "
            f"pose: {{position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, "
            "orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
        )
        command = [
            "gz",
            "service",
            "-s",
            f"/world/{self.world_name}/create",
            "--reqtype",
            "gz.msgs.EntityFactory",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "2000",
            "--req",
            request,
        ]
        try:
            result = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=3.0,
            )
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f"{label}: create timed out")
            return False

        ok = result.returncode == 0 and "data: true" in result.stdout
        if not ok:
            stderr = result.stderr.strip()
            stdout = result.stdout.strip()
            self.get_logger().warn(
                f"{label}: create failed: {stderr or stdout or result.returncode}"
            )
        return ok

    def _attach_harvested_tomato_to_robot_base(self, harvested_name):
        if self._attach_model_to_robot_link(
            harvested_name,
            "base_link",
            "basket_contact_attach",
        ):
            self.get_logger().info(
                f"Locked harvested tomato {harvested_name} to robot base/basket frame"
            )
            return True

        robot_id = self._gazebo_model_id(self.robot_model_name)
        if robot_id is None:
            self.get_logger().warn(
                f"Could not attach {harvested_name} to basket carrier; "
                f"model {self.robot_model_name} was not found."
            )
            return False

        attach_topic = f"/basket_attach/{harvested_name}"
        detach_topic = f"/basket_detach/{harvested_name}"
        state_topic = f"/basket_attach_state/{harvested_name}"
        innerxml = (
            "<parent_link>base_link</parent_link>"
            f"<child_model>{harvested_name}</child_model>"
            "<child_link>link</child_link>"
            f"<detach_topic>{detach_topic}</detach_topic>"
            f"<attach_topic>{attach_topic}</attach_topic>"
            f"<output_topic>{state_topic}</output_topic>"
        )
        request = (
            f'entity {{ id: {robot_id} name: "{self.robot_model_name}" type: MODEL }} '
            "plugins { "
            'name: "gz::sim::systems::DetachableJoint" '
            'filename: "gz-sim-detachable-joint-system" '
            f'innerxml: "{innerxml}" '
            "}"
        )
        command = [
            "gz",
            "service",
            "-s",
            f"/world/{self.world_name}/entity/system/add",
            "--reqtype",
            "gz.msgs.EntityPlugin_V",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            request,
        ]
        try:
            result = subprocess.run(
                command, check=False, capture_output=True, text=True, timeout=7.0
            )
        except (subprocess.TimeoutExpired, OSError) as exc:
            self.get_logger().warn(f"Basket tomato attachment plugin add failed: {exc}")
            return False

        if result.returncode != 0 or "data: true" not in result.stdout:
            stderr = result.stderr.strip()
            stdout = result.stdout.strip()
            self.get_logger().warn(
                "Basket tomato attachment plugin add was not accepted: "
                f"{stderr or stdout or result.returncode}"
            )
            return False

        time.sleep(0.15)
        return self._publish_gz_empty(attach_topic, "basket tomato attach", attempts=3)

    def _basket_slot_local_pose(self, harvested_name):
        basket_pose = BASKET_SETTLE_POSES[self.basket]
        digest = hashlib.sha1(harvested_name.encode("utf-8")).hexdigest()
        slot_index = int(digest[:8], 16) % len(BASKET_SLOT_OFFSETS)
        dx, dy, dz = BASKET_SLOT_OFFSETS[slot_index]
        return (
            basket_pose[0] + dx,
            basket_pose[1] + dy,
            basket_pose[2] + dz + self.args.freeze_basket_z_offset,
            slot_index,
        )

    def _activate_carried_visual_proxy(self):
        source_model = self._event_tomato_model()
        if not source_model or source_model == "tomato_center_topic_target":
            return False

        proxy_name = f"carried_{self._safe_model_name(source_model)}"
        self._remove_gazebo_model(proxy_name, "carried tomato proxy", warn=False)

        position = self._get_tcp_world_pose()
        if position is None and self._last_tomato_pose is not None:
            position = self._base_point_to_world(self._last_tomato_pose.pose.position)
        if position is None:
            self.get_logger().warn(
                "Could not create carried tomato proxy because TCP and target pose are unavailable"
            )
            return False

        if not self._create_static_harvested_tomato(
            proxy_name,
            position,
            source_model=source_model,
            label="carried tomato proxy",
            static=True,
        ):
            return False

        if not self._remove_gazebo_model(source_model, "carried tomato proxy"):
            self.get_logger().warn(
                f"Original tomato model {source_model} could not be removed after proxy creation; "
                "visual duplicate may remain."
            )

        self._carried_proxy_model = proxy_name
        self._visual_source_model = source_model
        self.tomato_model = proxy_name
        self.detach_topic = ""
        self.get_logger().info(
            f"Using carried visual proxy {proxy_name} for picked tomato {source_model}"
        )
        return True

    def _freeze_tomato_in_basket(self):
        if not self.args.freeze_harvested_tomato:
            return False

        source_model = self._event_tomato_model()
        safe_name = self._safe_model_name(source_model)
        harvested_name = f"harvested_{safe_name}"
        local_x, local_y, local_z, slot_index = self._basket_slot_local_pose(harvested_name)
        position = self._local_to_world((local_x, local_y, local_z))

        self.get_logger().info(
            f"Spawning harvested tomato in {self.basket} basket slot {slot_index} "
            f"as {harvested_name}"
        )
        if self._gazebo_model_id(harvested_name) is not None:
            self._remove_gazebo_model(
                harvested_name, "stale harvested tomato freeze", warn=False
            )

        model_to_remove = self._carried_proxy_model or self.tomato_model
        if model_to_remove and model_to_remove != harvested_name:
            if not self._remove_gazebo_model(
                model_to_remove, "harvested tomato source removal"
            ):
                self.get_logger().warn(
                    f"Not creating {harvested_name} because source model "
                    f"{model_to_remove} could not be removed; avoiding visual duplicate."
                )
                return False

        created = self._create_static_harvested_tomato(
            harvested_name,
            position,
            source_model=self._source_visual_model(),
            label="harvested tomato freeze",
            static=False,
        )
        if not created:
            return False
        if not self.args.lock_harvested_to_basket:
            self.get_logger().info(
                f"Leaving {harvested_name} dynamic in basket; Gazebo physics handles "
                "rolling and tomato-to-tomato collisions."
            )
            return True

        if self._attach_harvested_tomato_to_robot_base(harvested_name):
            return True

        self.get_logger().warn(
            f"Could not lock {harvested_name} to the basket frame; replacing it "
            "with a static basket visual so it cannot fall through the basket."
        )
        self._remove_gazebo_model(
            harvested_name, "unlocked harvested tomato cleanup", warn=False
        )
        return self._create_static_harvested_tomato(
            harvested_name,
            position,
            source_model=self._source_visual_model(),
            label="static harvested tomato fallback",
            static=True,
        )

    def _carry_offset_for_candidate(self, tomato_base, candidate):
        """Return tomato center offset in the TCP frame at the selected pick pose."""
        tomato = tomato_base.pose.position
        pick = candidate["pick"]
        delta_base = (
            tomato.x - pick.position.x,
            tomato.y - pick.position.y,
            tomato.z - pick.position.z,
        )
        return _rotate_vector(
            _quat_conjugate(_normalize_quaternion(pick.orientation)),
            delta_base,
        )

    def _place_in_basket(self):
        hover = f"{self.basket}_hover"
        drop = f"{self.basket}_drop"
        missing = [
            pose_name
            for pose_name in ("swing_transit", hover, drop)
            if pose_name not in self.poses
        ]
        if missing:
            self.get_logger().error(
                "Basket placement pose(s) missing from profile "
                f"{self.args.profile}: {', '.join(missing)}"
            )
            return False

        self.get_logger().info(
            f"Placing picked tomato into {self.basket} basket via {hover} -> {drop}"
        )

        forced_carry_for_basket = False
        attached_for_basket = self._attachment_active
        if (
            self.detach_topic
            and self.args.carry_pose_stabilization
            and not self._attachment_active
        ):
            self._carry_active = True
            forced_carry_for_basket = True
            self._set_tomato_pose_at_tcp(
                "pre-basket carry lock", repeats=2, interval=0.05
            )
        elif self._attachment_active:
            self.get_logger().info(
                "Using Gazebo gripper attachment for basket carry; TCP set_pose carry lock is disabled"
            )

        try:
            self._arm(["swing_transit", hover], 6.0, f"swing to {self.basket} basket")
            time.sleep(0.3)

            self._arm([drop], 2.0, f"lower into {self.basket} basket")
            time.sleep(0.3)
        except RuntimeError as exc:
            self._carry_active = False
            self.get_logger().error(f"Basket placement failed before release: {exc}")
            return False

        self._carry_active = False
        released_from_attachment = self._detach_tomato_from_gripper()
        if attached_for_basket or forced_carry_for_basket:
            if not released_from_attachment:
                self.get_logger().warn(
                    "Gripper attachment release publish failed; skipping pre-release basket settle"
                )
            else:
                self._settle_tomato_in_basket("pre-release tomato placement", repeats=3)
                time.sleep(0.2)

        try:
            self._gripper(GRIPPER_OPEN_TOTAL_WIDTH, "release tomato")
            time.sleep(0.3)

            self._settle_tomato_in_basket("post-release tomato placement", repeats=8)
            tomato_frozen = self._freeze_tomato_in_basket()
            time.sleep(0.3)

            self._arm([hover, "swing_transit"], 4.0, "retract from basket")
            time.sleep(0.3)

            if not tomato_frozen:
                self._settle_tomato_in_basket("post-retract tomato placement", repeats=5)
            self._arm(["home"], 4.0, "return home")
            if not tomato_frozen:
                self._settle_tomato_in_basket("final tomato placement", repeats=5)
        except RuntimeError as exc:
            self.get_logger().error(f"Basket placement failed after release: {exc}")
            return False

        self.get_logger().info(f"Tomato placed in {self.basket} basket")
        self._publish_picked_event("placed")
        return True

    def _validate_args(self):
        try:
            self._pre_detect_joint_positions = _parse_joint_positions(self.args.pre_detect_joints)
        except ValueError as exc:
            raise RuntimeError(f"invalid pre_detect_joints: {exc}") from exc
        try:
            _parse_float_list(self.args.grasp_angle_offsets_deg, [0.0])
            _parse_float_list(self.args.grasp_lateral_offsets, [self.args.grasp_lateral_offset])
            _parse_float_list(self.args.grasp_z_offsets, [0.0])
            self._pick_distance_candidates = _parse_float_list(self.args.pick_distances, [])
            fixed_direction = _parse_float_list(self.args.fixed_approach_direction, [])
        except ValueError as exc:
            raise RuntimeError(f"invalid grasp candidate offset list: {exc}") from exc
        if fixed_direction:
            if len(fixed_direction) < 2:
                raise RuntimeError("fixed_approach_direction must contain at least x,y")
            x = fixed_direction[0]
            y = fixed_direction[1]
            norm = math.hypot(x, y)
            if norm < 1e-6:
                raise RuntimeError("fixed_approach_direction xy norm must be non-zero")
            self._fixed_approach_direction = (x / norm, y / norm, 0.0)
        else:
            self._fixed_approach_direction = None

        if any(distance <= 0.0 for distance in self._pick_distance_candidates):
            raise RuntimeError("pick_distances values must be positive")
        max_pick_distance = self.args.max_pick_distance if self.args.use_radius_topic else self.args.pick_distance
        if self._pick_distance_candidates:
            max_pick_distance = max(max_pick_distance, max(self._pick_distance_candidates))
        if self.args.approach_distance <= max_pick_distance:
            raise RuntimeError("approach_distance must be greater than the maximum pick distance")
        if self.args.pick_distance <= 0.0:
            raise RuntimeError("pick_distance must be positive")
        if self.args.surface_clearance < 0.0:
            raise RuntimeError("surface_clearance must be non-negative")
        if self.args.tcp_front_offset < 0.0:
            raise RuntimeError("tcp_front_offset must be non-negative")
        if self.args.min_pick_distance <= 0.0 or self.args.max_pick_distance <= 0.0:
            raise RuntimeError("min_pick_distance and max_pick_distance must be positive")
        if self.args.min_pick_distance > self.args.max_pick_distance:
            raise RuntimeError("min_pick_distance must be <= max_pick_distance")
        if self.args.min_target_distance <= 0.0 or self.args.max_target_distance <= 0.0:
            raise RuntimeError("min_target_distance and max_target_distance must be positive")
        if self.args.min_target_distance >= self.args.max_target_distance:
            raise RuntimeError("min_target_distance must be < max_target_distance")
        for name in ("velocity_scaling", "acceleration_scaling"):
            value = getattr(self.args, name)
            if value <= 0.0 or value > 1.0:
                raise RuntimeError(f"{name} must be in (0.0, 1.0]")
        if self.args.max_cartesian_speed <= 0.0:
            raise RuntimeError("max_cartesian_speed must be positive")
        if self.args.post_motion_position_tolerance <= 0.0:
            raise RuntimeError("post_motion_position_tolerance must be positive")
        if self.args.post_motion_wait_timeout < 0.0:
            raise RuntimeError("post_motion_wait_timeout must be non-negative")
        if self.args.pre_detect_duration <= 0.0:
            raise RuntimeError("pre_detect_duration must be positive")
        if self.args.observation_settle < 0.0:
            raise RuntimeError("observation_settle must be non-negative")
        if self.args.stable_target_samples <= 0:
            raise RuntimeError("stable_target_samples must be positive")
        if self.args.target_stability_tolerance <= 0.0:
            raise RuntimeError("target_stability_tolerance must be positive")
        if self.args.max_approach_base_distance <= 0.0:
            raise RuntimeError("max_approach_base_distance must be positive")
        if self.args.max_pick_base_distance <= 0.0:
            raise RuntimeError("max_pick_base_distance must be positive")
        if self.args.auto_detach_max_distance <= 0.0:
            raise RuntimeError("auto_detach_max_distance must be positive")
        if self.args.nearby_tomato_collision_distance <= 0.0:
            raise RuntimeError("nearby_tomato_collision_distance must be positive")
        if self.args.nearby_tomato_collision_radius < 0.0:
            raise RuntimeError("nearby_tomato_collision_radius must be non-negative")
        if self.args.nearby_tomato_collision_padding < 0.0:
            raise RuntimeError("nearby_tomato_collision_padding must be non-negative")
        if self.args.freeze_basket_z_offset < 0.0:
            raise RuntimeError("freeze_basket_z_offset must be non-negative")
        if self.args.harvested_tomato_collision_radius <= 0.0:
            raise RuntimeError("harvested_tomato_collision_radius must be positive")
        if self.args.gripper_close_width < 0.0 or self.args.gripper_close_width > GRIPPER_OPEN_TOTAL_WIDTH:
            raise RuntimeError(
                f"gripper_close_width must be in [0.0, {GRIPPER_OPEN_TOTAL_WIDTH:.3f}]"
            )
        if self.args.pick_pipeline != PILZ_PIPELINE and self.args.pick_planner_id == "LIN":
            self.args.pick_planner_id = self.args.approach_planner_id
        if self.args.retreat_pipeline != PILZ_PIPELINE and self.args.retreat_planner_id == "LIN":
            self.args.retreat_planner_id = self.args.approach_planner_id

    def run(self):
        self._validate_args()
        self.get_logger().info(
            "Pick pipeline parameters: "
            f"planning_group={self.args.planning_group}, base_frame={self.args.base_frame}, "
            f"ee_link={self.args.ee_link}, approach_distance={self.args.approach_distance:.3f}, "
            f"pick_distance={self.args.pick_distance:.3f}, grasp_roll={self.args.grasp_roll:.3f}, "
            f"pick_distances={self._pick_distance_candidates or [self.args.pick_distance]}, "
            f"surface_clearance={self.args.surface_clearance:.3f}, "
            f"tcp_front_offset={self.args.tcp_front_offset:.3f}, "
            f"gripper_close_width={self.args.gripper_close_width:.3f}, "
            f"use_radius_topic={self.args.use_radius_topic}, tomato_topic={self.args.tomato_topic}, "
            f"auto_detach_nearest={self.args.auto_detach_nearest}, skip_detach={self.args.skip_detach}, "
            f"auto_grasp_candidates={self.args.auto_grasp_candidates}, "
            f"max_pick_candidates={self.args.max_pick_candidates}, "
            f"pick_pipeline={self.args.pick_pipeline}/{self.args.pick_planner_id}, "
            f"retreat_pipeline={self.args.retreat_pipeline}/{self.args.retreat_planner_id}, "
            f"fixed_approach_direction={self._fixed_approach_direction}, "
            f"approach_direction_source={self.args.approach_direction_source}, "
            f"nearby_tomato_avoidance={self.args.avoid_nearby_tomatoes}, "
            f"post_motion_check={self.args.post_motion_position_tolerance:.3f}m/"
            f"{self.args.post_motion_wait_timeout:.1f}s, "
            f"pre_detect_pose={'off' if self.args.skip_pre_detect_pose else self.args.pre_detect_joints}"
        )

        self._wait_for_servers()
        self._wait_for_joint_state()
        self._move_to_pre_detect_pose()

        stable_target = self._manual_tomato_target()
        if stable_target is None:
            stable_target = self._wait_for_stable_tomato_target()
        if stable_target is None:
            return False
        tomato_base, pick_distance = stable_target
        self._configure_auto_detach_target(tomato_base)

        candidates = self._compute_pick_pose_candidates(tomato_base, pick_distance)
        if not candidates:
            self.get_logger().error("Pose computation failed; no arm motion will run")
            return False

        if not self.args.auto_grasp_candidates:
            candidates = candidates[:1]
        max_pick_candidates = max(0, int(self.args.max_pick_candidates))
        if max_pick_candidates and len(candidates) > max_pick_candidates:
            selected_candidates = []
            selected_ids = set()
            selected_angles = set()
            for candidate in candidates:
                angle_key = round(candidate["angle_offset_deg"], 4)
                if angle_key in selected_angles:
                    continue
                selected_candidates.append(candidate)
                selected_ids.add(id(candidate))
                selected_angles.add(angle_key)
                if len(selected_candidates) >= max_pick_candidates:
                    break
            if len(selected_candidates) < max_pick_candidates:
                for candidate in candidates:
                    if id(candidate) in selected_ids:
                        continue
                    selected_candidates.append(candidate)
                    selected_ids.add(id(candidate))
                    if len(selected_candidates) >= max_pick_candidates:
                        break
            self.get_logger().warn(
                f"Limiting pick candidates from {len(candidates)} to {len(selected_candidates)}; "
                "remaining candidates will be skipped for this target"
            )
            candidates = selected_candidates

        for index, candidate in enumerate(candidates, start=1):
            self.get_logger().info(
                f"Trying pick candidate {index}/{len(candidates)}: "
                f"pick_distance={candidate['pick_distance']:.3f}, "
                f"angle={candidate['angle_offset_deg']:.1f}deg, "
                f"z_offset={candidate['z_offset']:.3f}, "
                f"lateral_offset={candidate['lateral_offset']:.3f}"
            )
            if self._execute_two_stage_pick(
                candidate["approach"],
                candidate["pick"],
                candidate["retreat"],
                self._carry_offset_for_candidate(tomato_base, candidate),
            ):
                return True
            self.get_logger().warn(f"Pick candidate {index} failed; trying next candidate")

        self.get_logger().error("All pick candidates failed")
        return False


def _parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("--basket", choices=("good", "bad"), default="good")
    parser.add_argument("--profile", choices=sorted(POSE_PROFILES.keys()), default="empty")
    parser.add_argument("--world-name", default=DEFAULT_WORLD_NAME)
    parser.add_argument("--tomato-model", default="")
    parser.add_argument("--detach-topic", default="")
    parser.add_argument("--place-in-basket", dest="place_in_basket", action="store_true")
    parser.add_argument("--no-place-in-basket", dest="place_in_basket", action="store_false")
    parser.set_defaults(place_in_basket=True)
    parser.add_argument("--skip-detach", action="store_true")
    parser.add_argument("--auto-detach-nearest", dest="auto_detach_nearest", action="store_true")
    parser.add_argument("--no-auto-detach-nearest", dest="auto_detach_nearest", action="store_false")
    parser.set_defaults(auto_detach_nearest=True)
    parser.add_argument("--auto-detach-max-distance", type=float, default=0.30)
    parser.add_argument("--pose-stabilization", dest="pose_stabilization", action="store_true")
    parser.add_argument("--no-pose-stabilization", dest="pose_stabilization", action="store_false")
    parser.set_defaults(pose_stabilization=True)
    parser.add_argument("--carry-pose-stabilization", dest="carry_pose_stabilization", action="store_true")
    parser.add_argument("--no-carry-pose-stabilization", dest="carry_pose_stabilization", action="store_false")
    parser.set_defaults(carry_pose_stabilization=False)
    parser.add_argument("--gripper-attachment", dest="gripper_attachment", action="store_true")
    parser.add_argument("--no-gripper-attachment", dest="gripper_attachment", action="store_false")
    parser.set_defaults(gripper_attachment=True)

    parser.add_argument("--planning-group", "--moveit-group", default=DEFAULT_PLANNING_GROUP)
    parser.add_argument("--base-frame", "--ik-frame", default=DEFAULT_BASE_FRAME)
    parser.add_argument("--ee-link", "--ik-link", default=DEFAULT_EE_LINK)
    parser.add_argument("--approach-distance", "--approach-offset", type=float, default=0.20)
    parser.add_argument("--pick-distance", type=float, default=0.085)
    parser.add_argument("--pick-distances", default="")
    parser.add_argument("--grasp-roll", type=float, default=math.pi / 2.0)
    parser.add_argument("--grasp-lateral-offset", type=float, default=0.0)
    parser.add_argument("--auto-grasp-candidates", dest="auto_grasp_candidates", action="store_true")
    parser.add_argument("--no-auto-grasp-candidates", dest="auto_grasp_candidates", action="store_false")
    parser.set_defaults(auto_grasp_candidates=False)
    parser.add_argument(
        "--grasp-angle-offsets-deg",
        default="0,-25,25,-50,50,-75,75,100,-100,140,-140,180",
    )
    parser.add_argument("--grasp-lateral-offsets", default="")
    parser.add_argument("--grasp-z-offsets", default="0.0,0.035,-0.025,0.070")
    parser.add_argument("--max-pick-candidates", type=int, default=0)
    parser.add_argument("--fixed-approach-direction", default="")
    parser.add_argument(
        "--approach-direction-source",
        choices=("tcp_to_target", "base_to_target"),
        default="tcp_to_target",
    )
    parser.add_argument("--avoid-nearby-tomatoes", dest="avoid_nearby_tomatoes", action="store_true")
    parser.add_argument("--no-avoid-nearby-tomatoes", dest="avoid_nearby_tomatoes", action="store_false")
    parser.set_defaults(avoid_nearby_tomatoes=False)
    parser.add_argument("--nearby-tomato-collision-distance", type=float, default=0.60)
    parser.add_argument("--nearby-tomato-collision-radius", type=float, default=0.0)
    parser.add_argument("--nearby-tomato-collision-padding", type=float, default=0.010)
    parser.add_argument("--nearby-tomato-collision-max-count", type=int, default=18)
    parser.add_argument("--gripper-close-width", type=float, default=0.070)
    parser.add_argument("--tomato-topic", default=DEFAULT_TOMATO_TOPIC)
    parser.add_argument("--tomato-radius-topic", default=DEFAULT_TOMATO_RADIUS_TOPIC)
    parser.add_argument("--picked-topic", default="/tomato_harvest/picked")
    parser.add_argument("--freeze-harvested-tomato", dest="freeze_harvested_tomato", action="store_true")
    parser.add_argument("--no-freeze-harvested-tomato", dest="freeze_harvested_tomato", action="store_false")
    parser.set_defaults(freeze_harvested_tomato=True)
    parser.add_argument("--freeze-basket-z-offset", type=float, default=0.055)
    parser.add_argument(
        "--harvested-tomato-collision-radius",
        type=float,
        default=DEFAULT_HARVESTED_TOMATO_COLLISION_RADIUS,
    )
    parser.add_argument("--lock-harvested-to-basket", dest="lock_harvested_to_basket", action="store_true")
    parser.add_argument("--no-lock-harvested-to-basket", dest="lock_harvested_to_basket", action="store_false")
    parser.set_defaults(lock_harvested_to_basket=False)
    parser.add_argument("--manual-target-frame", default="")
    parser.add_argument("--manual-target-x", type=float, default=None)
    parser.add_argument("--manual-target-y", type=float, default=None)
    parser.add_argument("--manual-target-z", type=float, default=None)
    parser.add_argument("--use-radius-topic", action="store_true")
    parser.add_argument("--require-radius", action="store_true")
    parser.add_argument("--surface-clearance", "--grasp-surface-clearance", dest="surface_clearance", type=float, default=0.030)
    parser.add_argument("--tcp-front-offset", type=float, default=0.020)
    parser.add_argument("--min-pick-distance", type=float, default=0.055)
    parser.add_argument("--max-pick-distance", type=float, default=0.095)
    parser.add_argument("--min-target-distance", type=float, default=0.08)
    parser.add_argument("--max-target-distance", type=float, default=1.20)
    parser.add_argument("--velocity-scaling", type=float, default=0.20)
    parser.add_argument("--acceleration-scaling", type=float, default=0.20)

    parser.add_argument("--approach-pipeline", default="ompl")
    parser.add_argument("--approach-planner-id", default="RRTConnectkConfigDefault")
    parser.add_argument("--pick-pipeline", default=PILZ_PIPELINE)
    parser.add_argument("--pick-planner-id", default="LIN")
    parser.add_argument("--pick-fallback-pipeline", default="")
    parser.add_argument("--pick-fallback-planner-id", default="")
    parser.add_argument("--retreat-pipeline", default=PILZ_PIPELINE)
    parser.add_argument("--retreat-planner-id", default="LIN")
    parser.add_argument("--move-action", default="/move_action")
    parser.add_argument("--move-action-timeout", type=float, default=45.0)
    parser.add_argument("--max-cartesian-speed", type=float, default=0.12)
    parser.add_argument("--post-motion-position-tolerance", type=float, default=0.04)
    parser.add_argument("--post-motion-wait-timeout", type=float, default=4.0)
    parser.add_argument("--position-tolerance", type=float, default=0.01)
    parser.add_argument("--orientation-tolerance", type=float, default=0.04)
    parser.add_argument("--allowed-planning-time", type=float, default=8.0)
    parser.add_argument("--num-planning-attempts", type=int, default=5)
    parser.add_argument("--replan", action="store_true")
    parser.add_argument("--replan-attempts", type=int, default=1)
    parser.add_argument("--tf-timeout", type=float, default=2.0)
    parser.add_argument("--use-latest-tomato-tf", dest="use_latest_tomato_tf", action="store_true")
    parser.add_argument("--no-use-latest-tomato-tf", dest="use_latest_tomato_tf", action="store_false")
    parser.set_defaults(use_latest_tomato_tf=True)
    parser.add_argument("--pre-detect-joints", default=DEFAULT_PRE_DETECT_JOINTS)
    parser.add_argument("--skip-pre-detect-pose", action="store_true")
    parser.add_argument("--pre-detect-duration", type=float, default=3.0)
    parser.add_argument("--observation-settle", type=float, default=1.0)
    parser.add_argument("--stable-target-samples", type=int, default=4)
    parser.add_argument("--target-stability-tolerance", type=float, default=0.06)
    parser.add_argument("--max-approach-base-distance", type=float, default=1.10)
    parser.add_argument("--max-pick-base-distance", type=float, default=1.18)
    parser.add_argument("--tomato-timeout", type=float, default=15.0)
    parser.add_argument("--tomato-radius-timeout", type=float, default=3.0)
    parser.add_argument("--joint-state-timeout", type=float, default=15.0)

    # Legacy launch arguments accepted for compatibility with older commands.
    parser.add_argument("--world-file", default="")
    parser.add_argument("--robot-x", type=float, default=37.62)
    parser.add_argument("--robot-y", type=float, default=8.50)
    parser.add_argument("--robot-z", type=float, default=0.20)
    parser.add_argument("--robot-yaw", type=float, default=1.56)
    parser.add_argument("--robot-pitch", type=float, default=0.09)
    parser.add_argument("--target-prefix", default="")
    parser.add_argument("--target-side", default="")
    parser.add_argument("--min-forward", type=float, default=0.0)
    parser.add_argument("--max-forward", type=float, default=0.0)
    parser.add_argument("--max-lateral", type=float, default=0.0)
    parser.add_argument("--max-picks", type=int, default=1)
    parser.add_argument("--min-z", type=float, default=0.0)
    parser.add_argument("--max-z", type=float, default=0.0)
    parser.add_argument("--grasp-local-x", type=float, default=0.0)
    parser.add_argument("--grasp-local-y", type=float, default=0.0)
    parser.add_argument("--grasp-local-z", type=float, default=0.0)
    parser.add_argument("--grasp-tolerance", type=float, default=0.0)
    parser.add_argument("--grasp-offset", type=float, default=0.0)
    parser.add_argument("--grip-clearance", type=float, default=0.0)
    parser.add_argument("--pre-grasp-z-offset", type=float, default=0.0)
    parser.add_argument("--lift-offset", type=float, default=0.0)
    return parser.parse_args(remove_ros_args(args=argv)[1:])


def main():
    args = _parse_args(sys.argv)
    rclpy.init(args=sys.argv)
    node = GreenhouseNearestPickPlace(args)
    exit_code = 0
    try:
        ok = node.run()
        if not ok:
            node.get_logger().error("Pick pipeline did not complete")
            exit_code = 1
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted")
        exit_code = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
