import argparse
import math
import re
import subprocess
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint


ARM_JOINT_NAMES = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
]

GRIPPER_JOINT_NAMES = [
    "fr3_finger_joint1",
    "fr3_finger_joint2",
]

POSE_PROFILES = {
    "empty": {
        "home": [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
        "pre_grasp": [-0.033, -0.169, 0.330, -3.000, 0.673, 3.700, 0.0],
        "grasp_mid": [-0.155, 0.036, 0.191, -2.787, 0.102, 3.700, 0.0],
        "grasp": [-0.161, 0.207, 0.160, -2.572, 0.002, 3.700, 0.0],
        "lift": [-0.207, -0.421, 0.213, -2.996, -0.019, 3.700, 0.0],
        "swing_transit": [1.45, -0.40, 0.0, -2.50, 0.0, 1.60, 0.0],
        "good_hover": [2.890, 0.097, 0.235, -2.149, 0.0, 2.531, 0.0],
        "good_drop": [2.890, 0.347, 0.235, -2.149, 0.0, 2.531, 0.0],
        "bad_hover": [2.890, 0.753, 0.266, -1.300, -0.078, 2.791, 0.0],
        "bad_drop": [2.890, 1.003, 0.266, -1.300, -0.078, 2.791, 0.0],
    },
    "greenhouse": {
        "home": [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
        "pre_grasp": [0.866, -0.916, 0.578, -2.803, 2.487, 2.467, 0.0],
        "grasp_mid": [0.743, -0.655, 0.760, -2.555, 2.800, 2.557, 0.0],
        "grasp": [0.546, -0.401, 0.973, -2.297, 2.800, 2.606, 0.0],
        "lift": [1.216, -0.825, 0.383, -2.433, 2.528, 3.023, 0.0],
        "swing_transit": [1.45, -0.40, 0.0, -2.50, 0.0, 1.60, 0.0],
        "good_hover": [1.429, -0.600, 1.323, -1.689, 0.572, 1.536, 0.0],
        "good_drop": [1.427, -0.697, 1.451, -1.983, 0.713, 1.800, 0.0],
        "bad_hover": [2.890, 0.753, 0.266, -1.300, -0.078, 2.791, 0.0],
        "bad_drop": [2.890, 1.003, 0.266, -1.300, -0.078, 2.791, 0.0],
    },
}

GRIPPER_OPEN_TOTAL_WIDTH = 0.08
GRIPPER_CLOSED_TOTAL_WIDTH = 0.0
DEFAULT_WORLD_NAME = "pick_place_detachable_test"
DEFAULT_TOMATO_MODEL = "tomato_ripe_pick_0"
DEFAULT_POSE_PROFILE = "empty"
BASKET_SETTLE_POSES = {
    "good": (-0.55, 0.0, 0.18),
    "bad": (-0.90, 0.0, 0.18),
}
DEFAULT_ROBOT_MODEL_NAME = "combined_robot"
DEFAULT_ATTACHMENT_LINK = "fr3_hand_tcp"
DEFAULT_GRIPPER_ATTACH_PARENT_LINK = "fr3_link7"


def gazebo_gripper_parent_link(link_name):
    """Return the Gazebo link that survives URDF fixed-joint reduction."""
    normalized = (link_name or "").strip()
    if normalized in ("fr3_hand", "fr3_hand_tcp", "fr3_camera_link"):
        return DEFAULT_GRIPPER_ATTACH_PARENT_LINK
    return normalized or DEFAULT_GRIPPER_ATTACH_PARENT_LINK


def _rotate_vector_by_quaternion(q, v):
    x, y, z = v
    qx = q.x
    qy = q.y
    qz = q.z
    qw = q.w

    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    return (
        x + qw * tx + (qy * tz - qz * ty),
        y + qw * ty + (qz * tx - qx * tz),
        z + qw * tz + (qx * ty - qy * tx),
    )


def _duration(seconds):
    whole = int(seconds)
    return Duration(sec=whole, nanosec=int((seconds - whole) * 1e9))


class DetachablePickPlace(Node):
    def __init__(
        self,
        basket,
        detach_topic,
        skip_detach,
        stabilize_pose,
        world_name,
        tomato_model,
        pose_profile,
        world_x,
        world_y,
        world_z,
        world_yaw,
    ):
        super().__init__("detachable_pick_place")
        self.basket = basket
        self.detach_topic = detach_topic
        self.skip_detach = skip_detach
        self.stabilize_pose = stabilize_pose
        self.world_name = world_name
        self.tomato_model = tomato_model
        self.poses = POSE_PROFILES[pose_profile]
        self.world_x = world_x
        self.world_y = world_y
        self.world_z = world_z
        self.world_yaw = world_yaw
        self.robot_model_name = DEFAULT_ROBOT_MODEL_NAME
        self.robot_base_frame = "base_link"
        self.robot_tcp_link = DEFAULT_ATTACHMENT_LINK
        self.gripper_attach_parent_link = gazebo_gripper_parent_link(
            DEFAULT_GRIPPER_ATTACH_PARENT_LINK
        )
        self.enable_gripper_attachment = True
        self.carry_tcp_offset = (0.0, 0.0, 0.0)
        self._carry_active = False
        self._carry_warning_logged = False
        self._attachment_active = False
        self._attachment_backend = ""
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.05, self._carry_pose_update)
        self.arm_client = ActionClient(
            self, FollowJointTrajectory, "/fr3_arm_controller/follow_joint_trajectory"
        )
        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/fr3_gripper_controller/follow_joint_trajectory",
        )

    def set_target(self, tomato_model, detach_topic):
        self.tomato_model = tomato_model
        self.detach_topic = detach_topic
        self._carry_active = False
        self._carry_warning_logged = False
        self._attachment_active = False
        self._attachment_backend = ""

    def _gz_topic_exists(self, topic, timeout=4.0):
        command = [
            "gz",
            "topic",
            "-i",
            "-t",
            topic,
        ]
        try:
            result = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=timeout,
            )
        except (subprocess.TimeoutExpired, OSError):
            return False
        if result.returncode != 0:
            return False
        info = "\n".join([result.stdout, result.stderr])
        has_subscriber = "Subscribers" in info and "No subscribers" not in info
        has_publisher = "Publishers" in info and "No publishers" not in info
        return has_subscriber or has_publisher

    def _wait_for_gz_topic(self, topic, timeout=8.0, poll_interval=0.25):
        deadline = time.monotonic() + max(0.0, timeout)
        while True:
            if self._gz_topic_exists(topic, timeout=min(0.75, max(0.1, timeout))):
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(poll_interval)

    def _publish_gz_empty(self, topic, label, attempts=1, retry_delay=0.12):
        command = [
            "gz",
            "topic",
            "-t",
            topic,
            "-m",
            "gz.msgs.Empty",
            "-p",
            "",
        ]
        attempts = max(1, int(attempts))
        for attempt in range(1, attempts + 1):
            suffix = f" ({attempt}/{attempts})" if attempts > 1 else ""
            self.get_logger().info(f"Publishing Gazebo {label} topic: {topic}{suffix}")
            result = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=5.0,
            )
            if result.returncode == 0:
                if attempt < attempts:
                    time.sleep(retry_delay)
                continue

            stderr = result.stderr.strip()
            self.get_logger().warn(
                f"Gazebo {label} publish failed: {stderr or result.returncode}"
            )
            return False
        return True

    @staticmethod
    def _safe_gz_name(name):
        return "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in str(name))

    def _call_contact_attach_service(self, action, fields, label, timeout_ms=3000):
        service = f"/world/{self.world_name}/contact_attach/{action}"
        payload = ",".join(f"{key}={value}" for key, value in fields.items())
        command = [
            "gz",
            "service",
            "-s",
            service,
            "--reqtype",
            "gz.msgs.StringMsg",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            str(timeout_ms),
            "--req",
            f'data: "{payload}"',
        ]
        try:
            result = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=max(2.0, timeout_ms / 1000.0 + 1.0),
            )
        except (subprocess.TimeoutExpired, OSError) as exc:
            self.get_logger().warn(f"{label}: contact fixed-joint service failed: {exc}")
            return False

        output = "\n".join([result.stdout, result.stderr]).strip()
        ok = result.returncode == 0 and "data: true" in output
        if not ok:
            self.get_logger().warn(
                f"{label}: contact fixed-joint service rejected request: "
                f"{output or result.returncode}"
            )
        return ok

    def _attach_model_to_robot_link(self, child_model, parent_link, label):
        joint_name = f"{label}_{self._safe_gz_name(child_model)}"
        return self._call_contact_attach_service(
            "attach",
            {
                "parent_model": self.robot_model_name,
                "parent_link": gazebo_gripper_parent_link(parent_link),
                "child_model": child_model,
                "child_link": "link",
                "joint_name": joint_name,
            },
            label,
        )

    def _detach_model_from_contact_joint(self, child_model, label):
        return self._call_contact_attach_service(
            "detach",
            {
                "child_model": child_model,
                "child_link": "link",
            },
            label,
        )

    def _attach_tomato_to_gripper(self):
        if self._attachment_active:
            return True
        if not getattr(self, "enable_gripper_attachment", True):
            self.get_logger().info("Gazebo gripper attachment disabled; using physical gripper contact")
            return False
        if self.skip_detach:
            self.get_logger().warn("Skipping gripper attachment by request")
            return False

        if self._attach_model_to_robot_link(
            self.tomato_model,
            self.gripper_attach_parent_link,
            "gripper_contact_attach",
        ):
            self._attachment_active = True
            self._attachment_backend = "contact_fixed_joint"
            self.get_logger().info(
                f"Contact-triggered fixed joint attached {self.tomato_model} to "
                f"{self.robot_model_name}::{self.gripper_attach_parent_link}"
            )
            return True

        self.get_logger().warn(
            "Contact fixed-joint attach is unavailable; falling back to Gazebo "
            "DetachableJoint topic path."
        )

        topic = f"/gripper_attach/{self.tomato_model}"
        if not self._gz_topic_exists(topic):
            if not self._add_gripper_attach_plugin():
                self.get_logger().warn(
                    f"Runtime gripper attachment plugin did not expose topic yet: {topic}"
                )
        if not self._wait_for_gz_topic(topic, timeout=20.0):
            self.get_logger().warn(
                f"Gazebo gripper attachment topic is not available: {topic}"
            )
            return False
        self.get_logger().info(
            "Publishing Gazebo gripper attach topic: "
            f"{topic} ({self.robot_model_name}::{self.gripper_attach_parent_link} -> "
            f"{self.tomato_model}::link)"
        )
        if not self._publish_gz_empty(topic, "gripper attach"):
            return False
        self._attachment_active = True
        self._attachment_backend = "detachable_joint_topic"
        return True

    def _add_gripper_attach_plugin(self):
        attach_topic = f"/gripper_attach/{self.tomato_model}"
        detach_topic = f"/gripper_detach/{self.tomato_model}"
        state_topic = f"/gripper_attach_state/{self.tomato_model}"
        robot_model_id = self._gazebo_model_id(self.robot_model_name)
        if robot_model_id is None:
            self.get_logger().warn(
                f"Could not resolve Gazebo entity id for model {self.robot_model_name}"
            )
            return False
        innerxml = (
            f"<parent_link>{self.gripper_attach_parent_link}</parent_link>"
            f"<child_model>{self.tomato_model}</child_model>"
            "<child_link>link</child_link>"
            f"<detach_topic>{detach_topic}</detach_topic>"
            f"<attach_topic>{attach_topic}</attach_topic>"
            f"<output_topic>{state_topic}</output_topic>"
        )
        request = (
            f'entity {{ id: {robot_model_id} name: "{self.robot_model_name}" type: MODEL }} '
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
        self.get_logger().info(
            "Adding runtime Gazebo gripper attachment plugin: "
            f"{self.robot_model_name}::{self.gripper_attach_parent_link} -> "
            f"{self.tomato_model}::link"
        )
        try:
            result = subprocess.run(
                command, check=False, capture_output=True, text=True, timeout=7.0
            )
        except (subprocess.TimeoutExpired, OSError) as exc:
            self.get_logger().warn(f"Runtime gripper attachment plugin add failed: {exc}")
            if self._wait_for_gz_topic(attach_topic, timeout=20.0):
                self.get_logger().info(
                    "Runtime gripper attachment plugin topic appeared after timeout"
                )
                return True
            return False

        if result.returncode != 0 or "data: true" not in result.stdout:
            stderr = result.stderr.strip()
            stdout = result.stdout.strip()
            self.get_logger().warn(
                "Runtime gripper attachment plugin add was not accepted: "
                f"{stderr or stdout or result.returncode}"
            )
            if self._wait_for_gz_topic(attach_topic, timeout=12.0):
                self.get_logger().info(
                    "Runtime gripper attachment plugin topic is available despite service warning"
                )
                return True
            return False

        return self._wait_for_gz_topic(attach_topic, timeout=20.0)

    def _gazebo_model_id(self, model_name):
        command = [
            "gz",
            "service",
            "-s",
            f"/world/{self.world_name}/scene/info",
            "--reqtype",
            "gz.msgs.Empty",
            "--reptype",
            "gz.msgs.Scene",
            "--timeout",
            "2000",
            "--req",
            "",
        ]
        try:
            result = subprocess.run(
                command, check=False, capture_output=True, text=True, timeout=4.0
            )
        except (subprocess.TimeoutExpired, OSError):
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

    def _detach_tomato_from_gripper(self):
        if not self._attachment_active:
            return True
        if self._attachment_backend == "contact_fixed_joint":
            detached = self._detach_model_from_contact_joint(
                self.tomato_model,
                "gripper_contact_detach",
            )
            self._attachment_active = False
            self._attachment_backend = ""
            return detached
        topic = f"/gripper_detach/{self.tomato_model}"
        detached = self._publish_gz_empty(topic, "gripper detach")
        self._attachment_active = False
        self._attachment_backend = ""
        return detached

    def _refresh_robot_world_pose_from_gazebo(self):
        command = [
            "gz",
            "topic",
            "-e",
            "-n",
            "1",
            "-t",
            f"/world/{self.world_name}/dynamic_pose/info",
        ]
        try:
            result = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=4.0,
            )
        except (subprocess.TimeoutExpired, OSError) as exc:
            self.get_logger().warn(
                "Could not query Gazebo robot pose; using configured "
                f"robot pose ({self.world_x:.3f}, {self.world_y:.3f}, "
                f"{self.world_z:.3f}, yaw={self.world_yaw:.3f}): {exc}"
            )
            return False

        if result.returncode != 0:
            stderr = result.stderr.strip()
            self.get_logger().warn(
                "Could not query Gazebo robot pose; using configured "
                f"robot pose ({self.world_x:.3f}, {self.world_y:.3f}, "
                f"{self.world_z:.3f}, yaw={self.world_yaw:.3f}): "
                f"{stderr or result.returncode}"
            )
            return False

        pose = self._parse_gazebo_model_pose(
            result.stdout,
            self.robot_model_name,
        )
        if pose is None:
            self.get_logger().warn(
                f"Gazebo pose topic did not include {self.robot_model_name}; "
                "using configured robot pose"
            )
            return False

        self.world_x, self.world_y, self.world_z, self.world_yaw = pose
        self.get_logger().info(
            "Using Gazebo robot pose for basket/carry transforms: "
            f"x={self.world_x:.3f}, y={self.world_y:.3f}, "
            f"z={self.world_z:.3f}, yaw={self.world_yaw:.3f}"
        )
        return True

    @staticmethod
    def _parse_gazebo_model_pose(text, model_name):
        block = []
        depth = 0
        in_pose = False
        for line in text.splitlines():
            stripped = line.strip()
            if not in_pose and stripped == "pose {":
                block = [line]
                depth = 1
                in_pose = True
                continue
            if not in_pose:
                continue

            block.append(line)
            depth += stripped.count("{") - stripped.count("}")
            if depth > 0:
                continue

            block_text = "\n".join(block)
            in_pose = False
            if f'name: "{model_name}"' not in block_text:
                continue

            x = DetachablePickPlace._extract_gz_pose_value(block_text, "position", "x")
            y = DetachablePickPlace._extract_gz_pose_value(block_text, "position", "y")
            z = DetachablePickPlace._extract_gz_pose_value(block_text, "position", "z")
            qx = DetachablePickPlace._extract_gz_pose_value(block_text, "orientation", "x")
            qy = DetachablePickPlace._extract_gz_pose_value(block_text, "orientation", "y")
            qz = DetachablePickPlace._extract_gz_pose_value(block_text, "orientation", "z")
            qw = DetachablePickPlace._extract_gz_pose_value(
                block_text,
                "orientation",
                "w",
                default=1.0,
            )
            yaw = math.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy * qy + qz * qz),
            )
            return (x, y, z, yaw)
        return None

    @staticmethod
    def _extract_gz_pose_value(block_text, section, field, default=0.0):
        section_match = re.search(rf"{section}\s*\{{(.*?)\n\s*\}}", block_text, re.S)
        if not section_match:
            return default
        field_match = re.search(
            rf"{field}:\s*([-+0-9.eE]+)",
            section_match.group(1),
        )
        if not field_match:
            return default
        return float(field_match.group(1))

    def _wait_for_servers(self):
        for name, client in (
            ("arm", self.arm_client),
            ("gripper", self.gripper_client),
        ):
            self.get_logger().info(f"Waiting for {name} action server...")
            if not client.wait_for_server(timeout_sec=30.0):
                raise RuntimeError(f"{name} action server is not available")
        self._refresh_robot_world_pose_from_gazebo()

    def _send_trajectory(self, client, joint_names, waypoints, total_time, label):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)

        step_time = total_time / float(len(waypoints))
        for index, positions in enumerate(waypoints, start=1):
            point = JointTrajectoryPoint()
            point.positions = list(positions)
            point.time_from_start = _duration(step_time * index)
            goal.trajectory.points.append(point)

        self.get_logger().info(f"{label}: sending {len(waypoints)} waypoint(s)")
        goal_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"{label}: goal rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        if getattr(result, "error_code", 0) != 0:
            raise RuntimeError(f"{label}: controller error_code={result.error_code}")
        self.get_logger().info(f"{label}: done")

    def _arm(self, pose_names, total_time, label):
        self._send_trajectory(
            self.arm_client,
            ARM_JOINT_NAMES,
            [self.poses[name] for name in pose_names],
            total_time,
            label,
        )

    def _gripper(self, total_width, label):
        half_width = total_width / 2.0
        self._send_trajectory(
            self.gripper_client,
            GRIPPER_JOINT_NAMES,
            [[half_width, half_width]],
            1.2,
            label,
        )

    def _detach_tomato(self):
        if self.skip_detach:
            self.get_logger().warn("Skipping detachable joint publish by request")
            return

        command = [
            "gz",
            "topic",
            "-t",
            self.detach_topic,
            "-m",
            "gz.msgs.Empty",
            "-p",
            "",
        ]
        for attempt in range(1, 4):
            suffix = f" ({attempt}/3)" if attempt > 1 else ""
            self.get_logger().info(f"Publishing Gazebo detach topic: {self.detach_topic}{suffix}")
            result = subprocess.run(
                command,
                check=False,
                capture_output=True,
                text=True,
                timeout=5.0,
            )
            if result.returncode != 0:
                stderr = result.stderr.strip()
                raise RuntimeError(f"detach publish failed: {stderr or result.returncode}")
            if attempt < 3:
                time.sleep(0.12)
        self.get_logger().info("Detach publish complete")

    def _local_to_world(self, pose):
        x, y, z = pose
        cos_yaw = math.cos(self.world_yaw)
        sin_yaw = math.sin(self.world_yaw)
        return (
            self.world_x + cos_yaw * x - sin_yaw * y,
            self.world_y + sin_yaw * x + cos_yaw * y,
            self.world_z + z,
        )

    def _set_tomato_pose(self, pose, label, repeats=1, interval=0.1):
        if not self.stabilize_pose:
            return

        x, y, z = self._local_to_world(pose)
        request = (
            f'name: "{self.tomato_model}", '
            f"position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, "
            "orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
        )
        command = [
            "gz",
            "service",
            "-s",
            f"/world/{self.world_name}/set_pose",
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "1000",
            "--req",
            request,
        ]

        self.get_logger().info(f"{label}: stabilizing {self.tomato_model}")
        for index in range(repeats):
            try:
                result = subprocess.run(
                    command,
                    check=False,
                    capture_output=True,
                    text=True,
                    timeout=3.0,
                )
                if result.returncode != 0:
                    stderr = result.stderr.strip()
                    self.get_logger().warn(
                        f"{label}: set_pose failed (attempt {index+1}/{repeats}): "
                        f"{stderr or result.returncode}"
                    )
                elif "data: true" not in result.stdout:
                    self.get_logger().warn(
                        f"{label}: set_pose empty response (attempt {index+1}/{repeats})"
                    )
            except subprocess.TimeoutExpired:
                self.get_logger().warn(
                    f"{label}: set_pose timed out (attempt {index+1}/{repeats})"
                )
            if index + 1 < repeats:
                time.sleep(interval)

    def _get_tcp_world_pose(self):
        """Get the carried tomato center in world frame, or None if TF unavailable."""
        base_frame = getattr(self, "robot_base_frame", "base_link")
        tcp_link = getattr(self, "robot_tcp_link", DEFAULT_ATTACHMENT_LINK)
        try:
            transform = self.tf_buffer.lookup_transform(base_frame, tcp_link, Time())
        except TransformException:
            return None
        t = transform.transform.translation
        offset = getattr(self, "carry_tcp_offset", (0.0, 0.0, 0.0))
        ox, oy, oz = _rotate_vector_by_quaternion(transform.transform.rotation, offset)
        return self._local_to_world((t.x + ox, t.y + oy, t.z + oz))

    def _carry_pose_update(self):
        if not self.stabilize_pose or not self._carry_active:
            return
        if self._attachment_active:
            return

        world_pose = self._get_tcp_world_pose()
        if world_pose is None:
            if not self._carry_warning_logged:
                self.get_logger().warn("Carry TF unavailable")
                self._carry_warning_logged = True
            return

        x, y, z = world_pose
        request = (
            f'name: "{self.tomato_model}", '
            f"position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, "
            "orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
        )
        command = [
            "gz",
            "service",
            "-s",
            f"/world/{self.world_name}/set_pose",
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "500",
            "--req",
            request,
        ]
        subprocess.run(command, check=False, capture_output=True, text=True, timeout=1.0)

    def _set_tomato_pose_at_tcp(self, label, repeats=5, interval=0.15):
        """Force the tomato to the current TCP world position, repeated for stability."""
        if not self.stabilize_pose:
            return
        if self._attachment_active:
            self.get_logger().info(
                f"{label}: skipping TCP set_pose because Gazebo gripper attachment is active"
            )
            return

        self.get_logger().info(f"{label}: stabilizing {self.tomato_model} at TCP")
        for index in range(repeats):
            world_pose = self._get_tcp_world_pose()
            if world_pose is None:
                self.get_logger().warn(f"{label}: TF unavailable on attempt {index+1}")
                time.sleep(interval)
                continue

            x, y, z = world_pose
            request = (
                f'name: "{self.tomato_model}", '
                f"position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, "
                "orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
            )
            command = [
                "gz",
                "service",
                "-s",
                f"/world/{self.world_name}/set_pose",
                "--reqtype",
                "gz.msgs.Pose",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "1000",
                "--req",
                request,
            ]
            result = subprocess.run(
                command, check=False, capture_output=True, text=True, timeout=3.0
            )
            if result.returncode != 0:
                self.get_logger().warn(
                    f"{label}: set_pose failed on attempt {index+1}"
                )
            if index + 1 < repeats:
                time.sleep(interval)

    def _settle_tomato_in_basket(self, label, repeats=5):
        self._set_tomato_pose(BASKET_SETTLE_POSES[self.basket], label, repeats=repeats)

    def run_pick(self):
        hover = f"{self.basket}_hover"
        drop = f"{self.basket}_drop"

        self.get_logger().info(
            f"Starting detachable tomato pick-place sequence for {self.tomato_model}"
        )
        self._arm(["home"], 5.0, "home")
        time.sleep(0.5)

        self._gripper(GRIPPER_OPEN_TOTAL_WIDTH, "open gripper")
        time.sleep(0.3)

        self._arm(["pre_grasp"], 3.5, "pre-grasp")
        time.sleep(0.3)

        self._arm(["grasp_mid", "grasp"], 3.2, "descend to tomato")
        time.sleep(0.3)

        self._gripper(GRIPPER_CLOSED_TOTAL_WIDTH, "close gripper firmly")
        time.sleep(1.0)

        gripper_attached = self._attach_tomato_to_gripper()
        if gripper_attached:
            self.get_logger().info(
                f"Attached {self.tomato_model} to "
                f"{self.robot_model_name}::{self.gripper_attach_parent_link} before plant detach"
            )
        self._detach_tomato()
        time.sleep(0.15)
        if not gripper_attached and self.stabilize_pose:
            self._carry_active = True
        time.sleep(0.3)

        self._arm(["lift"], 2.5, "lift detached tomato")
        time.sleep(0.3)

        self._arm(["swing_transit", hover], 6.0, f"swing to {self.basket} basket")
        time.sleep(0.3)

        self._arm([drop], 2.0, f"lower into {self.basket} basket")
        time.sleep(0.3)

        self._detach_tomato_from_gripper()
        self._carry_active = False
        self._settle_tomato_in_basket("pre-release tomato placement", repeats=3)
        time.sleep(0.2)

        self._gripper(GRIPPER_OPEN_TOTAL_WIDTH, "release tomato")
        time.sleep(0.3)

        self._settle_tomato_in_basket("post-release tomato placement", repeats=8)
        time.sleep(0.3)

        self._arm([hover, "swing_transit"], 4.0, "retract")
        time.sleep(0.3)

        self._settle_tomato_in_basket("post-retract tomato placement", repeats=5)

        self._arm(["home"], 4.0, "return home")
        self._settle_tomato_in_basket("final tomato placement", repeats=5)
        self.get_logger().info(
            f"Detachable tomato pick-place sequence complete for {self.tomato_model}"
        )

    def run(self):
        self._wait_for_servers()
        self.run_pick()


def _parse_args(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument("basket", nargs="?", choices=("good", "bad"), default="good")
    parser.add_argument(
        "--detach-topic",
        default="/detach/tomato_ripe_pick_0",
        help="Gazebo Empty topic consumed by the DetachableJoint system.",
    )
    parser.add_argument(
        "--skip-detach",
        action="store_true",
        help="Run the arm and gripper sequence without publishing the detach topic.",
    )
    parser.add_argument(
        "--no-pose-stabilization",
        dest="pose_stabilization",
        action="store_false",
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--pose-stabilization",
        dest="pose_stabilization",
        action="store_true",
        help="Enable Gazebo set_pose carry/settling. This is a visual/debug aid, not physical pick-place.",
    )
    parser.add_argument(
        "--world-name",
        default=DEFAULT_WORLD_NAME,
        help="Gazebo world name used for the /world/<name>/set_pose service.",
    )
    parser.add_argument(
        "--tomato-model",
        default=DEFAULT_TOMATO_MODEL,
        help="Gazebo model name of the tomato to settle after release.",
    )
    parser.add_argument(
        "--profile",
        choices=sorted(POSE_PROFILES.keys()),
        default=DEFAULT_POSE_PROFILE,
        help="Named arm pose profile for the target tomato height.",
    )
    parser.add_argument(
        "--world-x",
        type=float,
        default=0.0,
        help="World X offset of the robot/base frame used by Gazebo set_pose.",
    )
    parser.add_argument(
        "--world-y",
        type=float,
        default=0.0,
        help="World Y offset of the robot/base frame used by Gazebo set_pose.",
    )
    parser.add_argument(
        "--world-z",
        type=float,
        default=0.0,
        help="World Z offset of the robot/base frame used by Gazebo set_pose.",
    )
    parser.add_argument(
        "--world-yaw",
        type=float,
        default=0.0,
        help="World yaw offset of the robot/base frame used by Gazebo set_pose.",
    )
    parser.set_defaults(pose_stabilization=False)
    return parser.parse_args(remove_ros_args(args=argv)[1:])


def main():
    args = _parse_args(sys.argv)
    rclpy.init(args=sys.argv)
    node = DetachablePickPlace(
        args.basket,
        args.detach_topic,
        args.skip_detach,
        args.pose_stabilization,
        args.world_name,
        args.tomato_model,
        args.profile,
        args.world_x,
        args.world_y,
        args.world_z,
        args.world_yaw,
    )
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
