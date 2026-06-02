#!/usr/bin/env python3
"""Small Tk GUI for Cartesian FR3 TCP jogs through MoveIt."""

from __future__ import annotations

import math
import threading
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.duration import Duration as RclpyDuration
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.msg import MoveItErrorCodes
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectoryPoint


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _quat_normalize(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(v * v for v in q))
    if norm <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return tuple(v / norm for v in q)  # type: ignore[return-value]


def _quat_multiply(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return _quat_normalize(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )
    )


def _axis_angle_quat(axis: str, angle_rad: float) -> tuple[float, float, float, float]:
    half = 0.5 * angle_rad
    s = math.sin(half)
    c = math.cos(half)
    if axis == "x":
        return s, 0.0, 0.0, c
    if axis == "y":
        return 0.0, s, 0.0, c
    return 0.0, 0.0, s, c


def _quat_to_rpy(q: tuple[float, float, float, float]) -> tuple[float, float, float]:
    x, y, z, w = _quat_normalize(q)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class ArmCartesianGui(Node):
    def __init__(self) -> None:
        super().__init__("arm_cartesian_gui")

        self.declare_parameter("planning_group", "fr3_arm")
        self.declare_parameter("base_frame", "fr3_link0")
        self.declare_parameter("ee_link", "fr3_hand_tcp")
        self.declare_parameter("move_action", "/move_action")
        self.declare_parameter("pipeline_id", "ompl")
        self.declare_parameter("planner_id", "RRTConnectkConfigDefault")
        self.declare_parameter("num_planning_attempts", 5)
        self.declare_parameter("allowed_planning_time", 5.0)
        self.declare_parameter("velocity_scaling", 0.10)
        self.declare_parameter("acceleration_scaling", 0.10)
        self.declare_parameter("position_tolerance", 0.006)
        self.declare_parameter("orientation_tolerance", 0.06)
        self.declare_parameter("replan", True)
        self.declare_parameter("replan_attempts", 2)
        self.declare_parameter("move_action_timeout", 45.0)
        self.declare_parameter("linear_step_m", 0.02)
        self.declare_parameter("angular_step_deg", 5.0)
        self.declare_parameter("gripper_action", "/fr3_gripper_controller/follow_joint_trajectory")
        self.declare_parameter("gripper_open_width", 0.08)
        self.declare_parameter("gripper_close_width", 0.02)
        self.declare_parameter("gripper_duration", 1.0)

        self.planning_group = self._str_param("planning_group")
        self.base_frame = self._str_param("base_frame")
        self.ee_link = self._str_param("ee_link")
        self.move_action = self._str_param("move_action")
        self.pipeline_id = self._str_param("pipeline_id")
        self.planner_id = self._str_param("planner_id")
        self.num_planning_attempts = int(self.get_parameter("num_planning_attempts").value)
        self.allowed_planning_time = float(self.get_parameter("allowed_planning_time").value)
        self.velocity_scaling = float(self.get_parameter("velocity_scaling").value)
        self.acceleration_scaling = float(self.get_parameter("acceleration_scaling").value)
        self.position_tolerance = float(self.get_parameter("position_tolerance").value)
        self.orientation_tolerance = float(self.get_parameter("orientation_tolerance").value)
        self.replan = bool(self.get_parameter("replan").value)
        self.replan_attempts = int(self.get_parameter("replan_attempts").value)
        self.move_action_timeout = float(self.get_parameter("move_action_timeout").value)
        self.default_linear_step = float(self.get_parameter("linear_step_m").value)
        self.default_angular_step_deg = float(self.get_parameter("angular_step_deg").value)
        self.gripper_action = self._str_param("gripper_action")
        self.gripper_open_width = float(self.get_parameter("gripper_open_width").value)
        self.gripper_close_width = float(self.get_parameter("gripper_close_width").value)
        self.gripper_duration = float(self.get_parameter("gripper_duration").value)

        self.move_group_client = ActionClient(self, MoveGroup, self.move_action)
        self.gripper_client = ActionClient(self, FollowJointTrajectory, self.gripper_action)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.joint_state: JointState | None = None
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)

        self._busy = False
        self._active_goal_handle = None
        self._active_goal_started = 0.0
        self._closing = False
        self._last_feedback_state = ""
        self._last_feedback_log_time = 0.0

        self.root = tk.Tk()
        self.root.title("FR3 Cartesian Jog - MoveIt OMPL")
        self.root.geometry("640x600")
        self.root.minsize(600, 560)
        self.root.configure(bg="#171923")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self._build_gui()

        self._spin_thread = threading.Thread(target=self._spin, daemon=True)

    def _str_param(self, name: str) -> str:
        value = self.get_parameter(name).value
        return "" if value is None else str(value)

    def _joint_state_cb(self, msg: JointState) -> None:
        self.joint_state = msg

    def _build_gui(self) -> None:
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background="#171923")
        style.configure("TLabelframe", background="#171923", foreground="#e5e7eb")
        style.configure("TLabelframe.Label", background="#171923", foreground="#e5e7eb")
        style.configure("TLabel", background="#171923", foreground="#e5e7eb")
        style.configure("TButton", padding=6)
        style.configure("Busy.TLabel", background="#171923", foreground="#fbbf24")
        style.configure("Ok.TLabel", background="#171923", foreground="#86efac")
        style.configure("Err.TLabel", background="#171923", foreground="#fca5a5")

        header = ttk.Frame(self.root)
        header.pack(fill="x", padx=14, pady=(12, 6))
        ttk.Label(
            header,
            text=f"TCP jog: {self.base_frame} -> {self.ee_link}",
            font=("Segoe UI", 14, "bold"),
        ).pack(anchor="w")
        ttk.Label(
            header,
            text=f"group={self.planning_group}, pipeline={self.pipeline_id}, planner={self.planner_id or '<default>'}",
            font=("Segoe UI", 9),
        ).pack(anchor="w", pady=(2, 0))

        pose_frame = ttk.LabelFrame(self.root, text="Current TCP pose")
        pose_frame.pack(fill="x", padx=14, pady=6)
        self.pose_var = tk.StringVar(value="TF bekleniyor...")
        ttk.Label(
            pose_frame,
            textvariable=self.pose_var,
            font=("Consolas", 10),
            justify="left",
        ).pack(anchor="w", padx=10, pady=8)

        step_frame = ttk.LabelFrame(self.root, text="Step settings")
        step_frame.pack(fill="x", padx=14, pady=6)
        row = ttk.Frame(step_frame)
        row.pack(fill="x", padx=10, pady=8)
        ttk.Label(row, text="Linear step (m)").grid(row=0, column=0, sticky="w")
        self.linear_step_var = tk.DoubleVar(value=self.default_linear_step)
        tk.Spinbox(
            row,
            from_=0.002,
            to=0.200,
            increment=0.005,
            textvariable=self.linear_step_var,
            width=8,
            format="%.3f",
        ).grid(row=0, column=1, padx=(8, 24), sticky="w")
        ttk.Label(row, text="Angular step (deg)").grid(row=0, column=2, sticky="w")
        self.angular_step_var = tk.DoubleVar(value=self.default_angular_step_deg)
        tk.Spinbox(
            row,
            from_=1.0,
            to=30.0,
            increment=1.0,
            textvariable=self.angular_step_var,
            width=8,
            format="%.1f",
        ).grid(row=0, column=3, padx=(8, 24), sticky="w")
        ttk.Label(row, text="Vel scale").grid(row=1, column=0, pady=(8, 0), sticky="w")
        self.velocity_var = tk.DoubleVar(value=self.velocity_scaling)
        tk.Spinbox(
            row,
            from_=0.02,
            to=0.50,
            increment=0.02,
            textvariable=self.velocity_var,
            width=8,
            format="%.2f",
        ).grid(row=1, column=1, padx=(8, 24), pady=(8, 0), sticky="w")
        ttk.Label(row, text="Acc scale").grid(row=1, column=2, pady=(8, 0), sticky="w")
        self.acceleration_var = tk.DoubleVar(value=self.acceleration_scaling)
        tk.Spinbox(
            row,
            from_=0.02,
            to=0.50,
            increment=0.02,
            textvariable=self.acceleration_var,
            width=8,
            format="%.2f",
        ).grid(row=1, column=3, padx=(8, 24), pady=(8, 0), sticky="w")

        move_frame = ttk.LabelFrame(self.root, text="Position jog - base frame axes")
        move_frame.pack(fill="x", padx=14, pady=6)
        grid = ttk.Frame(move_frame)
        grid.pack(padx=10, pady=10)
        ttk.Button(grid, text="X+ / Up", command=lambda: self.jog_translation("x", 1.0)).grid(row=0, column=1, padx=4, pady=4)
        ttk.Button(grid, text="Y+ / Left", command=lambda: self.jog_translation("y", 1.0)).grid(row=1, column=0, padx=4, pady=4)
        ttk.Button(grid, text="Z+ / PgUp", command=lambda: self.jog_translation("z", 1.0)).grid(row=1, column=1, padx=4, pady=4)
        ttk.Button(grid, text="Y- / Right", command=lambda: self.jog_translation("y", -1.0)).grid(row=1, column=2, padx=4, pady=4)
        ttk.Button(grid, text="X- / Down", command=lambda: self.jog_translation("x", -1.0)).grid(row=2, column=1, padx=4, pady=4)
        ttk.Button(grid, text="Z- / PgDn", command=lambda: self.jog_translation("z", -1.0)).grid(row=3, column=1, padx=4, pady=4)

        rot_frame = ttk.LabelFrame(self.root, text="Rotation jog - base frame axes")
        rot_frame.pack(fill="x", padx=14, pady=6)
        rot = ttk.Frame(rot_frame)
        rot.pack(padx=10, pady=10)
        ttk.Button(rot, text="Roll - (Z)", command=lambda: self.jog_rotation("x", -1.0)).grid(row=0, column=0, padx=4, pady=4)
        ttk.Button(rot, text="Roll + (X)", command=lambda: self.jog_rotation("x", 1.0)).grid(row=0, column=1, padx=4, pady=4)
        ttk.Button(rot, text="Pitch - (F)", command=lambda: self.jog_rotation("y", -1.0)).grid(row=0, column=2, padx=4, pady=4)
        ttk.Button(rot, text="Pitch + (R)", command=lambda: self.jog_rotation("y", 1.0)).grid(row=0, column=3, padx=4, pady=4)
        ttk.Button(rot, text="Yaw - (Q)", command=lambda: self.jog_rotation("z", -1.0)).grid(row=1, column=1, padx=4, pady=4)
        ttk.Button(rot, text="Yaw + (E)", command=lambda: self.jog_rotation("z", 1.0)).grid(row=1, column=2, padx=4, pady=4)

        grip_frame = ttk.LabelFrame(self.root, text="Gripper")
        grip_frame.pack(fill="x", padx=14, pady=6)
        grip = ttk.Frame(grip_frame)
        grip.pack(fill="x", padx=10, pady=10)
        ttk.Label(grip, text="Width (m)").pack(side="left")
        self.gripper_width_var = tk.DoubleVar(value=self.gripper_close_width)
        tk.Spinbox(
            grip,
            from_=0.0,
            to=0.08,
            increment=0.005,
            textvariable=self.gripper_width_var,
            width=8,
            format="%.3f",
        ).pack(side="left", padx=(8, 16))
        ttk.Button(grip, text="Open", command=lambda: self.send_gripper(self.gripper_open_width)).pack(side="left", padx=4)
        ttk.Button(grip, text="Close", command=lambda: self.send_gripper(self.gripper_width_var.get())).pack(side="left", padx=4)

        status_frame = ttk.Frame(self.root)
        status_frame.pack(fill="x", padx=14, pady=(8, 12))
        self.status_style = "Ok.TLabel"
        self.status_var = tk.StringVar(value="Hazir. Ok tuslari X/Y, PageUp/PageDown Z hareketidir.")
        self.status_label = ttk.Label(
            status_frame,
            textvariable=self.status_var,
            style=self.status_style,
            font=("Segoe UI", 10, "bold"),
        )
        self.status_label.pack(anchor="w")

        self.root.bind("<Up>", lambda event: self.jog_translation("x", 1.0))
        self.root.bind("<Down>", lambda event: self.jog_translation("x", -1.0))
        self.root.bind("<Left>", lambda event: self.jog_translation("y", 1.0))
        self.root.bind("<Right>", lambda event: self.jog_translation("y", -1.0))
        self.root.bind("<Prior>", lambda event: self.jog_translation("z", 1.0))
        self.root.bind("<Next>", lambda event: self.jog_translation("z", -1.0))
        self.root.bind("<q>", lambda event: self.jog_rotation("z", -1.0))
        self.root.bind("<e>", lambda event: self.jog_rotation("z", 1.0))
        self.root.bind("<r>", lambda event: self.jog_rotation("y", 1.0))
        self.root.bind("<f>", lambda event: self.jog_rotation("y", -1.0))
        self.root.bind("<z>", lambda event: self.jog_rotation("x", -1.0))
        self.root.bind("<x>", lambda event: self.jog_rotation("x", 1.0))
        self.root.bind("<Escape>", lambda event: self.cancel_active_goal())

    def run(self) -> None:
        self._spin_thread.start()
        self.root.after(250, self._refresh_pose)
        self.root.after(1000, self._watchdog)
        self.root.focus_force()
        self.root.mainloop()

    def _spin(self) -> None:
        try:
            rclpy.spin(self)
        except ExternalShutdownException:
            pass

    def close(self) -> None:
        self._closing = True
        try:
            self.root.quit()
            self.root.destroy()
        except tk.TclError:
            pass

    def _set_status(self, text: str, kind: str = "ok") -> None:
        style = {"ok": "Ok.TLabel", "busy": "Busy.TLabel", "err": "Err.TLabel"}.get(kind, "Ok.TLabel")

        def apply() -> None:
            if self._closing:
                return
            self.status_var.set(text)
            self.status_label.configure(style=style)

        try:
            self.root.after(0, apply)
        except tk.TclError:
            pass

    def _refresh_pose(self) -> None:
        if self._closing:
            return
        pose = self._lookup_tcp_pose()
        if pose is None:
            self.pose_var.set(f"TF yok: {self.base_frame} -> {self.ee_link}")
        else:
            q = (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
            roll, pitch, yaw = _quat_to_rpy(q)
            self.pose_var.set(
                "pos: "
                f"x={pose.position.x:+.4f}  y={pose.position.y:+.4f}  z={pose.position.z:+.4f}\n"
                "rpy: "
                f"roll={math.degrees(roll):+.1f}  pitch={math.degrees(pitch):+.1f}  yaw={math.degrees(yaw):+.1f}"
            )
        self.root.after(500, self._refresh_pose)

    def _watchdog(self) -> None:
        if self._closing:
            return
        if self._busy and self._active_goal_handle is not None:
            elapsed = time.monotonic() - self._active_goal_started
            if elapsed > self.move_action_timeout:
                self._set_status("MoveIt timeout; goal cancel ediliyor.", "err")
                try:
                    self._active_goal_handle.cancel_goal_async()
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f"Cancel failed: {exc}")
                self._busy = False
                self._active_goal_handle = None
        self.root.after(1000, self._watchdog)

    def _lookup_tcp_pose(self) -> Pose | None:
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_link,
                Time(),
                timeout=RclpyDuration(seconds=0.05),
            )
        except TransformException:
            return None
        pose = Pose()
        pose.position.x = tf.transform.translation.x
        pose.position.y = tf.transform.translation.y
        pose.position.z = tf.transform.translation.z
        pose.orientation = tf.transform.rotation
        return pose

    def jog_translation(self, axis: str, sign: float) -> None:
        pose = self._lookup_tcp_pose()
        if pose is None:
            self._set_status(f"TF bekleniyor: {self.base_frame} -> {self.ee_link}", "err")
            return
        step = abs(float(self.linear_step_var.get())) * sign
        if axis == "x":
            pose.position.x += step
        elif axis == "y":
            pose.position.y += step
        else:
            pose.position.z += step
        self._send_pose_goal(pose, f"{axis.upper()}{step:+.3f}m")

    def jog_rotation(self, axis: str, sign: float) -> None:
        pose = self._lookup_tcp_pose()
        if pose is None:
            self._set_status(f"TF bekleniyor: {self.base_frame} -> {self.ee_link}", "err")
            return
        angle = math.radians(abs(float(self.angular_step_var.get())) * sign)
        current = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        rotated = _quat_multiply(_axis_angle_quat(axis, angle), current)
        pose.orientation.x = rotated[0]
        pose.orientation.y = rotated[1]
        pose.orientation.z = rotated[2]
        pose.orientation.w = rotated[3]
        self._send_pose_goal(pose, f"{axis.upper()}rot{math.degrees(angle):+.1f}deg")

    def _current_robot_state(self) -> RobotState:
        state = RobotState()
        state.is_diff = True
        if self.joint_state is not None:
            state.joint_state.name = list(self.joint_state.name)
            state.joint_state.position = list(self.joint_state.position)
            state.joint_state.velocity = list(self.joint_state.velocity)
            state.joint_state.effort = list(self.joint_state.effort)
        return state

    def _pose_constraints(self, pose: Pose, label: str) -> Constraints:
        bounds_pose = Pose()
        bounds_pose.position.x = pose.position.x
        bounds_pose.position.y = pose.position.y
        bounds_pose.position.z = pose.position.z
        bounds_pose.orientation.w = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [float(self.position_tolerance)]

        volume = BoundingVolume()
        volume.primitives.append(sphere)
        volume.primitive_poses.append(bounds_pose)

        pos = PositionConstraint()
        pos.header.frame_id = self.base_frame
        pos.link_name = self.ee_link
        pos.constraint_region = volume
        pos.weight = 1.0

        orient = OrientationConstraint()
        orient.header.frame_id = self.base_frame
        orient.link_name = self.ee_link
        orient.orientation = pose.orientation
        orient.absolute_x_axis_tolerance = float(self.orientation_tolerance)
        orient.absolute_y_axis_tolerance = float(self.orientation_tolerance)
        orient.absolute_z_axis_tolerance = float(self.orientation_tolerance)
        orient.weight = 1.0

        constraints = Constraints()
        constraints.name = label
        constraints.position_constraints.append(pos)
        constraints.orientation_constraints.append(orient)
        return constraints

    def _make_motion_request(self, pose: Pose, label: str) -> MotionPlanRequest:
        req = MotionPlanRequest()
        req.pipeline_id = self.pipeline_id
        req.planner_id = self.planner_id
        req.group_name = self.planning_group
        req.num_planning_attempts = int(self.num_planning_attempts)
        req.allowed_planning_time = float(self.allowed_planning_time)
        req.max_velocity_scaling_factor = _clamp(float(self.velocity_var.get()), 0.01, 1.0)
        req.max_acceleration_scaling_factor = _clamp(float(self.acceleration_var.get()), 0.01, 1.0)
        req.start_state = self._current_robot_state()
        req.goal_constraints.append(self._pose_constraints(pose, label))
        return req

    def _send_pose_goal(self, pose: Pose, label: str) -> None:
        if self._busy:
            self._set_status("Onceki MoveIt hedefi bitmeden yeni hedef gonderilmedi.", "busy")
            return
        if not self.move_group_client.server_is_ready():
            self._set_status("/move_action bekleniyor. Launch tamamen acildi mi?", "err")
            return

        goal = MoveGroup.Goal()
        goal.request = self._make_motion_request(pose, label)
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = self.replan
        goal.planning_options.replan_attempts = self.replan_attempts
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        self._busy = True
        self._active_goal_started = time.monotonic()
        self._last_feedback_state = ""
        self._last_feedback_log_time = 0.0
        self._set_status(f"Plan/execution: {label}", "busy")
        future = self.move_group_client.send_goal_async(goal, feedback_callback=self._feedback_cb)
        future.add_done_callback(lambda f: self._goal_response_cb(f, label))

    def _feedback_cb(self, msg) -> None:  # noqa: ANN001
        state = msg.feedback.state
        now = time.monotonic()
        if state != self._last_feedback_state or now - self._last_feedback_log_time > 2.0:
            self._last_feedback_state = state
            self._last_feedback_log_time = now
            self._set_status(f"MoveIt: {state}", "busy")

    def _goal_response_cb(self, future, label: str) -> None:  # noqa: ANN001
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._busy = False
            self._active_goal_handle = None
            self._set_status(f"MoveIt goal hatasi: {exc}", "err")
            return
        if goal_handle is None or not goal_handle.accepted:
            self._busy = False
            self._active_goal_handle = None
            self._set_status("MoveIt goal rejected.", "err")
            return
        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self._result_cb(f, label))

    def _result_cb(self, future, label: str) -> None:  # noqa: ANN001
        self._busy = False
        self._active_goal_handle = None
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as exc:  # noqa: BLE001
            self._set_status(f"MoveIt result hatasi: {exc}", "err")
            return
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self._set_status(f"Basarili: {label}, planning_time={result.planning_time:.3f}s", "ok")
            return
        self._set_status(f"MoveIt error_code={result.error_code.val}", "err")

    def cancel_active_goal(self) -> None:
        if self._active_goal_handle is None:
            self._set_status("Aktif MoveIt goal yok.", "ok")
            return
        try:
            self._active_goal_handle.cancel_goal_async()
            self._set_status("Cancel istegi gonderildi.", "busy")
        except Exception as exc:  # noqa: BLE001
            self._set_status(f"Cancel hatasi: {exc}", "err")

    def send_gripper(self, total_width: float) -> None:
        if not self.gripper_client.server_is_ready():
            self._set_status("Gripper action bekleniyor.", "err")
            return
        finger = _clamp(total_width * 0.5, 0.0, 0.04)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["fr3_finger_joint1", "fr3_finger_joint2"]
        point = JointTrajectoryPoint()
        point.positions = [finger, finger]
        duration = max(0.2, float(self.gripper_duration))
        point.time_from_start = Duration(sec=int(duration), nanosec=int((duration % 1.0) * 1e9))
        goal.trajectory.points.append(point)
        self.gripper_client.send_goal_async(goal)
        self._set_status(f"Gripper width={total_width:.3f}m", "busy")


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    gui = ArmCartesianGui()
    try:
        gui.run()
    finally:
        gui.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
