#!/usr/bin/env python3
"""
Tkinter GUI to:
  1. Move Franka FR3 arm joints via sliders  (action interface)
  2. Drive the Panther UGV with WASD keys     (cmd_vel publisher)

WASD controls (press & hold, release to stop):
  W / S  — forward / backward
  A / D  — rotate left / right
  Q / E  — strafe diagonals (optional)
  SPACE  — emergency stop
"""

import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Duration
import threading
import math
import sys


# ── Joint definitions ────────────────────────────────────────────────────────
ARM_JOINTS = [
    ("fr3_joint1", -2.3093, 2.3093, 0.0),
    ("fr3_joint2", -1.5133, 1.5133, -0.7854),
    ("fr3_joint3", -2.3093, 2.3093, 0.0),
    ("fr3_joint4", -2.7925, -0.1518, -2.356),
    ("fr3_joint5", -2.3093, 2.3093, 0.0),
    ("fr3_joint6", 0.5, 4.1888, 0.7854),
    ("fr3_joint7", -2.6895, 2.6895, 0.0),
]

GRIPPER_JOINTS = [
    ("fr3_finger_joint1", 0.0, 0.04, 0.0),
    ("fr3_finger_joint2", 0.0, 0.04, 0.0),
]

# ── Teleop defaults ──────────────────────────────────────────────────────────
DEFAULT_LINEAR_SPEED = 0.5   # m/s
DEFAULT_ANGULAR_SPEED = 1.0  # rad/s


class RobotControlGUI:
    def __init__(self):
        rclpy.init()
        self.node = Node("robot_control_gui")

        # Action clients for arm / gripper
        self.arm_client = ActionClient(
            self.node, FollowJointTrajectory,
            "/fr3_arm_controller/follow_joint_trajectory",
        )
        self.grip_client = ActionClient(
            self.node, FollowJointTrajectory,
            "/fr3_gripper_controller/follow_joint_trajectory",
        )

        # Publisher for platform drive
        self.cmd_pub = self.node.create_publisher(TwistStamped, "/cmd_vel", 10)

        # Subscriber for odometry (robot position)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_z = 0.0
        self.odom_yaw = 0.0
        self.node.create_subscription(Odometry, "/odometry/wheels", self._odom_cb, 10)

        # Spin ROS in background
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.spin_thread.start()

        # Track which keys are currently held down
        self.keys_pressed: set[str] = set()
        self.linear_speed = DEFAULT_LINEAR_SPEED
        self.angular_speed = DEFAULT_ANGULAR_SPEED

        # ── Build the GUI ─────────────────────────────────────────────────────
        self.root = tk.Tk()
        self.root.title("🤖 Robot Kontrol Paneli — Kol + Platform")
        self.root.configure(bg="#1e1e2e")
        self.root.geometry("750x1050")

        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Title.TLabel", foreground="#cdd6f4", background="#1e1e2e",
                         font=("Segoe UI", 15, "bold"))
        style.configure("Sub.TLabel", foreground="#bac2de", background="#1e1e2e",
                         font=("Segoe UI", 12, "bold"))
        style.configure("Joint.TLabel", foreground="#a6adc8", background="#1e1e2e",
                         font=("Segoe UI", 10))
        style.configure("Val.TLabel", foreground="#f38ba8", background="#1e1e2e",
                         font=("Segoe UI", 10, "bold"))

        # ── Title ─────────────────────────────────────────────────────────────
        ttk.Label(self.root, text="Robot Kontrol Paneli", style="Title.TLabel").pack(pady=(10, 2))

        # ── ARM sliders ───────────────────────────────────────────────────────
        arm_frame = tk.LabelFrame(self.root, text=" 🦾 Kol Eklemleri (7 DOF) ",
                                   bg="#1e1e2e", fg="#89b4fa", font=("Segoe UI", 11, "bold"))
        arm_frame.pack(fill="x", padx=12, pady=4)

        self.arm_sliders = []
        self.arm_labels = []
        for name, lo, hi, default in ARM_JOINTS:
            row = tk.Frame(arm_frame, bg="#1e1e2e")
            row.pack(fill="x", padx=8, pady=1)
            ttk.Label(row, text=name, style="Joint.TLabel", width=16).pack(side="left")
            val_label = ttk.Label(row, text=f"{default:.3f}", style="Val.TLabel", width=8)
            val_label.pack(side="right")
            slider = tk.Scale(
                row, from_=lo, to=hi, resolution=0.001, orient="horizontal",
                bg="#313244", fg="#cdd6f4", troughcolor="#45475a",
                highlightthickness=0, length=340,
                command=lambda v, lbl=val_label: lbl.configure(text=f"{float(v):.3f}"),
            )
            slider.set(default)
            slider.pack(side="right", padx=4)
            self.arm_sliders.append(slider)
            self.arm_labels.append(val_label)

        # ── GRIPPER sliders ───────────────────────────────────────────────────
        grip_frame = tk.LabelFrame(self.root, text=" 🤏 Gripper ",
                                    bg="#1e1e2e", fg="#a6e3a1", font=("Segoe UI", 11, "bold"))
        grip_frame.pack(fill="x", padx=12, pady=4)

        self.grip_sliders = []
        for name, lo, hi, default in GRIPPER_JOINTS:
            row = tk.Frame(grip_frame, bg="#1e1e2e")
            row.pack(fill="x", padx=8, pady=1)
            ttk.Label(row, text=name, style="Joint.TLabel", width=16).pack(side="left")
            val_label = ttk.Label(row, text=f"{default:.4f}", style="Val.TLabel", width=8)
            val_label.pack(side="right")
            slider = tk.Scale(
                row, from_=lo, to=hi, resolution=0.001, orient="horizontal",
                bg="#313244", fg="#cdd6f4", troughcolor="#45475a",
                highlightthickness=0, length=340,
                command=lambda v, lbl=val_label: lbl.configure(text=f"{float(v):.4f}"),
            )
            slider.set(default)
            slider.pack(side="right", padx=4)
            self.grip_sliders.append(slider)

        # ── Arm/gripper buttons ───────────────────────────────────────────────
        btn_frame = tk.Frame(self.root, bg="#1e1e2e")
        btn_frame.pack(pady=8)

        tk.Button(btn_frame, text="🦾 Kolu Gönder", font=("Segoe UI", 11, "bold"),
                  bg="#89b4fa", fg="#1e1e2e", activebackground="#74c7ec",
                  command=self.send_arm, width=14, height=1).pack(side="left", padx=6)
        tk.Button(btn_frame, text="🤏 Gripper", font=("Segoe UI", 11, "bold"),
                  bg="#a6e3a1", fg="#1e1e2e", activebackground="#94e2d5",
                  command=self.send_gripper, width=14, height=1).pack(side="left", padx=6)
        tk.Button(btn_frame, text="🏠 Home", font=("Segoe UI", 11, "bold"),
                  bg="#f9e2af", fg="#1e1e2e", activebackground="#fab387",
                  command=self.go_home, width=14, height=1).pack(side="left", padx=6)

        # ── WASD Teleop section ───────────────────────────────────────────────
        teleop_frame = tk.LabelFrame(
            self.root, text=" 🚗 Platform Sürüş (WASD) — Pencere odakta iken kullan ",
            bg="#1e1e2e", fg="#fab387", font=("Segoe UI", 11, "bold"),
        )
        teleop_frame.pack(fill="x", padx=12, pady=6)

        # Visual key display
        key_canvas = tk.Frame(teleop_frame, bg="#1e1e2e")
        key_canvas.pack(pady=6)

        self.key_buttons: dict[str, tk.Label] = {}
        key_layout = [
            [None, "W", None],
            ["A", "S", "D"],
        ]
        key_map_display = {"W": "▲ İleri", "A": "◄ Sol", "S": "▼ Geri", "D": "► Sağ"}

        for r, row_keys in enumerate(key_layout):
            for c, key in enumerate(row_keys):
                if key is None:
                    tk.Label(key_canvas, text="", width=10, bg="#1e1e2e").grid(row=r, column=c, padx=3, pady=3)
                    continue
                lbl = tk.Label(
                    key_canvas, text=f"{key}\n{key_map_display[key]}",
                    font=("Segoe UI", 11, "bold"), width=10, height=3,
                    bg="#313244", fg="#cdd6f4", relief="raised", borderwidth=2,
                )
                lbl.grid(row=r, column=c, padx=3, pady=3)
                self.key_buttons[key.lower()] = lbl

        # SPACE bar
        space_lbl = tk.Label(
            key_canvas, text="SPACE — ⛔ Dur",
            font=("Segoe UI", 11, "bold"), width=34, height=2,
            bg="#f38ba8", fg="#1e1e2e", relief="raised", borderwidth=2,
        )
        space_lbl.grid(row=2, column=0, columnspan=3, padx=3, pady=4)
        self.key_buttons["space"] = space_lbl

        # Speed controls
        speed_frame = tk.Frame(teleop_frame, bg="#1e1e2e")
        speed_frame.pack(fill="x", padx=10, pady=4)

        tk.Label(speed_frame, text="Lineer Hız (m/s):", bg="#1e1e2e", fg="#cdd6f4",
                 font=("Segoe UI", 10)).pack(side="left")
        self.linear_scale = tk.Scale(
            speed_frame, from_=0.1, to=2.0, resolution=0.1, orient="horizontal",
            bg="#313244", fg="#cdd6f4", troughcolor="#45475a",
            highlightthickness=0, length=150,
            command=lambda v: setattr(self, 'linear_speed', float(v)),
        )
        self.linear_scale.set(DEFAULT_LINEAR_SPEED)
        self.linear_scale.pack(side="left", padx=5)

        tk.Label(speed_frame, text="   Açısal Hız (rad/s):", bg="#1e1e2e", fg="#cdd6f4",
                 font=("Segoe UI", 10)).pack(side="left")
        self.angular_scale = tk.Scale(
            speed_frame, from_=0.2, to=3.0, resolution=0.1, orient="horizontal",
            bg="#313244", fg="#cdd6f4", troughcolor="#45475a",
            highlightthickness=0, length=150,
            command=lambda v: setattr(self, 'angular_speed', float(v)),
        )
        self.angular_scale.set(DEFAULT_ANGULAR_SPEED)
        self.angular_scale.pack(side="left", padx=5)

        # ── Velocity display ─────────────────────────────────────────────────
        self.vel_var = tk.StringVar(value="lin: 0.00  ang: 0.00")
        tk.Label(teleop_frame, textvariable=self.vel_var, bg="#1e1e2e",
                 fg="#89dceb", font=("Segoe UI Mono", 11, "bold")).pack(pady=2)

        # ── Robot Position Display ────────────────────────────────────────────
        pos_frame = tk.LabelFrame(
            self.root, text=" 📍 Robot Konumu (Odometry) ",
            bg="#1e1e2e", fg="#cba6f7", font=("Segoe UI", 11, "bold"),
        )
        pos_frame.pack(fill="x", padx=12, pady=6)

        pos_inner = tk.Frame(pos_frame, bg="#1e1e2e")
        pos_inner.pack(pady=6)

        self.pos_labels = {}
        pos_items = [
            ("X", "#f38ba8"),
            ("Y", "#a6e3a1"),
            ("Z", "#89b4fa"),
            ("Yaw", "#f9e2af"),
        ]
        for col, (name, color) in enumerate(pos_items):
            box = tk.Frame(pos_inner, bg="#313244", padx=12, pady=6, relief="groove", borderwidth=1)
            box.grid(row=0, column=col, padx=6, pady=2)
            tk.Label(box, text=name, bg="#313244", fg="#a6adc8",
                     font=("Segoe UI", 10, "bold")).pack()
            val = tk.Label(box, text="0.000", bg="#313244", fg=color,
                           font=("Segoe UI Mono", 14, "bold"), width=8)
            val.pack()
            unit = "rad" if name == "Yaw" else "m"
            tk.Label(box, text=unit, bg="#313244", fg="#585b70",
                     font=("Segoe UI", 8)).pack()
            self.pos_labels[name] = val

        # ── Status bar ────────────────────────────────────────────────────────
        self.status_var = tk.StringVar(value="Hazır.  WASD ile platformu sür, slider ile kolu kontrol et.")
        tk.Label(self.root, textvariable=self.status_var, bg="#1e1e2e",
                 fg="#94e2d5", font=("Segoe UI", 10)).pack(pady=4)

        # ── Key bindings ──────────────────────────────────────────────────────
        self.root.bind("<KeyPress>", self._on_key_press)
        self.root.bind("<KeyRelease>", self._on_key_release)
        self.root.focus_set()

        # Periodic cmd_vel publisher (10 Hz)
        self._teleop_loop()

        # Periodic position display update (5 Hz)
        self._update_position_display()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ── Odometry callback ─────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_z = msg.pose.pose.position.z
        # Quaternion → yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _update_position_display(self):
        """Update position labels at ~5 Hz."""
        self.pos_labels["X"].configure(text=f"{self.odom_x:+.3f}")
        self.pos_labels["Y"].configure(text=f"{self.odom_y:+.3f}")
        self.pos_labels["Z"].configure(text=f"{self.odom_z:+.3f}")
        self.pos_labels["Yaw"].configure(text=f"{self.odom_yaw:+.3f}")
        self.root.after(200, self._update_position_display)

    # ── Teleop ────────────────────────────────────────────────────────────────
    def _on_key_press(self, event):
        key = event.keysym.lower()
        if key in ("w", "a", "s", "d", "space"):
            self.keys_pressed.add(key)
            self._highlight_key(key, True)

    def _on_key_release(self, event):
        key = event.keysym.lower()
        self.keys_pressed.discard(key)
        self._highlight_key(key, False)

    def _highlight_key(self, key, active):
        lbl = self.key_buttons.get(key)
        if lbl is None:
            return
        if key == "space":
            lbl.configure(bg="#ff6b6b" if active else "#f38ba8")
        else:
            lbl.configure(bg="#89b4fa" if active else "#313244")

    def _teleop_loop(self):
        """Runs at ~10 Hz, publishes cmd_vel based on currently held keys."""
        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        if "space" in self.keys_pressed:
            # Emergency stop — zero everything
            pass
        else:
            if "w" in self.keys_pressed:
                msg.twist.linear.x += self.linear_speed
            if "s" in self.keys_pressed:
                msg.twist.linear.x -= self.linear_speed
            if "a" in self.keys_pressed:
                msg.twist.angular.z += self.angular_speed
            if "d" in self.keys_pressed:
                msg.twist.angular.z -= self.angular_speed

        self.cmd_pub.publish(msg)
        self.vel_var.set(f"lin: {msg.twist.linear.x:+.2f} m/s   ang: {msg.twist.angular.z:+.2f} rad/s")

        # Reschedule
        self.root.after(100, self._teleop_loop)

    # ── Arm / Gripper ─────────────────────────────────────────────────────────
    def send_arm(self):
        positions = [s.get() for s in self.arm_sliders]
        names = [j[0] for j in ARM_JOINTS]
        self._send_goal(self.arm_client, names, positions, "Kol")

    def send_gripper(self):
        positions = [s.get() for s in self.grip_sliders]
        names = [j[0] for j in GRIPPER_JOINTS]
        self._send_goal(self.grip_client, names, positions, "Gripper")

    def go_home(self):
        for slider, (_, _, _, default) in zip(self.arm_sliders, ARM_JOINTS):
            slider.set(default)
        self.send_arm()

    def _send_goal(self, client, names, positions, label):
        if not client.wait_for_server(timeout_sec=2.0):
            self.status_var.set(f"⚠️ {label} action server bulunamadı!")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=2, nanosec=0)
        goal.trajectory.points = [point]

        self.status_var.set(f"⏳ {label} hareketi gönderiliyor...")
        future = client.send_goal_async(goal)
        future.add_done_callback(
            lambda f, lbl=label: self._on_goal_response(f, lbl)
        )

    def _on_goal_response(self, future, label):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.root.after(0, lambda: self.status_var.set(f"❌ {label} hedef reddedildi!"))
            return
        self.root.after(0, lambda: self.status_var.set(f"✅ {label} hareketi kabul edildi..."))
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f, lbl=label: self.root.after(
                0, lambda: self.status_var.set(f"✅ {lbl} hareketi tamamlandı!")
            )
        )

    # ── Cleanup ───────────────────────────────────────────────────────────────
    def on_close(self):
        # Send stop before closing
        self.cmd_pub.publish(TwistStamped())
        self.node.destroy_node()
        rclpy.shutdown()
        self.root.destroy()
        sys.exit(0)

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    gui = RobotControlGUI()
    gui.run()
