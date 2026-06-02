#!/usr/bin/env python3
"""
pick_and_place.py -- FR3 pick-and-place with FK verification
=============================================================

Basket mapping (from panther_macro.urdf.xacro):
  GOOD basket (CLOSE) : xyz=(-0.55, 0.0, -0.12) from base_link
  BAD  basket (FAR)   : xyz=(-0.90, 0.0, -0.12) from base_link

Object: red sphere (tomato_placeholder.sdf), radius=30mm, spawned at
        world (0.678, 0, 0.677) on a static pedestal.

Joint startup fix:
  Gazebo inits all joints at 0.0. FR3 limits require:
    j4 in [-3.077, -0.117]  -- 0.0 is INVALID
    j6 in [ 0.440,  4.622]  -- 0.0 is INVALID
  Step 0 always homes the arm first.

Usage:
  python3 ~/combined_ws/pick_and_place.py good   # CLOSE basket
  python3 ~/combined_ws/pick_and_place.py bad    # FAR   basket
"""

import sys
import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import tf2_ros

from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# =============================================================================
# FR3 Forward Kinematics  (offsets from kinematics.yaml)
# =============================================================================
class FR3FK:
    """
    Analytic FK from fr3_link0 to fr3_hand_tcp.
    Each frame: [tx, ty, tz, roll, pitch, yaw] then Rz(q_i).
    """
    _FRAMES = [
        [ 0.000,  0.000,  0.333,  0.000,            0, 0],   # joint1
        [ 0.000,  0.000,  0.000, -math.pi / 2,      0, 0],   # joint2
        [ 0.000, -0.316,  0.000,  math.pi / 2,      0, 0],   # joint3
        [ 0.0825, 0.000,  0.000,  math.pi / 2,      0, 0],   # joint4
        [-0.0825, 0.384,  0.000, -math.pi / 2,      0, 0],   # joint5
        [ 0.000,  0.000,  0.000,  math.pi / 2,      0, 0],   # joint6
        [ 0.088,  0.000,  0.000,  math.pi / 2,      0, 0],   # joint7
        [ 0.000,  0.000,  0.107,  0.000,            0, 0],   # flange->tcp
    ]

    @staticmethod
    def _rpy(r, p, y):
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)
        Rx = np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])
        Ry = np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])
        Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
        return Rz @ Ry @ Rx

    @staticmethod
    def _Rz(t):
        c, s = math.cos(t), math.sin(t)
        return np.array([[c,-s,0],[s,c,0],[0,0,1]])

    @classmethod
    def tcp(cls, q):
        """Return 4x4 transform from fr3_link0 to hand_tcp."""
        T = np.eye(4)
        for i, (tx, ty, tz, r, p, y) in enumerate(cls._FRAMES):
            R = cls._rpy(r, p, y)
            Tf = np.eye(4); Tf[:3,:3] = R; Tf[:3,3] = [tx, ty, tz]
            T = T @ Tf
            if i < len(q):
                Tj = np.eye(4); Tj[:3,:3] = cls._Rz(q[i]); T = T @ Tj
        return T

    @classmethod
    def pos(cls, q):
        return cls.tcp(q)[:3, 3]

    @classmethod
    def show(cls, label, q):
        T = cls.tcp(q)
        p = T[:3, 3]
        sy = math.sqrt(T[0,0]**2 + T[1,0]**2)
        roll  = math.atan2( T[2,1],  T[2,2])
        pitch = math.atan2(-T[2,0],  sy)
        yaw   = math.atan2( T[1,0],  T[0,0])
        print(f"  [{label}]")
        print(f"    TCP  x={p[0]:+.4f}  y={p[1]:+.4f}  z={p[2]:+.4f} m  "
              f"(rpy={math.degrees(roll):.1f} {math.degrees(pitch):.1f} {math.degrees(yaw):.1f} deg)")
        print(f"    j7 = {math.degrees(q[6]):+.1f} deg   joints={[round(v,3) for v in q]}")


# =============================================================================
# Joint names
# =============================================================================
JOINT_NAMES = [
    'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
    'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
]

# =============================================================================
# Pose Library
# =============================================================================
#
# Object (sphere, r=30mm) spawned at world (0.678, 0, 0.677).
# fr3_link0 is at world z ~= 0.354m (base_link 0.1825 + mount 0.1715).
#
# Basket drop angles are USER-VERIFIED in RViz with joint-state GUI.
# Hover = drop with j2 reduced by 0.25 rad to clear basket rim (~8cm).
#
# j1 limit: [-2.9007, +2.9007].  Both baskets use j1=+2.901 (within limit).
# j4 limit: [-3.077, -0.117].    Never zero.
# j6 limit: [+0.440, +4.622].    Never zero.
#
POSES = {
    # -- Home (Franka standard ready) -----------------------------------------
    'home': [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],

    # -- Pre-grasp: TCP ~15cm above sphere ------------------------------------
    # Sphere centre at world z=0.677m; fr3_link0 at z=0.354m
    # --> TCP needs z ~= 0.677-0.354+0.15 = +0.473m from fr3_link0
    # j7=0.0 -> fingers symmetric (good for round tomato-sized objects)
    'pre_grasp': [0.0, 0.10, 0.0, -2.10, 0.0, 2.00, 0.0],

    # -- Grasp: TCP at sphere centre ------------------------------------------
    # z target ~= 0.677-0.354 = +0.323m from fr3_link0
    'grasp': [0.0, 0.50, 0.0, -1.65, 0.0, 2.15, 0.0],

    # -- Lift: raise object before swinging -----------------------------------
    # Same as pre_grasp but j7 already at 0.0 (kept for basket)
    'lift': [0.0, 0.10, 0.0, -2.10, 0.0, 2.00, 0.0],

    # -- Swing transit: compact while j1 sweeps 0 -> +2.901 ------------------
    # Mid-point at j1=1.45 keeps arm folded during large base rotation.
    'swing_transit': [1.45, -0.40, 0.0, -2.50, 0.0, 1.60, 0.0],

    # =========================================================================
    # GOOD basket (CLOSE, x=-0.55m from base_link)
    # Drop angles verified in RViz with joint-state GUI.
    # Hover: j2 backed off 0.25 rad from drop to clear basket rim.
    # =========================================================================
    'good_hover': [2.901,  0.097, 0.235, -2.149,  0.000, 2.531, 0.0],
    'good_drop':  [2.901,  0.347, 0.235, -2.149,  0.000, 2.531, 0.0],

    # =========================================================================
    # BAD basket (FAR, x=-0.90m from base_link)
    # Drop angles verified in RViz with joint-state GUI.
    # Hover: j2 backed off 0.25 rad from drop.
    # =========================================================================
    'bad_hover':  [2.901,  0.753, 0.266, -1.300, -0.078, 2.791, 0.0],
    'bad_drop':   [2.901,  1.003, 0.266, -1.300, -0.078, 2.791, 0.0],
}

# -- Gripper ------------------------------------------------------------------
# Sphere diameter = 60mm -> each finger 30mm from centre at contact
GRIPPER_OPEN   = 0.04   # m  (80mm total gap -- fully open)
GRIPPER_CLOSED = 0.025  # m  (50mm total gap -- grips 60mm sphere lightly)
GRIPPER_EFFORT = 60.0   # N


# =============================================================================
# PickAndPlace node
# =============================================================================
class PickAndPlace(Node):

    def __init__(self, basket: str):
        super().__init__('pick_and_place', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.basket = basket

        self._arm = ActionClient(
            self, FollowJointTrajectory,
            '/fr3_arm_controller/follow_joint_trajectory')
        self._grip = ActionClient(
            self, GripperCommand,
            '/fr3_gripper_controller/gripper_cmd')

        self._tf = tf2_ros.Buffer()
        self._tfl = tf2_ros.TransformListener(self._tf, self)

    # -------------------------------------------------------------------------
    # TF2 readout helpers
    # -------------------------------------------------------------------------
    def _tf_pos(self, child, parent='fr3_link0', label=''):
        try:
            t = self._tf.lookup_transform(parent, child, rclpy.time.Time())
            p = t.transform.translation
            self.get_logger().info(
                f"  [TF2/{child}] {label} "
                f"x={p.x:+.4f}  y={p.y:+.4f}  z={p.z:+.4f} m")
        except Exception:
            self.get_logger().warn(f"  [TF2] {child} not available yet")

    # -------------------------------------------------------------------------
    # Arm motion
    # -------------------------------------------------------------------------
    def _build_goal(self, waypoints, time_steps):
        points = []
        for q, t in zip(waypoints, time_steps):
            if isinstance(q, str):
                q = POSES[q]
            points.append(JointTrajectoryPoint(
                positions=list(q),
                velocities=[0.0] * 7,
                time_from_start=Duration(
                    sec=int(t),
                    nanosec=int((t % 1) * 1e9)),
            ))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(
            joint_names=JOINT_NAMES, points=points)
        return goal

    def _move(self, waypoints, time_steps, label):
        self.get_logger().info(
            f"\n  >> ARM: {label}  ({len(waypoints)} wp, "
            f"{time_steps[-1]:.1f}s)")

        # Print analytic FK for each waypoint
        for q, t in zip(waypoints, time_steps):
            qv = POSES[q] if isinstance(q, str) else q
            p  = FR3FK.pos(qv)
            nm = q if isinstance(q, str) else 'inline'
            self.get_logger().info(
                f"    [{t:.1f}s] {nm:<18}  TCP [{p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f}] m"
                f"  j7={math.degrees(qv[6]):+.0f}deg")

        self._arm.wait_for_server()
        fut = self._arm.send_goal_async(self._build_goal(waypoints, time_steps))
        rclpy.spin_until_future_complete(self, fut)
        rclpy.spin_until_future_complete(self, fut.result().get_result_async())
        self.get_logger().info(f"  OK: {label}")

    def _arm1(self, pose, secs):
        self._move([pose], [secs], pose)

    # -------------------------------------------------------------------------
    # Gripper
    # -------------------------------------------------------------------------
    def _gripper(self, pos, label):
        self.get_logger().info(
            f"  >> GRIPPER: {label}  ({pos*1000:.0f}mm each side)")
        self._grip.wait_for_server()
        goal = GripperCommand.Goal()
        goal.command.position = pos
        goal.command.max_effort = GRIPPER_EFFORT
        fut = self._grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        rclpy.spin_until_future_complete(self, fut.result().get_result_async())
        self.get_logger().info(f"  OK: gripper {label}")

    # -------------------------------------------------------------------------
    # Main sequence
    # -------------------------------------------------------------------------
    def run(self):
        SEP = "=" * 62
        bname = ("GOOD (CLOSE x=-0.55m)" if self.basket == 'good'
                 else "BAD  (FAR   x=-0.90m)")
        self.get_logger().info(f"\n{SEP}\n  Pick-and-Place -> {bname}\n{SEP}")

        # --- Print FK table for all poses ---
        self.get_logger().info("\n[FK TABLE] Analytic TCP from fr3_link0:")
        for name, q in POSES.items():
            FR3FK.show(name, q)

        time.sleep(1.0)

        # -------------------------------------------------------------------
        # STEP 0: Home  (fixes Gazebo zero-joint init which violates j4/j6)
        # -------------------------------------------------------------------
        self.get_logger().info("\n[0] Homing arm (j4, j6 must leave zero)...")
        self._arm1('home', 5.0)
        time.sleep(0.5)
        self._tf_pos('fr3_link7',    label='Home')
        self._tf_pos('fr3_hand_tcp', label='Home')

        # -------------------------------------------------------------------
        # STEP 1: Open gripper
        # -------------------------------------------------------------------
        self.get_logger().info("\n[1] Open gripper...")
        self._gripper(GRIPPER_OPEN, 'OPEN')
        time.sleep(0.3)

        # -------------------------------------------------------------------
        # STEP 2: Pre-grasp  (arm forward, TCP above sphere)
        # -------------------------------------------------------------------
        self.get_logger().info("\n[2] Pre-grasp (hover above sphere)...")
        self._arm1('pre_grasp', 3.5)
        time.sleep(0.3)
        self._tf_pos('fr3_hand_tcp', label='Pre-grasp')

        # -------------------------------------------------------------------
        # STEP 3: Grasp approach (two-waypoint smooth descent)
        # -------------------------------------------------------------------
        self.get_logger().info("\n[3] Descending to grasp...")
        mid = [0.0, 0.30, 0.0, -1.88, 0.0, 2.08, 0.0]
        self._move([mid, 'grasp'], [1.8, 3.2], 'descent -> grasp')
        time.sleep(0.3)
        self._tf_pos('fr3_link7',    label='At grasp')
        self._tf_pos('fr3_hand_tcp', label='At grasp')

        # -------------------------------------------------------------------
        # STEP 4: Close gripper
        # -------------------------------------------------------------------
        self.get_logger().info("\n[4] Close gripper (grip sphere)...")
        self._gripper(GRIPPER_CLOSED, 'CLOSE')
        time.sleep(0.8)

        # -------------------------------------------------------------------
        # STEP 5: Lift
        # -------------------------------------------------------------------
        self.get_logger().info("\n[5] Lifting object...")
        self._arm1('lift', 2.5)
        time.sleep(0.3)
        self._tf_pos('fr3_hand_tcp', label='Lifted')

        # -------------------------------------------------------------------
        # STEP 6: Swing to basket (via compact transit)
        # -------------------------------------------------------------------
        self.get_logger().info(
            f"\n[6] Swinging to {self.basket.upper()} basket...")
        hover = f'{self.basket}_hover'
        self._move(
            ['swing_transit', hover],
            [3.0, 6.0],
            f'swing -> {hover}')
        time.sleep(0.4)
        self._tf_pos('fr3_link7',    label=f'{self.basket} hover')
        self._tf_pos('fr3_hand_tcp', label=f'{self.basket} hover')

        # -------------------------------------------------------------------
        # STEP 7: Lower into basket
        # -------------------------------------------------------------------
        self.get_logger().info(
            f"\n[7] Lowering into {self.basket.upper()} basket...")
        drop = f'{self.basket}_drop'
        self._arm1(drop, 2.0)
        time.sleep(0.3)
        self._tf_pos('fr3_link7', label=f'Inside {self.basket}')

        # -------------------------------------------------------------------
        # STEP 8: Release
        # -------------------------------------------------------------------
        self.get_logger().info("\n[8] Releasing object...")
        self._gripper(GRIPPER_OPEN, 'OPEN (release)')
        time.sleep(0.8)

        # -------------------------------------------------------------------
        # STEP 9: Retract
        # -------------------------------------------------------------------
        self.get_logger().info("\n[9] Retracting from basket...")
        self._move(
            [hover, 'swing_transit'],
            [1.5, 4.0],
            'retract')
        time.sleep(0.3)

        # -------------------------------------------------------------------
        # STEP 10: Home
        # -------------------------------------------------------------------
        self.get_logger().info("\n[10] Returning home...")
        self._arm1('home', 4.0)
        self._tf_pos('fr3_link7',    label='Home (final)')
        self._tf_pos('fr3_hand_tcp', label='Home (final)')

        self.get_logger().info(
            f"\n{SEP}\n"
            f"  DONE -- object placed in {bname}\n"
            f"{SEP}")


# =============================================================================
# Entry point
# =============================================================================
def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ('good', 'bad'):
        print(__doc__)
        print("ERROR: pass 'good' or 'bad'")
        sys.exit(1)

    rclpy.init()
    node = PickAndPlace(sys.argv[1])
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted -- going home")
        try:
            node._arm1('home', 4.0)
        except Exception:
            pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
