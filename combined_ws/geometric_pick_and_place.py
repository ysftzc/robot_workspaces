#!/usr/bin/env python3
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from scipy.optimize import minimize

JOINT_NAMES = [
    'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
    'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
]

def rot_x(roll):
    c, s = np.cos(roll), np.sin(roll)
    return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])

def rot_y(pitch):
    c, s = np.cos(pitch), np.sin(pitch)
    return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])

def rot_z(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

def trans(x, y, z):
    return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])

def urdf_joint(theta, x, y, z, roll, pitch, yaw):
    T_origin = trans(x, y, z) @ rot_x(roll) @ rot_y(pitch) @ rot_z(yaw)
    T_joint = rot_z(theta)
    return T_origin @ T_joint

def franka_fk(q):
    """Exact Forward Kinematics matching FR3 URDF"""
    T = np.eye(4)
    T = T @ urdf_joint(q[0], 0, 0, 0.333, 0, 0, 0)
    T = T @ urdf_joint(q[1], 0, 0, 0, -1.57079632679, 0, 0)
    T = T @ urdf_joint(q[2], 0, -0.316, 0, 1.57079632679, 0, 0)
    T = T @ urdf_joint(q[3], 0.0825, 0, 0, 1.57079632679, 0, 0)
    T = T @ urdf_joint(q[4], -0.0825, 0.384, 0, -1.57079632679, 0, 0)
    T = T @ urdf_joint(q[5], 0, 0, 0, 1.57079632679, 0, 0)
    T = T @ urdf_joint(q[6], 0.088, 0, 0, 1.57079632679, 0, 0)
    # Joint8 (fixed flange)
    T = T @ trans(0, 0, 0.107)
    # fr3_hand -> tcp
    T = T @ trans(0, 0, 0.1034)
    return T

def solve_ik(target_xyz, target_z_dir, q_guess):
    """Numerical Inverse Kinematics with Orientation Control"""
    def objective(q):
        T = franka_fk(q)
        pos = T[:3, 3]
        z_axis = T[:3, 2] # Direction fingers point
        
        dist = np.linalg.norm(pos - target_xyz)
        ori = 1.0 - np.dot(z_axis, target_z_dir)
        
        # Keep joints close to guess to prevent wild swings
        penalty = 0.05 * np.sum((np.array(q) - np.array(q_guess))**2)
        return (dist * 20.0) + (ori * 5.0) + penalty
    
    bounds = [
        (-2.8, 2.8), (-1.5, 1.5), (-2.8, 2.8), 
        (-3.0, -0.1), (-2.8, 2.8), (0.0, 3.7), (-2.8, 2.8)
    ]
    res = minimize(objective, q_guess, bounds=bounds, method='SLSQP', tol=1e-6)
    return res.x

class GeometricPickAndPlace(Node):
    def __init__(self):
        super().__init__('geometric_pick_and_place', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        
        self._arm = ActionClient(self, FollowJointTrajectory, '/fr3_arm_controller/follow_joint_trajectory')
        # Changed gripper controller to FollowJointTrajectory to control BOTH fingers and bypass mimic bugs
        self._grip = ActionClient(self, FollowJointTrajectory, '/fr3_gripper_controller/follow_joint_trajectory')

        # Define the 4 coordinates we spawned in the launch file
        self.targets = [
            (0.750, 0.000),
            (0.700, 0.300),
            (0.700, -0.300),
            (0.800, -0.150)
        ]

    def _execute_trajectory(self, waypoints, total_time):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        
        step_time = total_time / len(waypoints)
        for i, joints in enumerate(waypoints):
            pt = JointTrajectoryPoint()
            pt.positions = list(joints)
            t = step_time * (i + 1)
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t % 1) * 1e9)
            goal.trajectory.points.append(pt)
            
        self._arm.wait_for_server()
        fut = self._arm.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        rclpy.spin_until_future_complete(self, fut.result().get_result_async())

    def _grip_cmd(self, width):
        # For JointTrajectoryController, width is TOTAL width. We divide by 2 for each finger.
        half_width = width / 2.0
        
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['fr3_finger_joint1', 'fr3_finger_joint2']
        
        pt = JointTrajectoryPoint()
        pt.positions = [half_width, half_width]
        pt.time_from_start.sec = 1
        pt.time_from_start.nanosec = 0
        goal.trajectory.points.append(pt)
        
        self._grip.wait_for_server()
        fut = self._grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        
        # Kapanma işlemi sırasinda ROS düğümünün (node) donmamasi lazim!
        # Eylem sunucusunun parmaklari fiziksel olarak kapatabilmesi için 2 saniye boyunca düğümü 'döndürmeye' (spin) devam ediyoruz.
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < 2.0:
            rclpy.spin_once(self, timeout_sec=0.1)

    def generate_cartesian_path(self, start_joints, start_xyz, end_xyz, steps=5):
        """Generates waypoints to ensure the arm moves in a perfectly straight line."""
        waypoints = []
        current_joints = start_joints
        for i in range(1, steps + 1):
            target = start_xyz + (end_xyz - start_xyz) * (i / steps)
            current_joints = solve_ik(target, np.array([0.0, 0.0, -1.0]), current_joints)
            waypoints.append(current_joints)
        return waypoints

    def run(self):
        import random
        # Son eklem (fr3_joint7) 0.785 (45 derece) olduğu için kamera çapraz bakıyordu.
        # Kameranın tam karşıya bakması için 0.0 veya 1.571 (90 derece) olarak ayarlıyoruz.
        home_joints = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.0]
        
        for index, (tx, ty) in enumerate(self.targets):
            self.get_logger().info(f"\n--- STARTING CYCLE {index+1} for object at X={tx}, Y={ty} ---")
            
            # 1. Homing & Pre-Grasp
            self.get_logger().info("Homing arm...")
            self._execute_trajectory([home_joints], 4.0)
            
            self.get_logger().info("Opening Gripper fully...")
            self._grip_cmd(0.08) # 0.08 total width = 0.04 per finger (maximum physical limit)

            # Calculate geometries
            obj_z_center = 0.500 - 0.3540
            grasp_z = obj_z_center - 0.02
            approach_z = grasp_z + 0.12
            retreat_z = grasp_z + 0.15
            
            target_xyz = np.array([tx, ty, grasp_z])
            hover_xyz = np.array([tx, ty, approach_z])

            # 2. Compute the exact Approach Pose (Hover)
            self.get_logger().info(f"Computing Hover Pose...")
            down_guess = [0.0, 0.3, 0.0, -2.1, 0.0, 2.4, 0.785]
            hover_joints = solve_ik(hover_xyz, np.array([0.0, 0.0, -1.0]), down_guess)
            self._execute_trajectory([hover_joints], 4.0)

            # 3. Approach (Descend straight down)
            self.get_logger().info(f"Descending precisely to Grasp Pose...")
            approach_path = self.generate_cartesian_path(hover_joints, hover_xyz, target_xyz, steps=5)
            self._execute_trajectory(approach_path, 3.0)
            
            # 4. Grab
            self.get_logger().info("Target reached! Closing Gripper securely...")
            self._grip_cmd(0.0) # 0.0 total width = fully closed. It will squeeze the box with max force.
            
            # 5. Retreat
            self.get_logger().info("Retreating with payload...")
            retreat_path = approach_path[::-1]
            self._execute_trajectory(retreat_path, 3.0)
            
            # 6. Random Basket Selection
            basket = random.choice(['good', 'bad'])
            self.get_logger().info(f"Sorting into {basket.upper()} basket...")
            basket_joints = [2.901, 0.347, 0.235, -2.149, 0.0, 2.531, 0.0] if basket == 'good' else [2.901, 1.003, 0.266, -1.3, -0.078, 2.791, 0.0]
            self._execute_trajectory([basket_joints], 5.0)
            
            # 7. Release
            self.get_logger().info("Releasing object...")
            self._grip_cmd(0.08) # 0.08 total width = fully open, guarantee the drop!
            
            self.get_logger().info(f"--- CYCLE {index+1} COMPLETE ---\n")

        self.get_logger().info("All objects processed! Final Homing...")
        self._execute_trajectory([home_joints], 4.0)
        self.get_logger().info("Multi-Object Geometric Pick and Place Complete!")

def main():
    rclpy.init()
    node = GeometricPickAndPlace()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
