#!/usr/bin/env python3
import sys
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import tf2_ros
from scipy.optimize import minimize

JOINT_NAMES = [
    'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
    'fr3_joint5', 'fr3_joint6', 'fr3_joint7',
]

def franka_fk(q):
    """Simplified Forward Kinematics to calculate X,Y,Z of the TCP"""
    dh = [
        (0.333, 0, 0),
        (0, 0, -np.pi/2),
        (0.316, 0, np.pi/2),
        (0, 0.0825, np.pi/2),
        (0.384, -0.0825, -np.pi/2),
        (0, 0, np.pi/2),
        (0, 0.088, np.pi/2)
    ]
    T = np.eye(4)
    for i in range(7):
        d, a, alpha = dh[i]
        theta = q[i]
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        T_i = np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [ 0,  sa,     ca,    d],
            [ 0,  0,      0,     1]
        ])
        T = T @ T_i
    
    # Exact distance from joint7 to fr3_hand_tcp in URDF (0.107 + 0.1034)
    T_tcp = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0.2104],
        [0, 0, 0, 1]
    ])
    T = T @ T_tcp
    return T[:3, 3]

def solve_ik(target_xyz, q_guess):
    """Custom Numerical Inverse Kinematics Solver"""
    def objective(q):
        pos = franka_fk(q)
        dist = np.linalg.norm(pos - target_xyz)
        # Keep J3, J5, J7 near zero for an elegant elbow-up posture
        penalty = 0.05 * (q[2]**2 + q[4]**2 + q[6]**2)
        return dist + penalty
    
    bounds = [
        (-2.8, 2.8), (-1.7, 1.7), (-2.8, 2.8), 
        (-3.0, -0.1), (-2.8, 2.8), (-0.01, 3.7), (-2.8, 2.8)
    ]
    res = minimize(objective, q_guess, bounds=bounds, method='SLSQP')
    return res.x

class VisionPickAndPlace(Node):
    def __init__(self, basket: str):
        super().__init__('vision_pick_and_place', parameter_overrides=[rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.basket = basket
        self.bridge = CvBridge()
        self.target_3d = None
        self.camera_info = None
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._arm = ActionClient(self, FollowJointTrajectory, '/fr3_arm_controller/follow_joint_trajectory')
        self._grip = ActionClient(self, GripperCommand, '/fr3_gripper_controller/gripper_cmd')

        self.create_subscription(CameraInfo, '/camera/color/camera_info', self.info_cb, 10)
        self.create_subscription(Image, '/camera/color/image_raw', self.image_cb, 10)

    def info_cb(self, msg):
        self.camera_info = msg

    def image_cb(self, msg):
        if self.target_3d is not None or self.camera_info is None:
            return
            
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Detect red tomato
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)
        
        M = cv2.moments(mask)
        if M["m00"] > 500: # large enough blob
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            self.get_logger().info(f"[VISION] Tomato detected at pixel: ({cX}, {cY})")
            
            # Pinhole Camera Projection
            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]
            
            vx = (cX - cx) / fx
            vy = (cY - cy) / fy
            
            try:
                trans = self.tf_buffer.lookup_transform('fr3_link0', 'fr3_camera_color_optical_frame', rclpy.time.Time())
                cam_x = trans.transform.translation.x
                cam_y = trans.transform.translation.y
                cam_z = trans.transform.translation.z
                
                # Exact tomato top surface height in fr3_link0 frame
                tomato_surface_z = 0.285 
                depth = cam_z - tomato_surface_z 
                
                # Because the camera looks down: Camera Y is X, Camera X is -Y
                world_x = cam_x + vy * depth 
                world_y = cam_y - vx * depth 
                
                self.target_3d = np.array([world_x, world_y, 0.32])
                self.get_logger().info(f"[VISION] 3D Target Calculated: X={world_x:.3f}, Y={world_y:.3f}, Z={tomato_z:.3f}")
                
            except Exception as e:
                self.get_logger().warn(f"Waiting for TF tree... {e}")

    def _arm_cmd(self, joints, time_sec):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = joints
        pt.time_from_start.sec = int(time_sec)
        pt.time_from_start.nanosec = int((time_sec % 1) * 1e9)
        goal.trajectory.points.append(pt)
        
        self._arm.wait_for_server()
        fut = self._arm.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        rclpy.spin_until_future_complete(self, fut.result().get_result_async())

    def _grip_cmd(self, width):
        goal = GripperCommand.Goal()
        goal.command.position = width
        goal.command.max_effort = 50.0
        self._grip.wait_for_server()
        fut = self._grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        rclpy.spin_until_future_complete(self, fut.result().get_result_async())

    def run(self):
        self.get_logger().info("Homing arm...")
        home_joints = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        self._arm_cmd(home_joints, 4.0)
        
        self.get_logger().info("Moving to Pre-Grasp Camera position...")
        pre_grasp_joints = [0.0, 0.1, 0.0, -2.1, 0.0, 2.0, 0.0]
        self._arm_cmd(pre_grasp_joints, 3.0)
        
        self.get_logger().info("Opening Gripper...")
        self._grip_cmd(0.04)
        
        self.get_logger().info("Searching for tomato...")
        while self.target_3d is None:
            rclpy.spin_once(self)
            time.sleep(0.1)
            
        self.get_logger().info("Solving Inverse Kinematics for precise grasp...")
        grasp_joints = solve_ik(self.target_3d, pre_grasp_joints)
        
        self.get_logger().info("Descending to Grasp...")
        self._arm_cmd(list(grasp_joints), 3.0)
        
        self.get_logger().info("Closing Gripper...")
        self._grip_cmd(0.025)
        
        self.get_logger().info("Lifting...")
        self._arm_cmd(pre_grasp_joints, 3.0)
        
        self.get_logger().info(f"Swinging to {self.basket.upper()} basket...")
        basket_joints = [2.901, 0.347, 0.235, -2.149, 0.0, 2.531, 0.0] if self.basket == 'good' else [2.901, 1.003, 0.266, -1.3, -0.078, 2.791, 0.0]
        self._arm_cmd(basket_joints, 5.0)
        
        self.get_logger().info("Releasing...")
        self._grip_cmd(0.04)
        
        self.get_logger().info("Homing...")
        self._arm_cmd(home_joints, 4.0)
        self.get_logger().info("Harvesting Complete!")

def main():
    rclpy.init()
    node = VisionPickAndPlace(sys.argv[1] if len(sys.argv) > 1 else 'good')
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
