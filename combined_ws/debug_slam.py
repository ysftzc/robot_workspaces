import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
import time
import threading

class SlamDebugger(Node):
    def __init__(self):
        super().__init__('slam_debugger')
        self.counts = {
            '/scan': 0,
            '/lidar/scan': 0,
            '/drive_controller/odom': 0,
            '/imu/data_raw': 0,
            'tf_odom_base': 0
        }
        
        self.create_subscription(LaserScan, '/scan', lambda msg: self.cb('/scan'), 10)
        self.create_subscription(LaserScan, '/lidar/scan', lambda msg: self.cb('/lidar/scan'), 10)
        self.create_subscription(Odometry, '/drive_controller/odom', lambda msg: self.cb('/drive_controller/odom'), 10)
        self.create_subscription(Imu, '/imu/data_raw', lambda msg: self.cb('/imu/data_raw'), 10)
        self.create_subscription(TFMessage, '/tf', self.tf_cb, 10)
        
    def cb(self, topic):
        self.counts[topic] += 1
        
    def tf_cb(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == 'odom' and t.child_frame_id == 'base_link':
                self.counts['tf_odom_base'] += 1

def main():
    rclpy.init()
    node = SlamDebugger()
    print("Listening to ROS 2 topics for 5 seconds...")
    
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    time.sleep(5.0)
    
    print("\n=== DIAGNOSTIC REPORT ===")
    for topic, count in node.counts.items():
        status = "✅ OK" if count > 0 else "❌ MISSING"
        print(f"{topic.ljust(25)} : {count} messages -> {status}")
    print("=========================\n")
    
    if node.counts['/imu/data_raw'] == 0:
        print("ROOT CAUSE: The IMU is missing! The EKF filter needs IMU to calculate Odometry TF.")
    elif node.counts['tf_odom_base'] == 0:
        print("ROOT CAUSE: EKF is not publishing odom->base_link TF.")
    elif node.counts['/scan'] == 0:
        print("ROOT CAUSE: Lidar /scan is missing. Gazebo bridge might be failing.")
    else:
        print("Everything seems to be publishing correctly! Check RViz again.")
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()
