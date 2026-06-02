import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import time
import threading

class TopicChecker(Node):
    def __init__(self):
        super().__init__('topic_checker')
        self.scan_count = 0
        self.tf_count = 0
        self.odom_count = 0
        
        self.create_subscription(LaserScan, '/lidar/scan', self.scan_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan2_cb, 10)
        self.create_subscription(TFMessage, '/tf', self.tf_cb, 10)
        
    def scan_cb(self, msg):
        self.scan_count += 1
        
    def scan2_cb(self, msg):
        print("WARNING: Data received on /scan instead of /lidar/scan!")
        self.scan_count += 1
        
    def tf_cb(self, msg):
        self.tf_count += 1
        for transform in msg.transforms:
            if transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_link':
                self.odom_count += 1

def main():
    rclpy.init()
    node = TopicChecker()
    
    print("Listening to topics for 5 seconds...")
    
    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    
    time.sleep(5.0)
    
    print("\n--- DIAGNOSTICS RESULTS ---")
    print(f"Lidar messages received (/lidar/scan or /scan): {node.scan_count}")
    print(f"Total TF messages received: {node.tf_count}")
    print(f"TF (odom -> base_link) messages received: {node.odom_count}")
    print("---------------------------\n")
    
    if node.scan_count == 0:
        print("ERROR: NO LIDAR DATA! Check if Gazebo is unpaused and the bridge is working.")
    if node.odom_count == 0:
        print("ERROR: NO ODOMETRY TF! EKF is not generating odom->base_link.")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
