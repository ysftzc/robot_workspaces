#!/usr/bin/env python3
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

MOVE_BINDINGS = {
    'w': (0.80, 0.0),
    's': (-0.80, 0.0),
    'a': (0.0, 1.20),
    'd': (0.0, -1.20),
    'x': (0.0, 0.0),
}

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

class TeleopNode(Node):
    def __init__(self):
        super().__init__('twist_stamped_teleop')
        self.pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)

    def publish_cmd(self, linear_x, angular_z):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = TeleopNode()
    print("WASD ile sür, x ile dur, q ile çık")
    try:
        while rclpy.ok():
            key = get_key()
            if key == 'q':
                break
            linear_x, angular_z = MOVE_BINDINGS.get(key, (0.0, 0.0))
            node.publish_cmd(linear_x, angular_z)
    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
