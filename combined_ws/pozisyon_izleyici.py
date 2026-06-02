import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

class PositionMonitor(Node):
    def __init__(self):
        super().__init__('position_monitor')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('Robot Kolunun Gerçek (Fiziksel) Pozisyonu İzleniyor... (fr3_link0 -> fr3_hand_tcp)')

    def timer_callback(self):
        try:
            t = self.tf_buffer.lookup_transform('fr3_link0', 'fr3_hand_tcp', rclpy.time.Time())
            x = t.transform.translation.x
            y = t.transform.translation.y
            z = t.transform.translation.z
            self.get_logger().info(f'Gerçek Koordinat -> X: {x:.3f}, Y: {y:.3f}, Z: {z:.3f}')
        except Exception:
            pass

def main():
    rclpy.init()
    node = PositionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
