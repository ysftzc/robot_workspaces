import math
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        self.declare_parameter('initial_pose_file', '')
        self.declare_parameter('publish_delay_sec', 8.0)
        self.declare_parameter('publish_period_sec', 1.0)
        self.declare_parameter('publish_count', 45)

        self._initial_pose_file = self.get_parameter('initial_pose_file').value
        self._publish_delay_sec = float(self.get_parameter('publish_delay_sec').value)
        self._publish_period_sec = float(self.get_parameter('publish_period_sec').value)
        self._publish_count = int(self.get_parameter('publish_count').value)

        self._pose = self._load_initial_pose(self._initial_pose_file)
        self._publisher = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10
        )
        self._remaining = max(1, self._publish_count)
        self._published_count = 0
        self._timer = self.create_timer(self._publish_delay_sec, self._publish_once)

        self.get_logger().info(
            f'Loaded initial pose from "{self._initial_pose_file}". '
            f'Publishing {self._remaining} time(s) after {self._publish_delay_sec:.1f}s.'
        )

    def _load_initial_pose(self, initial_pose_file):
        if not initial_pose_file:
            raise RuntimeError('initial_pose_file parameter is empty')

        path = Path(initial_pose_file)
        if not path.exists():
            raise RuntimeError(f'Initial pose file does not exist: {path}')

        with path.open('r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}

        pose = data.get('initial_pose')
        if not isinstance(pose, dict):
            raise RuntimeError(f'Initial pose file must contain initial_pose: {path}')

        return {
            'frame_id': str(pose.get('frame_id', 'map')),
            'x': float(pose['x']),
            'y': float(pose['y']),
            'yaw': float(pose['yaw']),
            'covariance_xy': float(pose.get('covariance_xy', 0.25)),
            'covariance_yaw': float(pose.get('covariance_yaw', 0.06853891909122467)),
        }

    def _publish_once(self):
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

        msg = self._make_message()
        self._publisher.publish(msg)
        self._remaining -= 1
        self._published_count += 1

        self.get_logger().info(
            f'Published initial pose: x={self._pose["x"]:.3f}, '
            f'y={self._pose["y"]:.3f}, yaw={self._pose["yaw"]:.3f}'
        )

        if self._remaining > 0:
            self._timer = self.create_timer(self._publish_period_sec, self._publish_once)
        else:
            self.get_logger().info('Initial pose publishing complete.')

    def _make_message(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._pose['frame_id']
        msg.pose.pose.position.x = self._pose['x']
        msg.pose.pose.position.y = self._pose['y']

        half_yaw = self._pose['yaw'] * 0.5
        msg.pose.pose.orientation.z = math.sin(half_yaw)
        msg.pose.pose.orientation.w = math.cos(half_yaw)

        msg.pose.covariance[0] = self._pose['covariance_xy']
        msg.pose.covariance[7] = self._pose['covariance_xy']
        msg.pose.covariance[35] = self._pose['covariance_yaw']
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
