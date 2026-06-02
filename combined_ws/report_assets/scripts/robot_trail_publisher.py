#!/usr/bin/env python3
"""Publish the robot's traveled path from AMCL poses for RViz screenshots."""

from math import hypot

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node


class RobotTrailPublisher(Node):
    def __init__(self) -> None:
        super().__init__("robot_trail_publisher")
        self.declare_parameter("min_distance_m", 0.04)
        self.declare_parameter("max_points", 5000)
        self.declare_parameter("trail_topic", "/robot_trail")

        self.min_distance_m = float(self.get_parameter("min_distance_m").value)
        self.max_points = int(self.get_parameter("max_points").value)
        trail_topic = str(self.get_parameter("trail_topic").value)

        self.path = Path()
        self.path.header.frame_id = "map"
        self.last_x: float | None = None
        self.last_y: float | None = None

        self.publisher = self.create_publisher(Path, trail_topic, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._pose_cb, 20)
        self.get_logger().info(f"Publishing AMCL trail on {trail_topic}")

    def _pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_x is not None and hypot(x - self.last_x, y - self.last_y) < self.min_distance_m:
            return

        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = "map"
        pose.pose = msg.pose.pose

        self.path.header.stamp = msg.header.stamp
        self.path.poses.append(pose)
        if len(self.path.poses) > self.max_points:
            self.path.poses = self.path.poses[-self.max_points :]

        self.last_x = x
        self.last_y = y
        self.publisher.publish(self.path)


def main() -> None:
    rclpy.init()
    node = RobotTrailPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
