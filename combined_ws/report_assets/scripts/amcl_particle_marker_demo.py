#!/usr/bin/env python3
import math
import random

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from geometry_msgs.msg import PoseWithCovarianceStamped


class AmclParticleMarkerDemo(Node):
    def __init__(self):
        super().__init__("amcl_particle_marker_demo")
        self.pose = None
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.on_pose,
            10,
        )
        self.pub = self.create_publisher(MarkerArray, "/amcl_particle_markers", 10)
        self.timer = self.create_timer(0.5, self.publish_markers)

    def on_pose(self, msg):
        self.pose = msg.pose.pose

    def publish_markers(self):
        pose = self.pose
        if pose is None:
            x = 0.87
            y = 0.01
        else:
            x = pose.position.x
            y = pose.position.y

        rng = random.Random(42)
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "amcl_particles"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.045
        marker.scale.y = 0.045
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.22
        marker.color.a = 1.0

        # Compact deterministic cloud around the current AMCL pose for reporting.
        for i in range(180):
            angle = rng.random() * math.tau
            radius = abs(rng.gauss(0.0, 0.22))
            stretch = 1.0 + 0.45 * math.sin(angle)
            marker.points.append(
                Point(
                    x=x + math.cos(angle) * radius * stretch,
                    y=y + math.sin(angle) * radius * 0.65,
                    z=0.03,
                )
            )

        self.pub.publish(MarkerArray(markers=[marker]))


def main():
    rclpy.init()
    node = AmclParticleMarkerDemo()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
