#!/usr/bin/env python3
"""
SLAM Diagnostic Tool
====================
Checks TF chain, topic health, EKF odometry stability, and map updates.
Run this AFTER launching the simulation and SLAM.

Usage:
  python3 ~/robot_workspaces/combined_ws/diagnose_slam.py
"""

import sys
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped


class SlamDiagnostics(Node):
    def __init__(self):
        super().__init__('slam_diagnostics')

        # Counters
        self.counts = {
            '/scan': 0,
            '/drive_controller/odom': 0,
            '/imu/data_raw': 0,
            'tf: odom→base_link': 0,
            'tf: map→odom': 0,
            '/map': 0,
        }

        # Scan frame_id tracking
        self.scan_frame_id = None

        # EKF odom tracking for jump detection
        self.last_odom_x = None
        self.last_odom_y = None
        self.last_odom_yaw = None
        self.odom_jumps = 0
        self.max_odom_delta = 0.0

        # Map update tracking
        self.map_timestamps = []

        # Subscribe to all relevant topics
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(
            Odometry, '/drive_controller/odom', self.odom_raw_cb, 10
        )
        self.create_subscription(Imu, '/imu/data_raw', self.imu_cb, 10)
        self.create_subscription(TFMessage, '/tf', self.tf_cb, 10)

        # Map uses transient local durability
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, map_qos)

        # EKF filtered odom for jump detection
        self.create_subscription(
            Odometry, '/odometry/filtered', self.ekf_odom_cb, 10
        )

    def scan_cb(self, msg):
        self.counts['/scan'] += 1
        self.scan_frame_id = msg.header.frame_id

    def odom_raw_cb(self, msg):
        self.counts['/drive_controller/odom'] += 1

    def imu_cb(self, msg):
        self.counts['/imu/data_raw'] += 1

    def tf_cb(self, msg):
        for t in msg.transforms:
            key = f"tf: {t.header.frame_id}→{t.child_frame_id}"
            if key in self.counts:
                self.counts[key] += 1

    def map_cb(self, msg):
        self.counts['/map'] += 1
        now = time.time()
        self.map_timestamps.append(now)

    def ekf_odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)

        if self.last_odom_x is not None:
            dx = x - self.last_odom_x
            dy = y - self.last_odom_y
            dist = math.sqrt(dx * dx + dy * dy)
            self.max_odom_delta = max(self.max_odom_delta, dist)
            # A jump > 0.5m in a single update is suspicious
            if dist > 0.5:
                self.odom_jumps += 1

        self.last_odom_x = x
        self.last_odom_y = y
        self.last_odom_yaw = yaw


def main():
    rclpy.init()
    node = SlamDiagnostics()

    duration = 10.0
    print(f"🔍 SLAM Diagnostics — Listening for {duration:.0f} seconds...")
    print("   (Move the robot with teleop during this time for best results)\n")

    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    time.sleep(duration)

    # ── Report ──
    print("=" * 60)
    print("  SLAM DIAGNOSTIC REPORT")
    print("=" * 60)

    all_ok = True
    issues = []

    # Topic health
    print("\n📡 Topic Health:")
    for topic, count in node.counts.items():
        hz = count / duration
        if count > 0:
            status = f"✅ {count} msgs ({hz:.1f} Hz)"
        else:
            status = "❌ NO DATA"
            all_ok = False
        print(f"  {topic:30s} : {status}")

    # Scan frame_id check
    print(f"\n🔗 LaserScan frame_id: ", end="")
    if node.scan_frame_id is None:
        print("❌ No /scan received — cannot check")
        all_ok = False
        issues.append("No /scan data received. Is lidar_bridge running?")
    elif node.scan_frame_id == 'laser':
        print(f"✅ '{node.scan_frame_id}' (matches URDF)")
    else:
        print(f"⚠️  '{node.scan_frame_id}' (expected 'laser')")
        issues.append(
            f"LaserScan frame_id is '{node.scan_frame_id}' but URDF has 'laser'. "
            "TF chain may be broken. Consider using frame_id_override in lidar_bridge."
        )

    # EKF stability
    print(f"\n📊 EKF Odometry Stability:")
    print(f"  Max single-step delta : {node.max_odom_delta:.4f} m")
    print(f"  Jumps (> 0.5m)        : {node.odom_jumps}")
    if node.odom_jumps > 0:
        all_ok = False
        issues.append(
            f"EKF produced {node.odom_jumps} odometry jump(s) > 0.5m. "
            "This breaks scan matching. Check sensor_timeout in EKF config."
        )
    elif node.max_odom_delta > 0.2:
        issues.append(
            f"Max odom delta is {node.max_odom_delta:.3f}m — borderline. "
            "Reduce robot speed or increase EKF frequency."
        )

    # Map updates
    print(f"\n🗺️  Map Updates:")
    map_count = len(node.map_timestamps)
    if map_count == 0:
        print("  ❌ No map updates received!")
        all_ok = False
        issues.append(
            "slam_toolbox is not publishing /map updates. "
            "Possible causes: scan matching failure, TF chain broken, or "
            "minimum_travel_distance not reached."
        )
    elif map_count == 1:
        print(f"  ⚠️  Only 1 map update (initial). Map is NOT growing!")
        all_ok = False
        issues.append(
            "Only 1 map update = map frozen after initial scan. "
            "The scan matcher is failing. Try: reduce penalties, "
            "widen correlation_search_space_dimension, move robot slowly."
        )
    else:
        avg_interval = (
            (node.map_timestamps[-1] - node.map_timestamps[0]) / (map_count - 1)
        )
        print(f"  ✅ {map_count} updates, avg interval: {avg_interval:.1f}s")

    # TF chain
    print(f"\n🔗 TF Chain:")
    chain_ok = True
    if node.counts['tf: odom→base_link'] == 0:
        print("  ❌ odom → base_link : MISSING (EKF not publishing)")
        chain_ok = False
        issues.append(
            "EKF is not publishing odom→base_link TF. Check that EKF is "
            "receiving both /drive_controller/odom and /imu/data_raw."
        )
    else:
        print(f"  ✅ odom → base_link : OK ({node.counts['tf: odom→base_link']} msgs)")
    if node.counts['tf: map→odom'] == 0:
        print("  ❌ map → odom : MISSING (slam_toolbox not publishing)")
        chain_ok = False
        issues.append(
            "slam_toolbox is not publishing map→odom TF. This means SLAM "
            "scan matching has failed completely."
        )
    else:
        print(f"  ✅ map → odom : OK ({node.counts['tf: map→odom']} msgs)")

    # Summary
    print("\n" + "=" * 60)
    if all_ok:
        print("✅ EVERYTHING LOOKS GOOD! SLAM should be working.")
    else:
        print("❌ ISSUES DETECTED:")
        for i, issue in enumerate(issues, 1):
            print(f"   {i}. {issue}")
    print("=" * 60)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
