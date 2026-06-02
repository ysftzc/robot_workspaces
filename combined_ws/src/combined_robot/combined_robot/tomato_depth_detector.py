"""Depth-based red tomato target publisher.

Publishes:
  /tomato_center  geometry_msgs/PoseStamped in the camera optical frame
  /tomato_radius  std_msgs/Float32 in meters
"""

import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32

try:
    import cv2
except ImportError:
    cv2 = None


class TomatoDepthDetector(Node):
    def __init__(self):
        super().__init__("tomato_depth_detector")

        self.color_topic = self.declare_parameter(
            "color_topic", "/camera/color/image_raw"
        ).value
        self.depth_topic = self.declare_parameter(
            "depth_topic", "/camera/depth/image"
        ).value
        self.camera_info_topic = self.declare_parameter(
            "camera_info_topic", "/camera/depth/camera_info"
        ).value
        self.output_pose_topic = self.declare_parameter(
            "output_pose_topic", "/tomato_center"
        ).value
        self.output_radius_topic = self.declare_parameter(
            "output_radius_topic", "/tomato_radius"
        ).value
        self.output_frame_id = self.declare_parameter(
            "output_frame_id", "fr3_camera_depth_optical_frame"
        ).value

        self.red_min = int(self.declare_parameter("red_min", 90).value)
        self.red_green_margin = int(
            self.declare_parameter("red_green_margin", 25).value
        )
        self.red_blue_margin = int(self.declare_parameter("red_blue_margin", 25).value)
        self.green_max = int(self.declare_parameter("green_max", 170).value)
        self.blue_max = int(self.declare_parameter("blue_max", 170).value)
        self.min_component_area = int(
            self.declare_parameter("min_component_area", 80).value
        )
        self.max_components = int(self.declare_parameter("max_components", 12).value)
        self.min_depth = float(self.declare_parameter("min_depth", 0.08).value)
        self.max_depth = float(self.declare_parameter("max_depth", 4.0).value)
        self.min_radius = float(self.declare_parameter("min_radius", 0.025).value)
        self.max_radius = float(self.declare_parameter("max_radius", 0.055).value)
        self.min_center_depth = float(
            self.declare_parameter("min_center_depth", 0.12).value
        )
        self.max_center_depth = float(
            self.declare_parameter("max_center_depth", 0.90).value
        )
        self.max_abs_x = float(self.declare_parameter("max_abs_x", 0.42).value)
        self.max_abs_y = float(self.declare_parameter("max_abs_y", 0.45).value)
        self.center_preference_weight = float(
            self.declare_parameter("center_preference_weight", 0.35).value
        )
        self.center_method = str(
            self.declare_parameter("center_method", "bbox").value
        ).lower()
        if self.center_method not in ("bbox", "centroid", "circle"):
            self.get_logger().warn(
                f"Unknown center_method='{self.center_method}', falling back to bbox"
            )
            self.center_method = "bbox"
        self.roi_width_fraction = self._clamp_fraction(
            self.declare_parameter("roi_width_fraction", 1.0).value,
            "roi_width_fraction",
        )
        self.roi_height_fraction = self._clamp_fraction(
            self.declare_parameter("roi_height_fraction", 1.0).value,
            "roi_height_fraction",
        )
        self.roi_center_u_fraction = self._clamp_center_fraction(
            self.declare_parameter("roi_center_u_fraction", 0.5).value,
            "roi_center_u_fraction",
        )
        self.roi_center_v_fraction = self._clamp_center_fraction(
            self.declare_parameter("roi_center_v_fraction", 0.5).value,
            "roi_center_v_fraction",
        )
        self.publish_rate = float(self.declare_parameter("publish_rate", 5.0).value)
        self.depth_sample_limit = int(
            self.declare_parameter("depth_sample_limit", 5000).value
        )
        self.mirror_depth_u = bool(self.declare_parameter("mirror_depth_u", False).value)
        self.mirror_depth_projection = bool(
            self.declare_parameter("mirror_depth_projection", False).value
        )
        self.surface_depth_percentile = float(
            self.declare_parameter("surface_depth_percentile", 20.0).value
        )
        self.surface_depth_percentile = min(max(self.surface_depth_percentile, 0.0), 100.0)
        self.target_strategy = self.declare_parameter(
            "target_strategy", "nearest"
        ).value

        self._last_depth = None
        self._last_camera_info = None
        self._last_publish_time = 0.0
        self._last_status_time = 0.0

        self.pose_pub = self.create_publisher(PoseStamped, self.output_pose_topic, 10)
        self.radius_pub = self.create_publisher(Float32, self.output_radius_topic, 10)

        self.create_subscription(Image, self.color_topic, self._color_cb, 10)
        self.create_subscription(Image, self.depth_topic, self._depth_cb, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self._info_cb, 10)

        self.get_logger().info(
            "Depth tomato detector started: "
            f"color={self.color_topic}, depth={self.depth_topic}, "
            f"camera_info={self.camera_info_topic}, pose_out={self.output_pose_topic}, "
            f"radius_out={self.output_radius_topic}, target_strategy={self.target_strategy}, "
            f"center_method={self.center_method}, "
            f"roi=({self.roi_width_fraction:.2f}w, {self.roi_height_fraction:.2f}h "
            f"at {self.roi_center_u_fraction:.2f}u, {self.roi_center_v_fraction:.2f}v)"
        )

    def _info_cb(self, msg):
        self._last_camera_info = msg

    def _depth_cb(self, msg):
        try:
            self._last_depth = (msg, self._image_to_depth(msg))
        except ValueError as exc:
            self.get_logger().warn(f"Depth image rejected: {exc}")

    def _color_cb(self, msg):
        now = time.monotonic()
        min_period = 1.0 / self.publish_rate if self.publish_rate > 0.0 else 0.0
        if now - self._last_publish_time < min_period:
            return
        self._last_publish_time = now

        if self._last_depth is None or self._last_camera_info is None:
            self._status("Waiting for depth image and camera info")
            return

        try:
            rgb = self._image_to_rgb(msg)
        except ValueError as exc:
            self.get_logger().warn(f"Color image rejected: {exc}")
            return

        depth_msg, depth = self._last_depth
        mask = self._apply_center_roi(self._red_mask(rgb))
        red_pixels = int(np.count_nonzero(mask))
        candidates = self._component_candidates(mask, depth, self._last_camera_info)
        candidates = [
            candidate
            for candidate in candidates
            if self._candidate_passes_filters(candidate)
        ]
        if not candidates:
            self._status(f"No valid red tomato component with depth; red_pixels={red_pixels}")
            return

        candidate = self._select_candidate(candidates)
        self._publish_candidate(candidate, msg, depth_msg, self._last_camera_info)

    def _status(self, text):
        now = time.monotonic()
        if now - self._last_status_time >= 2.0:
            self._last_status_time = now
            self.get_logger().info(text)

    def _image_to_rgb(self, msg):
        encoding = msg.encoding.lower()
        channel_count = {
            "rgb8": 3,
            "bgr8": 3,
            "rgba8": 4,
            "bgra8": 4,
        }.get(encoding)
        if channel_count is None:
            raise ValueError(f"unsupported color encoding '{msg.encoding}'")

        row = np.frombuffer(msg.data, dtype=np.uint8)
        expected_step = msg.width * channel_count
        if msg.step < expected_step:
            raise ValueError(
                f"image step {msg.step} is smaller than expected {expected_step}"
            )
        row = row.reshape((msg.height, msg.step))
        pixels = row[:, :expected_step].reshape((msg.height, msg.width, channel_count))
        if encoding == "rgb8":
            return pixels[:, :, :3]
        if encoding == "bgr8":
            return pixels[:, :, [2, 1, 0]]
        if encoding == "rgba8":
            return pixels[:, :, :3]
        return pixels[:, :, [2, 1, 0]]

    def _image_to_depth(self, msg):
        encoding = msg.encoding.lower()
        if encoding in ("32fc1", "32fc"):
            dtype = np.float32
            scale = 1.0
        elif encoding in ("16uc1", "mono16"):
            dtype = np.uint16
            scale = 0.001
        else:
            raise ValueError(f"unsupported depth encoding '{msg.encoding}'")

        item_size = np.dtype(dtype).itemsize
        expected_step = msg.width * item_size
        if msg.step < expected_step:
            raise ValueError(
                f"image step {msg.step} is smaller than expected {expected_step}"
            )

        arr = np.frombuffer(msg.data, dtype=dtype)
        if msg.is_bigendian != (sys.byteorder == "big"):
            arr = arr.byteswap()
        row_elems = msg.step // item_size
        depth = arr.reshape((msg.height, row_elems))[:, : msg.width].astype(np.float32)
        return depth * scale

    def _red_mask(self, rgb):
        values = rgb.astype(np.int16)
        r = values[:, :, 0]
        g = values[:, :, 1]
        b = values[:, :, 2]
        return (
            (r >= self.red_min)
            & (r - g >= self.red_green_margin)
            & (r - b >= self.red_blue_margin)
            & (g <= self.green_max)
            & (b <= self.blue_max)
        )

    def _clamp_fraction(self, value, name):
        fraction = float(value)
        if fraction <= 0.0:
            self.get_logger().warn(f"{name} must be > 0; using 1.0")
            return 1.0
        return min(fraction, 1.0)

    def _clamp_center_fraction(self, value, name):
        fraction = float(value)
        if fraction < 0.0 or fraction > 1.0:
            clamped = min(max(fraction, 0.0), 1.0)
            self.get_logger().warn(f"{name} must be in [0, 1]; using {clamped:.2f}")
            return clamped
        return fraction

    def _apply_center_roi(self, mask):
        if self.roi_width_fraction >= 1.0 and self.roi_height_fraction >= 1.0:
            return mask

        height, width = mask.shape
        roi_width = max(1, int(round(width * self.roi_width_fraction)))
        roi_height = max(1, int(round(height * self.roi_height_fraction)))
        center_u = int(round((width - 1) * self.roi_center_u_fraction))
        center_v = int(round((height - 1) * self.roi_center_v_fraction))
        u0 = min(max(0, center_u - roi_width // 2), max(0, width - roi_width))
        v0 = min(max(0, center_v - roi_height // 2), max(0, height - roi_height))
        u1 = min(width, u0 + roi_width)
        v1 = min(height, v0 + roi_height)

        cropped = np.zeros_like(mask, dtype=bool)
        cropped[v0:v1, u0:u1] = mask[v0:v1, u0:u1]
        return cropped

    def _component_candidates(self, mask, depth, camera_info):
        if cv2 is not None:
            count, labels, stats, centroids = cv2.connectedComponentsWithStats(
                mask.astype(np.uint8), 8
            )
            order = sorted(
                range(1, count),
                key=lambda i: stats[i, cv2.CC_STAT_AREA],
                reverse=True,
            )[: self.max_components]
            components = [
                (
                    i,
                    int(stats[i, cv2.CC_STAT_AREA]),
                    float(centroids[i][0]),
                    float(centroids[i][1]),
                )
                for i in order
                if int(stats[i, cv2.CC_STAT_AREA]) >= self.min_component_area
            ]
            return [
                candidate
                for component in components
                if (candidate := self._candidate_from_component(labels, component, depth, camera_info))
                is not None
            ]

        return self._fallback_component_candidates(mask, depth, camera_info)

    def _fallback_component_candidates(self, mask, depth, camera_info):
        visited = np.zeros(mask.shape, dtype=bool)
        components = []
        height, width = mask.shape
        for start_v, start_u in np.argwhere(mask):
            if visited[start_v, start_u]:
                continue
            stack = [(int(start_v), int(start_u))]
            visited[start_v, start_u] = True
            pixels = []
            while stack:
                v, u = stack.pop()
                pixels.append((v, u))
                for nv in (v - 1, v, v + 1):
                    for nu in (u - 1, u, u + 1):
                        if (
                            0 <= nv < height
                            and 0 <= nu < width
                            and not visited[nv, nu]
                            and mask[nv, nu]
                        ):
                            visited[nv, nu] = True
                            stack.append((nv, nu))
            if len(pixels) >= self.min_component_area:
                coords = np.asarray(pixels)
                area = len(pixels)
                components.append(
                    (coords, area, float(coords[:, 1].mean()), float(coords[:, 0].mean()))
                )
        components.sort(key=lambda item: item[1], reverse=True)
        candidates = []
        for coords, area, centroid_u, centroid_v in components[: self.max_components]:
            candidate = self._candidate_from_coords(
                coords, area, centroid_u, centroid_v, mask.shape, depth, camera_info
            )
            if candidate is not None:
                candidates.append(candidate)
        return candidates

    def _candidate_from_component(self, labels, component, depth, camera_info):
        label, area, centroid_u, centroid_v = component
        coords = np.argwhere(labels == label)
        return self._candidate_from_coords(
            coords, area, centroid_u, centroid_v, labels.shape, depth, camera_info
        )

    def _candidate_from_coords(self, coords, area, centroid_u, centroid_v, color_shape, depth, camera_info):
        if coords.size == 0:
            return None
        center_u, center_v, bbox_radius = self._target_pixel_center_and_radius(
            coords, centroid_u, centroid_v
        )
        if coords.shape[0] > self.depth_sample_limit:
            step = max(1, coords.shape[0] // self.depth_sample_limit)
            coords = coords[::step]

        color_h, color_w = color_shape
        depth_h, depth_w = depth.shape
        scale_u = depth_w / float(color_w)
        scale_v = depth_h / float(color_h)
        color_u = center_u * scale_u
        v = center_v * scale_v
        depth_center_u = (
            (depth_w - 1) - color_u
            if self.mirror_depth_u and self.mirror_depth_projection
            else color_u
        )
        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            return None

        focal = 0.5 * (fx + fy)
        area_radius = math.sqrt(area / math.pi)
        pixel_radius = max(area_radius, bbox_radius) * 0.5 * (scale_u + scale_v)
        depth_u = np.clip(np.rint(coords[:, 1] * scale_u).astype(np.int32), 0, depth_w - 1)
        if self.mirror_depth_u:
            depth_u = (depth_w - 1) - depth_u
        depth_v = np.clip(np.rint(coords[:, 0] * scale_v).astype(np.int32), 0, depth_h - 1)
        samples = depth[depth_v, depth_u]
        valid = np.isfinite(samples) & (samples >= self.min_depth) & (samples <= self.max_depth)
        valid_count = int(np.count_nonzero(valid))
        if valid_count >= 10:
            surface_depth = float(
                np.percentile(samples[valid], self.surface_depth_percentile)
            )
        else:
            surface_depth = self._centroid_depth_fallback(depth, depth_center_u, v, pixel_radius)
        if surface_depth is None:
            return None

        radius = pixel_radius * surface_depth / focal
        radius = min(max(radius, self.min_radius), self.max_radius)
        center_depth = surface_depth + radius
        x = (depth_center_u - cx) * center_depth / fx
        y = (v - cy) * center_depth / fy

        return {
            "area": area,
            "color_pixel": (color_u, v),
            "depth_pixel": (depth_center_u, v),
            "surface_depth": surface_depth,
            "radius": radius,
            "center": (x, y, center_depth),
            "center_method": self.center_method,
        }

    def _candidate_passes_filters(self, candidate):
        x, y, z = candidate["center"]
        return (
            self.min_center_depth <= z <= self.max_center_depth
            and abs(x) <= self.max_abs_x
            and abs(y) <= self.max_abs_y
        )

    def _centroid_depth_fallback(self, depth, u, v, pixel_radius):
        height, width = depth.shape
        half_size = int(min(max(pixel_radius * 0.7, 4.0), 30.0))
        center_u = int(round(u))
        center_v = int(round(v))
        u0 = max(0, center_u - half_size)
        u1 = min(width, center_u + half_size + 1)
        v0 = max(0, center_v - half_size)
        v1 = min(height, center_v + half_size + 1)
        if u0 >= u1 or v0 >= v1:
            return None
        window = depth[v0:v1, u0:u1]
        valid = window[
            np.isfinite(window)
            & (window >= self.min_depth)
            & (window <= self.max_depth)
        ]
        if valid.size < 10:
            return None
        return float(np.percentile(valid, 20))

    def _target_pixel_center_and_radius(self, coords, centroid_u, centroid_v):
        u_values = coords[:, 1].astype(np.float32)
        v_values = coords[:, 0].astype(np.float32)
        min_u = float(np.min(u_values))
        max_u = float(np.max(u_values))
        min_v = float(np.min(v_values))
        max_v = float(np.max(v_values))
        bbox_center_u = 0.5 * (min_u + max_u)
        bbox_center_v = 0.5 * (min_v + max_v)
        bbox_radius = 0.5 * max(max_u - min_u + 1.0, max_v - min_v + 1.0)

        if self.center_method == "centroid":
            return float(centroid_u), float(centroid_v), bbox_radius
        if self.center_method == "circle" and cv2 is not None:
            points = np.column_stack((u_values, v_values)).astype(np.float32)
            (circle_u, circle_v), circle_radius = cv2.minEnclosingCircle(points)
            return float(circle_u), float(circle_v), max(bbox_radius, float(circle_radius))
        return bbox_center_u, bbox_center_v, bbox_radius

    def _select_candidate(self, candidates):
        if self.target_strategy == "largest":
            return max(candidates, key=lambda item: item["area"])
        if self.target_strategy == "center":
            return min(
                candidates,
                key=lambda item: math.hypot(item["center"][0], item["center"][1]),
            )
        if self.target_strategy == "center_nearest":
            return min(
                candidates,
                key=lambda item: item["surface_depth"]
                + self.center_preference_weight
                * math.hypot(item["center"][0], item["center"][1]),
            )
        return min(candidates, key=lambda item: item["surface_depth"])

    def _publish_candidate(self, candidate, color_msg, depth_msg, camera_info):
        frame_id = self.output_frame_id or (
            camera_info.header.frame_id
            or depth_msg.header.frame_id
            or color_msg.header.frame_id
        )
        stamp = depth_msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = color_msg.header.stamp

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = stamp
        pose.pose.position.x = candidate["center"][0]
        pose.pose.position.y = candidate["center"][1]
        pose.pose.position.z = candidate["center"][2]
        pose.pose.orientation.w = 1.0

        radius = Float32()
        radius.data = float(candidate["radius"])

        self.pose_pub.publish(pose)
        self.radius_pub.publish(radius)
        self.get_logger().info(
            "Published depth tomato target: "
            f"frame={frame_id}, center=({pose.pose.position.x:.3f}, "
            f"{pose.pose.position.y:.3f}, {pose.pose.position.z:.3f}), "
            f"radius={radius.data:.3f}, surface_depth={candidate['surface_depth']:.3f}, "
            f"center_method={candidate['center_method']}, "
            f"depth_pixel=({candidate['depth_pixel'][0]:.1f}, {candidate['depth_pixel'][1]:.1f}), "
            f"area={candidate['area']}"
        )


def main():
    rclpy.init()
    node = TomatoDepthDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
