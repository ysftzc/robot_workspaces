"""RGB-D pixel-to-3D tomato mapper with detector adapters.

The node intentionally does not depend on a YOLO message type. Current test
adapters provide one pixel per frame; a future YOLO adapter can feed bbox
centers into the same depth, TF, merge, panel, and JSON publishing pipeline.
"""

from __future__ import annotations

import json
import math
import re
import subprocess
import sys
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Iterable
from xml.etree import ElementTree as ET

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


PARAMETER_DESCRIPTOR = ParameterDescriptor(dynamic_typing=True)


@dataclass(frozen=True)
class PixelDetection:
    u: float
    v: float
    mode: str
    class_id: int | None = None
    class_name: str = ""
    confidence: float | None = None
    detection_id: str = ""
    bbox: list[float] | None = None


@dataclass(frozen=True)
class TomatoModel:
    name: str
    x: float
    y: float
    z: float
    radius_m: float
    gazebo_x: float = 0.0
    gazebo_y: float = 0.0
    gazebo_z: float = 0.0


class DetectionAdapter:
    def detections(self, node: "TomatoDepthMapper", color_msg: Image) -> list[PixelDetection]:
        raise NotImplementedError


class TestCenterPixelAdapter(DetectionAdapter):
    def detections(self, node: "TomatoDepthMapper", color_msg: Image) -> list[PixelDetection]:
        return [
            PixelDetection(
                u=(float(color_msg.width) - 1.0) * 0.5,
                v=(float(color_msg.height) - 1.0) * 0.5,
                mode="test_center_pixel",
            )
        ]


class ManualPixelAdapter(DetectionAdapter):
    def detections(self, node: "TomatoDepthMapper", _color_msg: Image) -> list[PixelDetection]:
        return [
            PixelDetection(
                u=float(node.pixel_u),
                v=float(node.pixel_v),
                mode="manual_pixel",
            )
        ]


class YoloTopicFutureAdapter(DetectionAdapter):
    """Placeholder adapter using generic JSON, not a YOLO-specific message type.

    Accepted future std_msgs/String formats:
      {"detections": [{"center_u": 320, "center_v": 240}, ...]}
      {"detections": [{"bbox": [x, y, w, h]}, ...]}
      [{"center_u": 320, "center_v": 240}, ...]
    """

    def detections(self, node: "TomatoDepthMapper", _color_msg: Image) -> list[PixelDetection]:
        if not node._future_detections:
            node._status("yolo_topic_future selected, waiting for generic JSON detections")
            return []
        return [
            replace(detection, mode="yolo_topic_future")
            for detection in node._future_detections
        ]


class TomatoDepthMapper(Node):
    def __init__(self):
        super().__init__("tomato_depth_mapper")

        self.rgb_topic = self._string_param("rgb_topic", "/camera/color/image_raw")
        self.depth_topic = self._string_param("depth_topic", "/camera/depth/image")
        self.camera_info_topic = self._string_param(
            "camera_info_topic", "/camera/depth/camera_info"
        )
        self.current_waypoint_topic = self._string_param(
            "current_waypoint_topic", "/mission_manager/status"
        )
        self.current_waypoint_default = self._string_param(
            "current_waypoint", "unknown"
        )
        self.amcl_pose_topic = self._string_param("amcl_pose_topic", "/amcl_pose")
        self.future_detection_topic = self._string_param(
            "future_detection_topic", "/yolo/tomato_detections_json"
        )
        self.output_topic = self._string_param("output_topic", "/tomato_map/list")

        self.detection_mode = self._string_param(
            "detection_mode", "test_center_pixel"
        ).strip().lower()
        self.pixel_u = self._float_param("pixel_u", 320.0)
        self.pixel_v = self._float_param("pixel_v", 240.0)
        self.min_depth_m = self._float_param("min_depth_m", 0.15)
        self.max_depth_m = self._float_param("max_depth_m", 3.0)
        self.merge_distance_m = self._float_param("merge_distance_m", 0.05)
        self.publish_rate_hz = self._float_param("publish_rate_hz", 5.0)
        self.panel_rate_hz = self._float_param("panel_rate_hz", 0.0)
        self.max_panel_rows = self._int_param("max_panel_rows", 30)
        self.depth_window_radius_px = self._int_param("depth_window_radius_px", 2)
        self.depth_bbox_prefer_inner_roi = self._bool_param(
            "depth_bbox_prefer_inner_roi", True
        )
        self.depth_bbox_inner_fraction = self._float_param(
            "depth_bbox_inner_fraction", 0.30
        )
        self.depth_bbox_min_valid_samples = self._int_param(
            "depth_bbox_min_valid_samples", 5
        )
        self.depth_bbox_fallback_enabled = self._bool_param(
            "depth_bbox_fallback_enabled", True
        )
        self.depth_bbox_percentile = self._float_param("depth_bbox_percentile", 50.0)
        self.tf_timeout_sec = self._float_param("tf_timeout_sec", 0.08)
        self.use_latest_tf = self._bool_param("use_latest_tf", True)
        self.model_filter_enabled = self._bool_param("model_filter_enabled", False)
        self.model_filter_world_file = self._string_param("model_filter_world_file", "")
        self.model_filter_frame = self._string_param("model_filter_frame", "map")
        self.model_filter_max_distance_m = self._float_param(
            "model_filter_max_distance_m", 0.50
        )
        self.model_filter_map_x_from_gazebo_y_offset = self._float_param(
            "model_filter_map_x_from_gazebo_y_offset", -4.93
        )
        self.model_filter_map_y_from_gazebo_x_origin = self._float_param(
            "model_filter_map_y_from_gazebo_x_origin", 35.83
        )
        self.model_filter_use_live_gazebo_pose = self._bool_param(
            "model_filter_use_live_gazebo_pose", True
        )
        self.model_filter_robot_model = self._string_param(
            "model_filter_robot_model", "combined_robot"
        )
        self.model_filter_robot_base_z_offset = self._float_param(
            "model_filter_robot_base_z_offset", 0.1715
        )
        self.model_filter_live_pose_timeout_sec = self._float_param(
            "model_filter_live_pose_timeout_sec", 0.8
        )
        self.model_filter_live_pose_query_period_sec = self._float_param(
            "model_filter_live_pose_query_period_sec", 0.5
        )
        self.model_filter_snap_to_model_center = self._bool_param(
            "model_filter_snap_to_model_center", False
        )
        self.model_filter_match_strategy = self._string_param(
            "model_filter_match_strategy", "hybrid"
        ).strip().lower()
        self.model_filter_projection_max_center_px = self._float_param(
            "model_filter_projection_max_center_px", 110.0
        )
        self.model_filter_projection_bbox_margin = self._float_param(
            "model_filter_projection_bbox_margin", 0.60
        )
        self.model_filter_projection_class_gate = self._bool_param(
            "model_filter_projection_class_gate", True
        )
        self.correct_depth_to_center = self._bool_param("correct_depth_to_center", True)

        self.camera_frame = self._string_param(
            "camera_frame", "fr3_camera_depth_optical_frame"
        )
        self.target_frame = self._string_param("target_frame", "base_link")
        self.global_frame = self._string_param("global_frame", "map")
        self.fallback_global_frame = self._string_param("fallback_global_frame", "odom")
        self.prefer_global_frame = self._bool_param("prefer_global_frame", True)

        self._last_depth: tuple[Image, np.ndarray] | None = None
        self._last_camera_info: CameraInfo | None = None
        self._latest_amcl_pose: PoseWithCovarianceStamped | None = None
        self._current_waypoint = self.current_waypoint_default
        self._future_detections: list[PixelDetection] = []
        self._records: list[dict] = []
        self._next_id = 1
        self._last_publish_time = 0.0
        self._last_panel_time = 0.0
        self._last_status_time = 0.0
        self._filtered_reject_count = 0
        self._last_model_filter_robot_pose_time = 0.0
        self._cached_model_filter_robot_pose_value: tuple[float, float, float, float] | None = None
        self._tomato_models = self._load_tomato_models()

        self._adapter = self._make_adapter(self.detection_mode)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._list_pub = self.create_publisher(String, self.output_topic, 10)
        self.create_subscription(Image, self.rgb_topic, self._rgb_cb, 10)
        self.create_subscription(Image, self.depth_topic, self._depth_cb, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self._info_cb, 10)
        if self.current_waypoint_topic:
            self.create_subscription(String, self.current_waypoint_topic, self._waypoint_cb, 10)
        if self.amcl_pose_topic:
            amcl_qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self.create_subscription(
                PoseWithCovarianceStamped,
                self.amcl_pose_topic,
                self._amcl_pose_cb,
                amcl_qos,
            )
        if self.future_detection_topic:
            self.create_subscription(String, self.future_detection_topic, self._future_cb, 10)

        self._dynamic_params = [
            'model_filter_max_distance_m', 'merge_distance_m',
            'min_depth_m', 'max_depth_m',
            'depth_bbox_inner_fraction', 'depth_bbox_percentile',
        ]
        self.create_timer(2.0, self._refresh_dynamic_params)

        self.get_logger().info(
            "Tomato depth mapper started: "
            f"mode={self.detection_mode}, rgb={self.rgb_topic}, depth={self.depth_topic}, "
            f"camera_info={self.camera_info_topic}, output={self.output_topic}, "
            f"camera_frame={self.camera_frame}, target_frame={self.target_frame}, "
            f"global_frame={self.global_frame}, fallback_global_frame={self.fallback_global_frame}, "
            f"use_latest_tf={self.use_latest_tf}"
        )
        if self.model_filter_enabled:
            self.get_logger().info(
                "Tomato model filter enabled: "
                f"models={len(self._tomato_models)}, frame={self.model_filter_frame}, "
                f"max_distance={self.model_filter_max_distance_m:.2f}m, "
                f"snap_to_model_center={self.model_filter_snap_to_model_center}, "
                f"match_strategy={self.model_filter_match_strategy}, "
                f"projection_limit={self.model_filter_projection_max_center_px:.1f}px"
            )

    def _string_param(self, name: str, default: str) -> str:
        self.declare_parameter(name, default, PARAMETER_DESCRIPTOR)
        value = self.get_parameter(name).value
        return default if value is None else str(value)

    def _float_param(self, name: str, default: float) -> float:
        self.declare_parameter(name, default, PARAMETER_DESCRIPTOR)
        try:
            return float(self.get_parameter(name).value)
        except (TypeError, ValueError):
            self.get_logger().warn(f"Invalid {name}; using {default}")
            return float(default)

    def _int_param(self, name: str, default: int) -> int:
        self.declare_parameter(name, default, PARAMETER_DESCRIPTOR)
        try:
            return int(float(self.get_parameter(name).value))
        except (TypeError, ValueError):
            self.get_logger().warn(f"Invalid {name}; using {default}")
            return int(default)

    def _bool_param(self, name: str, default: bool) -> bool:
        self.declare_parameter(name, default, PARAMETER_DESCRIPTOR)
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _refresh_dynamic_params(self) -> None:
        for name in self._dynamic_params:
            try:
                value = self.get_parameter(name).value
                if value is not None:
                    setattr(self, name, float(value))
            except Exception:  # noqa: BLE001
                pass

    def _make_adapter(self, mode: str) -> DetectionAdapter:
        if mode == "test_center_pixel":
            return TestCenterPixelAdapter()
        if mode == "manual_pixel":
            return ManualPixelAdapter()
        if mode == "yolo_topic_future":
            return YoloTopicFutureAdapter()
        self.get_logger().warn(
            f"Unknown detection_mode='{mode}', falling back to test_center_pixel"
        )
        self.detection_mode = "test_center_pixel"
        return TestCenterPixelAdapter()

    def _info_cb(self, msg: CameraInfo) -> None:
        self._last_camera_info = msg

    def _depth_cb(self, msg: Image) -> None:
        try:
            self._last_depth = (msg, self._image_to_depth(msg))
        except ValueError as exc:
            self.get_logger().warn(f"Depth image rejected: {exc}")

    def _waypoint_cb(self, msg: String) -> None:
        self._current_waypoint = self._parse_waypoint_status(msg.data)

    def _amcl_pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        self._latest_amcl_pose = msg

    def _future_cb(self, msg: String) -> None:
        try:
            parsed = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Future detection JSON rejected: {exc}")
            return
        self._future_detections = list(self._detections_from_future_json(parsed))

    def _rgb_cb(self, msg: Image) -> None:
        now = time.monotonic()
        min_period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.0
        if now - self._last_publish_time < min_period:
            return
        self._last_publish_time = now

        if self._last_depth is None or self._last_camera_info is None:
            self._status("Waiting for depth image and camera info")
            return

        depth_msg, depth = self._last_depth
        detections = self._adapter.detections(self, msg)
        if not detections:
            self._publish_list()
            self._maybe_print_panel()
            return

        for detection in detections:
            record = self._record_from_detection(detection, msg, depth_msg, depth)
            if record is not None:
                self._merge_or_add(record)

        self._publish_list()
        self._maybe_print_panel()

    def _record_from_detection(
        self,
        detection: PixelDetection,
        color_msg: Image,
        depth_msg: Image,
        depth: np.ndarray,
    ) -> dict | None:
        camera_info = self._last_camera_info
        if camera_info is None:
            return None

        color_u = float(detection.u)
        color_v = float(detection.v)
        if not (0.0 <= color_u < float(color_msg.width) and 0.0 <= color_v < float(color_msg.height)):
            self._status(
                f"Detection pixel outside RGB image: u={color_u:.1f}, v={color_v:.1f}"
            )
            return None

        source_frame = (
            self.camera_frame
            or camera_info.header.frame_id
            or depth_msg.header.frame_id
            or color_msg.header.frame_id
        )
        stamp = depth_msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = color_msg.header.stamp

        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            self._status("Invalid CameraInfo intrinsics: fx/fy must be positive")
            return None

        projection_match = self._match_model_filter_by_projection(
            detection,
            color_msg,
            camera_info,
            source_frame,
            stamp,
        )

        depth_h, depth_w = depth.shape
        depth_u = color_u * depth_w / max(float(color_msg.width), 1.0)
        depth_v = color_v * depth_h / max(float(color_msg.height), 1.0)
        depth_m = None
        depth_source = ""
        if detection.bbox and self.depth_bbox_prefer_inner_roi:
            depth_m = self._sample_depth_in_bbox(
                depth,
                detection.bbox,
                color_msg.width,
                color_msg.height,
                inner_fraction=self.depth_bbox_inner_fraction,
            )
            if depth_m is not None:
                depth_source = "bbox_inner_roi"

        if depth_m is None:
            depth_m = self._sample_depth(depth, depth_u, depth_v)
            depth_source = "center_window"

        if (
            depth_m is None
            and detection.bbox
            and self.depth_bbox_fallback_enabled
        ):
            depth_m = self._sample_depth_in_bbox(
                depth,
                detection.bbox,
                color_msg.width,
                color_msg.height,
                inner_fraction=1.0,
            )
            if depth_m is not None:
                depth_source = "bbox_fallback"
        if depth_m is None:
            if projection_match is None or not self.model_filter_snap_to_model_center:
                self._status(
                    f"No valid depth at pixel u={color_u:.1f}, v={color_v:.1f}"
                )
                return None
            depth_m = projection_match["camera_point"][2]
            depth_source = "model_projection"

        if depth_source == "model_projection" and projection_match is not None:
            camera_point = projection_match["camera_point"]
        else:
            camera_point = (
                (depth_u - cx) * depth_m / fx,
                (depth_v - cy) * depth_m / fy,
                depth_m,
            )

        if (
            projection_match is not None
            and self.model_filter_snap_to_model_center
            and projection_match.get("target_point") is not None
            and self.target_frame
        ):
            target_raw_point = self._try_transform_point(
                source_frame,
                self.target_frame,
                stamp,
                camera_point,
            )
            if target_raw_point is not None:
                frame_id, raw_point = self.target_frame, target_raw_point
            else:
                frame_id, raw_point = source_frame, camera_point
        else:
            frame_id, raw_point = self._select_output_point(source_frame, stamp, camera_point)
        matched_model_source = ""
        projection_info = None
        if projection_match is not None:
            matched_model = (projection_match["model"], 0.0)
            matched_model_source = "projection_2d"
            projection_info = projection_match
        else:
            matched_model = self._match_model_filter(frame_id, raw_point, stamp)
            if matched_model is not None:
                matched_model_source = "spatial_3d"
        if self.model_filter_enabled and self._tomato_models and matched_model is None:
            return None

        estimated_radius = None
        if detection.bbox:
            estimated_radius = self._estimate_radius_from_bbox(detection.bbox, depth_m, fx, fy)

        model_radius = matched_model[0].radius_m if matched_model is not None else None
        correction_radius = model_radius if model_radius is not None else estimated_radius
        target_frame_id = frame_id
        target_point = raw_point
        target_source = "depth_surface"
        if self.correct_depth_to_center and correction_radius is not None:
            corrected_camera_point = self._correct_depth_surface_to_center(
                camera_point,
                correction_radius,
            )
            target_frame_id, target_point = self._select_output_point(
                source_frame,
                stamp,
                corrected_camera_point,
            )
            target_source = "depth_radius_center"

        if matched_model is not None and self.model_filter_snap_to_model_center:
            model, _distance = matched_model
            if projection_info is not None and projection_info.get("target_point") is not None:
                target_frame_id = self.target_frame
                target_point = projection_info["target_point"]
            else:
                target_frame_id = self.model_filter_frame
                target_point = (model.x, model.y, model.z)
            target_source = "model_filter_center"

        timestamp = f"{stamp.sec}.{stamp.nanosec:09d}"
        record = {
            "tomato_id": "",
            "current_waypoint": self._current_waypoint,
            "detection_mode": detection.mode,
            "pixel_u": round(color_u, 2),
            "pixel_v": round(color_v, 2),
            "depth_m": round(depth_m, 4),
            "depth_source": depth_source,
            "x": round(target_point[0], 4),
            "y": round(target_point[1], 4),
            "z": round(target_point[2], 4),
            "frame_id": target_frame_id,
            "target_source": target_source,
            "raw_surface_x": round(raw_point[0], 4),
            "raw_surface_y": round(raw_point[1], 4),
            "raw_surface_z": round(raw_point[2], 4),
            "raw_surface_frame_id": frame_id,
            "timestamp": timestamp,
            "updated_count": 1,
        }
        if detection.class_id is not None:
            record["class_id"] = int(detection.class_id)
        if detection.class_name:
            record["tomato_class"] = detection.class_name
        if detection.confidence is not None:
            record["confidence"] = round(float(detection.confidence), 4)
        if detection.detection_id:
            record["source_detection_id"] = detection.detection_id
        if detection.bbox:
            record["bbox"] = detection.bbox
            if estimated_radius is not None:
                record["estimated_radius_m"] = round(estimated_radius, 4)
        if matched_model is not None:
            model, distance = matched_model
            record["model_name"] = model.name
            record["model_distance_m"] = round(distance, 4)
            record["model_match_source"] = matched_model_source
            record["radius_m"] = round(model.radius_m, 4)
            record["model_center_x"] = round(model.x, 4)
            record["model_center_y"] = round(model.y, 4)
            record["model_center_z"] = round(model.z, 4)
            record["model_center_frame_id"] = self.model_filter_frame
            if projection_info is not None:
                record["model_projection_u"] = round(projection_info["u"], 2)
                record["model_projection_v"] = round(projection_info["v"], 2)
                record["model_projection_error_px"] = round(
                    projection_info["error_px"], 2
                )
                if projection_info.get("target_point") is not None:
                    target_x, target_y, target_z = projection_info["target_point"]
                    record["model_center_target_x"] = round(target_x, 4)
                    record["model_center_target_y"] = round(target_y, 4)
                    record["model_center_target_z"] = round(target_z, 4)
                    record["model_center_target_frame_id"] = self.target_frame
        return record

    @staticmethod
    def _correct_depth_surface_to_center(
        camera_point: tuple[float, float, float],
        radius_m: float,
    ) -> tuple[float, float, float]:
        """Move a depth surface sample toward the estimated sphere center."""
        if radius_m <= 0.0 or not math.isfinite(radius_m):
            return camera_point
        ray_length = math.sqrt(sum(component * component for component in camera_point))
        if ray_length < 1e-9:
            return camera_point
        scale = radius_m / ray_length
        return tuple(component * (1.0 + scale) for component in camera_point)

    def _load_tomato_models(self) -> list[TomatoModel]:
        if not self.model_filter_enabled:
            return []

        path = self._resolve_world_file(self.model_filter_world_file)
        if path is None:
            self.get_logger().warn(
                "Tomato model filter is enabled, but model_filter_world_file is empty or unresolved. "
                "Detections will pass without model filtering."
            )
            return []

        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as exc:
            self.get_logger().warn(f"Tomato model filter SDF parse failed: {path}: {exc}")
            return []
        except OSError as exc:
            self.get_logger().warn(f"Tomato model filter SDF read failed: {path}: {exc}")
            return []

        models: list[TomatoModel] = []
        for model in root.findall(".//model"):
            name = model.get("name", "")
            if not name.startswith("tomato_"):
                continue
            pose_text = (model.findtext("pose") or "").strip()
            values = pose_text.split()
            if len(values) < 3:
                continue
            try:
                gazebo_x = float(values[0])
                gazebo_y = float(values[1])
                gazebo_z = float(values[2])
            except ValueError:
                continue
            radius_m = self._model_collision_radius(model)
            models.append(
                TomatoModel(
                    name=name,
                    x=gazebo_y + self.model_filter_map_x_from_gazebo_y_offset,
                    y=self.model_filter_map_y_from_gazebo_x_origin - gazebo_x,
                    z=gazebo_z,
                    radius_m=radius_m,
                    gazebo_x=gazebo_x,
                    gazebo_y=gazebo_y,
                    gazebo_z=gazebo_z,
                )
            )

        if not models:
            self.get_logger().warn(f"Tomato model filter found no tomato_* models in {path}")
        return models

    def _model_collision_radius(self, model: ET.Element) -> float:
        radius_text = model.findtext(".//collision/geometry/sphere/radius")
        if radius_text is None:
            radius_text = model.findtext(".//geometry/sphere/radius")
        try:
            radius = float((radius_text or "").strip())
        except ValueError:
            radius = 0.032
        if not math.isfinite(radius) or radius <= 0.0:
            return 0.032
        return radius

    def _estimate_radius_from_bbox(
        self,
        bbox: list[float],
        depth_m: float,
        fx: float,
        fy: float,
    ) -> float | None:
        if len(bbox) < 4 or fx <= 0.0 or fy <= 0.0:
            return None
        try:
            width_px = float(bbox[2])
            height_px = float(bbox[3])
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in (width_px, height_px, depth_m)):
            return None
        if width_px <= 0.0 or height_px <= 0.0 or depth_m <= 0.0:
            return None
        radius_x = 0.5 * width_px * depth_m / fx
        radius_y = 0.5 * height_px * depth_m / fy
        radius = 0.5 * (radius_x + radius_y)
        return min(max(radius, 0.015), 0.09)

    def _resolve_world_file(self, world_file: str) -> Path | None:
        if not world_file:
            return None
        path = Path(world_file).expanduser()
        if path.is_file():
            return path
        if not path.is_absolute() and get_package_share_directory is not None:
            try:
                candidate = (
                    Path(get_package_share_directory("combined_robot"))
                    / "worlds"
                    / path
                )
            except Exception:  # noqa: BLE001
                candidate = None
            if candidate is not None and candidate.is_file():
                return candidate
        return None

    def _gazebo_to_frame(
        self,
        model: "TomatoModel",
        target_frame: str,
        stamp,
    ) -> tuple[float, float, float] | None:
        """Convert a Gazebo tomato model pose to a ROS frame used by detections."""
        robot_frame = self.target_frame or "base_link"
        robot_point = self._gazebo_model_to_robot_frame(model)
        if robot_point is not None:
            if target_frame == robot_frame:
                return robot_point
            transformed = self._try_transform_point(robot_frame, target_frame, stamp, robot_point)
            if transformed is not None:
                return transformed

        map_point = (model.x, model.y, model.z)
        if target_frame == self.model_filter_frame:
            return map_point
        amcl_point = self._try_amcl_map_transform(
            self.model_filter_frame,
            target_frame,
            stamp,
            map_point,
        )
        if amcl_point is not None:
            return amcl_point
        transformed = self._try_transform_point(
            self.model_filter_frame, target_frame, stamp, map_point
        )
        if transformed is not None:
            return transformed
        return None

    def _gazebo_model_to_robot_frame(
        self,
        model: TomatoModel,
    ) -> tuple[float, float, float] | None:
        robot_pose = self._cached_model_filter_robot_pose()
        if robot_pose is None:
            return None

        robot_x, robot_y, robot_z, robot_yaw = robot_pose
        dx = model.gazebo_x - robot_x
        dy = model.gazebo_y - robot_y
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        return (
            cos_yaw * dx + sin_yaw * dy,
            -sin_yaw * dx + cos_yaw * dy,
            model.gazebo_z - (robot_z + self.model_filter_robot_base_z_offset),
        )

    def _cached_model_filter_robot_pose(self) -> tuple[float, float, float, float] | None:
        if not self.model_filter_use_live_gazebo_pose or not self.model_filter_robot_model:
            return None

        now = time.monotonic()
        if (
            self._cached_model_filter_robot_pose_value is not None
            and self.model_filter_live_pose_query_period_sec > 0.0
            and now - self._last_model_filter_robot_pose_time
            < self.model_filter_live_pose_query_period_sec
        ):
            return self._cached_model_filter_robot_pose_value

        self._last_model_filter_robot_pose_time = now
        pose = self._live_gazebo_model_pose(self.model_filter_robot_model)
        if pose is not None:
            self._cached_model_filter_robot_pose_value = pose
            return pose
        return self._cached_model_filter_robot_pose_value

    def _live_gazebo_model_pose(
        self,
        model_name: str,
    ) -> tuple[float, float, float, float] | None:
        timed_out = False
        try:
            result = subprocess.run(
                [
                    "gz",
                    "model",
                    "-m",
                    model_name,
                    "-p",
                    "--force-version",
                    "8",
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=max(0.2, self.model_filter_live_pose_timeout_sec),
            )
            output = f"{result.stdout}\n{result.stderr}"
            returncode = result.returncode
        except subprocess.TimeoutExpired as exc:
            timed_out = True
            stdout = exc.stdout or ""
            stderr = exc.stderr or ""
            if isinstance(stdout, bytes):
                stdout = stdout.decode(errors="replace")
            if isinstance(stderr, bytes):
                stderr = stderr.decode(errors="replace")
            output = f"{stdout}\n{stderr}"
            returncode = 0 if output else 124
        except (OSError, subprocess.SubprocessError) as exc:
            self._status(f"Model filter could not query Gazebo robot pose: {exc}")
            return None

        if returncode != 0:
            self._status(
                "Model filter Gazebo robot pose query failed: "
                f"{output.strip() or returncode}"
            )
            return None

        match = re.search(
            r"Pose\s*\[ XYZ \(m\) \] \[ RPY \(rad\) \]:\s*"
            r"\[\s*([^\]]+?)\s*\]\s*"
            r"\[\s*([^\]]+?)\s*\]",
            output,
            re.MULTILINE,
        )
        if not match:
            detail = " before timeout" if timed_out else ""
            self._status(f"Model filter could not parse Gazebo robot pose{detail}")
            return None

        try:
            xyz = [float(value) for value in match.group(1).split()]
            rpy = [float(value) for value in match.group(2).split()]
        except ValueError:
            self._status("Model filter received invalid Gazebo robot pose values")
            return None

        if len(xyz) != 3 or len(rpy) != 3:
            self._status("Model filter received incomplete Gazebo robot pose values")
            return None
        return xyz[0], xyz[1], xyz[2], rpy[2]

    def _match_model_filter(
        self,
        frame_id: str,
        point: tuple[float, float, float],
        stamp,
    ) -> tuple[TomatoModel, float] | None:
        if not self.model_filter_enabled:
            return None
        if not self._tomato_models:
            return None

        matches: list[tuple[float, TomatoModel, str]] = []

        # Strategy 1: compare in model_filter_frame, usually map.
        if frame_id != self.model_filter_frame:
            transformed = self._try_transform_point(
                frame_id, self.model_filter_frame, stamp, point
            )
            if transformed is not None:
                model_point = transformed
            else:
                model_point = None
        else:
            model_point = point

        if model_point is not None:
            nearest = min(
                self._tomato_models,
                key=lambda m: math.dist(model_point, (m.x, m.y, m.z)),
            )
            distance = math.dist(model_point, (nearest.x, nearest.y, nearest.z))
            matches.append((distance, nearest, self.model_filter_frame))
            if distance <= self.model_filter_max_distance_m:
                return nearest, distance

        # Strategy 2: compare in robot frame using live Gazebo robot pose. This
        # survives map/localization drift because both points are reduced to the
        # robot frame before matching.
        robot_match = self._match_model_filter_in_robot_frame(frame_id, point, stamp)
        if robot_match is not None:
            robot_model, robot_distance = robot_match
            matches.append((robot_distance, robot_model, self.target_frame or "base_link"))

        if not matches:
            self._filtered_reject_count += 1
            self._status(
                f"Model filter rejected detection: cannot compare models with {frame_id}"
            )
            return None

        distance, nearest, source = min(matches, key=lambda item: item[0])
        if distance > self.model_filter_max_distance_m:
            self._filtered_reject_count += 1
            self._status(
                "Model filter rejected detection: "
                f"nearest={nearest.name}, distance={distance:.3f}m, "
                f"limit={self.model_filter_max_distance_m:.3f}m, source={source}"
            )
            return None

        return nearest, distance

    def _match_model_filter_by_projection(
        self,
        detection: PixelDetection,
        color_msg: Image,
        camera_info: CameraInfo,
        camera_frame: str,
        stamp,
    ) -> dict | None:
        if not self.model_filter_enabled:
            return None
        if not self._tomato_models:
            return None
        if self.model_filter_match_strategy in ("3d", "spatial_3d", "depth_3d"):
            return None

        fx = float(camera_info.k[0])
        fy = float(camera_info.k[4])
        cx = float(camera_info.k[2])
        cy = float(camera_info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            return None

        bbox = detection.bbox if detection.bbox and len(detection.bbox) >= 4 else None
        try:
            if bbox:
                bbox_x, bbox_y, bbox_w, bbox_h = (float(value) for value in bbox[:4])
                bbox_center_u = bbox_x + 0.5 * bbox_w
                bbox_center_v = bbox_y + 0.5 * bbox_h
                bbox_margin = max(4.0, max(bbox_w, bbox_h) * self.model_filter_projection_bbox_margin)
            else:
                bbox_x = bbox_y = bbox_w = bbox_h = 0.0
                bbox_center_u = float(detection.u)
                bbox_center_v = float(detection.v)
                bbox_margin = self.model_filter_projection_max_center_px
        except (TypeError, ValueError):
            return None

        best: tuple[float, float, TomatoModel, tuple[float, float, float], float, float] | None = None
        for model in self._tomato_models:
            if (
                self.model_filter_projection_class_gate
                and not self._model_name_matches_detection_class(detection.class_name, model.name)
            ):
                continue

            camera_point = self._gazebo_to_frame(model, camera_frame, stamp)
            if camera_point is None:
                continue
            target_point = None
            if self.target_frame:
                target_point = self._gazebo_to_frame(model, self.target_frame, stamp)
            x, y, z = camera_point
            if not all(math.isfinite(value) for value in (x, y, z)) or z <= 0.02:
                continue

            u = fx * x / z + cx
            v = fy * y / z + cy
            if not all(math.isfinite(value) for value in (u, v)):
                continue
            if (
                u < -self.model_filter_projection_max_center_px
                or v < -self.model_filter_projection_max_center_px
                or u > float(color_msg.width) + self.model_filter_projection_max_center_px
                or v > float(color_msg.height) + self.model_filter_projection_max_center_px
            ):
                continue

            if bbox:
                min_u = bbox_x - bbox_margin
                max_u = bbox_x + bbox_w + bbox_margin
                min_v = bbox_y - bbox_margin
                max_v = bbox_y + bbox_h + bbox_margin
                outside_u = max(min_u - u, 0.0, u - max_u)
                outside_v = max(min_v - v, 0.0, v - max_v)
                outside_distance = math.hypot(outside_u, outside_v)
                if outside_distance > self.model_filter_projection_max_center_px:
                    continue
            else:
                outside_distance = 0.0

            center_error = math.hypot(u - bbox_center_u, v - bbox_center_v)
            if not bbox and center_error > self.model_filter_projection_max_center_px:
                continue

            # First prefer projected centers inside the expanded bbox, then
            # choose the one closest to the YOLO box center. This handles
            # leaf occlusion where the bbox center is a visible-surface center,
            # not the sphere center.
            score = outside_distance * 1000.0 + center_error
            if best is None or score < best[0]:
                best = (score, center_error, model, camera_point, target_point, u, v)

        if best is None:
            return None

        _score, center_error, model, camera_point, target_point, u, v = best
        return {
            "model": model,
            "camera_point": camera_point,
            "target_point": target_point,
            "u": u,
            "v": v,
            "error_px": center_error,
        }

    @staticmethod
    def _model_name_matches_detection_class(class_name: str, model_name: str) -> bool:
        label = (class_name or "").strip().lower()
        if not label:
            return True

        model = (model_name or "").strip().lower()
        if any(token in label for token in ("fully", "ripe", "ripened", "red")):
            return "ripe" in model and not any(
                token in model for token in ("unripe", "rotten", "diseased", "disease")
            )
        if any(token in label for token in ("green", "unripe")):
            return "unripe" in model or "green" in model
        if "disease" in label or "diseased" in label:
            return "disease" in model or "diseased" in model
        if "rot" in label:
            return "rotten" in model or "rot" in model
        return True

    def _match_model_filter_in_robot_frame(
        self,
        frame_id: str,
        point: tuple[float, float, float],
        stamp,
    ) -> tuple[TomatoModel, float] | None:
        robot_frame = self.target_frame or "base_link"
        if frame_id == robot_frame:
            robot_point = point
        else:
            robot_point = self._try_transform_point(frame_id, robot_frame, stamp, point)
            if robot_point is None:
                return None

        best_model = None
        best_distance = float("inf")
        for model in self._tomato_models:
            model_point = self._gazebo_model_to_robot_frame(model)
            if model_point is None:
                continue
            distance = math.dist(robot_point, model_point)
            if distance < best_distance:
                best_distance = distance
                best_model = model

        if best_model is None:
            return None
        return best_model, best_distance

    def _sample_depth(self, depth: np.ndarray, u: float, v: float) -> float | None:
        height, width = depth.shape
        center_u = int(round(u))
        center_v = int(round(v))
        if center_u < 0 or center_u >= width or center_v < 0 or center_v >= height:
            return None

        radius = max(0, int(self.depth_window_radius_px))
        u0 = max(0, center_u - radius)
        u1 = min(width, center_u + radius + 1)
        v0 = max(0, center_v - radius)
        v1 = min(height, center_v + radius + 1)
        window = depth[v0:v1, u0:u1]
        valid = window[
            np.isfinite(window)
            & (window > 0.0)
            & (window >= self.min_depth_m)
            & (window <= self.max_depth_m)
        ]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def _sample_depth_in_bbox(
        self,
        depth: np.ndarray,
        bbox: list[float],
        color_width: int,
        color_height: int,
        inner_fraction: float = 1.0,
    ) -> float | None:
        if len(bbox) < 4:
            return None
        height, width = depth.shape
        try:
            x, y, w, h = (float(value) for value in bbox[:4])
        except (TypeError, ValueError):
            return None
        if w <= 0.0 or h <= 0.0 or color_width <= 0 or color_height <= 0:
            return None

        scale_x = width / float(color_width)
        scale_y = height / float(color_height)
        fraction = min(max(float(inner_fraction), 0.05), 1.0)
        margin_x = 0.5 * (1.0 - fraction) * w
        margin_y = 0.5 * (1.0 - fraction) * h
        sample_x0 = x + margin_x
        sample_y0 = y + margin_y
        sample_x1 = x + w - margin_x
        sample_y1 = y + h - margin_y

        u0 = max(0, int(math.floor(sample_x0 * scale_x)))
        v0 = max(0, int(math.floor(sample_y0 * scale_y)))
        u1 = min(width, int(math.ceil(sample_x1 * scale_x)))
        v1 = min(height, int(math.ceil(sample_y1 * scale_y)))
        if u1 <= u0 or v1 <= v0:
            return None

        window = depth[v0:v1, u0:u1]
        valid = window[
            np.isfinite(window)
            & (window >= self.min_depth_m)
            & (window <= self.max_depth_m)
        ]
        if valid.size < max(1, int(self.depth_bbox_min_valid_samples)):
            return None

        percentile = min(max(float(self.depth_bbox_percentile), 0.0), 100.0)
        return float(np.percentile(valid, percentile))

    def _select_output_point(
        self, source_frame: str, stamp, camera_point: tuple[float, float, float]
    ) -> tuple[str, tuple[float, float, float]]:
        if self.prefer_global_frame:
            for frame in (self.global_frame, self.fallback_global_frame):
                if not frame:
                    continue
                point = self._try_transform_point(source_frame, frame, stamp, camera_point)
                if point is not None:
                    return frame, point

        if self.target_frame:
            point = self._try_transform_point(source_frame, self.target_frame, stamp, camera_point)
            if point is not None:
                return self.target_frame, point

        return source_frame, camera_point

    def _try_transform_point(
        self,
        source_frame: str,
        target_frame: str,
        stamp,
        point: tuple[float, float, float],
    ) -> tuple[float, float, float] | None:
        if source_frame == target_frame:
            return point
        if not source_frame or not target_frame:
            return None
        try:
            transform = self._lookup_transform(target_frame, source_frame, stamp)
        except TransformException as exc:
            fallback = self._try_amcl_map_transform(
                source_frame,
                target_frame,
                stamp,
                point,
            )
            if fallback is not None:
                return fallback
            self._status(f"TF unavailable {source_frame}->{target_frame}: {exc}")
            return None
        return self._apply_transform(point, transform)

    def _try_amcl_map_transform(
        self,
        source_frame: str,
        target_frame: str,
        stamp,
        point: tuple[float, float, float],
    ) -> tuple[float, float, float] | None:
        if source_frame != self.model_filter_frame:
            return None
        if self._latest_amcl_pose is None:
            return None
        if not self.target_frame:
            return None

        base_point = self._map_point_to_base_from_amcl(point)
        if base_point is None:
            return None
        if target_frame == self.target_frame:
            return base_point

        try:
            transform = self._lookup_transform_latest(target_frame, self.target_frame)
        except TransformException:
            try:
                transform = self._lookup_transform(target_frame, self.target_frame, stamp)
            except TransformException:
                return None
        return self._apply_transform(base_point, transform)

    def _lookup_transform_latest(self, target_frame: str, source_frame: str):
        timeout = Duration(seconds=max(0.0, self.tf_timeout_sec))
        return self._tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            Time(seconds=0, nanoseconds=0, clock_type=self.get_clock().clock_type),
            timeout=timeout,
        )

    def _map_point_to_base_from_amcl(
        self,
        point: tuple[float, float, float],
    ) -> tuple[float, float, float] | None:
        msg = self._latest_amcl_pose
        if msg is None:
            return None
        pose = msg.pose.pose
        q = pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        dx = point[0] - pose.position.x
        dy = point[1] - pose.position.y
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            cos_yaw * dx + sin_yaw * dy,
            -sin_yaw * dx + cos_yaw * dy,
            point[2] - pose.position.z,
        )

    def _lookup_transform(self, target_frame: str, source_frame: str, stamp):
        timeout = Duration(seconds=max(0.0, self.tf_timeout_sec))
        latest_time = Time(seconds=0, nanoseconds=0, clock_type=self.get_clock().clock_type)
        if self.use_latest_tf:
            try:
                return self._tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    latest_time,
                    timeout=timeout,
                )
            except TransformException:
                if stamp.sec == 0 and stamp.nanosec == 0:
                    raise
                return self._tf_buffer.lookup_transform(
                    target_frame,
                    source_frame,
                    Time.from_msg(stamp, clock_type=self.get_clock().clock_type),
                    timeout=timeout,
                )
        try:
            return self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time.from_msg(stamp, clock_type=self.get_clock().clock_type),
                timeout=timeout,
            )
        except TransformException:
            return self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                latest_time,
                timeout=timeout,
            )

    def _apply_transform(self, point: tuple[float, float, float], transform) -> tuple[float, float, float]:
        q = transform.transform.rotation
        t = transform.transform.translation
        rx, ry, rz = self._rotate_vector(point, (q.x, q.y, q.z, q.w))
        return rx + t.x, ry + t.y, rz + t.z

    @staticmethod
    def _rotate_vector(
        point: tuple[float, float, float], quat: tuple[float, float, float, float]
    ) -> tuple[float, float, float]:
        qx, qy, qz, qw = quat
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm == 0.0:
            return point
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
        px, py, pz = point

        # Optimized quaternion-vector rotation: v' = v + 2w(q x v) + 2q x(q x v)
        uvx = qy * pz - qz * py
        uvy = qz * px - qx * pz
        uvz = qx * py - qy * px
        uuvx = qy * uvz - qz * uvy
        uuvy = qz * uvx - qx * uvz
        uuvz = qx * uvy - qy * uvx
        return (
            px + 2.0 * (qw * uvx + uuvx),
            py + 2.0 * (qw * uvy + uuvy),
            pz + 2.0 * (qw * uvz + uuvz),
        )

    def _merge_or_add(self, record: dict) -> None:
        model_name = record.get("model_name")
        if model_name:
            for existing in self._records:
                if existing.get("model_name") != model_name:
                    continue
                tomato_id = existing["tomato_id"]
                updated_count = int(existing.get("updated_count", 1)) + 1
                existing.update(record)
                existing["tomato_id"] = tomato_id
                existing["updated_count"] = updated_count
                return

        for existing in self._records:
            if existing["current_waypoint"] != record["current_waypoint"]:
                continue
            if existing["frame_id"] != record["frame_id"]:
                continue
            distance = math.dist(
                (existing["x"], existing["y"], existing["z"]),
                (record["x"], record["y"], record["z"]),
            )
            if distance <= self.merge_distance_m:
                tomato_id = existing["tomato_id"]
                updated_count = int(existing.get("updated_count", 1)) + 1
                existing.update(record)
                existing["tomato_id"] = tomato_id
                existing["updated_count"] = updated_count
                return

        record["tomato_id"] = f"T{self._next_id:03d}"
        self._next_id += 1
        self._records.append(record)

    def _publish_list(self) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "count": len(self._records),
                "expected_model_count": len(self._tomato_models),
                "model_filter_enabled": self.model_filter_enabled,
                "filtered_reject_count": self._filtered_reject_count,
                "records": self._records,
            },
            ensure_ascii=True,
        )
        self._list_pub.publish(msg)

    def _maybe_print_panel(self) -> None:
        if self.panel_rate_hz <= 0.0:
            return
        now = time.monotonic()
        min_period = 1.0 / self.panel_rate_hz
        if now - self._last_panel_time < min_period:
            return
        self._last_panel_time = now
        self.get_logger().info("\n" + self._format_panel())

    def _format_panel(self) -> str:
        headers = [
            "ID",
            "Fidan/Waypoint",
            "Mode",
            "Pixel U",
            "Pixel V",
            "Depth",
            "X",
            "Y",
            "Z",
            "Frame",
            "Time",
        ]
        rows = []
        for record in self._records[-max(1, self.max_panel_rows):]:
            rows.append(
                [
                    record["tomato_id"],
                    record["current_waypoint"],
                    record["detection_mode"],
                    f'{record["pixel_u"]:.1f}',
                    f'{record["pixel_v"]:.1f}',
                    f'{record["depth_m"]:.3f}',
                    f'{record["x"]:.3f}',
                    f'{record["y"]:.3f}',
                    f'{record["z"]:.3f}',
                    record["frame_id"],
                    record["timestamp"],
                ]
            )
        if not rows:
            rows = [["-", self._current_waypoint, self.detection_mode, "-", "-", "-", "-", "-", "-", "-", "-"]]

        widths = [
            min(
                max(len(str(row[index])) for row in ([headers] + rows)),
                22 if index in (1, 2) else 16,
            )
            for index in range(len(headers))
        ]

        def clip(value: str, width: int) -> str:
            value = str(value)
            return value if len(value) <= width else value[: max(0, width - 1)] + "~"

        def line(values: list[str]) -> str:
            return "| " + " | ".join(
                clip(value, width).ljust(width)
                for value, width in zip(values, widths)
            ) + " |"

        separator = "|-" + "-|-".join("-" * width for width in widths) + "-|"
        return "\n".join([line(headers), separator] + [line(row) for row in rows])

    def _image_to_depth(self, msg: Image) -> np.ndarray:
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
        return arr.reshape((msg.height, row_elems))[:, : msg.width].astype(np.float32) * scale

    def _parse_waypoint_status(self, text: str) -> str:
        fields = {}
        for part in text.split(";"):
            if "=" not in part:
                continue
            key, value = part.split("=", 1)
            fields[key.strip()] = value.strip()
        if fields.get("current"):
            return fields["current"]
        stripped = text.strip()
        return stripped or self.current_waypoint_default

    def _detections_from_future_json(self, parsed) -> Iterable[PixelDetection]:
        detections = parsed.get("detections", parsed) if isinstance(parsed, dict) else parsed
        if not isinstance(detections, list):
            return []

        pixel_detections = []
        for item in detections:
            if not isinstance(item, dict):
                continue
            center = None
            bbox_values = None
            if "center_u" in item and "center_v" in item:
                center = (float(item["center_u"]), float(item["center_v"]))
            elif "pixel_u" in item and "pixel_v" in item:
                center = (float(item["pixel_u"]), float(item["pixel_v"]))
            else:
                bbox = item.get("bbox")
                if isinstance(bbox, list) and len(bbox) >= 4:
                    x, y, w, h = (float(value) for value in bbox[:4])
                    center = (x + 0.5 * w, y + 0.5 * h)
                    bbox_values = [x, y, w, h]

            if center is None:
                continue
            if bbox_values is None:
                bbox = item.get("bbox")
                if isinstance(bbox, list) and len(bbox) >= 4:
                    bbox_values = [float(value) for value in bbox[:4]]

            class_id = None
            if "class_id" in item:
                try:
                    class_id = int(item["class_id"])
                except (TypeError, ValueError):
                    class_id = None
            confidence = None
            if "confidence" in item:
                try:
                    confidence = float(item["confidence"])
                except (TypeError, ValueError):
                    confidence = None
            pixel_detections.append(
                PixelDetection(
                    u=center[0],
                    v=center[1],
                    mode="yolo_topic_future",
                    class_id=class_id,
                    class_name=str(item.get("class", item.get("class_name", "")) or ""),
                    confidence=confidence,
                    detection_id=str(item.get("detection_id", "") or ""),
                    bbox=bbox_values,
                )
            )
        return pixel_detections

    def _status(self, text: str) -> None:
        now = time.monotonic()
        if now - self._last_status_time >= 2.0:
            self._last_status_time = now
            self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = TomatoDepthMapper()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
