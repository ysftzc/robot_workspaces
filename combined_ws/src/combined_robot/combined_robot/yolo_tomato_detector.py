"""Ultralytics YOLO detector bridge for tomato RGB images.

Publishes generic JSON detections so the depth mapper stays independent from
any YOLO-specific ROS message type.
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


PARAMETER_DESCRIPTOR = ParameterDescriptor(dynamic_typing=True)


class YoloTomatoDetector(Node):
    def __init__(self):
        super().__init__("yolo_tomato_detector")

        self.yolo_site_packages = self._string_param(
            "yolo_site_packages", "/home/yusuf/yolo_env/lib/python3.12/site-packages"
        )
        self.model_path = self._string_param(
            "model_path",
            "/home/yusuf/robot_workspaces/combined_ws/yolo_models/tomato/best.pt",
        )
        self.image_topic = self._string_param("image_topic", "/camera/color/image_raw")
        self.output_topic = self._string_param(
            "output_topic", "/yolo/tomato_detections_json"
        )
        self.confidence_threshold = self._float_param("confidence_threshold", 0.35)
        self.iou_threshold = self._float_param("iou_threshold", 0.45)
        self.publish_rate_hz = self._float_param("publish_rate_hz", 4.0)
        self.imgsz = self._int_param("imgsz", 640)
        self.max_det = self._int_param("max_det", 80)
        self.device_param = self._string_param("device", "cuda:0")
        self.use_half = self._bool_param("half", True)
        self.class_filter = self._class_filter_param(
            self._string_param("class_filter", "all")
        )

        self._last_publish_time = 0.0
        self._last_status_time = 0.0
        self._model = None
        self._torch = None
        self._pil_image_cls = None
        self._device = "cpu"

        self._load_model()

        self._pub = self.create_publisher(String, self.output_topic, 10)
        self.create_subscription(Image, self.image_topic, self._image_cb, 10)
        self.get_logger().info(
            "YOLO tomato detector started: "
            f"image={self.image_topic}, output={self.output_topic}, "
            f"model={self.model_path}, device={self._device}, "
            f"classes={self._model.names}"
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

    @staticmethod
    def _class_filter_param(value: str) -> set[str]:
        value = value.strip()
        if not value or value.lower() == "all":
            return set()
        return {item.strip() for item in value.split(",") if item.strip()}

    def _load_model(self) -> None:
        yolo_site = Path(self.yolo_site_packages).expanduser()
        if yolo_site.is_dir() and str(yolo_site) not in sys.path:
            sys.path.insert(0, str(yolo_site))

        try:
            import torch
            from PIL import Image as PILImage
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "Cannot import YOLO dependencies. Check yolo_site_packages="
                f"{self.yolo_site_packages}"
            ) from exc

        model_path = Path(self.model_path).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(f"YOLO model file not found: {model_path}")

        cuda_available = bool(torch.cuda.is_available())
        requested = self.device_param.strip().lower()
        if requested.startswith("cuda") and not cuda_available:
            self.get_logger().warn("CUDA requested but unavailable; using CPU")
            self._device = "cpu"
        else:
            self._device = requested or ("cuda:0" if cuda_available else "cpu")

        self._torch = torch
        self._pil_image_cls = PILImage
        self._model = YOLO(str(model_path))

    def _image_cb(self, msg: Image) -> None:
        now = time.monotonic()
        min_period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.0
        if now - self._last_publish_time < min_period:
            return
        self._last_publish_time = now

        try:
            rgb = self._image_to_rgb(msg)
        except ValueError as exc:
            self._status(f"Image rejected: {exc}")
            return

        try:
            detections = self._predict(rgb)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"YOLO predict failed: {exc}")
            return

        stamp = msg.header.stamp
        payload = {
            "timestamp": f"{stamp.sec}.{stamp.nanosec:09d}",
            "frame_id": msg.header.frame_id,
            "image_width": msg.width,
            "image_height": msg.height,
            "model_path": self.model_path,
            "device": self._device,
            "confidence_threshold": self.confidence_threshold,
            "detections": detections,
        }
        out = String()
        out.data = json.dumps(payload, ensure_ascii=True)
        self._pub.publish(out)
        self._status(f"YOLO detections: {len(detections)}")

    def _predict(self, rgb: np.ndarray) -> list[dict]:
        source = self._pil_image_cls.fromarray(rgb)
        use_half = bool(self.use_half and self._device.startswith("cuda"))
        results = self._model.predict(
            source=source,
            imgsz=self.imgsz,
            conf=self.confidence_threshold,
            iou=self.iou_threshold,
            device=self._device,
            half=use_half,
            max_det=self.max_det,
            verbose=False,
        )
        if not results:
            return []

        result = results[0]
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return []

        names = result.names or self._model.names
        xyxy_values = boxes.xyxy.detach().cpu().numpy()
        xywh_values = boxes.xywh.detach().cpu().numpy()
        conf_values = boxes.conf.detach().cpu().numpy()
        cls_values = boxes.cls.detach().cpu().numpy().astype(int)

        detections = []
        for index, (xyxy, xywh, conf, class_id) in enumerate(
            zip(xyxy_values, xywh_values, conf_values, cls_values),
            start=1,
        ):
            class_name = str(names.get(int(class_id), int(class_id)))
            if self.class_filter and class_name not in self.class_filter:
                continue
            x1, y1, x2, y2 = (float(value) for value in xyxy)
            center_u, center_v, width, height = (float(value) for value in xywh)
            if not all(math.isfinite(value) for value in (x1, y1, x2, y2, center_u, center_v)):
                continue
            detections.append(
                {
                    "detection_id": f"Y{index:03d}",
                    "bbox": [
                        round(center_u - 0.5 * width, 2),
                        round(center_v - 0.5 * height, 2),
                        round(width, 2),
                        round(height, 2),
                    ],
                    "xyxy": [
                        round(x1, 2),
                        round(y1, 2),
                        round(x2, 2),
                        round(y2, 2),
                    ],
                    "center_u": round(center_u, 2),
                    "center_v": round(center_v, 2),
                    "class_id": int(class_id),
                    "class": class_name,
                    "confidence": round(float(conf), 4),
                }
            )
        return detections

    @staticmethod
    def _image_to_rgb(msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        channels_by_encoding = {
            "rgb8": 3,
            "bgr8": 3,
            "rgba8": 4,
            "bgra8": 4,
            "mono8": 1,
        }
        channels = channels_by_encoding.get(encoding)
        if channels is None:
            raise ValueError(f"unsupported RGB image encoding '{msg.encoding}'")

        expected_step = msg.width * channels
        if msg.step < expected_step:
            raise ValueError(
                f"image step {msg.step} is smaller than expected {expected_step}"
            )

        arr = np.frombuffer(msg.data, dtype=np.uint8)
        row_elems = msg.step
        image = arr.reshape((msg.height, row_elems))[:, :expected_step]
        image = image.reshape((msg.height, msg.width, channels))

        if encoding == "rgb8":
            return np.ascontiguousarray(image)
        if encoding == "bgr8":
            return np.ascontiguousarray(image[:, :, ::-1])
        if encoding == "rgba8":
            return np.ascontiguousarray(image[:, :, :3])
        if encoding == "bgra8":
            return np.ascontiguousarray(image[:, :, 2::-1])
        return np.ascontiguousarray(np.repeat(image, 3, axis=2))

    def _status(self, text: str) -> None:
        now = time.monotonic()
        if now - self._last_status_time >= 2.0:
            self._last_status_time = now
            self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = YoloTomatoDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
