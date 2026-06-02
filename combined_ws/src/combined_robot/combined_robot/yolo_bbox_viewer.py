"""Live Tkinter viewer for YOLO tomato bounding boxes."""

from __future__ import annotations

import json
import time
import tkinter as tk
from tkinter import ttk

import numpy as np
import rclpy
from PIL import Image as PILImage
from PIL import ImageDraw, ImageFont, ImageTk
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


PARAMETER_DESCRIPTOR = ParameterDescriptor(dynamic_typing=True)


CLASS_COLORS = {
    "fully_ripened": "#ff2d2d",
    "green": "#3bd14a",
    "rotten": "#ff9f1c",
}
DEFAULT_COLOR = "#00b7ff"


class YoloBboxViewerNode(Node):
    def __init__(self):
        super().__init__("yolo_bbox_viewer")
        self.image_topic = self._string_param("image_topic", "/camera/color/image_raw")
        self.detection_topic = self._string_param(
            "detection_topic", "/yolo/tomato_detections_json"
        )
        self.max_width = self._int_param("max_width", 960)
        self.max_height = self._int_param("max_height", 720)
        self.min_confidence = self._float_param("min_confidence", 0.0)
        self.stale_detection_sec = self._float_param("stale_detection_sec", 1.0)

        self.latest_image: np.ndarray | None = None
        self.latest_image_stamp = ""
        self.latest_image_frame = ""
        self.latest_detections: list[dict] = []
        self.latest_detection_wall_time = 0.0
        self.latest_detection_stamp = ""
        self.image_count = 0
        self.detection_message_count = 0
        self.last_error = ""
        self._version = 0

        self.create_subscription(Image, self.image_topic, self._image_cb, 10)
        self.create_subscription(String, self.detection_topic, self._detection_cb, 10)
        self.get_logger().info(
            "YOLO bbox viewer listening: "
            f"image={self.image_topic}, detections={self.detection_topic}"
        )

    def _string_param(self, name: str, default: str) -> str:
        self.declare_parameter(name, default, PARAMETER_DESCRIPTOR)
        value = self.get_parameter(name).value
        return default if value is None else str(value)

    def _int_param(self, name: str, default: int) -> int:
        self.declare_parameter(name, default, PARAMETER_DESCRIPTOR)
        try:
            return int(float(self.get_parameter(name).value))
        except (TypeError, ValueError):
            self.get_logger().warn(f"Invalid {name}; using {default}")
            return int(default)

    def _float_param(self, name: str, default: float) -> float:
        self.declare_parameter(name, default, PARAMETER_DESCRIPTOR)
        try:
            return float(self.get_parameter(name).value)
        except (TypeError, ValueError):
            self.get_logger().warn(f"Invalid {name}; using {default}")
            return float(default)

    def _image_cb(self, msg: Image) -> None:
        try:
            self.latest_image = self._image_to_rgb(msg)
        except ValueError as exc:
            self.last_error = str(exc)
            return
        stamp = msg.header.stamp
        self.latest_image_stamp = f"{stamp.sec}.{stamp.nanosec:09d}"
        self.latest_image_frame = msg.header.frame_id
        self.image_count += 1
        self.last_error = ""
        self._version += 1

    def _detection_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.last_error = f"Detection JSON parse error: {exc}"
            return
        if not isinstance(payload, dict):
            self.last_error = "Detection JSON is not an object"
            return

        detections = payload.get("detections", [])
        if not isinstance(detections, list):
            self.last_error = "Detection JSON has no detections list"
            return

        filtered = []
        for detection in detections:
            if not isinstance(detection, dict):
                continue
            try:
                confidence = float(detection.get("confidence", 0.0) or 0.0)
            except (TypeError, ValueError):
                confidence = 0.0
            if confidence < self.min_confidence:
                continue
            bbox = detection.get("bbox")
            if not isinstance(bbox, list) or len(bbox) < 4:
                continue
            filtered.append(detection)

        self.latest_detections = filtered
        self.latest_detection_stamp = str(payload.get("timestamp", "") or "")
        self.latest_detection_wall_time = time.monotonic()
        self.detection_message_count += 1
        self.last_error = ""
        self._version += 1

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
            raise ValueError(f"unsupported image encoding '{msg.encoding}'")

        expected_step = msg.width * channels
        if msg.step < expected_step:
            raise ValueError(
                f"image step {msg.step} is smaller than expected {expected_step}"
            )

        arr = np.frombuffer(msg.data, dtype=np.uint8)
        image = arr.reshape((msg.height, msg.step))[:, :expected_step]
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


class YoloBboxViewer:
    def __init__(self, node: YoloBboxViewerNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title("YOLO Tomato BBox Viewer")
        self.root.geometry("1000x760")
        self.root.minsize(640, 420)
        self._closed = False
        self._last_draw_version = -1
        self._photo = None

        self.status_var = tk.StringVar(value="Waiting for camera and YOLO detections")
        status = ttk.Label(self.root, textvariable=self.status_var, anchor="w")
        status.pack(fill=tk.X, padx=10, pady=(10, 6))

        self.image_label = ttk.Label(self.root, anchor="center")
        self.image_label.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))

        self.root.protocol("WM_DELETE_WINDOW", self._close)
        try:
            self._font = ImageFont.truetype("DejaVuSans.ttf", 14)
            self._small_font = ImageFont.truetype("DejaVuSans.ttf", 12)
        except OSError:
            self._font = ImageFont.load_default()
            self._small_font = ImageFont.load_default()

    def run(self) -> None:
        self.root.after(20, self._spin_ros)
        self.root.after(80, self._redraw)
        self.root.mainloop()

    def _spin_ros(self) -> None:
        if self._closed:
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except ExternalShutdownException:
            self._close()
            return
        self.root.after(20, self._spin_ros)

    def _redraw(self) -> None:
        if self._closed:
            return
        if self.node._version != self._last_draw_version:
            self._last_draw_version = self.node._version
            self._draw_frame()
        else:
            self._update_status()
        self.root.after(80, self._redraw)

    def _draw_frame(self) -> None:
        image = self.node.latest_image
        if image is None:
            self._update_status()
            return

        pil = PILImage.fromarray(image)
        source_width, source_height = pil.size
        scale = min(
            float(self.node.max_width) / max(source_width, 1),
            float(self.node.max_height) / max(source_height, 1),
            1.0,
        )
        display_width = max(1, int(source_width * scale))
        display_height = max(1, int(source_height * scale))
        if scale != 1.0:
            pil = pil.resize((display_width, display_height), PILImage.Resampling.BILINEAR)

        draw = ImageDraw.Draw(pil)
        detections = self._fresh_detections()
        for detection in detections:
            self._draw_detection(draw, detection, scale)

        self._draw_overlay(draw, pil.size, len(detections))
        self._photo = ImageTk.PhotoImage(pil)
        self.image_label.configure(image=self._photo)
        self._update_status()

    def _fresh_detections(self) -> list[dict]:
        if self.node.stale_detection_sec <= 0.0:
            return self.node.latest_detections
        age = time.monotonic() - self.node.latest_detection_wall_time
        if age > self.node.stale_detection_sec:
            return []
        return self.node.latest_detections

    def _draw_detection(self, draw: ImageDraw.ImageDraw, detection: dict, scale: float) -> None:
        bbox = detection.get("bbox", [])
        try:
            x, y, width, height = (float(value) * scale for value in bbox[:4])
        except (TypeError, ValueError):
            return

        class_name = str(detection.get("class", detection.get("class_name", "")) or "tomato")
        confidence = detection.get("confidence", "")
        try:
            confidence_text = f"{float(confidence):.2f}"
        except (TypeError, ValueError):
            confidence_text = ""

        color = CLASS_COLORS.get(class_name, DEFAULT_COLOR)
        x2 = x + width
        y2 = y + height
        for offset in range(3):
            draw.rectangle(
                [x - offset, y - offset, x2 + offset, y2 + offset],
                outline=color,
            )

        label = class_name if not confidence_text else f"{class_name} {confidence_text}"
        text_bbox = draw.textbbox((0, 0), label, font=self._font)
        text_width = text_bbox[2] - text_bbox[0]
        text_height = text_bbox[3] - text_bbox[1]
        label_y = max(0, y - text_height - 8)
        draw.rectangle(
            [x, label_y, x + text_width + 8, label_y + text_height + 6],
            fill=color,
        )
        draw.text((x + 4, label_y + 3), label, fill="#000000", font=self._font)

    def _draw_overlay(
        self, draw: ImageDraw.ImageDraw, size: tuple[int, int], detection_count: int
    ) -> None:
        width, _height = size
        text = (
            f"Detections: {detection_count}  "
            f"Images: {self.node.image_count}  "
            f"YOLO msgs: {self.node.detection_message_count}"
        )
        text_bbox = draw.textbbox((0, 0), text, font=self._small_font)
        text_width = min(width - 12, text_bbox[2] - text_bbox[0] + 12)
        draw.rectangle([6, 6, 6 + text_width, 28], fill="#111111")
        draw.text((12, 10), text, fill="#ffffff", font=self._small_font)

    def _update_status(self) -> None:
        if self.node.last_error:
            self.status_var.set(self.node.last_error)
            return
        age = time.monotonic() - self.node.latest_detection_wall_time
        stale = ""
        if self.node.latest_detection_wall_time > 0.0 and age > self.node.stale_detection_sec:
            stale = f"    Detections stale: {age:.1f}s"
        self.status_var.set(
            f"Image: {self.node.image_topic}    YOLO: {self.node.detection_topic}    "
            f"Frame: {self.node.latest_image_frame or '-'}    "
            f"Detections: {len(self.node.latest_detections)}{stale}"
        )

    def _close(self) -> None:
        self._closed = True
        try:
            self.root.destroy()
        except tk.TclError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = YoloBboxViewerNode()
    try:
        viewer = YoloBboxViewer(node)
        viewer.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
