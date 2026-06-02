"""Bridge mapped tomato records into the PoseStamped topic used by pick."""

from __future__ import annotations

import json
import math
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from std_msgs.msg import Float32, String


PARAMETER_DESCRIPTOR = ParameterDescriptor(dynamic_typing=True)


class TomatoMapPickTarget(Node):
    def __init__(self) -> None:
        super().__init__("tomato_map_pick_target")

        self.input_topic = self._string_param("input_topic", "/tomato_map/list")
        self.output_topic = self._string_param("output_topic", "/mission_pick/tomato_center")
        self.output_radius_topic = self._string_param(
            "output_radius_topic", "/mission_pick/tomato_radius"
        )
        self.selection_topic = self._string_param(
            "selection_topic", "/tomato_harvest/target_selection"
        )
        self.target_model_name = self._string_param("target_model_name", "")
        self.allowed_classes = self._name_set_param("allowed_classes", "fully_ripened,ripe")
        self.reject_classes = self._name_set_param("reject_classes", "green,unripe,rotten,disease")
        self.min_confidence = self._float_param("min_confidence", 0.35)
        self.min_updated_count = self._int_param("min_updated_count", 1)
        self.min_z_m = self._float_param("min_z_m", 0.20)
        self.max_z_m = self._float_param("max_z_m", 1.40)
        self.publish_rate_hz = self._float_param("publish_rate_hz", 5.0)
        self.selection_strategy = self._string_param("selection_strategy", "nearest_depth")

        self._records: list[dict[str, Any]] = []
        self._last_selected_key = ""

        self._pose_pub = self.create_publisher(PoseStamped, self.output_topic, 10)
        self._radius_pub = self.create_publisher(Float32, self.output_radius_topic, 10)
        self._selection_pub = self.create_publisher(String, self.selection_topic, 10)
        self.create_subscription(String, self.input_topic, self._list_cb, 10)

        period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.2
        self.create_timer(period, self._publish_selected)
        self.get_logger().info(
            "Tomato map pick target started: "
            f"input={self.input_topic}, output={self.output_topic}, "
            f"target_model={self.target_model_name or '<auto>'}"
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

    def _name_set_param(self, name: str, default: str) -> set[str]:
        value = self._string_param(name, default).strip().lower()
        if not value or value == "all":
            return set()
        return {item.strip() for item in value.split(",") if item.strip()}

    @staticmethod
    def _record_label(record: dict[str, Any]) -> str:
        parts = [
            str(record.get("tomato_class", "")),
            str(record.get("class", "")),
            str(record.get("class_name", "")),
            str(record.get("model_name", "")),
        ]
        return " ".join(parts).lower()

    def _list_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Invalid tomato map JSON: {exc}")
            return
        records = payload.get("records", [])
        if not isinstance(records, list):
            return
        self._records = [record for record in records if isinstance(record, dict)]

    def _is_candidate(self, record: dict[str, Any]) -> bool:
        model_name = str(record.get("model_name", ""))
        if self.target_model_name and model_name != self.target_model_name:
            return False

        label = self._record_label(record)
        if not self.target_model_name:
            if self.reject_classes and any(token in label for token in self.reject_classes):
                return False
            if self.allowed_classes and not any(token in label for token in self.allowed_classes):
                return False

        try:
            confidence = float(record.get("confidence", 1.0))
        except (TypeError, ValueError):
            confidence = 1.0
        if confidence < self.min_confidence:
            return False

        try:
            updated_count = int(float(record.get("updated_count", 1)))
        except (TypeError, ValueError):
            updated_count = 1
        if updated_count < self.min_updated_count:
            return False

        try:
            z = float(record["z"])
        except (KeyError, TypeError, ValueError):
            return False
        return self.min_z_m <= z <= self.max_z_m

    def _candidate_key(self, record: dict[str, Any]) -> tuple[float, ...]:
        if self.selection_strategy == "highest_confidence":
            try:
                confidence = float(record.get("confidence", 0.0))
            except (TypeError, ValueError):
                confidence = 0.0
            return (-confidence,)
        if self.selection_strategy == "nearest_base":
            try:
                return (
                    math.sqrt(
                        float(record["x"]) ** 2
                        + float(record["y"]) ** 2
                        + float(record["z"]) ** 2
                    ),
                )
            except (KeyError, TypeError, ValueError):
                return (float("inf"),)
        try:
            return (float(record.get("depth_m", float("inf"))),)
        except (TypeError, ValueError):
            return (float("inf"),)

    def _select_record(self) -> dict[str, Any] | None:
        candidates = [record for record in self._records if self._is_candidate(record)]
        if not candidates:
            return None
        return sorted(candidates, key=self._candidate_key)[0]

    def _publish_selected(self) -> None:
        record = self._select_record()
        if record is None:
            return

        try:
            x = float(record["x"])
            y = float(record["y"])
            z = float(record["z"])
        except (KeyError, TypeError, ValueError):
            return

        frame_id = str(record.get("frame_id", "") or "fr3_link0")
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        self._pose_pub.publish(pose)

        radius = record.get("radius_m", record.get("estimated_radius_m"))
        if radius is not None:
            try:
                self._radius_pub.publish(Float32(data=float(radius)))
            except (TypeError, ValueError):
                pass

        model_name = str(record.get("model_name", ""))
        key = f"{model_name}:{frame_id}:{x:.3f}:{y:.3f}:{z:.3f}"
        if key != self._last_selected_key:
            self._last_selected_key = key
            payload = {
                "status": "selected",
                "tomato_model": model_name,
                "model_name": model_name,
                "frame_id": frame_id,
                "x": x,
                "y": y,
                "z": z,
                "source": "tomato_map_pick_target",
                "record": record,
            }
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=True)
            self._selection_pub.publish(msg)
            self.get_logger().info(
                "Selected tomato pick target: "
                f"model={model_name or '<none>'}, frame={frame_id}, "
                f"pos=({x:.3f}, {y:.3f}, {z:.3f})"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TomatoMapPickTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
