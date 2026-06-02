"""Publish mapped tomato detections as RViz markers and MoveIt collision objects.

This node is intentionally separate from the pick-place pipeline. It consumes the
generic /tomato_map/list JSON produced by tomato_depth_mapper and mirrors stable
tomato records into the planning scene. Keep it disabled while validating
pick-place behavior unless collision avoidance is the test target.
"""

from __future__ import annotations

import json
import math
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


PARAMETER_DESCRIPTOR = ParameterDescriptor(dynamic_typing=True)


class TomatoCollisionSceneManager(Node):
    def __init__(self):
        super().__init__("tomato_collision_scene_manager")

        self.input_topic = self._string_param("input_topic", "/tomato_map/list")
        self.picked_topic = self._string_param("picked_topic", "/tomato_harvest/picked")
        self.target_topic = self._string_param(
            "target_topic", "/tomato_harvest/target_selection"
        )
        self.planning_scene_topic = self._string_param("planning_scene_topic", "/planning_scene")
        self.marker_topic = self._string_param(
            "marker_topic", "/tomato_collision_scene/markers"
        )
        self.object_prefix = self._string_param("object_prefix", "tomato_obstacle_")
        self.default_frame = self._string_param("default_frame", "map")
        self.planning_frame = self._string_param("planning_frame", "base_link")
        self.class_filter = self._name_set_param("class_filter", "all")
        self.exclude_classes = self._name_set_param("exclude_classes", "")
        self.exclude_model_names = self._name_set_param("exclude_model_names", "")

        self.publish_planning_scene = self._bool_param("publish_planning_scene", False)
        self.publish_markers = self._bool_param("publish_markers", True)
        self.collision_radius_m = max(0.01, self._float_param("collision_radius_m", 0.032))
        self.marker_radius_m = max(0.01, self._float_param("marker_radius_m", 0.032))
        self.radius_scale = max(0.1, self._float_param("radius_scale", 1.0))
        self.min_radius_m = max(0.001, self._float_param("min_radius_m", 0.015))
        self.max_radius_m = max(self.min_radius_m, self._float_param("max_radius_m", 0.08))
        self.min_confidence = self._float_param("min_confidence", 0.0)
        self.min_updated_count = max(1, self._int_param("min_updated_count", 1))
        self.max_records = max(1, self._int_param("max_records", 200))
        self.publish_period_sec = max(0.1, self._float_param("publish_period_sec", 1.0))
        self.tf_timeout_sec = max(0.0, self._float_param("tf_timeout_sec", 0.20))

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        scene_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._scene_pub = self.create_publisher(
            PlanningScene, self.planning_scene_topic, scene_qos
        )
        self._marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.create_subscription(String, self.input_topic, self._list_cb, 10)
        if self.picked_topic:
            self.create_subscription(String, self.picked_topic, self._picked_cb, 10)
        if self.target_topic:
            self.create_subscription(String, self.target_topic, self._target_cb, 10)

        self._records: dict[str, dict] = {}
        self._active_object_ids: set[str] = set()
        self._active_target_models: set[str] = set()
        self._last_publish_time = 0.0
        self._last_tf_warning_time = 0.0

        self.get_logger().info(
            "Tomato collision scene manager started: "
            f"input={self.input_topic}, planning_scene={self.planning_scene_topic}, "
            f"markers={self.marker_topic}, fallback_radius={self.collision_radius_m:.3f}m, "
            f"radius_scale={self.radius_scale:.2f}, "
            f"radius_limits=[{self.min_radius_m:.3f}, {self.max_radius_m:.3f}]m, "
            f"planning_frame={self.planning_frame}, publish_scene={self.publish_planning_scene}, "
            f"publish_markers={self.publish_markers}, class_filter={sorted(self.class_filter) or 'all'}, "
            f"exclude_classes={sorted(self.exclude_classes) or 'none'}, "
            f"target_topic={self.target_topic}"
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

    def _name_set_param(self, name: str, default: str) -> set[str]:
        raw = self._string_param(name, default).strip()
        if not raw or raw.lower() in ("all", "*"):
            return set()
        return {part.strip().lower() for part in raw.split(",") if part.strip()}

    def _list_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Tomato map JSON rejected: {exc}")
            return

        records = payload.get("records", []) if isinstance(payload, dict) else []
        if not isinstance(records, list):
            return

        for record in records[-self.max_records :]:
            if not isinstance(record, dict) or not self._record_passes_filters(record):
                continue
            key = self._record_key(record)
            self._records[key] = dict(record)

        now = time.monotonic()
        if now - self._last_publish_time >= self.publish_period_sec:
            self._last_publish_time = now
            self._publish_scene()
            self._publish_markers()

    def _picked_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        model_name = str(payload.get("tomato_model", "")).strip()
        if not model_name:
            return

        self._active_target_models.discard(model_name)
        remove_keys = [
            key
            for key, record in self._records.items()
            if str(record.get("model_name", "")) == model_name
        ]
        for key in remove_keys:
            self._records.pop(key, None)

        object_id = self._object_id(model_name)
        self._publish_remove_object(object_id)
        self._active_object_ids.discard(object_id)
        self._publish_markers()

    def _target_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        model_name = str(
            payload.get("model_name") or payload.get("tomato_model") or ""
        ).strip()
        if not model_name:
            return

        status = str(payload.get("status", "selected")).strip().lower()
        if status in ("selected", "active", "target"):
            self._active_target_models = {model_name}
            object_id = self._object_id(model_name)
            self._publish_remove_object(object_id)
            self._active_object_ids.discard(object_id)
            self.get_logger().info(
                f"Temporarily excluding harvest target from tomato obstacles: {model_name}"
            )
        elif status in ("clear", "cleared", "failed", "cancelled", "canceled"):
            self._active_target_models.discard(model_name)

        self._publish_scene()
        self._publish_markers()

    def _record_passes_filters(self, record: dict) -> bool:
        try:
            x = float(record.get("x"))
            y = float(record.get("y"))
            z = float(record.get("z"))
        except (TypeError, ValueError):
            return False
        if not all(math.isfinite(value) for value in (x, y, z)):
            return False

        model_name = str(record.get("model_name", "")).strip().lower()
        if model_name and model_name in self.exclude_model_names:
            return False
        if model_name and model_name in {name.lower() for name in self._active_target_models}:
            return False

        class_name = str(record.get("tomato_class", "")).strip().lower()
        if self.class_filter and class_name not in self.class_filter:
            return False
        model_tokens = {token for token in model_name.replace("-", "_").split("_") if token}
        if class_name and class_name in self.exclude_classes:
            return False
        if self.exclude_classes and model_tokens.intersection(self.exclude_classes):
            return False

        confidence = record.get("confidence")
        if confidence is not None:
            try:
                if float(confidence) < self.min_confidence:
                    return False
            except (TypeError, ValueError):
                return False

        try:
            updated_count = int(record.get("updated_count", 1))
        except (TypeError, ValueError):
            updated_count = 1
        return updated_count >= self.min_updated_count

    def _record_key(self, record: dict) -> str:
        model_name = str(record.get("model_name", "")).strip()
        if model_name:
            return model_name
        tomato_id = str(record.get("tomato_id", "")).strip()
        if tomato_id:
            return tomato_id
        return f"{record.get('frame_id', self.default_frame)}:{record.get('x')}:{record.get('y')}:{record.get('z')}"

    def _object_id(self, key: str) -> str:
        safe = "".join(ch if ch.isalnum() or ch == "_" else "_" for ch in key)
        return f"{self.object_prefix}{safe}"

    def _publish_scene(self) -> None:
        if not self.publish_planning_scene:
            return

        desired_ids = set()
        collision_objects = []
        for key, record in self._records.items():
            if self._record_is_active_target(record):
                continue
            object_id = self._object_id(key)
            collision_object = self._collision_object(object_id, record)
            if collision_object is None:
                continue
            desired_ids.add(object_id)
            collision_objects.append(collision_object)

        for stale_id in sorted(self._active_object_ids - desired_ids):
            collision_objects.append(self._remove_collision_object(stale_id))

        if not collision_objects:
            return

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.extend(collision_objects)
        self._scene_pub.publish(scene)
        self._active_object_ids = desired_ids

    def _publish_remove_object(self, object_id: str) -> None:
        if not self.publish_planning_scene:
            return
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(self._remove_collision_object(object_id))
        self._scene_pub.publish(scene)

    def _collision_object(self, object_id: str, record: dict) -> CollisionObject | None:
        planning_point = self._planning_point(record)
        if planning_point is None:
            return None

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [self._record_radius(record, self.collision_radius_m)]

        pose = Pose()
        pose.position.x = planning_point[0]
        pose.position.y = planning_point[1]
        pose.position.z = planning_point[2]
        pose.orientation.w = 1.0

        collision_object = CollisionObject()
        collision_object.header.frame_id = self.planning_frame or self.default_frame
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = object_id
        collision_object.operation = CollisionObject.ADD
        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        return collision_object

    def _planning_point(self, record: dict) -> tuple[float, float, float] | None:
        source_frame = str(record.get("frame_id") or self.default_frame).strip()
        try:
            point = (float(record["x"]), float(record["y"]), float(record["z"]))
        except (KeyError, TypeError, ValueError):
            return None

        target_frame = (self.planning_frame or source_frame).strip()
        if not target_frame or source_frame == target_frame:
            return point

        try:
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException as exc:
            self._warn_tf_throttled(
                f"Skipping tomato collision object; TF unavailable {source_frame}->{target_frame}: {exc}"
            )
            return None

        return self._apply_transform(point, transform)

    def _apply_transform(self, point: tuple[float, float, float], transform) -> tuple[float, float, float]:
        q = transform.transform.rotation
        t = transform.transform.translation
        x, y, z = point
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm == 0.0:
            return (x + t.x, y + t.y, z + t.z)
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qz * qw)
        r02 = 2.0 * (qx * qz + qy * qw)
        r10 = 2.0 * (qx * qy + qz * qw)
        r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
        r12 = 2.0 * (qy * qz - qx * qw)
        r20 = 2.0 * (qx * qz - qy * qw)
        r21 = 2.0 * (qy * qz + qx * qw)
        r22 = 1.0 - 2.0 * (qx * qx + qy * qy)

        return (
            r00 * x + r01 * y + r02 * z + t.x,
            r10 * x + r11 * y + r12 * z + t.y,
            r20 * x + r21 * y + r22 * z + t.z,
        )

    def _warn_tf_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_tf_warning_time < 2.0:
            return
        self._last_tf_warning_time = now
        self.get_logger().warn(message)

    def _remove_collision_object(self, object_id: str) -> CollisionObject:
        collision_object = CollisionObject()
        collision_object.header.frame_id = self.planning_frame or self.default_frame
        collision_object.header.stamp = self.get_clock().now().to_msg()
        collision_object.id = object_id
        collision_object.operation = CollisionObject.REMOVE
        return collision_object

    def _publish_markers(self) -> None:
        if not self.publish_markers:
            return

        markers = MarkerArray()
        delete_all = Marker()
        delete_all.header.frame_id = self.default_frame
        delete_all.header.stamp = self.get_clock().now().to_msg()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        visible_records = [
            (key, record)
            for key, record in sorted(self._records.items())
            if not self._record_is_active_target(record)
        ]
        for index, (key, record) in enumerate(visible_records, start=1):
            markers.markers.append(self._marker(index, key, record))

        self._marker_pub.publish(markers)

    def _marker(self, marker_id: int, key: str, record: dict) -> Marker:
        marker = Marker()
        marker.header.frame_id = str(record.get("frame_id") or self.default_frame)
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "tomato_collision_scene"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(record["x"])
        marker.pose.position.y = float(record["y"])
        marker.pose.position.z = float(record["z"])
        marker.pose.orientation.w = 1.0
        radius = self._record_radius(record, self.marker_radius_m)
        marker.scale.x = radius * 2.0
        marker.scale.y = radius * 2.0
        marker.scale.z = radius * 2.0

        class_name = str(record.get("tomato_class", "")).lower()
        if "green" in class_name or "unripe" in class_name:
            marker.color.r, marker.color.g, marker.color.b = 0.25, 0.9, 0.25
        elif "rotten" in class_name or "disease" in class_name:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.45, 0.05
        else:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.05, 0.05
        marker.color.a = 0.45
        marker.lifetime.sec = int(max(0.0, self.publish_period_sec * 3.0))
        marker.text = key
        return marker

    def _record_radius(self, record: dict, fallback_radius: float) -> float:
        for field in ("radius_m", "collision_radius_m", "tomato_radius_m", "estimated_radius_m", "radius"):
            value = record.get(field)
            if value is None:
                continue
            try:
                radius = float(value)
            except (TypeError, ValueError):
                continue
            if math.isfinite(radius) and radius > 0.0:
                return min(max(radius * self.radius_scale, self.min_radius_m), self.max_radius_m)
        scaled_fallback = float(fallback_radius) * self.radius_scale
        return min(max(scaled_fallback, self.min_radius_m), self.max_radius_m)

    def _record_is_active_target(self, record: dict) -> bool:
        model_name = str(record.get("model_name", "")).strip()
        if not model_name:
            return False
        return model_name in self._active_target_models


def main(args=None):
    rclpy.init(args=args)
    node = TomatoCollisionSceneManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
