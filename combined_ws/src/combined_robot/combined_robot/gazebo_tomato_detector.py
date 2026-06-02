"""Gazebo ground-truth tomato detector.

This is a temporary detector adapter for the greenhouse workflow. It reads
tomato model poses from the SDF world, combines them with the live Gazebo robot
pose, and publishes the selected target in the same shape expected by the
existing pick pipeline:

  /tomato_center  geometry_msgs/PoseStamped
  /tomato_radius  std_msgs/Float32

YOLO can later replace this node without changing the pick/place pipeline.
"""

from __future__ import annotations

import json
import math
import re
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree as ET

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


@dataclass(frozen=True)
class TomatoModel:
    name: str
    tomato_class: str
    row: str
    plant_index: int | None
    fruit_index: int | None
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


@dataclass(frozen=True)
class RobotPose:
    x: float
    y: float
    z: float
    yaw: float
    source: str


@dataclass(frozen=True)
class Candidate:
    model: TomatoModel
    base_x: float
    base_y: float
    base_z: float
    distance_m: float
    score: float


class GazeboTomatoDetector(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_tomato_detector")

        self.world_file = self.declare_parameter(
            "world_file", "tomato_farm_sera.sdf"
        ).value
        self.world_name = self.declare_parameter("world_name", "tomato_farm").value
        self.robot_model = self.declare_parameter(
            "robot_model", "combined_robot"
        ).value
        self.base_frame = self.declare_parameter("base_frame", "fr3_link0").value
        self.tf_base_frame = str(
            self.declare_parameter("tf_base_frame", self.base_frame).value
        )
        self.map_frame = str(self.declare_parameter("map_frame", "map").value)
        self.pose_source = str(
            self.declare_parameter("pose_source", "tf").value
        ).strip().lower()

        self.selected_model = str(
            self.declare_parameter("selected_model", "").value
        ).strip()
        self.class_filter = str(
            self.declare_parameter("class_filter", "ripe").value
        ).strip().lower()
        if self.class_filter in ("all", "any", "none", "*"):
            self.class_filter = ""
        self.row_filter = str(self.declare_parameter("row_filter", "").value).strip()
        self.plant_index_filter = int(
            self.declare_parameter("plant_index_filter", -1).value
        )

        self.output_pose_topic = self.declare_parameter(
            "output_pose_topic", "/tomato_center"
        ).value
        self.output_radius_topic = self.declare_parameter(
            "output_radius_topic", "/tomato_radius"
        ).value
        self.output_selected_topic = self.declare_parameter(
            "output_selected_topic", "/gazebo_tomato_detector/selected"
        ).value
        self.output_list_topic = self.declare_parameter(
            "output_list_topic", "/gazebo_tomato_detector/list"
        ).value
        self.picked_topic = self.declare_parameter(
            "picked_topic", "/tomato_harvest/picked"
        ).value
        self.clear_picked_topic = self.declare_parameter(
            "clear_picked_topic", "/tomato_harvest/clear_picked"
        ).value
        self.initial_picked_models = str(
            self.declare_parameter("initial_picked_models", "").value
        )

        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 2.0).value
        )
        self.tomato_radius_m = float(
            self.declare_parameter("tomato_radius_m", 0.045).value
        )
        self.min_forward_m = float(
            self.declare_parameter("min_forward_m", 0.20).value
        )
        self.max_forward_m = float(
            self.declare_parameter("max_forward_m", 1.35).value
        )
        self.max_lateral_m = float(
            self.declare_parameter("max_lateral_m", 1.25).value
        )
        self.min_base_z_m = float(self.declare_parameter("min_base_z_m", 0.25).value)
        self.max_base_z_m = float(self.declare_parameter("max_base_z_m", 1.35).value)
        self.center_preference_weight = float(
            self.declare_parameter("center_preference_weight", 0.15).value
        )

        self.use_live_robot_pose = bool(
            self.declare_parameter("use_live_robot_pose", True).value
        )
        self.robot_x = float(self.declare_parameter("robot_x", 37.62).value)
        self.robot_y = float(self.declare_parameter("robot_y", 8.50).value)
        self.robot_z = float(self.declare_parameter("robot_z", 0.20).value)
        self.robot_yaw = float(self.declare_parameter("robot_yaw", 1.56).value)
        self.robot_base_z_offset = float(
            self.declare_parameter("robot_base_z_offset", 0.1715).value
        )
        self.map_x_from_gazebo_y_offset = float(
            self.declare_parameter("map_x_from_gazebo_y_offset", -4.93).value
        )
        self.map_y_from_gazebo_x_origin = float(
            self.declare_parameter("map_y_from_gazebo_x_origin", 35.83).value
        )
        self.gazebo_pose_timeout_sec = float(
            self.declare_parameter("gazebo_pose_timeout_sec", 2.0).value
        )
        self.live_pose_query_period_sec = float(
            self.declare_parameter("live_pose_query_period_sec", 2.0).value
        )

        self._world_path = self._resolve_world_file(self.world_file)
        self._tomatoes = self._load_tomato_models(self._world_path)
        self._picked_models = self._parse_model_set(self.initial_picked_models)
        self._last_status_time = 0.0
        self._last_live_pose_query_time = 0.0
        self._cached_live_pose: tuple[float, float, float, float] | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pose_pub = self.create_publisher(PoseStamped, self.output_pose_topic, 10)
        self.radius_pub = self.create_publisher(Float32, self.output_radius_topic, 10)
        self.selected_pub = self.create_publisher(String, self.output_selected_topic, 10)
        self.list_pub = self.create_publisher(String, self.output_list_topic, 10)
        if self.picked_topic:
            self.create_subscription(String, self.picked_topic, self._picked_cb, 10)
        if self.clear_picked_topic:
            self.create_subscription(String, self.clear_picked_topic, self._clear_picked_cb, 10)

        period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.5
        self.timer = self.create_timer(period, self._publish_tick)

        counts: dict[str, int] = {}
        for tomato in self._tomatoes:
            counts[tomato.tomato_class] = counts.get(tomato.tomato_class, 0) + 1
        self.get_logger().info(
            "Gazebo tomato detector started: "
            f"world={self._world_path}, tomatoes={len(self._tomatoes)}, "
            f"counts={counts}, selected_model={self.selected_model or '<auto>'}, "
            f"class_filter={self.class_filter or '<all>'}, base_frame={self.base_frame}, "
            f"pose_source={self.pose_source}, "
            f"picked_topic={self.picked_topic or '<off>'}, "
            f"initial_picked={len(self._picked_models)}, "
            f"publish_rate_hz={self.publish_rate_hz:.2f}, "
            f"live_pose_query_period_sec={self.live_pose_query_period_sec:.2f}"
        )

    @staticmethod
    def _parse_model_set(text: str) -> set[str]:
        models = set()
        for part in str(text or "").replace(";", ",").split(","):
            model = part.strip()
            if model:
                models.add(model)
        return models

    def _picked_cb(self, msg: String) -> None:
        model_name = self._model_name_from_message(msg.data)
        if not model_name:
            self.get_logger().warn(
                f"Picked message did not include a tomato model name: {msg.data!r}"
            )
            return
        if model_name in self._picked_models:
            return
        self._picked_models.add(model_name)
        self.get_logger().info(
            f"Marked {model_name} as picked; remaining candidates will skip it"
        )

    def _clear_picked_cb(self, msg: String) -> None:
        model_name = self._model_name_from_message(msg.data)
        if model_name:
            if model_name in self._picked_models:
                self._picked_models.remove(model_name)
                self.get_logger().info(f"Cleared picked marker for {model_name}")
            return
        count = len(self._picked_models)
        self._picked_models.clear()
        self.get_logger().info(f"Cleared all picked tomato markers ({count})")

    @staticmethod
    def _model_name_from_message(data: str) -> str:
        value = str(data or "").strip()
        if not value:
            return ""
        if value.startswith("{"):
            try:
                payload = json.loads(value)
            except json.JSONDecodeError:
                return ""
            for key in ("tomato_model", "model", "name"):
                model = str(payload.get(key, "")).strip()
                if model:
                    return model
            return ""
        return value

    def _resolve_world_file(self, world_file: str) -> Path:
        candidate = Path(str(world_file)).expanduser()
        if candidate.exists():
            return candidate

        package_candidates: list[Path] = []
        if get_package_share_directory is not None:
            try:
                package_candidates.append(
                    Path(get_package_share_directory("combined_robot")) / "worlds" / str(world_file)
                )
            except Exception:
                pass
        package_candidates.append(
            Path("/home/yusuf/robot_workspaces/combined_ws/src/combined_robot/worlds")
            / str(world_file)
        )

        for path in package_candidates:
            if path.exists():
                return path
        raise RuntimeError(f"world file does not exist: {world_file}")

    def _load_tomato_models(self, world_path: Path) -> list[TomatoModel]:
        root = ET.parse(world_path).getroot()
        tomatoes: list[TomatoModel] = []
        for model in root.findall(".//model"):
            name = model.get("name") or ""
            if not name.startswith("tomato_"):
                continue
            if name == "tomato_farm":
                continue

            pose_values = [float(value) for value in model.findtext("pose", "0 0 0 0 0 0").split()]
            while len(pose_values) < 6:
                pose_values.append(0.0)
            row, plant_index, fruit_index = self._parse_model_location(name)
            tomatoes.append(
                TomatoModel(
                    name=name,
                    tomato_class=self._class_from_name(name),
                    row=row,
                    plant_index=plant_index,
                    fruit_index=fruit_index,
                    x=pose_values[0],
                    y=pose_values[1],
                    z=pose_values[2],
                    roll=pose_values[3],
                    pitch=pose_values[4],
                    yaw=pose_values[5],
                )
            )
        return tomatoes

    @staticmethod
    def _class_from_name(name: str) -> str:
        if name.startswith("tomato_ripe"):
            return "ripe"
        if name.startswith("tomato_unripe"):
            return "unripe"
        if name.startswith("tomato_rotten") or name.startswith("tomato_diseased"):
            return "bad"
        return "unknown"

    @staticmethod
    def _parse_model_location(name: str) -> tuple[str, int | None, int | None]:
        parts = name.split("_")
        if len(parts) < 5:
            return "", None, None
        row = parts[-3]
        try:
            plant_index = int(parts[-2])
        except ValueError:
            plant_index = None
        try:
            fruit_index = int(parts[-1])
        except ValueError:
            fruit_index = None
        return row, plant_index, fruit_index

    def _publish_tick(self) -> None:
        robot_pose = self._robot_pose()
        candidates = self._candidate_targets(robot_pose)
        selected = candidates[0] if candidates else None

        self._publish_list(robot_pose, candidates, selected)
        if selected is None:
            self._status("No Gazebo tomato target passed filters")
            return

        stamp = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = selected.base_x
        pose.pose.position.y = selected.base_y
        pose.pose.position.z = selected.base_z
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        radius = Float32()
        radius.data = self.tomato_radius_m
        self.radius_pub.publish(radius)

        msg = String()
        msg.data = json.dumps(self._selected_payload(robot_pose, selected), sort_keys=True)
        self.selected_pub.publish(msg)

        self._status(
            f"Selected {selected.model.name}: "
            f"{self.base_frame}=({selected.base_x:.3f}, {selected.base_y:.3f}, "
            f"{selected.base_z:.3f}), dist={selected.distance_m:.3f}m, "
            f"robot_pose={robot_pose.source}"
        )

    def _candidate_targets(self, robot_pose: RobotPose) -> list[Candidate]:
        candidates: list[Candidate] = []
        for tomato in self._tomatoes:
            if tomato.name in self._picked_models:
                continue
            if self.selected_model and tomato.name != self.selected_model:
                continue
            if not self.selected_model and self.class_filter and tomato.tomato_class != self.class_filter:
                continue
            if self.row_filter and tomato.row != self.row_filter:
                continue
            if self.plant_index_filter >= 0 and tomato.plant_index != self.plant_index_filter:
                continue

            base_x, base_y, base_z = self._world_to_base(tomato, robot_pose)
            if not self.selected_model:
                if base_x < self.min_forward_m or base_x > self.max_forward_m:
                    continue
                if abs(base_y) > self.max_lateral_m:
                    continue
                if base_z < self.min_base_z_m or base_z > self.max_base_z_m:
                    continue

            distance = math.sqrt(base_x * base_x + base_y * base_y + base_z * base_z)
            score = distance + self.center_preference_weight * abs(base_y)
            candidates.append(
                Candidate(
                    model=tomato,
                    base_x=base_x,
                    base_y=base_y,
                    base_z=base_z,
                    distance_m=distance,
                    score=score,
                )
            )
        candidates.sort(key=lambda candidate: candidate.score)
        return candidates

    def _world_to_base(self, tomato: TomatoModel, robot_pose: RobotPose) -> tuple[float, float, float]:
        if robot_pose.source == "tf_map":
            tomato_x, tomato_y, tomato_z = self._gazebo_to_map_xyz(
                tomato.x, tomato.y, tomato.z
            )
            dx = tomato_x - robot_pose.x
            dy = tomato_y - robot_pose.y
            cos_yaw = math.cos(robot_pose.yaw)
            sin_yaw = math.sin(robot_pose.yaw)
            return (
                cos_yaw * dx + sin_yaw * dy,
                -sin_yaw * dx + cos_yaw * dy,
                tomato_z - robot_pose.z,
            )

        dx = tomato.x - robot_pose.x
        dy = tomato.y - robot_pose.y
        cos_yaw = math.cos(robot_pose.yaw)
        sin_yaw = math.sin(robot_pose.yaw)
        base_x = cos_yaw * dx + sin_yaw * dy
        base_y = -sin_yaw * dx + cos_yaw * dy
        base_z = tomato.z - robot_pose.z
        return base_x, base_y, base_z

    def _gazebo_to_map_xyz(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        return (
            y + self.map_x_from_gazebo_y_offset,
            self.map_y_from_gazebo_x_origin - x,
            z,
        )

    def _robot_pose(self) -> RobotPose:
        if self.pose_source in ("tf", "tf_map"):
            tf_pose = self._tf_robot_map_pose()
            if tf_pose is not None:
                x, y, z, yaw = tf_pose
                return RobotPose(x=x, y=y, z=z, yaw=yaw, source="tf_map")

        if self.pose_source in ("gazebo", "gazebo_cli") and self.use_live_robot_pose:
            live_pose = self._cached_or_live_gazebo_model_pose(self.robot_model)
            if live_pose is not None:
                x, y, z, yaw = live_pose
                return RobotPose(
                    x=x,
                    y=y,
                    z=z + self.robot_base_z_offset,
                    yaw=yaw,
                    source="gazebo",
                )
        return RobotPose(
            x=self.robot_x,
            y=self.robot_y,
            z=self.robot_z + self.robot_base_z_offset,
            yaw=self.robot_yaw,
            source="fallback_params",
        )

    def _tf_robot_map_pose(self) -> tuple[float, float, float, float] | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.tf_base_frame,
                Time(),
            )
        except TransformException as exc:
            self._status(
                f"Could not query TF {self.map_frame}->{self.tf_base_frame}; "
                f"using fallback params: {exc}"
            )
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            float(translation.x),
            float(translation.y),
            float(translation.z),
            self._yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w),
        )

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _cached_or_live_gazebo_model_pose(
        self, model_name: str
    ) -> tuple[float, float, float, float] | None:
        now = time.monotonic()
        if (
            self._cached_live_pose is not None
            and self.live_pose_query_period_sec > 0.0
            and now - self._last_live_pose_query_time < self.live_pose_query_period_sec
        ):
            return self._cached_live_pose

        live_pose = self._live_gazebo_model_pose(model_name)
        self._last_live_pose_query_time = now
        if live_pose is not None:
            self._cached_live_pose = live_pose
            return live_pose
        return self._cached_live_pose

    def _live_gazebo_model_pose(self, model_name: str) -> tuple[float, float, float, float] | None:
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
                timeout=max(0.5, self.gazebo_pose_timeout_sec),
            )
        except (OSError, subprocess.SubprocessError) as exc:
            self._status(f"Could not query Gazebo robot pose; using fallback params: {exc}")
            return None

        output = f"{result.stdout}\n{result.stderr}"
        if result.returncode != 0:
            self._status(
                "Gazebo robot pose query failed; using fallback params: "
                f"{output.strip() or result.returncode}"
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
            self._status("Could not parse Gazebo robot pose; using fallback params")
            return None

        try:
            xyz = [float(value) for value in match.group(1).split()]
            rpy = [float(value) for value in match.group(2).split()]
        except ValueError:
            self._status("Invalid Gazebo robot pose values; using fallback params")
            return None

        if len(xyz) != 3 or len(rpy) != 3:
            self._status("Incomplete Gazebo robot pose values; using fallback params")
            return None
        return xyz[0], xyz[1], xyz[2], rpy[2]

    def _publish_list(
        self,
        robot_pose: RobotPose,
        candidates: list[Candidate],
        selected: Candidate | None,
    ) -> None:
        payload = {
            "timestamp": time.time(),
            "world_file": str(self._world_path),
            "world_name": self.world_name,
            "robot": {
                "model": self.robot_model,
                "x": robot_pose.x,
                "y": robot_pose.y,
                "z": robot_pose.z,
                "yaw": robot_pose.yaw,
                "source": robot_pose.source,
            },
            "filters": {
                "selected_model": self.selected_model,
                "class_filter": self.class_filter,
                "row_filter": self.row_filter,
                "plant_index_filter": self.plant_index_filter,
            },
            "picked_models": sorted(self._picked_models),
            "picked_count": len(self._picked_models),
            "selected": self._selected_payload(robot_pose, selected) if selected else None,
            "targets": [
                self._candidate_payload(candidate)
                for candidate in candidates[:30]
            ],
            "records": [
                self._candidate_record(index, candidate, time.time())
                for index, candidate in enumerate(candidates[:30], start=1)
            ],
            "candidate_count": len(candidates),
            "total_tomatoes": len(self._tomatoes),
            "expected_model_count": len(self._tomatoes),
            "model_filter_enabled": False,
            "filtered_reject_count": 0,
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.list_pub.publish(msg)

    def _selected_payload(self, robot_pose: RobotPose, selected: Candidate) -> dict:
        payload = self._candidate_payload(selected)
        payload["detach_topic"] = f"/detach/{selected.model.name}"
        payload["tomato_model"] = selected.model.name
        payload["robot_pose_source"] = robot_pose.source
        payload["pose_topic"] = self.output_pose_topic
        payload["radius_topic"] = self.output_radius_topic
        return payload

    def _candidate_payload(self, candidate: Candidate) -> dict:
        tomato = candidate.model
        return {
            "name": tomato.name,
            "class": tomato.tomato_class,
            "row": tomato.row,
            "plant_index": tomato.plant_index,
            "fruit_index": tomato.fruit_index,
            "world": {
                "x": tomato.x,
                "y": tomato.y,
                "z": tomato.z,
                "roll": tomato.roll,
                "pitch": tomato.pitch,
                "yaw": tomato.yaw,
            },
            "base": {
                "frame_id": self.base_frame,
                "x": candidate.base_x,
                "y": candidate.base_y,
                "z": candidate.base_z,
            },
            "distance_m": candidate.distance_m,
            "score": candidate.score,
            "radius_m": self.tomato_radius_m,
        }

    def _candidate_record(self, index: int, candidate: Candidate, timestamp: float) -> dict:
        tomato = candidate.model
        waypoint = (
            f"{tomato.row}_{tomato.plant_index}"
            if tomato.row and tomato.plant_index is not None
            else ""
        )
        return {
            "tomato_id": f"G{index:03d}",
            "current_waypoint": waypoint,
            "detection_mode": "gazebo_model",
            "tomato_class": tomato.tomato_class,
            "pixel_u": "",
            "pixel_v": "",
            "depth_m": "",
            "x": round(candidate.base_x, 4),
            "y": round(candidate.base_y, 4),
            "z": round(candidate.base_z, 4),
            "frame_id": self.base_frame,
            "model_name": tomato.name,
            "model_distance_m": round(candidate.distance_m, 4),
            "timestamp": f"{timestamp:.3f}",
        }

    def _status(self, text: str) -> None:
        now = time.monotonic()
        if now - self._last_status_time >= 2.0:
            self._last_status_time = now
            self.get_logger().info(text)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboTomatoDetector()
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
