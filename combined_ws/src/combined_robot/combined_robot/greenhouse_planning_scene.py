#!/usr/bin/env python3

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive


class GreenhousePlanningScene(Node):
    def __init__(self):
        super().__init__('greenhouse_planning_scene')

        self.declare_parameter('world_file', '')
        self.declare_parameter('planning_scene_topic', '/planning_scene')
        self.declare_parameter('planning_frame', 'map')
        self.declare_parameter('frame_mode', 'map')
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('gazebo_map_x_from_gazebo_y_offset', -4.93)
        self.declare_parameter('gazebo_map_y_from_gazebo_x_origin', 35.83)
        self.declare_parameter('robot_spawn_x', 0.0)
        self.declare_parameter('robot_spawn_y', 0.0)
        self.declare_parameter('robot_spawn_z', 0.0)
        self.declare_parameter('robot_spawn_roll', 0.0)
        self.declare_parameter('robot_spawn_pitch', 0.0)
        self.declare_parameter('robot_spawn_yaw', 0.0)
        self.declare_parameter('base_to_planning_frame_z', 0.1715)
        self.declare_parameter('include_tomatoes', False)
        self.declare_parameter('include_soilbeds', True)
        self.declare_parameter('include_pots', True)
        self.declare_parameter('include_plants', False)
        self.declare_parameter('include_structures', True)
        self.declare_parameter('include_lamps', True)
        self.declare_parameter('include_baskets', True)
        self.declare_parameter('tomato_collision_radius_m', 0.085)
        self.declare_parameter('plant_collision_height_m', 1.85)
        self.declare_parameter('plant_collision_radius_m', 0.12)

        self._world_file = self._string_param('world_file')
        self._planning_scene_topic = self._string_param('planning_scene_topic')
        self._planning_frame = self._string_param('planning_frame')
        self._frame_mode = self._string_param('frame_mode').strip().lower() or 'map'
        self._publish_rate_hz = max(0.1, float(self.get_parameter('publish_rate_hz').value))
        self._gazebo_map_x_offset = float(
            self.get_parameter('gazebo_map_x_from_gazebo_y_offset').value
        )
        self._gazebo_map_y_origin = float(
            self.get_parameter('gazebo_map_y_from_gazebo_x_origin').value
        )
        self._robot_spawn = (
            float(self.get_parameter('robot_spawn_x').value),
            float(self.get_parameter('robot_spawn_y').value),
            float(self.get_parameter('robot_spawn_z').value),
            float(self.get_parameter('robot_spawn_roll').value),
            float(self.get_parameter('robot_spawn_pitch').value),
            float(self.get_parameter('robot_spawn_yaw').value),
        )
        self._base_to_planning_frame_z = float(
            self.get_parameter('base_to_planning_frame_z').value
        )
        self._include_tomatoes = self._as_bool(self.get_parameter('include_tomatoes').value)
        self._include_soilbeds = self._as_bool(self.get_parameter('include_soilbeds').value)
        self._include_pots = self._as_bool(self.get_parameter('include_pots').value)
        self._include_plants = self._as_bool(self.get_parameter('include_plants').value)
        self._include_structures = self._as_bool(self.get_parameter('include_structures').value)
        self._include_lamps = self._as_bool(self.get_parameter('include_lamps').value)
        self._include_baskets = self._as_bool(self.get_parameter('include_baskets').value)
        self._tomato_collision_radius_m = max(
            0.01, float(self.get_parameter('tomato_collision_radius_m').value)
        )
        self._plant_collision_height_m = float(
            self.get_parameter('plant_collision_height_m').value
        )
        self._plant_collision_radius_m = float(
            self.get_parameter('plant_collision_radius_m').value
        )

        self._world_path = self._resolve_world_file(self._world_file)
        self._objects = self._build_objects(self._world_path)
        self._publish_count = 0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(PlanningScene, self._planning_scene_topic, qos)
        self._timer = self.create_timer(1.0 / self._publish_rate_hz, self._publish_scene)

        self.get_logger().info(
            f'Loaded {len(self._objects)} collision object(s) from {self._world_path}'
        )

    def _string_param(self, name: str) -> str:
        value = self.get_parameter(name).value
        return '' if value is None else str(value)

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'on')
        return bool(value)

    def _resolve_world_file(self, world_file: str) -> Path:
        if not world_file:
            raise RuntimeError('world_file parameter is empty')

        candidate = Path(world_file).expanduser()
        if candidate.exists():
            return candidate

        share = Path(get_package_share_directory('combined_robot'))
        for path in (
            share / 'worlds' / world_file,
            Path('/home/yusuf/robot_workspaces/combined_ws/src/combined_robot/worlds') / world_file,
        ):
            if path.exists():
                return path

        raise RuntimeError(f'World file does not exist: {world_file}')

    def _build_objects(self, world_path: Path) -> list[CollisionObject]:
        root = ET.parse(world_path).getroot()
        world = root.find('world')
        if world is None:
            raise RuntimeError(f'No <world> element found in {world_path}')

        objects: list[CollisionObject] = []
        for model in world.findall('model'):
            name = model.get('name', '')
            if not name:
                continue

            primitive = self._primitive_for_model(name)
            if primitive is None:
                continue

            pose = self._parse_pose(model.findtext('pose', default='0 0 0 0 0 0'))
            planning_pose = self._gazebo_pose_to_planning_pose(pose)
            collision_object = CollisionObject()
            collision_object.header.frame_id = self._planning_frame
            collision_object.header.stamp = self.get_clock().now().to_msg()
            collision_object.id = name
            collision_object.operation = CollisionObject.ADD
            collision_object.primitives.append(primitive)
            collision_object.primitive_poses.append(self._primitive_pose(planning_pose, primitive))
            objects.append(collision_object)

        return objects

    def _primitive_for_model(self, model_name: str) -> SolidPrimitive | None:
        name = model_name.lower()
        primitive = SolidPrimitive()

        if name.startswith('tomato') and not self._include_tomatoes:
            return None
        if name.startswith('tomato'):
            primitive.type = SolidPrimitive.SPHERE
            primitive.dimensions = [self._tomato_collision_radius_m]
            return primitive
        if name in ('physics_ground_plane', 'ground_plane'):
            return None
        if name.startswith('sera_greenhouse'):
            return None

        if name.startswith('soilbed'):
            if not self._include_soilbeds:
                return None
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [0.90, 0.90, 0.45]
            return primitive
        if name.startswith('flowerpot'):
            if not self._include_pots:
                return None
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [0.28, 0.18]
            return primitive
        if name.startswith('plant') or name.startswith('modul_plant'):
            if not self._include_plants:
                return None
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [
                self._plant_collision_height_m,
                self._plant_collision_radius_m,
            ]
            return primitive
        if name.startswith('structure'):
            if not self._include_structures:
                return None
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [0.40, 0.40, 2.20]
            return primitive
        if name.startswith('lamp'):
            if not self._include_lamps:
                return None
            primitive.type = SolidPrimitive.CYLINDER
            primitive.dimensions = [1.80, 0.10]
            return primitive
        if name in ('good_pick_basket', 'bad_pick_basket'):
            if not self._include_baskets:
                return None
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [0.60, 0.45, 0.50]
            return primitive
        if name == 'pickup_pedestal':
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [0.70, 0.70, 0.35]
            return primitive

        return None

    @staticmethod
    def _parse_pose(pose_text: str) -> tuple[float, float, float, float, float, float]:
        values = [float(item) for item in pose_text.split()]
        while len(values) < 6:
            values.append(0.0)
        return tuple(values[:6])

    def _gazebo_pose_to_planning_pose(
        self, pose: tuple[float, float, float, float, float, float]
    ) -> tuple[float, float, float, float, float, float]:
        if self._frame_mode == 'robot_spawn':
            return self._gazebo_pose_to_robot_spawn_pose(pose)
        if self._frame_mode != 'map':
            self.get_logger().warn(
                f'Unknown frame_mode "{self._frame_mode}", falling back to map conversion.'
            )
        return self._gazebo_pose_to_map_pose(pose)

    def _gazebo_pose_to_map_pose(
        self, pose: tuple[float, float, float, float, float, float]
    ) -> tuple[float, float, float, float, float, float]:
        x, y, z, roll, pitch, yaw = pose
        map_x = y + self._gazebo_map_x_offset
        map_y = self._gazebo_map_y_origin - x
        return map_x, map_y, z, roll, pitch, yaw

    def _gazebo_pose_to_robot_spawn_pose(
        self, pose: tuple[float, float, float, float, float, float]
    ) -> tuple[float, float, float, float, float, float]:
        x, y, z, roll, pitch, yaw = pose
        sx, sy, sz, sroll, spitch, syaw = self._robot_spawn
        frame_origin = self._rotate_rpy(
            (0.0, 0.0, self._base_to_planning_frame_z),
            sroll,
            spitch,
            syaw,
        )
        dx = x - (sx + frame_origin[0])
        dy = y - (sy + frame_origin[1])
        dz = z - (sz + frame_origin[2])
        local_x, local_y, local_z = self._inverse_rotate_rpy(
            (dx, dy, dz),
            sroll,
            spitch,
            syaw,
        )
        return (
            local_x,
            local_y,
            local_z,
            roll - sroll,
            pitch - spitch,
            yaw - syaw,
        )

    @staticmethod
    def _rotate_rpy(
        vector: tuple[float, float, float], roll: float, pitch: float, yaw: float
    ) -> tuple[float, float, float]:
        x, y, z = vector
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        x1 = x
        y1 = cr * y - sr * z
        z1 = sr * y + cr * z

        x2 = cp * x1 + sp * z1
        y2 = y1
        z2 = -sp * x1 + cp * z1

        return (
            cy * x2 - sy * y2,
            sy * x2 + cy * y2,
            z2,
        )

    @classmethod
    def _inverse_rotate_rpy(
        cls, vector: tuple[float, float, float], roll: float, pitch: float, yaw: float
    ) -> tuple[float, float, float]:
        # Inverse of Rz(yaw) * Ry(pitch) * Rx(roll).
        x, y, z = vector
        cy = math.cos(-yaw)
        sy = math.sin(-yaw)
        x1 = cy * x - sy * y
        y1 = sy * x + cy * y
        z1 = z

        cp = math.cos(-pitch)
        sp = math.sin(-pitch)
        x2 = cp * x1 + sp * z1
        y2 = y1
        z2 = -sp * x1 + cp * z1

        cr = math.cos(-roll)
        sr = math.sin(-roll)
        return (
            x2,
            cr * y2 - sr * z2,
            sr * y2 + cr * z2,
        )

    @staticmethod
    def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
        half_roll = roll * 0.5
        half_pitch = pitch * 0.5
        half_yaw = yaw * 0.5

        cr = math.cos(half_roll)
        sr = math.sin(half_roll)
        cp = math.cos(half_pitch)
        sp = math.sin(half_pitch)
        cy = math.cos(half_yaw)
        sy = math.sin(half_yaw)

        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        w = cr * cp * cy + sr * sp * sy
        return x, y, z, w

    def _primitive_pose(
        self, pose: tuple[float, float, float, float, float, float], primitive: SolidPrimitive
    ) -> Pose:
        x, y, z, roll, pitch, yaw = pose
        center = Pose()
        center.position.x = x
        center.position.y = y
        if primitive.type == SolidPrimitive.CYLINDER:
            center.position.z = z + primitive.dimensions[0] * 0.5
        elif primitive.type == SolidPrimitive.SPHERE:
            center.position.z = z
        else:
            center.position.z = z + primitive.dimensions[2] * 0.5

        qx, qy, qz, qw = self._quat_from_rpy(roll, pitch, yaw)
        center.orientation.x = qx
        center.orientation.y = qy
        center.orientation.z = qz
        center.orientation.w = qw
        return center

    def _publish_scene(self):
        if not self._objects:
            return

        scene = PlanningScene()
        scene.is_diff = True
        scene.name = 'greenhouse_collision_scene'
        scene.world.collision_objects = list(self._objects)
        self._publisher.publish(scene)
        self._publish_count += 1

        if self._publish_count == 1:
            self.get_logger().info(
                f'Published planning scene with {len(self._objects)} collision object(s) '
                f'to {self._planning_scene_topic}'
            )


def main():
    rclpy.init()
    node = GreenhousePlanningScene()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
