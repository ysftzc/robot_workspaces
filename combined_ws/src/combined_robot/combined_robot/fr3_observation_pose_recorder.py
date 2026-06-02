#!/usr/bin/env python3
"""Record the current FR3 joint state into the observation-pose YAML file."""

from __future__ import annotations

import argparse
from copy import deepcopy
from pathlib import Path
import sys
from typing import Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import yaml


DEFAULT_JOINTS = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
]


class JointStateWaiter(Node):
    def __init__(self, topic: str, joint_names: Sequence[str]) -> None:
        super().__init__("fr3_observation_pose_recorder")
        self._joint_names = list(joint_names)
        self.positions: list[float] | None = None
        self.create_subscription(JointState, topic, self._joint_state_cb, 10)

    def _joint_state_cb(self, msg: JointState) -> None:
        by_name = dict(zip(msg.name, msg.position))
        if not all(name in by_name for name in self._joint_names):
            return
        self.positions = [float(by_name[name]) for name in self._joint_names]


def _default_config_path() -> Path:
    return (
        Path.home()
        / "robot_workspaces"
        / "combined_ws"
        / "src"
        / "combined_robot"
        / "config"
        / "fr3_observation_poses.yaml"
    )


def _parse_args(argv: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Save the current /joint_states values for fr3_joint1..7 into a "
            "combined_robot FR3 observation pose."
        )
    )
    parser.add_argument(
        "--pose",
        required=True,
        help="Pose name to update, for example harvest_b_scan_1.",
    )
    parser.add_argument(
        "--config",
        default=str(_default_config_path()),
        help="Path to fr3_observation_poses.yaml in the source workspace.",
    )
    parser.add_argument(
        "--topic",
        default="/joint_states",
        help="JointState topic to read.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Seconds to wait for a complete FR3 joint state.",
    )
    parser.add_argument(
        "--description",
        default=None,
        help="Optional description to store on the pose.",
    )
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=None,
        help="Optional observation motion duration. Existing value is preserved when omitted.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the values but do not write the YAML file.",
    )
    return parser.parse_args(argv)


def _load_yaml(path: Path) -> dict:
    if not path.exists():
        raise FileNotFoundError(f"config file does not exist: {path}")
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"config file root must be a mapping: {path}")
    return data


def _save_pose(
    path: Path,
    pose_name: str,
    positions: Sequence[float],
    description: str | None,
    duration_sec: float | None,
    dry_run: bool,
) -> None:
    data = _load_yaml(path)
    joints = data.get("joints") or DEFAULT_JOINTS
    if list(joints) != DEFAULT_JOINTS:
        raise ValueError(
            "this recorder expects fr3_joint1..7 in order; found "
            f"{list(joints)!r}"
        )

    poses = data.setdefault("poses", {})
    if not isinstance(poses, dict):
        raise ValueError("poses must be a mapping")

    previous = poses.get(pose_name) or {}
    if not isinstance(previous, dict):
        previous = {}
    updated = deepcopy(previous)
    updated.pop("mirror_from", None)
    updated["positions"] = [round(float(value), 4) for value in positions]

    if description is not None:
        updated["description"] = description
    elif "description" not in updated:
        updated["description"] = f"Recorded from current joint state for {pose_name}."

    if duration_sec is not None:
        updated["duration_sec"] = float(duration_sec)
    else:
        updated.setdefault("duration_sec", 2.0)

    updated.setdefault("gripper_positions", [0.04, 0.04])
    updated.setdefault("gripper_duration_sec", 1.0)
    poses[pose_name] = updated

    degrees = [value * 180.0 / 3.141592653589793 for value in positions]
    print(f"{pose_name} radians: {[round(v, 4) for v in positions]}")
    print(f"{pose_name} degrees: {[round(v, 2) for v in degrees]}")
    if dry_run:
        print("dry-run: YAML not written")
        return

    with path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=False, width=120)
    print(f"updated: {path}")


def main(argv: Sequence[str] | None = None) -> int:
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    config_path = Path(args.config).expanduser().resolve()

    rclpy.init()
    node = JointStateWaiter(args.topic, DEFAULT_JOINTS)
    try:
        end_time = node.get_clock().now().nanoseconds + int(args.timeout * 1_000_000_000)
        while rclpy.ok() and node.positions is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.get_clock().now().nanoseconds > end_time:
                raise TimeoutError(
                    f"no complete FR3 joint state received on {args.topic} within {args.timeout:.1f}s"
                )

        if node.positions is None:
            raise TimeoutError(f"no complete FR3 joint state received on {args.topic}")

        _save_pose(
            config_path,
            args.pose,
            node.positions,
            args.description,
            args.duration_sec,
            args.dry_run,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
