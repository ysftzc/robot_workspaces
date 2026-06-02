"""Tkinter panel for /tomato_map/list JSON records."""

from __future__ import annotations

import json
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


PARAMETER_DESCRIPTOR = ParameterDescriptor(dynamic_typing=True)


class TomatoMapPanelNode(Node):
    def __init__(self):
        super().__init__("tomato_map_panel")
        self.declare_parameter("topic", "/tomato_map/list", PARAMETER_DESCRIPTOR)
        self.declare_parameter("remember_records", True, PARAMETER_DESCRIPTOR)
        self.topic = str(self.get_parameter("topic").value)
        self.remember_records = bool(self.get_parameter("remember_records").value)
        self.records = []
        self._record_memory = {}
        self.visible_record_keys = set()
        self.visible_record_count = 0
        self.expected_model_count = 0
        self.model_filter_enabled = False
        self.filtered_reject_count = 0
        self.message_count = 0
        self.last_error = ""
        self.create_subscription(String, self.topic, self._list_cb, 10)

    def _list_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.last_error = f"JSON parse error: {exc}"
            return

        if not isinstance(payload, dict):
            self.last_error = "JSON payload is not an object"
            return

        incoming_records = self._extract_records(payload)
        if self.remember_records:
            self._merge_records(incoming_records)
        else:
            self.records = incoming_records
            self.visible_record_count = len(incoming_records)
            self.visible_record_keys = set()

        self.expected_model_count = int(
            payload.get("expected_model_count", payload.get("total_tomatoes", 0)) or 0
        )
        self.model_filter_enabled = bool(payload.get("model_filter_enabled", False))
        self.filtered_reject_count = int(payload.get("filtered_reject_count", 0) or 0)
        self.message_count += 1
        self.last_error = ""

    def _extract_records(self, payload: dict) -> list[dict]:
        records = payload.get("records", []) if isinstance(payload, dict) else []
        if not isinstance(records, list):
            self.last_error = "JSON payload has no records list"
            return []

        extracted = [record for record in records if isinstance(record, dict)]
        if extracted:
            return extracted

        targets = payload.get("targets", [])
        return self._records_from_gazebo_targets(payload, targets)

    def _merge_records(self, incoming_records: list[dict]) -> None:
        now = time.time()
        visible_keys = set()

        for record in incoming_records:
            key = self._record_key(record)
            if not key:
                continue
            visible_keys.add(key)
            merged = dict(self._record_memory.get(key, {}))
            merged.update(record)
            merged["_panel_key"] = key
            merged["_last_seen_wall_time"] = now
            merged["seen_state"] = "visible"
            self._record_memory[key] = merged

        for key, record in self._record_memory.items():
            if key not in visible_keys:
                record["seen_state"] = "stored"

        self.visible_record_keys = visible_keys
        self.visible_record_count = len(visible_keys)
        self.records = [
            record
            for _, record in sorted(
                self._record_memory.items(), key=lambda item: self._record_sort_key(item[1])
            )
        ]

    @staticmethod
    def _record_key(record: dict) -> str:
        model_name = str(record.get("model_name", "") or "").strip()
        if model_name:
            return f"model:{model_name}"

        tomato_id = str(record.get("tomato_id", "") or "").strip()
        if tomato_id:
            return f"id:{tomato_id}"

        waypoint = str(record.get("current_waypoint", "") or "").strip()
        x = record.get("x", "")
        y = record.get("y", "")
        z = record.get("z", "")
        if waypoint != "" and x != "" and y != "" and z != "":
            return f"pose:{waypoint}:{x}:{y}:{z}"

        return ""

    @staticmethod
    def _record_sort_key(record: dict) -> tuple[str, str, str]:
        return (
            str(record.get("current_waypoint", "") or ""),
            str(record.get("model_name", "") or ""),
            str(record.get("tomato_id", "") or record.get("_panel_key", "") or ""),
        )

    @staticmethod
    def _records_from_gazebo_targets(payload: dict, targets) -> list[dict]:
        if not isinstance(targets, list):
            return []

        timestamp = payload.get("timestamp", "")
        records = []
        for index, target in enumerate(targets, start=1):
            if not isinstance(target, dict):
                continue
            base = target.get("base", {})
            if not isinstance(base, dict):
                base = {}
            row = target.get("row", "")
            plant_index = target.get("plant_index", "")
            waypoint = f"{row}_{plant_index}" if row != "" and plant_index is not None else ""
            records.append(
                {
                    "tomato_id": f"G{index:03d}",
                    "current_waypoint": waypoint,
                    "detection_mode": "gazebo_model",
                    "pixel_u": "",
                    "pixel_v": "",
                    "depth_m": "",
                    "x": base.get("x", ""),
                    "y": base.get("y", ""),
                    "z": base.get("z", ""),
                    "frame_id": base.get("frame_id", ""),
                    "model_name": target.get("name", ""),
                    "tomato_class": target.get("class", ""),
                    "confidence": target.get("confidence", ""),
                    "model_distance_m": target.get("distance_m", ""),
                    "timestamp": timestamp,
                }
            )
        return records


class TomatoMapPanel:
    COLUMNS = [
        ("tomato_id", "ID", 70),
        ("current_waypoint", "Fidan/Waypoint", 180),
        ("detection_mode", "Mode", 150),
        ("tomato_class", "Class", 80),
        ("confidence", "Conf", 70),
        ("seen_state", "State", 80),
        ("pixel_u", "Pixel U", 80),
        ("pixel_v", "Pixel V", 80),
        ("depth_m", "Depth", 80),
        ("x", "X", 80),
        ("y", "Y", 80),
        ("z", "Z", 80),
        ("frame_id", "Frame", 130),
        ("model_name", "Model", 190),
        ("model_distance_m", "Model Dist", 90),
        ("timestamp", "Time", 160),
    ]

    def __init__(self, node: TomatoMapPanelNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title("Tomato Map Panel")
        self.root.geometry("1180x520")
        self.root.minsize(900, 360)
        self._closed = False
        self._last_message_count = -1

        self.status_var = tk.StringVar(value=f"Listening: {self.node.topic}")
        status = ttk.Label(self.root, textvariable=self.status_var, anchor="w")
        status.pack(fill=tk.X, padx=10, pady=(10, 4))

        frame = ttk.Frame(self.root)
        frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))

        column_ids = [item[0] for item in self.COLUMNS]
        self.tree = ttk.Treeview(frame, columns=column_ids, show="headings")
        for key, title, width in self.COLUMNS:
            self.tree.heading(key, text=title)
            self.tree.column(key, width=width, minwidth=50, anchor=tk.CENTER)
        self.tree.tag_configure("stored", foreground="#777777")

        y_scroll = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=self.tree.yview)
        x_scroll = ttk.Scrollbar(frame, orient=tk.HORIZONTAL, command=self.tree.xview)
        self.tree.configure(yscrollcommand=y_scroll.set, xscrollcommand=x_scroll.set)

        self.tree.grid(row=0, column=0, sticky="nsew")
        y_scroll.grid(row=0, column=1, sticky="ns")
        x_scroll.grid(row=1, column=0, sticky="ew")
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)

        self.root.protocol("WM_DELETE_WINDOW", self._close)

    def run(self) -> None:
        self.root.after(50, self._spin_ros)
        self.root.mainloop()

    def _spin_ros(self) -> None:
        if self._closed:
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except ExternalShutdownException:
            self._close()
            return

        if self.node.message_count != self._last_message_count:
            self._last_message_count = self.node.message_count
            self._refresh_table()

        self.root.after(50, self._spin_ros)

    def _refresh_table(self) -> None:
        for item in self.tree.get_children():
            self.tree.delete(item)

        for record in self.node.records:
            state = str(record.get("seen_state", "") or "")
            self.tree.insert(
                "",
                tk.END,
                values=[self._format_value(record.get(key, "")) for key, _, _ in self.COLUMNS],
                tags=("stored",) if state == "stored" else (),
            )

        if self.node.last_error:
            self.status_var.set(self.node.last_error)
            return

        expected = (
            f"/{self.node.expected_model_count}"
            if self.node.expected_model_count > 0
            else ""
        )
        filter_text = (
            f"    Filtered rejects: {self.node.filtered_reject_count}"
            if self.node.model_filter_enabled
            else ""
        )
        self.status_var.set(
            f"Listening: {self.node.topic}    Tomatoes: {len(self.node.records)}{expected}    "
            f"Visible: {self.node.visible_record_count}    "
            f"Messages: {self.node.message_count}{filter_text}"
        )

    @staticmethod
    def _format_value(value) -> str:
        if isinstance(value, float):
            return f"{value:.3f}"
        if value is None:
            return ""
        return str(value)

    def _close(self) -> None:
        self._closed = True
        try:
            self.root.destroy()
        except tk.TclError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = TomatoMapPanelNode()
    try:
        panel = TomatoMapPanel(node)
        panel.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
