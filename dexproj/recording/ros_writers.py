"""ROS-oriented episode writers for DexProj."""

from __future__ import annotations

import csv
import json
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage, Image
    from std_msgs.msg import Float32MultiArray
except ImportError:  # pragma: no cover
    rclpy = None
    SingleThreadedExecutor = None
    Node = object  # type: ignore[assignment]
    Image = object  # type: ignore[assignment]
    CompressedImage = object  # type: ignore[assignment]
    Float32MultiArray = object  # type: ignore[assignment]


@dataclass
class RosTopicRecorder:
    name: str
    topic: str
    path: Path
    schema: str
    _thread: threading.Thread | None = field(default=None, init=False)
    _stop_event: threading.Event = field(default_factory=threading.Event, init=False)
    _node: Node | None = field(default=None, init=False)
    _executor: SingleThreadedExecutor | None = field(default=None, init=False)
    _last_msg: dict | None = field(default=None, init=False)

    def start(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if rclpy is None or Node is object:
            self._write_placeholder()
            return
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node(f"dexproj_{self.name}_recorder")
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._subscribe()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._executor is not None and self._node is not None:
            self._executor.remove_node(self._node)
        if self._node is not None:
            self._node.destroy_node()

    def _subscribe(self) -> None:
        assert self._node is not None
        if self.schema == "image":
            self._node.create_subscription(Image, self.topic, self._image_cb, 10)
        elif self.schema == "compressed_image":
            self._node.create_subscription(CompressedImage, self.topic, self._compressed_image_cb, 10)
        else:
            self._node.create_subscription(Float32MultiArray, self.topic, self._float_array_cb, 10)

    def _spin(self) -> None:
        assert self._executor is not None
        while not self._stop_event.is_set() and rclpy is not None and rclpy.ok():
            self._executor.spin_once(timeout_sec=0.1)
            if self._last_msg is not None:
                with self.path.open("a", encoding="utf-8") as handle:
                    handle.write(json.dumps(self._last_msg, ensure_ascii=False) + "\n")
                self._last_msg = None

    def _write_placeholder(self) -> None:
        self.path.write_text(
            json.dumps(
                {
                    "topic": self.topic,
                    "schema": self.schema,
                    "note": "ROS2 unavailable in this environment; recorder placeholder only.",
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )

    def _image_cb(self, msg):
        self._last_msg = {
            "topic": self.topic,
            "schema": self.schema,
            "height": int(getattr(msg, "height", 0) or 0),
            "width": int(getattr(msg, "width", 0) or 0),
            "encoding": str(getattr(msg, "encoding", "") or ""),
            "step": int(getattr(msg, "step", 0) or 0),
        }

    def _compressed_image_cb(self, msg):
        self._last_msg = {
            "topic": self.topic,
            "schema": self.schema,
            "format": str(getattr(msg, "format", "") or ""),
            "data_len": int(len(getattr(msg, "data", b"") or b"")),
        }

    def _float_array_cb(self, msg):
        data = list(getattr(msg, "data", []) or [])
        self._last_msg = {
            "topic": self.topic,
            "schema": self.schema,
            "data_len": len(data),
            "preview": data[:8],
        }


@dataclass
class RosImageFrameRecorder:
    name: str
    topic: str
    image_dir: Path
    frames_csv_path: Path
    schema: str = "image"
    _thread: threading.Thread | None = field(default=None, init=False)
    _stop_event: threading.Event = field(default_factory=threading.Event, init=False)
    _node: Node | None = field(default=None, init=False)
    _executor: SingleThreadedExecutor | None = field(default=None, init=False)
    _frame_index: int = field(default=0, init=False)
    _csv_fp: object | None = field(default=None, init=False)
    _csv_writer: csv.writer | None = field(default=None, init=False)

    def start(self) -> None:
        self.image_dir.mkdir(parents=True, exist_ok=True)
        self.frames_csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._csv_fp = self.frames_csv_path.open("w", newline="", encoding="utf-8")
        self._csv_writer = csv.writer(self._csv_fp)
        self._csv_writer.writerow(["index", "timestamp_unix", "timestamp_ms", "image_path", "topic", "encoding", "width", "height", "format"])
        if rclpy is None or Node is object:
            self._write_placeholder()
            return
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node(f"dexproj_{self.name}_image_recorder")
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._subscribe()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._executor is not None and self._node is not None:
            self._executor.remove_node(self._node)
        if self._node is not None:
            self._node.destroy_node()
        if self._csv_fp is not None:
            self._csv_fp.close()

    def _subscribe(self) -> None:
        assert self._node is not None
        if self.schema == "compressed_image":
            self._node.create_subscription(CompressedImage, self.topic, self._compressed_cb, 10)
        else:
            self._node.create_subscription(Image, self.topic, self._image_cb, 10)

    def _spin(self) -> None:
        assert self._executor is not None
        while not self._stop_event.is_set() and rclpy is not None and rclpy.ok():
            self._executor.spin_once(timeout_sec=0.1)

    def _write_placeholder(self) -> None:
        self.frames_csv_path.write_text(
            "index,timestamp_unix,timestamp_ms,image_path,topic,encoding,width,height,format\n",
            encoding="utf-8",
        )

    def _write_frame_row(self, image_path: Path, timestamp_unix: float, topic: str, encoding: str, width: int, height: int, fmt: str = "") -> None:
        assert self._csv_writer is not None
        self._frame_index += 1
        self._csv_writer.writerow([
            self._frame_index,
            f"{timestamp_unix:.9f}",
            f"{timestamp_unix * 1000.0:.3f}",
            str(image_path),
            topic,
            encoding,
            width,
            height,
            fmt,
        ])
        if self._csv_fp is not None:
            self._csv_fp.flush()

    def _image_cb(self, msg):
        timestamp_unix = time.time()
        width = int(getattr(msg, "width", 0) or 0)
        height = int(getattr(msg, "height", 0) or 0)
        encoding = str(getattr(msg, "encoding", "") or "")
        image_path = self.image_dir / f"frame_{self._frame_index + 1:06d}.ppm"
        self._save_raw_image(image_path, msg, encoding, width, height)
        self._write_frame_row(image_path, timestamp_unix, self.topic, encoding, width, height)

    def _compressed_cb(self, msg):
        timestamp_unix = time.time()
        fmt = str(getattr(msg, "format", "") or "jpg")
        suffix = ".jpg" if "jpg" in fmt.lower() or "jpeg" in fmt.lower() else ".png"
        image_path = self.image_dir / f"frame_{self._frame_index + 1:06d}{suffix}"
        image_path.write_bytes(bytes(getattr(msg, "data", b"") or b""))
        self._write_frame_row(image_path, timestamp_unix, self.topic, "compressed", 0, 0, fmt)

    def _save_raw_image(self, image_path: Path, msg, encoding: str, width: int, height: int) -> None:
        data = bytes(getattr(msg, "data", b"") or b"")
        if encoding in {"rgb8", "bgr8"} and width > 0 and height > 0:
            array = np.frombuffer(data, dtype=np.uint8).reshape(height, width, 3)
            if encoding == "bgr8":
                array = array[:, :, ::-1]
            self._write_ppm(image_path, array)
            return
        if encoding == "mono8" and width > 0 and height > 0:
            array = np.frombuffer(data, dtype=np.uint8).reshape(height, width)
            self._write_pgm(image_path, array)
            return
        image_path.write_bytes(data)

    @staticmethod
    def _write_ppm(path: Path, array: np.ndarray) -> None:
        header = f"P6\n{array.shape[1]} {array.shape[0]}\n255\n".encode("ascii")
        path.write_bytes(header + array.astype(np.uint8).tobytes())

    @staticmethod
    def _write_pgm(path: Path, array: np.ndarray) -> None:
        header = f"P5\n{array.shape[1]} {array.shape[0]}\n255\n".encode("ascii")
        path.write_bytes(header + array.astype(np.uint8).tobytes())
