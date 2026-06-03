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
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage, Image, JointState
    from std_msgs.msg import Float32MultiArray
except ImportError:  # pragma: no cover
    rclpy = None
    SingleThreadedExecutor = None
    Node = object  # type: ignore[assignment]
    QoSProfile = None  # type: ignore[assignment]
    ReliabilityPolicy = None  # type: ignore[assignment]
    HistoryPolicy = None  # type: ignore[assignment]
    Image = object  # type: ignore[assignment]
    CompressedImage = object  # type: ignore[assignment]
    JointState = object  # type: ignore[assignment]
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
class JointCsvPairRecorder:
    name: str
    state_topics: dict[str, str]
    action_topics: dict[str, str]
    timestamp_path: Path
    observation_path: Path
    action_path: Path
    columns_by_side: dict[str, list[str]]
    interval_sec: float = 0.01
    side_order: tuple[str, str] = ("left", "right")
    _thread: threading.Thread | None = field(default=None, init=False)
    _stop_event: threading.Event = field(default_factory=threading.Event, init=False)
    _node: Node | None = field(default=None, init=False)
    _executor: SingleThreadedExecutor | None = field(default=None, init=False)
    _timestamp_fp: object | None = field(default=None, init=False)
    _observation_fp: object | None = field(default=None, init=False)
    _action_fp: object | None = field(default=None, init=False)
    _timestamp_writer: csv.writer | None = field(default=None, init=False)
    _observation_writer: csv.writer | None = field(default=None, init=False)
    _action_writer: csv.writer | None = field(default=None, init=False)
    _latest_state: dict[str, list[float]] = field(default_factory=dict, init=False)
    _latest_action: dict[str, list[float]] = field(default_factory=dict, init=False)
    _lock: threading.Lock = field(default_factory=threading.Lock, init=False)
    _row_index: int = field(default=0, init=False)

    def start(self) -> None:
        self.timestamp_path.parent.mkdir(parents=True, exist_ok=True)
        self._open_csvs()
        if rclpy is None or Node is object or JointState is object:
            self._write_placeholder()
            return
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node(f"dexproj_{self.name}_joint_recorder")
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._subscribe()
        self._thread = threading.Thread(target=self._spin_and_sample, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._executor is not None and self._node is not None:
            self._executor.remove_node(self._node)
        if self._node is not None:
            self._node.destroy_node()
        for handle in [self._timestamp_fp, self._observation_fp, self._action_fp]:
            if handle is not None:
                handle.close()

    def _open_csvs(self) -> None:
        self._timestamp_fp = self.timestamp_path.open("w", newline="", encoding="utf-8")
        self._observation_fp = self.observation_path.open("w", newline="", encoding="utf-8")
        self._action_fp = self.action_path.open("w", newline="", encoding="utf-8")
        self._timestamp_writer = csv.writer(self._timestamp_fp)
        self._observation_writer = csv.writer(self._observation_fp)
        self._action_writer = csv.writer(self._action_fp)
        columns = self._columns()
        self._timestamp_writer.writerow(["index", "timestamp_unix"])
        self._observation_writer.writerow(columns)
        self._action_writer.writerow(columns)

    def _write_placeholder(self) -> None:
        self._flush_all()

    def _subscribe(self) -> None:
        assert self._node is not None
        qos = self._joint_qos()
        for side, topic in self.state_topics.items():
            self._node.create_subscription(
                JointState,
                topic,
                lambda msg, side=side: self._joint_cb(msg, side, is_action=False),
                qos,
            )
        for side, topic in self.action_topics.items():
            self._node.create_subscription(
                JointState,
                topic,
                lambda msg, side=side: self._joint_cb(msg, side, is_action=True),
                qos,
            )

    def _joint_qos(self):
        if QoSProfile is None:
            return 10
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

    def _spin_and_sample(self) -> None:
        assert self._executor is not None
        next_sample = time.time()
        while not self._stop_event.is_set() and rclpy is not None and rclpy.ok():
            self._executor.spin_once(timeout_sec=0.001)
            now = time.time()
            if now < next_sample:
                continue
            self._write_sample(now)
            next_sample += self.interval_sec
            if next_sample < now:
                next_sample = now + self.interval_sec

    def _joint_cb(self, msg, side: str, is_action: bool) -> None:
        values = [float(value) for value in list(getattr(msg, "position", []) or [])]
        expected = len(self.columns_by_side.get(side, []))
        if expected > 0:
            values = values[:expected] + [float("nan")] * max(expected - len(values), 0)
        with self._lock:
            target = self._latest_action if is_action else self._latest_state
            target[side] = values

    def _write_sample(self, timestamp_unix: float) -> None:
        assert self._timestamp_writer is not None
        assert self._observation_writer is not None
        assert self._action_writer is not None
        with self._lock:
            if not self._latest_state and not self._latest_action:
                return
            state_row = self._row_from_latest(self._latest_state)
            action_row = self._row_from_latest(self._latest_action)
        self._row_index += 1
        self._timestamp_writer.writerow([self._row_index, f"{timestamp_unix:.9f}"])
        self._observation_writer.writerow(state_row)
        self._action_writer.writerow(action_row)
        self._flush_all()

    def _row_from_latest(self, latest: dict[str, list[float]]) -> list[float]:
        row: list[float] = []
        for side in self.side_order:
            width = len(self.columns_by_side.get(side, []))
            values = latest.get(side)
            if values is None:
                row.extend([float("nan")] * width)
            else:
                row.extend(values[:width] + [float("nan")] * max(width - len(values), 0))
        return row

    def _columns(self) -> list[str]:
        columns: list[str] = []
        for side in self.side_order:
            columns.extend(self.columns_by_side.get(side, []))
        return columns

    def _flush_all(self) -> None:
        for handle in [self._timestamp_fp, self._observation_fp, self._action_fp]:
            if handle is not None:
                handle.flush()


@dataclass
class RosImageFrameRecorder:
    name: str
    topic: str
    image_dir: Path
    frames_csv_path: Path
    schema: str = "image"
    reliability: str = "best_effort"
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
        qos = self._image_qos()
        if self.schema == "compressed_image":
            self._node.create_subscription(CompressedImage, self.topic, self._compressed_cb, qos)
        else:
            self._node.create_subscription(Image, self.topic, self._image_cb, qos)

    def _image_qos(self):
        if QoSProfile is None:
            return 10
        reliability = ReliabilityPolicy.BEST_EFFORT
        if str(self.reliability).lower() == "reliable":
            reliability = ReliabilityPolicy.RELIABLE
        return QoSProfile(
            reliability=reliability,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

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
        suffix = self._image_suffix_for_encoding(encoding)
        image_path = self.image_dir / f"frame_{self._frame_index + 1:06d}{suffix}"
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
            self._write_jpeg(image_path, array, encoding)
            return
        if encoding == "mono8" and width > 0 and height > 0:
            array = np.frombuffer(data, dtype=np.uint8).reshape(height, width)
            self._write_png_gray(image_path, array)
            return
        image_path.write_bytes(data)

    @staticmethod
    def _image_suffix_for_encoding(encoding: str) -> str:
        if encoding in {"rgb8", "bgr8", "mono8"}:
            return ".jpg"
        return ".bin"

    @staticmethod
    def _write_jpeg(path: Path, array: np.ndarray, encoding: str) -> None:
        try:
            import cv2  # type: ignore
        except ImportError:
            if encoding == "bgr8":
                array = array[:, :, ::-1]
            header = f"P6\n{array.shape[1]} {array.shape[0]}\n255\n".encode("ascii")
            path.with_suffix(".ppm").write_bytes(header + array.astype(np.uint8).tobytes())
            return
        encode_input = array if encoding == "bgr8" else array[:, :, ::-1]
        ok, encoded = cv2.imencode(".jpg", encode_input, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        if not ok:
            raise RuntimeError(f"failed to encode JPEG: {path}")
        path.write_bytes(encoded.tobytes())

    @staticmethod
    def _write_png_gray(path: Path, array: np.ndarray) -> None:
        try:
            import cv2  # type: ignore
        except ImportError:
            header = f"P5\n{array.shape[1]} {array.shape[0]}\n255\n".encode("ascii")
            path.with_suffix(".pgm").write_bytes(header + array.astype(np.uint8).tobytes())
            return
        ok, encoded = cv2.imencode(".jpg", array, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        if not ok:
            raise RuntimeError(f"failed to encode grayscale JPEG: {path}")
        path.write_bytes(encoded.tobytes())
