"""ROS2 node that streams Wuji glove hand data and publishes ROS topics."""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np

from ament_index_python.packages import get_package_share_directory

try:
    import yaml
except ImportError as exc:
    raise ImportError("PyYAML is required to load Wuji glove input configuration files.") from exc

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
    from rclpy.utilities import remove_ros_args
    from std_msgs.msg import Float32MultiArray
except ImportError as exc:
    raise ImportError("This module requires ROS 2 Python packages (rclpy, std_msgs).") from exc

try:
    from wuji_sdk import SdkManager
except ImportError as exc:
    raise ImportError("wuji_sdk is required to use Wuji glove input.") from exc


@dataclass
class WujiGloveInputConfig:
    """Configuration for the Wuji glove input streaming node."""

    config_path: Optional[str] = None

    publish_rate_hz: float = 50.0
    publish_hand_topic: str = "/hand_input"
    include_right_hand: bool = True
    include_left_hand: bool = True

    left_glove_sn: Optional[str] = None
    right_glove_sn: Optional[str] = None
    left_device_name: str = "glove_left"
    right_device_name: str = "glove_right"

    @classmethod
    def from_file(cls, path: str | Path) -> "WujiGloveInputConfig":
        cfg_path = Path(path).expanduser().resolve()
        if not cfg_path.exists():
            raise FileNotFoundError(f"Config file not found: {cfg_path}")

        with cfg_path.open("r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle) or {}

        if not isinstance(raw, dict):
            raise ValueError("WujiGloveInputConfig expects a mapping at the root of the YAML file.")

        valid_fields = {field.name for field in fields(cls)}
        data: Dict[str, Any] = {"config_path": str(cfg_path)}
        ignored_keys: list[str] = []

        for key, value in raw.items():
            if key in valid_fields:
                data[key] = value
            else:
                ignored_keys.append(key)

        if ignored_keys:
            print(f"[WujiGloveInputConfig] Ignoring unknown keys: {', '.join(sorted(ignored_keys))}")

        return cls(**data)


class _GloveStream:
    def __init__(self, side: str, device_name: str, glove_sn: Optional[str]):
        manager = SdkManager.instance()
        if glove_sn:
            self._device = manager.connect(sn=glove_sn, device_name=device_name)
        else:
            self._device = manager.auto_connect(device_name=device_name)
        self._sub = self._device.hand_skeleton().subscribe()
        self.side = side

    def recv_latest(self) -> Optional[np.ndarray]:
        skeleton = self._sub.recv()
        if skeleton is None:
            return None

        while True:
            newer = self._sub.recv()
            if newer is None:
                break
            skeleton = newer

        keypoints = np.array([joint.pose.position for joint in skeleton.joints], dtype=np.float32)
        if keypoints.shape != (21, 3):
            return None
        return keypoints


class WujiGloveInputNode(Node):
    """ROS2 node that reads Wuji glove data and publishes /hand_input."""

    def __init__(self, config: WujiGloveInputConfig):
        super().__init__("wuji_glove_input")
        self.config = config
        self._left_stream: Optional[_GloveStream] = None
        self._right_stream: Optional[_GloveStream] = None
        self._left_fingers: Optional[np.ndarray] = None
        self._right_fingers: Optional[np.ndarray] = None

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.hand_publisher = self.create_publisher(
            Float32MultiArray, self.config.publish_hand_topic, qos_profile
        )

        if self.config.include_left_hand:
            self._left_stream = _GloveStream(
                side="left",
                device_name=self.config.left_device_name,
                glove_sn=self.config.left_glove_sn,
            )
        if self.config.include_right_hand:
            self._right_stream = _GloveStream(
                side="right",
                device_name=self.config.right_device_name,
                glove_sn=self.config.right_glove_sn,
            )

        self.timer = self.create_timer(
            1.0 / max(self.config.publish_rate_hz, 1.0), self._publish_latest_frame
        )
        self.get_logger().info(
            f"[Wuji Glove Input] Publishing hand data on '{self.config.publish_hand_topic}' "
            f"at {self.config.publish_rate_hz:.1f} Hz."
        )

    def _publish_latest_frame(self) -> None:
        updated = False

        if self._right_stream is not None:
            latest = self._right_stream.recv_latest()
            if latest is not None:
                self._right_fingers = latest
                updated = True

        if self._left_stream is not None:
            latest = self._left_stream.recv_latest()
            if latest is not None:
                self._left_fingers = latest
                updated = True

        if not updated:
            return

        if self.config.include_right_hand and self._right_fingers is None:
            return
        if self.config.include_left_hand and self._left_fingers is None:
            return

        payloads = []
        if self.config.include_right_hand:
            payloads.append(self._right_fingers.flatten())
        if self.config.include_left_hand:
            payloads.append(self._left_fingers.flatten())

        if not payloads:
            return

        hand_msg = Float32MultiArray()
        hand_msg.data = np.concatenate(payloads).astype(np.float32).tolist()
        self.hand_publisher.publish(hand_msg)


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish Wuji glove hand data to ROS2 topics.")
    parser.add_argument(
        "-c",
        "--config",
        default=None,
        help="Path to a Wuji glove input YAML configuration file.",
    )
    parser.add_argument("--left-glove-sn", default=None, help="Left glove serial number override.")
    parser.add_argument("--right-glove-sn", default=None, help="Right glove serial number override.")
    parser.add_argument("--left-device-name", default=None, help="Left glove device name override.")
    parser.add_argument("--right-device-name", default=None, help="Right glove device name override.")
    parser.add_argument(
        "--include-left-hand",
        choices=["true", "false"],
        default=None,
        help="Whether to publish left hand data.",
    )
    parser.add_argument(
        "--include-right-hand",
        choices=["true", "false"],
        default=None,
        help="Whether to publish right hand data.",
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> None:
    program_name = sys.argv[0] if sys.argv else "wuji_glove_input"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]
    args = _parse_args(cli_argv)

    if args.config:
        config_path = Path(args.config)
    else:
        share_dir = Path(get_package_share_directory("wuji_glove_input_py"))
        config_path = share_dir / "config" / "wuji_glove_input.yaml"

    config = WujiGloveInputConfig.from_file(str(config_path))
    if args.left_glove_sn is not None:
        config.left_glove_sn = args.left_glove_sn or None
    if args.right_glove_sn is not None:
        config.right_glove_sn = args.right_glove_sn or None
    if args.left_device_name is not None:
        config.left_device_name = args.left_device_name
    if args.right_device_name is not None:
        config.right_device_name = args.right_device_name
    if args.include_left_hand is not None:
        config.include_left_hand = args.include_left_hand == "true"
    if args.include_right_hand is not None:
        config.include_right_hand = args.include_right_hand == "true"

    rclpy.init(args=raw_argv)
    node = WujiGloveInputNode(config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as exc:
            if "rcl_shutdown already called" not in str(exc):
                raise


if __name__ == "__main__":
    main(sys.argv[1:])
