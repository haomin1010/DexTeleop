"""Direct Wuji glove to Wuji hand teleoperation.

This node intentionally bypasses the wujihandros2 driver. It mirrors the
working ``wuji-retargeting/example/teleop_real.py`` path:

    wuji_sdk glove skeleton -> Retargeter -> wujihandpy realtime_controller

ROS2 is used as a process supervisor and to publish both commanded joint targets
and hardware feedback for recording/monitoring.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

_DEXPROJ_RETARGETING_ROOT = Path(
    os.environ.get("DEXPROJ_RETARGETING_ROOT", "/workspace/DexProj/wuji-retargeting")
)
if _DEXPROJ_RETARGETING_ROOT.exists():
    sys.path.insert(0, str(_DEXPROJ_RETARGETING_ROOT))

try:
    import wujihandpy
except ImportError as exc:  # pragma: no cover - hardware runtime dependency
    raise ImportError("wujihandpy is required for direct Wuji hand control.") from exc

try:
    from wuji_sdk import SdkManager
except ImportError as exc:  # pragma: no cover - hardware runtime dependency
    raise ImportError("wuji_sdk is required for Wuji glove input.") from exc

try:
    import wuji_retargeting
    from wuji_retargeting import Retargeter
except ImportError as exc:  # pragma: no cover - hardware runtime dependency
    raise ImportError("wuji_retargeting is required for glove retargeting.") from exc

from ament_index_python.packages import get_package_share_directory


CONTROL_RATE_HZ = 1000.0

JOINT_NAMES = [
    "thumb_joint_0", "thumb_joint_1", "thumb_joint_2", "thumb_joint_3",
    "index_joint_0", "index_joint_1", "index_joint_2", "index_joint_3",
    "middle_joint_0", "middle_joint_1", "middle_joint_2", "middle_joint_3",
    "ring_joint_0", "ring_joint_1", "ring_joint_2", "ring_joint_3",
    "pinky_joint_0", "pinky_joint_1", "pinky_joint_2", "pinky_joint_3",
]


def _default_retarget_config(side: str) -> str:
    config_dir = Path(get_package_share_directory("wujihand_output")) / "config"
    return str(config_dir / f"retarget_wuji_glove_{side}.yaml")


def _sdk_params_dir() -> Path:
    override = os.environ.get("WUJI_SDK_PARAMS_DIR")
    if override:
        return Path(override).expanduser()
    return Path.home() / ".wuji" / "sdk" / "params"


def _sdk_param_file(sn: str) -> Path:
    return _sdk_params_dir() / f"{sn}.toml"


def _read_robot_name(path: Path) -> str:
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return "unknown"
    match = re.search(r"<robot\s+name=\\?\"([^\"\\]+)", text)
    return match.group(1) if match else "unknown"


def _summarize_retarget_config(path: str) -> str:
    try:
        with open(path, "r", encoding="utf-8") as handle:
            config = yaml.safe_load(handle) or {}
    except Exception as exc:
        return f"unreadable: {exc!r}"
    retarget = config.get("retarget", {}) if isinstance(config, dict) else {}
    return (
        f"rotation={retarget.get('mediapipe_rotation')}, "
        f"wrist_offset_cm={retarget.get('wrist_offset_cm')}, "
        f"thumb_offset_cm={retarget.get('thumb_offset_cm')}"
    )


def _as_joint_vector(raw) -> Optional[np.ndarray]:
    if raw is None:
        return None
    array = np.asarray(raw, dtype=np.float32).reshape(-1)
    if array.size < len(JOINT_NAMES):
        return None
    return array[: len(JOINT_NAMES)]


class _GloveStream:
    def __init__(self, *, side: str, device_name: str, glove_sn: str):
        manager = SdkManager.instance()
        self._device = self._connect_with_retry(manager, glove_sn, device_name)
        self._sub = self._device.hand_skeleton().subscribe()
        self.side = side

    @staticmethod
    def _connect_with_retry(manager, glove_sn: str, device_name: str, attempts: int = 3):
        last_error = None
        for attempt in range(1, attempts + 1):
            try:
                return manager.connect(sn=glove_sn, device_name=device_name)
            except Exception as exc:
                last_error = exc
                if attempt == attempts:
                    break
                print(
                    f"Wuji glove connect failed for {glove_sn} "
                    f"(attempt {attempt}/{attempts}): {exc}; retrying...",
                    flush=True,
                )
                time.sleep(1.0)
        raise last_error

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

    def close(self) -> None:
        self._sub = None
        self._device = None


@dataclass
class _DirectHand:
    side: str
    hand_sn: str
    glove_sn: str
    glove_device_name: str
    retarget_config: str

    def __post_init__(self) -> None:
        self.glove: Optional[_GloveStream] = None
        self.glove = _GloveStream(
            side=self.side,
            device_name=self.glove_device_name,
            glove_sn=self.glove_sn,
        )
        print(f"Direct {self.side} retarget config: {self.retarget_config}", flush=True)
        print(
            f"Direct {self.side} retarget summary: "
            f"{_summarize_retarget_config(self.retarget_config)}",
            flush=True,
        )
        self.retargeter = Retargeter.from_yaml(self.retarget_config, self.side)
        self.hand = None
        self.controller = None
        self.latest_command: Optional[np.ndarray] = None
        self._warned_state_read = False

    def connect_hand(self) -> None:
        self.hand = wujihandpy.Hand(serial_number=self.hand_sn or None)
        self.hand.disable_thread_safe_check()
        self.hand.write_joint_enabled(True)
        self.controller = self.hand.realtime_controller(
            enable_upstream=False,
            filter=wujihandpy.filter.LowPass(cutoff_freq=5.0),
        )

    def step(self) -> Optional[np.ndarray]:
        if self.glove is None or self.controller is None:
            return None
        keypoints = self.glove.recv_latest()
        if keypoints is None or np.allclose(keypoints, 0):
            return None

        qpos = np.asarray(self.retargeter.retarget(keypoints), dtype=np.float32)
        self.controller.set_joint_target_position(qpos.reshape(5, 4))
        self.latest_command = qpos
        return qpos

    def read_actual_position(self) -> Optional[np.ndarray]:
        if self.hand is None:
            return None

        try:
            reader = getattr(self.controller, "get_joint_actual_position", None)
            if callable(reader):
                return _as_joint_vector(reader())

            reader = getattr(self.hand, "get_joint_actual_position", None)
            if callable(reader):
                return _as_joint_vector(reader())

            reader = getattr(self.hand, "read_joint_actual_position", None)
            if callable(reader):
                return _as_joint_vector(reader())
        except Exception as exc:
            if not self._warned_state_read:
                print(
                    f"Direct {self.side} actual joint read unavailable: {exc!r}",
                    flush=True,
                )
                self._warned_state_read = True
        return None

    def close(self) -> None:
        if self.hand is not None:
            try:
                self.hand.write_joint_enabled(False)
            except Exception:
                pass
        if self.glove is not None:
            self.glove.close()


class WujiHandDirectControllerNode(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("wujihand_direct_controller")

        self._hands: dict[str, _DirectHand] = {}
        self._command_publishers = {
            "left": self.create_publisher(JointState, "/wuji_hand/left/joint_command", 10),
            "right": self.create_publisher(JointState, "/wuji_hand/right/joint_command", 10),
        }
        self._state_publishers = {
            "left": self.create_publisher(JointState, "/wuji_hand/left/joint_state", 10),
            "right": self.create_publisher(JointState, "/wuji_hand/right/joint_state", 10),
        }
        self.get_logger().info(
            f"wuji_retargeting loaded from: {getattr(wuji_retargeting, '__file__', 'unknown')}"
        )

        if args.include_left_hand == "true":
            self._log_sdk_params("left", args.left_glove_sn)
            self._hands["left"] = _DirectHand(
                side="left",
                hand_sn=args.left_hand_sn,
                glove_sn=args.left_glove_sn,
                glove_device_name=args.left_device_name,
                retarget_config=args.left_retarget_config or _default_retarget_config("left"),
            )
            self.get_logger().info(
                f"Direct left hand: hand_sn={args.left_hand_sn}, glove_sn={args.left_glove_sn}"
            )

        if args.include_right_hand == "true":
            self._log_sdk_params("right", args.right_glove_sn)
            self._hands["right"] = _DirectHand(
                side="right",
                hand_sn=args.right_hand_sn,
                glove_sn=args.right_glove_sn,
                glove_device_name=args.right_device_name,
                retarget_config=args.right_retarget_config or _default_retarget_config("right"),
            )
            self.get_logger().info(
                f"Direct right hand: hand_sn={args.right_hand_sn}, glove_sn={args.right_glove_sn}"
            )

        if not self._hands:
            raise RuntimeError("No direct Wuji hand channels enabled.")

        for side, hand in self._hands.items():
            hand.connect_hand()
            self.get_logger().info(f"Direct {side} Wuji hand hardware connected")

        self.create_timer(1.0 / CONTROL_RATE_HZ, self._control_loop)
        self.get_logger().info(
            f"Python direct Wuji hand control active at {CONTROL_RATE_HZ:.1f} Hz"
        )

    def _log_sdk_params(self, side: str, glove_sn: str) -> None:
        path = _sdk_param_file(glove_sn)
        if path.exists():
            self.get_logger().info(
                f"Wuji SDK params for {side} glove {glove_sn}: {path} "
                f"(robot={_read_robot_name(path)})"
            )
        else:
            self.get_logger().warning(
                f"Wuji SDK params missing for {side} glove {glove_sn}: {path}"
            )

    def _control_loop(self) -> None:
        for side, hand in self._hands.items():
            try:
                command = hand.step()
            except Exception as exc:
                self.get_logger().error(f"{side} direct control failed: {exc!r}")
                continue

            if command is not None:
                self._publish_joint_command(side, command)

            actual_position = hand.read_actual_position()
            if actual_position is not None:
                self._publish_joint_state(side, actual_position)

    def _publish_joint_command(self, side: str, command: np.ndarray) -> None:
        self._publish_joint_message(
            self._command_publishers[side],
            side,
            command,
        )

    def _publish_joint_state(self, side: str, position: np.ndarray) -> None:
        self._publish_joint_message(
            self._state_publishers[side],
            side,
            position,
        )

    def _publish_joint_message(self, publisher, side: str, values: np.ndarray) -> None:
        msg = JointState()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id=f"{side}_hand")
        msg.name = JOINT_NAMES
        msg.position = np.asarray(values, dtype=float).reshape(-1).tolist()
        publisher.publish(msg)

    def shutdown(self) -> None:
        self.get_logger().info("Shutting down Python direct Wuji hand control...")
        for hand in self._hands.values():
            try:
                hand.close()
            except Exception as exc:
                self.get_logger().warning(f"Error while closing hand: {exc!r}")


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Direct Wuji glove to Wuji hand controller")
    parser.add_argument("--left-hand-sn", default="", help="Left Wuji hand serial number")
    parser.add_argument("--right-hand-sn", default="", help="Right Wuji hand serial number")
    parser.add_argument("--left-glove-sn", default="", help="Left Wuji glove serial number")
    parser.add_argument("--right-glove-sn", default="", help="Right Wuji glove serial number")
    parser.add_argument("--left-device-name", default="glove_left", help="Left Wuji SDK device alias")
    parser.add_argument("--right-device-name", default="glove_right", help="Right Wuji SDK device alias")
    parser.add_argument("--left-retarget-config", default="", help="Left retarget YAML")
    parser.add_argument("--right-retarget-config", default="", help="Right retarget YAML")
    parser.add_argument("--include-left-hand", choices=["true", "false"], default="true")
    parser.add_argument("--include-right-hand", choices=["true", "false"], default="true")
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> None:
    program_name = sys.argv[0] if sys.argv else "wujihand_direct_controller"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]
    args = _parse_args(cli_argv)

    rclpy.init(args=raw_argv)
    node = WujiHandDirectControllerNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as exc:
            if "rcl_shutdown already called" not in str(exc):
                raise


if __name__ == "__main__":
    main()
