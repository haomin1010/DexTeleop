from __future__ import annotations

import argparse
import time
from typing import Optional, Set

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState

from tianji_output import TianjiArmController
from .common import ROS2LoggerAdapter, get_default_qos, get_package_config_path, load_yaml_config


LEFT_ARM_CMD_TOPIC = "/tianji_arm/left/joint_command"
RIGHT_ARM_CMD_TOPIC = "/tianji_arm/right/joint_command"
LEFT_ARM_STATE_HW_TOPIC = "/tianji_arm/left/joint_state_hw"
RIGHT_ARM_STATE_HW_TOPIC = "/tianji_arm/right/joint_state_hw"


def _parse_active_sides(value: str) -> Set[str]:
    sides = str(value).lower()
    if sides == "both":
        return {"left", "right"}
    if sides in ("left", "right"):
        return {sides}
    raise ValueError("teleop_active_sides must be 'both', 'left', or 'right'")


class TianjiSdkExecutorNode(Node):
    """Execute joint_command on real robot via Marvin SDK."""

    def __init__(
        self,
        controller_config: str,
        control_rate_hz: float = 100.0,
        publish_hw_state: bool = True,
    ) -> None:
        super().__init__("tianji_sdk_executor")
        config = load_yaml_config(controller_config)
        robot_ip = str(config.get("robot_ip", "192.168.8.166"))
        config_path = get_package_config_path("tianji_output", "ccs_m6.MvKDCfg")
        if config_path is None:
            raise FileNotFoundError("Cannot find tianji_output/config/ccs_m6.MvKDCfg")

        self._active_sides = _parse_active_sides(config.get("teleop_active_sides", "right"))
        self._init_move_sides = str(config.get("init_move_sides", "both")).lower()
        self._init_move_duration_sec = float(config.get("init_move_duration_sec", 3.0))

        self.controller = TianjiArmController(
            robot_ip=robot_ip,
            config_path=str(config_path),
            logger=ROS2LoggerAdapter(self.get_logger()),
            dry_run=False,
            read_only=False,
            feedback_handshake=False,
            prefer_last_ik_reference=False,
            ik_subprocess_isolate=False,
        )
        try:
            self.controller.set_impedance_mode(mode="joint")
            self.get_logger().info("SDK executor: joint impedance mode configured")
        except Exception as exc:
            self.get_logger().warn(f"SDK executor: failed to set joint impedance mode: {exc}")

        if self._init_move_duration_sec > 0.0:
            try:
                self.get_logger().info(
                    f"SDK executor: init move sides={self._init_move_sides} "
                    f"duration={self._init_move_duration_sec:.1f}s"
                )
                self.controller.move_to_init(
                    wait=True,
                    timeout=1,
                    duration=self._init_move_duration_sec,
                    sides=self._init_move_sides,
                )
            except Exception as exc:
                self.get_logger().warn(f"SDK executor: init move failed: {exc}")

        self._last_left_cmd: Optional[list] = None
        self._last_right_cmd: Optional[list] = None
        self._last_send_time = 0.0
        self._min_period = 1.0 / max(float(control_rate_hz), 1.0)
        self._publish_hw_state = bool(publish_hw_state)

        qos = get_default_qos()
        if "left" in self._active_sides:
            self.left_sub = self.create_subscription(
                JointState, LEFT_ARM_CMD_TOPIC, self._left_cmd_cb, qos
            )
        else:
            self.left_sub = None
        if "right" in self._active_sides:
            self.right_sub = self.create_subscription(
                JointState, RIGHT_ARM_CMD_TOPIC, self._right_cmd_cb, qos
            )
        else:
            self.right_sub = None
        if self._publish_hw_state:
            self.left_state_pub = self.create_publisher(JointState, LEFT_ARM_STATE_HW_TOPIC, qos)
            self.right_state_pub = self.create_publisher(JointState, RIGHT_ARM_STATE_HW_TOPIC, qos)
            self.create_timer(self._min_period, self._publish_hw_state_cb)

        self.get_logger().info(
            "SDK executor ready: active_sides="
            f"{sorted(self._active_sides)}, init_move_sides={self._init_move_sides}, "
            f"publish_hw_state={self._publish_hw_state}"
        )

    def _left_cmd_cb(self, msg: JointState) -> None:
        if msg.position:
            self._last_left_cmd = list(msg.position)
            self._send_latest()

    def _right_cmd_cb(self, msg: JointState) -> None:
        if msg.position:
            self._last_right_cmd = list(msg.position)
            self._send_latest()

    def _send_latest(self) -> None:
        now = time.monotonic()
        if now - self._last_send_time < self._min_period:
            return
        self._last_send_time = now

        left_cmd = self._last_left_cmd if "left" in self._active_sides else None
        right_cmd = self._last_right_cmd if "right" in self._active_sides else None
        if left_cmd is None and right_cmd is None:
            return
        self.controller.move_to_joints_direct(
            left_joints=left_cmd,
            right_joints=right_cmd,
        )

    def _publish_hw_state_cb(self) -> None:
        try:
            left_joints, right_joints = self.controller.get_current_joints()
        except Exception as exc:
            self.get_logger().warn(f"Failed to read robot feedback: {exc}")
            return
        stamp = self.get_clock().now().to_msg()
        if left_joints is not None:
            msg = JointState()
            msg.header.stamp = stamp
            msg.header.frame_id = "left_base_state_hw"
            msg.name = [f"left_joint_{i+1}" for i in range(7)]
            msg.position = list(left_joints)
            self.left_state_pub.publish(msg)
        if right_joints is not None:
            msg = JointState()
            msg.header.stamp = stamp
            msg.header.frame_id = "right_base_state_hw"
            msg.name = [f"right_joint_{i+1}" for i in range(7)]
            msg.position = list(right_joints)
            self.right_state_pub.publish(msg)


def main(argv: Optional[list[str]] = None) -> None:
    cli_args = remove_ros_args(args=argv if argv is None else None)[1:]
    parser = argparse.ArgumentParser(description="Tianji SDK executor node")
    parser.add_argument("-c", "--config", required=True, help="Path to controller yaml")
    parser.add_argument("--rate", type=float, default=100.0, help="Max SDK send rate")
    parser.add_argument(
        "--no-hw-state-pub",
        action="store_true",
        help="Disable publishing /tianji_arm/*/joint_state_hw",
    )
    args = parser.parse_args(cli_args)

    rclpy.init(args=None)
    node = TianjiSdkExecutorNode(
        controller_config=args.config,
        control_rate_hz=args.rate,
        publish_hw_state=not args.no_hw_state_pub,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
