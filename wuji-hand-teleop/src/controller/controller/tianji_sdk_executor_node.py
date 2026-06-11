from __future__ import annotations

import argparse
import math
import time
from typing import Optional, Sequence

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

        self._last_left_cmd: Optional[list] = None
        self._last_right_cmd: Optional[list] = None
        self._last_sent_left: Optional[list] = None
        self._last_sent_right: Optional[list] = None
        self._max_joint_step_deg = max(float(config.get("max_joint_step_deg", 2.5)), 0.0)
        self._last_send_time = 0.0
        self._min_period = 1.0 / max(float(control_rate_hz), 1.0)
        self._publish_hw_state = bool(publish_hw_state)
        self._seed_current_joints()

        qos = get_default_qos()
        self.left_sub = self.create_subscription(JointState, LEFT_ARM_CMD_TOPIC, self._left_cmd_cb, qos)
        self.right_sub = self.create_subscription(JointState, RIGHT_ARM_CMD_TOPIC, self._right_cmd_cb, qos)
        if self._publish_hw_state:
            self.left_state_pub = self.create_publisher(JointState, LEFT_ARM_STATE_HW_TOPIC, qos)
            self.right_state_pub = self.create_publisher(JointState, RIGHT_ARM_STATE_HW_TOPIC, qos)
            self.create_timer(self._min_period, self._publish_hw_state_cb)

        self.get_logger().info(
            "SDK executor ready: subscribe joint_command, execute on robot, "
            f"publish_hw_state={self._publish_hw_state}, "
            f"max_joint_step_deg={self._max_joint_step_deg:.2f}"
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
        if self._last_left_cmd is None and self._last_right_cmd is None:
            return
        left_cmd = self._apply_joint_step_limit(self._last_left_cmd, self._last_sent_left)
        right_cmd = self._apply_joint_step_limit(self._last_right_cmd, self._last_sent_right)
        self.controller.move_to_joints_direct(
            left_joints=left_cmd,
            right_joints=right_cmd,
        )
        if left_cmd is not None:
            self._last_sent_left = list(left_cmd)
        if right_cmd is not None:
            self._last_sent_right = list(right_cmd)

    def _seed_current_joints(self) -> None:
        try:
            left_joints, right_joints = self.controller.get_current_joints()
        except Exception as exc:
            self.get_logger().warn(f"SDK executor: failed to read startup joints: {exc}")
            return

        if left_joints is not None:
            self._last_left_cmd = [float(v) for v in left_joints]
            self._last_sent_left = list(self._last_left_cmd)
        if right_joints is not None:
            self._last_right_cmd = [float(v) for v in right_joints]
            self._last_sent_right = list(self._last_right_cmd)

        if self._last_left_cmd is not None or self._last_right_cmd is not None:
            self.controller.move_to_joints_direct(
                left_joints=self._last_left_cmd,
                right_joints=self._last_right_cmd,
            )
            self.get_logger().info(
                "SDK executor: seeded startup hold command from current robot joints"
            )

    def _apply_joint_step_limit(
        self,
        target: Optional[Sequence[float]],
        previous: Optional[Sequence[float]],
    ) -> Optional[list]:
        if target is None:
            return None
        target_values = [float(v) for v in target]
        if self._max_joint_step_deg <= 0.0 or previous is None:
            return target_values
        if len(previous) != len(target_values):
            return target_values

        limited: list[float] = []
        for goal, last in zip(target_values, previous):
            delta = goal - float(last)
            if not math.isfinite(delta):
                limited.append(float(last))
                continue
            delta = max(-self._max_joint_step_deg, min(self._max_joint_step_deg, delta))
            limited.append(float(last) + delta)
        return limited

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
    cli_args = remove_ros_args(args=argv if argv is not None else None)[1:]
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
