"""Bridge Tianji controller joint states to the right-arm URDF joint names."""

from __future__ import annotations

import argparse
import math
import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState


class TianjiJointStateBridge(Node):
    """Convert Tianji degree joint states into URDF-compatible radians."""

    def __init__(
        self,
        input_topic: str = "/tianji_arm/right/joint_state",
        output_topic: str = "/joint_states",
        side: str = "right",
    ):
        super().__init__("tianji_joint_state_bridge")
        self.side = side
        suffix = "R" if side == "right" else "L"
        self.urdf_joint_names = [f"Joint{i}_{suffix}" for i in range(1, 8)]
        self.pub = self.create_publisher(JointState, output_topic, 10)
        self.sub = self.create_subscription(JointState, input_topic, self._callback, 10)
        self.get_logger().info(
            f"Bridging {input_topic} degrees -> {output_topic} radians "
            f"as {self.urdf_joint_names}"
        )

    def _callback(self, msg: JointState) -> None:
        if len(msg.position) < 7:
            return
        out = JointState()
        out.header = msg.header
        out.name = self.urdf_joint_names
        out.position = [math.radians(float(x)) for x in msg.position[:7]]
        self.pub.publish(out)


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Bridge Tianji joint states to URDF joint states.")
    parser.add_argument("--input-topic", default="/tianji_arm/right/joint_state")
    parser.add_argument("--output-topic", default="/joint_states")
    parser.add_argument("--side", default="right", choices=["left", "right"])
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> None:
    program_name = sys.argv[0] if sys.argv else "tianji_joint_state_bridge"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    args = _parse_args(remove_ros_args(raw_argv)[1:])

    rclpy.init(args=raw_argv)
    node = TianjiJointStateBridge(
        input_topic=args.input_topic,
        output_topic=args.output_topic,
        side=args.side,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
