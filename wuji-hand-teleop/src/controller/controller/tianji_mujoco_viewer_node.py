"""MuJoCo viewer for the Tianji M6 arm (left or right)."""

from __future__ import annotations

import argparse
import math
import sys
import tempfile
import time
from pathlib import Path
from typing import Optional

from ament_index_python.packages import get_package_share_directory
import mujoco
import mujoco.viewer
import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState

from .common import get_default_qos


class TianjiMujocoViewer(Node):
    """Display Tianji joint states and tracker target commands in MuJoCo."""

    def __init__(
        self,
        side: str = "right",
        joint_topic: str = "/tianji_arm/right/joint_command",
        initial_joint_topic: str = "/tianji_arm/right/joint_state",
    ):
        side = str(side).lower()
        if side not in ("left", "right"):
            raise ValueError(f"side must be 'left' or 'right', got {side!r}")

        super().__init__(f"tianji_mujoco_viewer_{side}")
        self.side = side
        self.joint_topic = joint_topic
        self.initial_joint_topic = initial_joint_topic
        suffix = "L" if side == "left" else "R"
        self.joint_names = [f"Joint{i}_{suffix}" for i in range(1, 8)]
        self.latest_joints_deg: Optional[list[float]] = None
        self._initialized_from_state = False
        self._received_command = False
        self._logged_initial = False
        self._logged_command = False
        self._logged_apply = False

        self.model = self._load_model()
        self.data = mujoco.MjData(self.model)
        self.qpos_addr = self._resolve_qpos_addresses()
        self.get_logger().info(
            f"MuJoCo qpos addresses: {dict(zip(self.joint_names, self.qpos_addr))}"
        )

        qos = get_default_qos()
        self.initial_sub = self.create_subscription(
            JointState,
            initial_joint_topic,
            self._initial_joint_callback,
            qos,
        )
        self.command_sub = self.create_subscription(
            JointState,
            joint_topic,
            self._command_joint_callback,
            qos,
        )
        self.get_logger().info(
            f"MuJoCo viewer initial pose from {initial_joint_topic}; command target from {joint_topic}"
        )

    def _load_model(self) -> mujoco.MjModel:
        share_dir = Path(get_package_share_directory("tianji_urdf"))
        urdf_name = "left.urdf" if self.side == "left" else "right.urdf"
        urdf_path = share_dir / "urdf" / urdf_name
        mesh_dir = share_dir / "meshes"
        xml = urdf_path.read_text()
        xml = xml.replace("package://tianji_urdf/meshes/", f"{mesh_dir}/")

        tmp = tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False)
        tmp.write(xml)
        tmp.close()
        self.get_logger().info(f"Loading MuJoCo model from {urdf_path}")
        return mujoco.MjModel.from_xml_path(tmp.name)

    def _resolve_qpos_addresses(self) -> list[int]:
        addrs = []
        for name in self.joint_names:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise RuntimeError(f"MuJoCo model does not contain joint {name}")
            addrs.append(int(self.model.jnt_qposadr[joint_id]))
        return addrs

    def _extract_joints(self, msg: JointState) -> Optional[list[float]]:
        if len(msg.position) < 7:
            return None
        return [float(x) for x in msg.position[:7]]

    def _initial_joint_callback(self, msg: JointState) -> None:
        if self._received_command:
            return
        joints = self._extract_joints(msg)
        if joints is not None:
            if not self._logged_initial:
                self.get_logger().info(f"Received initial joint_state degrees: {joints}")
                self._logged_initial = True
            self._initialized_from_state = True
            self.latest_joints_deg = joints

    def _command_joint_callback(self, msg: JointState) -> None:
        if not self._initialized_from_state:
            return
        joints = self._extract_joints(msg)
        if joints is not None:
            if not self._logged_command:
                self.get_logger().info(f"Received MuJoCo drive joint degrees: {joints}")
                self._logged_command = True
            self._received_command = True
            self.latest_joints_deg = joints

    def apply_latest_joints(self) -> None:
        if self.latest_joints_deg is None:
            return
        radians = [math.radians(value_deg) for value_deg in self.latest_joints_deg]
        for addr, value_deg in zip(self.qpos_addr, self.latest_joints_deg):
            self.data.qpos[addr] = math.radians(value_deg)
        mujoco.mj_forward(self.model, self.data)
        if not self._logged_apply:
            self.get_logger().info(f"Applied MuJoCo qpos radians: {radians}")
            self._logged_apply = True


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Show Tianji arm joint states in MuJoCo.")
    parser.add_argument("--side", default="right", choices=["left", "right"])
    parser.add_argument("--joint-topic", default=None)
    parser.add_argument("--initial-joint-topic", default=None)
    args = parser.parse_args(argv)
    if args.joint_topic is None:
        args.joint_topic = f"/tianji_arm/{args.side}/joint_command"
    if args.initial_joint_topic is None:
        args.initial_joint_topic = f"/tianji_arm/{args.side}/joint_state"
    return args


def main(argv: Optional[list[str]] = None) -> None:
    program_name = sys.argv[0] if sys.argv else "tianji_mujoco_viewer"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    args = _parse_args(remove_ros_args(raw_argv)[1:])

    rclpy.init(args=raw_argv)
    node = TianjiMujocoViewer(
        side=args.side,
        joint_topic=args.joint_topic,
        initial_joint_topic=args.initial_joint_topic,
    )

    try:
        with mujoco.viewer.launch_passive(node.model, node.data) as viewer:
            while rclpy.ok() and viewer.is_running():
                rclpy.spin_once(node, timeout_sec=0.0)
                node.apply_latest_joints()
                viewer.sync()
                time.sleep(1.0 / 60.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
