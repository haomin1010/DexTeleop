#!/usr/bin/env python3
"""Print right-arm tracker target, IK pose, and command freshness.

This is a read-only ROS 2 diagnostic helper for the HTC -> Tianji right-arm
chain. It helps separate coordinate/workspace issues from tracker dropouts by
showing whether the target keeps moving while joint commands stop updating.
"""

from __future__ import annotations

import argparse
import math
import time
from typing import Optional

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


RIGHT_INIT_POS = np.array([0.5733, -0.2237, 0.2762], dtype=np.float64)


def _fmt(values, precision: int = 3) -> str:
    return "[" + ", ".join(f"{float(v):.{precision}f}" for v in values) + "]"


def _age(stamp: Optional[float]) -> str:
    if stamp is None:
        return "none"
    return f"{time.monotonic() - stamp:.2f}s"


class RightArmReachDiagnostic(Node):
    def __init__(self, interval_sec: float):
        super().__init__("right_arm_reach_diagnostic")
        self.interval_sec = max(float(interval_sec), 0.1)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.ee_pose: Optional[list[float]] = None
        self.ee_pose_stamp: Optional[float] = None
        self.zsp: Optional[list[float]] = None
        self.zsp_stamp: Optional[float] = None
        self.cmd: Optional[list[float]] = None
        self.cmd_stamp: Optional[float] = None
        self.state: Optional[list[float]] = None
        self.state_stamp: Optional[float] = None

        self.create_subscription(
            Float64MultiArray,
            "/tianji_arm/right/right_ee_pose",
            self._ee_pose_cb,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/tianji_arm/right/right_zsp_para",
            self._zsp_cb,
            10,
        )
        self.create_subscription(
            JointState,
            "/tianji_arm/right/joint_command",
            self._cmd_cb,
            10,
        )
        self.create_subscription(
            JointState,
            "/tianji_arm/right/joint_state",
            self._state_cb,
            10,
        )

        self.create_timer(self.interval_sec, self._print_snapshot)

    def _ee_pose_cb(self, msg: Float64MultiArray) -> None:
        self.ee_pose = list(msg.data)
        self.ee_pose_stamp = time.monotonic()

    def _zsp_cb(self, msg: Float64MultiArray) -> None:
        self.zsp = list(msg.data)
        self.zsp_stamp = time.monotonic()

    def _cmd_cb(self, msg: JointState) -> None:
        self.cmd = list(msg.position)
        self.cmd_stamp = time.monotonic()

    def _state_cb(self, msg: JointState) -> None:
        self.state = list(msg.position)
        self.state_stamp = time.monotonic()

    def _lookup(self, target_frame: str, source_frame: str):
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            tf2_ros.TransformException,
        ):
            return None

    @staticmethod
    def _tf_xyz_rpy(tf_msg):
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        quat = [q.x, q.y, q.z, q.w]
        rpy = R.from_quat(quat).as_euler("xyz", degrees=True)
        return np.array([t.x, t.y, t.z], dtype=np.float64), rpy

    def _print_snapshot(self) -> None:
        tianji_tf = self._lookup("right_chest", "tianji_right")
        wrist_tf = self._lookup("right_chest", "right_wrist")
        arm_tf = self._lookup("right_chest", "right_arm")

        print("\n=== right arm reach diagnostic ===", flush=True)
        if tianji_tf is not None:
            xyz, rpy = self._tf_xyz_rpy(tianji_tf)
            delta = xyz - RIGHT_INIT_POS
            print(
                "tf right_chest->tianji_right "
                f"xyz={_fmt(xyz)} delta_from_init={_fmt(delta)} "
                f"dist_from_init={np.linalg.norm(delta):.3f} rpy_deg={_fmt(rpy, 1)}"
            )
        else:
            print("tf right_chest->tianji_right missing")

        if wrist_tf is not None:
            xyz, rpy = self._tf_xyz_rpy(wrist_tf)
            print(f"tf right_chest->right_wrist  xyz={_fmt(xyz)} rpy_deg={_fmt(rpy, 1)}")
        else:
            print("tf right_chest->right_wrist missing")

        if arm_tf is not None:
            xyz, _ = self._tf_xyz_rpy(arm_tf)
            print(f"tf right_chest->right_arm    xyz={_fmt(xyz)}")
        else:
            print("tf right_chest->right_arm missing")

        if self.ee_pose is not None and len(self.ee_pose) >= 6:
            pos = np.array(self.ee_pose[:3], dtype=np.float64)
            delta = pos - RIGHT_INIT_POS
            print(
                "right_ee_pose "
                f"xyz={_fmt(pos)} delta_from_init={_fmt(delta)} "
                f"dist_from_init={np.linalg.norm(delta):.3f} "
                f"abc_deg={_fmt(self.ee_pose[3:6], 1)} age={_age(self.ee_pose_stamp)}"
            )
        else:
            print(f"right_ee_pose missing age={_age(self.ee_pose_stamp)}")

        if self.zsp is not None:
            zsp_norm = math.sqrt(sum(float(v) * float(v) for v in self.zsp[:3]))
            print(f"right_zsp={_fmt(self.zsp)} norm3={zsp_norm:.3f} age={_age(self.zsp_stamp)}")
        else:
            print(f"right_zsp missing age={_age(self.zsp_stamp)}")

        if self.cmd is not None:
            print(f"joint_command={_fmt(self.cmd, 1)} age={_age(self.cmd_stamp)}")
        else:
            print(f"joint_command missing age={_age(self.cmd_stamp)}")

        if self.state is not None:
            print(f"joint_state  ={_fmt(self.state, 1)} age={_age(self.state_stamp)}")
        else:
            print(f"joint_state missing age={_age(self.state_stamp)}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Diagnose right-arm reach failures.")
    parser.add_argument("--interval", type=float, default=0.5, help="Print interval in seconds.")
    args = parser.parse_args()

    rclpy.init()
    node = RightArmReachDiagnostic(interval_sec=args.interval)
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
