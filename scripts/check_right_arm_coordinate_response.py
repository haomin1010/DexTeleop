#!/usr/bin/env python3
"""Interactive coordinate-response check for the right HTC -> Tianji arm path."""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
import tf2_ros
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float64MultiArray


POSES = [
    ("neutral", "Put the right arm in a comfortable middle pose, then press Enter."),
    ("forward", "Move the right arm straight forward from neutral, then press Enter."),
    ("down", "Move the right arm downward from neutral, then press Enter."),
    ("outward", "Move the right arm outward/right from neutral, then press Enter."),
]


@dataclass
class Sample:
    name: str
    tianji_xyz: Optional[np.ndarray]
    tianji_rpy: Optional[np.ndarray]
    wrist_xyz: Optional[np.ndarray]
    wrist_rpy: Optional[np.ndarray]
    ee_pose: Optional[np.ndarray]


def _fmt(values: Optional[np.ndarray], precision: int = 3) -> str:
    if values is None:
        return "missing"
    return "[" + ", ".join(f"{float(v):.{precision}f}" for v in values) + "]"


class CoordinateResponseNode(Node):
    def __init__(self):
        super().__init__("right_arm_coordinate_response_check")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.ee_pose: Optional[np.ndarray] = None
        self.create_subscription(
            Float64MultiArray,
            "/tianji_arm/right/right_ee_pose",
            self._ee_pose_cb,
            10,
        )

    def _ee_pose_cb(self, msg: Float64MultiArray) -> None:
        self.ee_pose = np.array(list(msg.data), dtype=np.float64)

    def _lookup(self, target_frame: str, source_frame: str):
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
            tf2_ros.TransformException,
        ):
            return None

    @staticmethod
    def _xyz_rpy(tf_msg) -> tuple[np.ndarray, np.ndarray]:
        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        xyz = np.array([t.x, t.y, t.z], dtype=np.float64)
        rpy = R.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz", degrees=True)
        return xyz, rpy

    def sample(self, name: str, settle_sec: float) -> Sample:
        end = time.monotonic() + settle_sec
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

        tianji_tf = self._lookup("right_chest", "tianji_right")
        wrist_tf = self._lookup("right_chest", "right_wrist")

        tianji_xyz = tianji_rpy = wrist_xyz = wrist_rpy = None
        if tianji_tf is not None:
            tianji_xyz, tianji_rpy = self._xyz_rpy(tianji_tf)
        if wrist_tf is not None:
            wrist_xyz, wrist_rpy = self._xyz_rpy(wrist_tf)

        ee_pose = None if self.ee_pose is None else np.array(self.ee_pose, dtype=np.float64)
        return Sample(name, tianji_xyz, tianji_rpy, wrist_xyz, wrist_rpy, ee_pose)


def _dominant_axis(delta: Optional[np.ndarray]) -> str:
    if delta is None:
        return "missing"
    labels = ["x", "y", "z"]
    idx = int(np.argmax(np.abs(delta)))
    sign = "+" if delta[idx] >= 0 else "-"
    return f"{sign}{labels[idx]} ({delta[idx]:.3f} m)"


def _print_sample(sample: Sample) -> None:
    print(f"\n[{sample.name}]")
    print(f"  tianji xyz={_fmt(sample.tianji_xyz)} rpy={_fmt(sample.tianji_rpy, 1)}")
    print(f"  wrist  xyz={_fmt(sample.wrist_xyz)} rpy={_fmt(sample.wrist_rpy, 1)}")
    if sample.ee_pose is not None and len(sample.ee_pose) >= 6:
        print(f"  ee_pose xyz={_fmt(sample.ee_pose[:3])} abc={_fmt(sample.ee_pose[3:6], 1)}")
    else:
        print("  ee_pose missing")


def _print_delta(neutral: Sample, sample: Sample) -> None:
    def diff(a: Optional[np.ndarray], b: Optional[np.ndarray]) -> Optional[np.ndarray]:
        if a is None or b is None:
            return None
        return b - a

    tianji_delta = diff(neutral.tianji_xyz, sample.tianji_xyz)
    wrist_delta = diff(neutral.wrist_xyz, sample.wrist_xyz)
    ee_delta = None
    if neutral.ee_pose is not None and sample.ee_pose is not None:
        ee_delta = sample.ee_pose[:3] - neutral.ee_pose[:3]

    print(f"\nDelta neutral -> {sample.name}:")
    print(f"  tianji delta={_fmt(tianji_delta)} dominant={_dominant_axis(tianji_delta)}")
    print(f"  wrist  delta={_fmt(wrist_delta)} dominant={_dominant_axis(wrist_delta)}")
    print(f"  ee     delta={_fmt(ee_delta)} dominant={_dominant_axis(ee_delta)}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Check right-arm coordinate response.")
    parser.add_argument("--settle-sec", type=float, default=0.5)
    args = parser.parse_args()

    rclpy.init()
    node = CoordinateResponseNode()
    samples: list[Sample] = []
    try:
        for name, prompt in POSES:
            input(f"\n{prompt} ")
            sample = node.sample(name, settle_sec=args.settle_sec)
            _print_sample(sample)
            samples.append(sample)
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if not samples:
        return 1
    neutral = samples[0]
    for sample in samples[1:]:
        _print_delta(neutral, sample)

    print(
        "\nExpected broad pattern for right_chest frame depends on the configured "
        "axis convention, but each physical move should have one clear dominant "
        "axis and tianji/wrist/ee deltas should agree in direction. If a forward "
        "move mostly changes vertical/side axis, or tianji and ee disagree, the "
        "conversion being tested is wrong."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
