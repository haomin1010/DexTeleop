#!/usr/bin/env python3
"""Log right_wrist frame axes and rotation deltas for RViz / static-TF diagnosis.

Run while teleop is publishing TF (read_only is enough). Move the wrist slowly:
neutral a few seconds, then roll-like / pitch-like / yaw-like motion. Paste logs
with prefix [WRIST_FRAME_DEBUG] for analysis.

Example:
  python3 scripts/log_wrist_frame_axes.py --side right --duration-sec 45
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time


def _reexec_in_docker_if_needed() -> None:
    script_path = os.path.realpath(__file__)
    root_dir = os.path.dirname(os.path.dirname(script_path))
    ensure_script = os.path.join(root_dir, "scripts", "ensure_docker_exec.sh")
    container_script = "/workspace/DexProj/scripts/log_wrist_frame_axes.py"
    container_activate = "/workspace/DexProj/scripts/activate_dexproj_env.sh"

    if os.path.exists("/.dockerenv") or not os.path.isfile(ensure_script):
        return

    print(
        "[wrist_frame_log] Re-running inside DexProj Docker...",
        file=sys.stderr,
    )
    cmd = [
        ensure_script,
        "--",
        "bash",
        "-lc",
        "source "
        + container_activate
        + " && python3 "
        + container_script
        + " "
        + " ".join(subprocess.list2cmdline([arg]) for arg in sys.argv[1:]),
    ]
    os.execv(ensure_script, cmd)


try:
    import numpy as np
    import rclpy
    import tf2_ros
    from rclpy.node import Node
    from scipy.spatial.transform import Rotation as R
except ModuleNotFoundError as exc:
    if exc.name in {"rclpy", "tf2_ros"}:
        _reexec_in_docker_if_needed()
    raise


LOG_TAG = "[WRIST_FRAME_DEBUG]"


class WristFrameLogger(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("wrist_frame_logger")
        self.args = args
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.zero_rot: np.ndarray | None = None
        self.zero_pos: np.ndarray | None = None
        self.sample_count = 0
        self.dominant_counts = {"X": 0, "Y": 0, "Z": 0, "·": 0}

    def lookup_rot_pos(self, parent: str, child: str) -> tuple[np.ndarray, np.ndarray] | None:
        try:
            tf = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None
        t = tf.transform
        quat = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
        rot = R.from_quat(quat).as_matrix()
        pos = np.array([t.translation.x, t.translation.y, t.translation.z], dtype=np.float64)
        return rot, pos

    def wait_for_tf(self) -> bool:
        deadline = time.monotonic() + self.args.tf_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.lookup_rot_pos(self.args.chest_frame, self.args.wrist_frame) is not None:
                return True
        return False

    def log_static_snapshot(self) -> None:
        chest = self.args.chest_frame
        wrist = self.args.wrist_frame
        tianji = self.args.tianji_frame

        rw = self.lookup_rot_pos(chest, wrist)
        if rw is None:
            self.get_logger().error(f"{LOG_TAG} missing TF {chest} -> {wrist}")
            return
        rot, pos = rw
        self._log_axes_and_rpy(chest, wrist, rot, pos, header="snapshot")

        wt = self.lookup_rot_pos(wrist, tianji)
        if wt is not None:
            t_rot, t_pos = wt
            self.get_logger().info(
                f"{LOG_TAG} static {wrist}->{tianji} "
                f"trans_m={[round(float(v), 4) for v in t_pos]} "
                f"quat_xyzw={[round(float(v), 4) for v in R.from_matrix(t_rot).as_quat()]}"
            )
            self._log_rpy_labels(f"{wrist}->{tianji}", t_rot)
        else:
            self.get_logger().warn(f"{LOG_TAG} missing TF {wrist} -> {tianji}")

        tr = self.lookup_rot_pos(chest, tianji)
        if tr is not None:
            self._log_axes_and_rpy(chest, tianji, tr[0], tr[1], header="tianji_in_chest")

    def _log_rpy_labels(self, label: str, rot: np.ndarray) -> None:
        # Intrinsic (body-fixed) Euler in parent frame — common RViz mental model.
        xyz = R.from_matrix(rot).as_euler("xyz", degrees=True)
        zyx = R.from_matrix(rot).as_euler("ZYX", degrees=True)
        self.get_logger().info(
            f"{LOG_TAG} {label} intrinsic_xyz_deg "
            f"(about_local_X,about_local_Y,about_local_Z)="
            f"[{xyz[0]:.2f}, {xyz[1]:.2f}, {xyz[2]:.2f}]"
        )
        self.get_logger().info(
            f"{LOG_TAG} {label} intrinsic_ZYX_deg "
            f"(about_local_Z,about_local_Y,about_local_X)="
            f"[{zyx[0]:.2f}, {zyx[1]:.2f}, {zyx[2]:.2f}]"
        )

    def _log_axes_and_rpy(
        self,
        parent: str,
        child: str,
        rot: np.ndarray,
        pos: np.ndarray,
        header: str,
    ) -> None:
        x_axis = rot[:, 0]
        y_axis = rot[:, 1]
        z_axis = rot[:, 2]
        self.get_logger().info(
            f"{LOG_TAG} {header} {parent}->{child} pos_m="
            f"[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]"
        )
        self.get_logger().info(
            f"{LOG_TAG} {header} RViz_RGB_axes in {parent}: "
            f"X_red=[{x_axis[0]:.3f},{x_axis[1]:.3f},{x_axis[2]:.3f}] "
            f"Y_green=[{y_axis[0]:.3f},{y_axis[1]:.3f},{y_axis[2]:.3f}] "
            f"Z_blue=[{z_axis[0]:.3f},{z_axis[1]:.3f},{z_axis[2]:.3f}]"
        )
        self._log_rpy_labels(f"{parent}->{child}", rot)

    def capture_zero(self) -> None:
        rw = self.lookup_rot_pos(self.args.chest_frame, self.args.wrist_frame)
        if rw is None:
            raise RuntimeError("Cannot capture zero: wrist TF missing")
        self.zero_rot, self.zero_pos = rw
        self.get_logger().info(f"{LOG_TAG} ZERO captured — now move wrist slowly")

    def log_delta(self) -> None:
        rw = self.lookup_rot_pos(self.args.chest_frame, self.args.wrist_frame)
        if rw is None or self.zero_rot is None:
            return
        rot, pos = rw
        delta_rot = self.zero_rot.T @ rot
        angle_deg = float(np.degrees(R.from_matrix(delta_rot).magnitude()))
        if angle_deg < self.args.motion_threshold_deg:
            self.dominant_counts["·"] += 1
            return

        # Rotation vector expressed in wrist frame at zero (local).
        rotvec_local = R.from_matrix(delta_rot).as_rotvec()
        rotvec_local_deg = np.degrees(rotvec_local)
        abs_comp = np.abs(rotvec_local_deg)
        dominant = "XYZ"[int(np.argmax(abs_comp))]
        self.dominant_counts[dominant] += 1
        self.sample_count += 1

        labels = {
            "X": "绕 right_wrist 红色 X",
            "Y": "绕 right_wrist 绿色 Y",
            "Z": "绕 right_wrist 蓝色 Z",
        }
        self.get_logger().info(
            f"{LOG_TAG} motion #{self.sample_count} angle_deg={angle_deg:.2f} "
            f"local_rotvec_deg=[{rotvec_local_deg[0]:.2f}, {rotvec_local_deg[1]:.2f}, {rotvec_local_deg[2]:.2f}] "
            f"dominant={dominant} ({labels[dominant]})"
        )

        rotvec_chest = np.degrees(self.zero_rot @ rotvec_local)
        self.get_logger().info(
            f"{LOG_TAG} motion #{self.sample_count} chest_frame_rotvec_deg="
            f"[{rotvec_chest[0]:.2f}, {rotvec_chest[1]:.2f}, {rotvec_chest[2]:.2f}]"
        )

    def log_summary(self) -> None:
        total = sum(self.dominant_counts.values())
        self.get_logger().info(
            f"{LOG_TAG} SUMMARY samples_with_motion={self.sample_count} "
            f"dominant_axis_counts={self.dominant_counts} (total_ticks={total})"
        )
        self.get_logger().info(
            f"{LOG_TAG} 解读: dominant=Z 表示你主要在绕 RViz 里 right_wrist 的蓝轴转; "
            f"若你自称 roll 且多为 Z，则 roll≈绕腕 Z"
        )


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=["right", "left"], default="right")
    parser.add_argument("--chest-frame", default=None)
    parser.add_argument("--wrist-frame", default=None)
    parser.add_argument("--tianji-frame", default=None)
    parser.add_argument("--duration-sec", type=float, default=45.0)
    parser.add_argument("--rate-hz", type=float, default=2.0)
    parser.add_argument("--tf-timeout", type=float, default=30.0)
    parser.add_argument("--motion-threshold-deg", type=float, default=2.0)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv or sys.argv[1:])
    side = args.side
    args.chest_frame = args.chest_frame or f"{side}_chest"
    args.wrist_frame = args.wrist_frame or f"{side}_wrist"
    args.tianji_frame = args.tianji_frame or f"tianji_{side}"

    print(
        "\n=== wrist frame 诊断 ===\n"
        "1) 另开终端先起 tracker（read_only 即可）\n"
        "2) 本脚本会先打一帧静态轴向，再采 ZERO\n"
        "3) 中立 3–5 秒后，分别做：拧小臂(roll)、点头(pitch)、招手(yaw)\n"
        "4) 把终端里 [WRIST_FRAME_DEBUG] 行复制给助手\n"
    )

    rclpy.init()
    node = WristFrameLogger(args)
    try:
        if not node.wait_for_tf():
            node.get_logger().error(
                f"{LOG_TAG} timeout waiting for {args.chest_frame}->{args.wrist_frame}"
            )
            return 1

        node.log_static_snapshot()
        print("\n保持 right_wrist 中立，按 Enter 采 ZERO …")
        input()
        node.capture_zero()

        period = 1.0 / max(args.rate_hz, 0.5)
        end = time.monotonic() + max(args.duration_sec, 5.0)
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            node.log_delta()
            time.sleep(period)

        node.log_summary()
        return 0
    except KeyboardInterrupt:
        node.log_summary()
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
