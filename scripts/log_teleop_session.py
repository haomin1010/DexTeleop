#!/usr/bin/env python3
"""Record tracker TF + robot joint feedback/command for teleop diagnosis.

Does not change tianji_arm_controller. Run in a second terminal while launch runs.

Output: [TELEOP_SESSION] lines on stdout and optional CSV file.
"""
from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as R


def _reexec_in_docker_if_needed() -> None:
    script_path = os.path.realpath(__file__)
    root_dir = os.path.dirname(os.path.dirname(script_path))
    ensure_script = os.path.join(root_dir, "scripts", "ensure_docker_exec.sh")
    container_script = "/workspace/DexProj/scripts/log_teleop_session.py"
    container_activate = "/workspace/DexProj/scripts/activate_dexproj_env.sh"

    if os.path.exists("/.dockerenv") or not os.path.isfile(ensure_script):
        return

    print("[teleop_session] Re-running inside DexProj Docker...", file=sys.stderr)
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
    import rclpy
    import tf2_ros
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray
except ModuleNotFoundError as exc:
    if exc.name in {"rclpy", "tf2_ros"}:
        _reexec_in_docker_if_needed()
    raise


LOG_TAG = "[TELEOP_SESSION]"


class TeleopSessionLogger(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("teleop_session_logger")
        self.args = args
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.zero_wrist_rot: np.ndarray | None = None
        self.last_state: list[float] | None = None
        self.last_cmd: list[float] | None = None
        self.last_ee_pose: list[float] | None = None
        self.csv_writer = None
        self.csv_file = None
        if args.log_file:
            path = Path(args.log_file)
            path.parent.mkdir(parents=True, exist_ok=True)
            self.csv_file = path.open("w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                [
                    "t_mono",
                    "note",
                    "wrist_x_m",
                    "wrist_y_m",
                    "wrist_z_m",
                    "wrist_rx_deg",
                    "wrist_ry_deg",
                    "wrist_rz_deg",
                    "wrist_local_drx_deg",
                    "wrist_local_dry_deg",
                    "wrist_local_drz_deg",
                    "wrist_local_delta_deg",
                    "tianji_x_m",
                    "tianji_y_m",
                    "tianji_z_m",
                    "tianji_rx_deg",
                    "tianji_ry_deg",
                    "tianji_rz_deg",
                    "j1",
                    "j2",
                    "j3",
                    "j4",
                    "j5",
                    "j6",
                    "j7",
                    "cmd_j1",
                    "cmd_j2",
                    "cmd_j3",
                    "cmd_j4",
                    "cmd_j5",
                    "cmd_j6",
                    "cmd_j7",
                    "ee_x_m",
                    "ee_y_m",
                    "ee_z_m",
                    "ee_rx_deg",
                    "ee_ry_deg",
                    "ee_rz_deg",
                ]
            )

        side = args.side
        self.chest = args.chest_frame or f"{side}_chest"
        self.wrist = args.wrist_frame or f"{side}_wrist"
        self.tianji = args.tianji_frame or f"tianji_{side}"
        prefix = f"/tianji_arm/{side}"
        self.create_subscription(
            JointState,
            f"{prefix}/joint_state",
            self._on_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            JointState,
            f"{prefix}/joint_command",
            self._on_cmd,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float64MultiArray,
            f"{prefix}/{side}_ee_pose",
            self._on_ee_pose,
            qos_profile_sensor_data,
        )
        period = 1.0 / max(args.rate_hz, 0.2)
        self.create_timer(period, self._tick)

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.close()
        super().destroy_node()

    def _on_state(self, msg: JointState) -> None:
        if msg.position:
            self.last_state = [float(v) for v in msg.position[:7]]

    def _on_cmd(self, msg: JointState) -> None:
        if msg.position:
            self.last_cmd = [float(v) for v in msg.position[:7]]

    def _on_ee_pose(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= 6:
            self.last_ee_pose = [float(v) for v in msg.data[:6]]

    def lookup(self, parent: str, child: str) -> np.ndarray | None:
        try:
            tf = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ):
            return None
        t = tf.transform
        mat = np.eye(4, dtype=np.float64)
        mat[:3, :3] = R.from_quat(
            [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
        ).as_matrix()
        mat[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
        return mat

    @staticmethod
    def _rpy_intrinsic_xyz(rot: np.ndarray) -> np.ndarray:
        return R.from_matrix(rot).as_euler("xyz", degrees=True)

    def _tick(self) -> None:
        wrist_mat = self.lookup(self.chest, self.wrist)
        tianji_mat = self.lookup(self.chest, self.tianji)
        if wrist_mat is None:
            self.get_logger().warn(f"{LOG_TAG} missing TF {self.chest}->{self.wrist}")
            return

        wrist_rot = wrist_mat[:3, :3]
        wrist_pos = wrist_mat[:3, 3]
        wrist_rpy = self._rpy_intrinsic_xyz(wrist_rot)

        if self.zero_wrist_rot is None:
            self.zero_wrist_rot = wrist_rot.copy()
            self.get_logger().info(f"{LOG_TAG} ZERO wrist rotation captured")

        delta_rot = self.zero_wrist_rot.T @ wrist_rot
        local_rotvec = np.degrees(R.from_matrix(delta_rot).as_rotvec())
        delta_deg = float(np.degrees(R.from_matrix(delta_rot).magnitude()))

        tianji_pos = [float("nan")] * 3
        tianji_rpy = [float("nan")] * 3
        if tianji_mat is not None:
            tianji_pos = tianji_mat[:3, 3].tolist()
            tianji_rpy = self._rpy_intrinsic_xyz(tianji_mat[:3, :3]).tolist()

        state = self.last_state or [float("nan")] * 7
        cmd = self.last_cmd or [float("nan")] * 7
        ee = self.last_ee_pose or [float("nan")] * 6

        dominant = "XYZ"[int(np.argmax(np.abs(local_rotvec)))] if delta_deg >= 2.0 else "-"
        self.get_logger().info(
            f"{LOG_TAG} wrist_in_{self.chest} pos_m={[round(v, 4) for v in wrist_pos]} "
            f"rpy_xyz_deg={[round(v, 2) for v in wrist_rpy]} "
            f"local_rotvec_deg={[round(v, 2) for v in local_rotvec]} "
            f"delta_deg={delta_deg:.2f} dominant_axis={dominant}"
        )
        self.get_logger().info(
            f"{LOG_TAG} tianji_in_{self.chest} pos_m={[round(v, 4) for v in tianji_pos]} "
            f"rpy_xyz_deg={[round(v, 2) for v in tianji_rpy]}"
        )
        self.get_logger().info(
            f"{LOG_TAG} motor_feedback_deg={[round(v, 2) for v in state]} "
            f"joint_cmd_deg={[round(v, 2) for v in cmd]}"
        )
        if not all(np.isnan(v) for v in ee[:3]):
            self.get_logger().info(
                f"{LOG_TAG} ee_pose_topic mm_deg="
                f"[{ee[0]*1000:.1f}, {ee[1]*1000:.1f}, {ee[2]*1000:.1f}, "
                f"{ee[3]:.2f}, {ee[4]:.2f}, {ee[5]:.2f}]"
            )

        if self.csv_writer is not None:
            self.csv_writer.writerow(
                [
                    f"{time.monotonic():.3f}",
                    "",
                    *wrist_pos,
                    *wrist_rpy,
                    *local_rotvec,
                    delta_deg,
                    *tianji_pos,
                    *tianji_rpy,
                    *state,
                    *cmd,
                    *ee,
                ]
            )
            self.csv_file.flush()

    def log_note(self, note: str) -> None:
        self.get_logger().info(f"{LOG_TAG} NOTE {note}")
        if self.csv_writer is not None:
            self.csv_writer.writerow([f"{time.monotonic():.3f}", note] + [""] * 28)
            self.csv_file.flush()


def _parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--side", choices=["right", "left"], default="right")
    p.add_argument("--chest-frame", default=None)
    p.add_argument("--wrist-frame", default=None)
    p.add_argument("--tianji-frame", default=None)
    p.add_argument("--rate-hz", type=float, default=2.0)
    p.add_argument(
        "--log-file",
        default="",
        help="Optional CSV path, e.g. /workspace/DexProj/logs/teleop_session.csv",
    )
    p.add_argument("--duration-sec", type=float, default=0.0, help="0 = run until Ctrl+C")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    _reexec_in_docker_if_needed()
    args = _parse_args(argv or sys.argv[1:])

    print(
        "\n=== 遥操作会话记录（不改控制器）===\n"
        "与 launch 同时跑。建议动作顺序（每步 5–8 秒，并在此终端输入标记）：\n"
        "  1) 中立\n"
        "  2) 只向前伸手（胸部 +X）\n"
        "  3) 只绕 right_wrist 蓝轴(Z) 转 — 你认为的 roll\n"
        "  4) 只绕绿轴(Y) — pitch\n"
        "  5) 只绕红轴(X) — yaw\n"
        "输入格式: note 中立 / note roll正 等，回车写入日志\n"
    )

    rclpy.init()
    node = TeleopSessionLogger(args)
    end = time.monotonic() + args.duration_sec if args.duration_sec > 0 else None

    try:
        import select

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline().strip()
                if line:
                    node.log_note(line)
            if end is not None and time.monotonic() >= end:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
