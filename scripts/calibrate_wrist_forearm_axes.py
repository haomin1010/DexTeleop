#!/usr/bin/env python3
"""Calibrate wrist rotation axes relative to the forearm tracker.

Run this while openvr_input publishes TF. The script resolves the same relative
rotation used by tianji_arm_controller, derives the roll axis from the forearm
tracker-to-wrist geometry, asks for pitch/yaw wrist motions, and prints a
wrist_orientation_axis_basis block for tianji_output.yaml.
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
    container_script = "/workspace/DexProj/scripts/calibrate_wrist_forearm_axes.py"
    container_activate = "/workspace/DexProj/scripts/activate_dexproj_env.sh"

    if os.path.exists("/.dockerenv") or not os.path.isfile(ensure_script):
        return

    print(
        "[calibrate] ROS Python packages are not active; re-running inside DexProj Docker...",
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


class WristAxisCalibrator(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("wrist_axis_calibrator")
        self.args = args
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def lookup_matrix(self, parent: str, child: str) -> np.ndarray | None:
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
        mat = np.eye(4, dtype=np.float64)
        mat[:3, :3] = R.from_quat(quat).as_matrix()
        mat[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
        return mat

    def relative_rotation_for(self, parent_frame: str, wrist_frame: str) -> np.ndarray | None:
        arm = self.lookup_matrix(parent_frame, self.args.arm_frame)
        wrist = self.lookup_matrix(parent_frame, wrist_frame)
        if arm is None or wrist is None:
            return None
        return arm[:3, :3].T @ wrist[:3, :3]

    def relative_rotation(self) -> np.ndarray | None:
        return self.relative_rotation_for(self.args.chest_frame, self.args.wrist_frame)

    def forearm_axis_in_delta_frame(self, zero_rel: np.ndarray) -> np.ndarray:
        arm = self.lookup_matrix(self.args.chest_frame, self.args.arm_frame)
        wrist = self.lookup_matrix(self.args.chest_frame, self.args.wrist_frame)
        if arm is None or wrist is None:
            raise RuntimeError("Cannot compute forearm axis because the locked TF pair is unavailable.")
        direction_parent = wrist[:3, 3] - arm[:3, 3]
        norm = float(np.linalg.norm(direction_parent))
        if norm < 1e-5:
            raise RuntimeError(
                "Cannot compute forearm axis: arm and wrist tracker positions are nearly identical."
            )
        direction_arm = arm[:3, :3].T @ (direction_parent / norm)
        direction_delta = zero_rel.T @ direction_arm
        direction_delta /= np.linalg.norm(direction_delta)
        return direction_delta

    def _tf_summary(self) -> str:
        text = self.tf_buffer.all_frames_as_yaml()
        if not text.strip() or text.strip() == "[]":
            return (
                "(no TF frames received)\n"
                "Start the tracker/arm launch in another terminal first, for example:\n"
                "  scripts/ensure_docker_exec.sh -- bash -lc 'source /workspace/DexProj/scripts/activate_dexproj_env.sh && "
                "ros2 launch wuji_teleop_bringup wuji_teleop_arm.launch.py arm_input:=tracker read_only:=true "
                "feedback_handshake:=true'\n"
                "If it is already running, check that both terminals use the same ROS_DOMAIN_ID."
            )
        lines = text.splitlines()
        if len(lines) > 80:
            lines = lines[:80] + ["..."]
        return "\n".join(lines)

    def _available_frames(self) -> set[str]:
        frames = set()
        for line in self.tf_buffer.all_frames_as_yaml().splitlines():
            if line and not line.startswith(" ") and line.rstrip().endswith(":"):
                frames.add(line.rstrip()[:-1])
        return frames

    def _missing_tf_hint(self) -> str:
        frames = self._available_frames()
        if not frames:
            return ""

        hints = []
        if self.args.arm_frame in frames:
            hints.append(f"arm frame '{self.args.arm_frame}' is visible")
        else:
            hints.append(f"arm frame '{self.args.arm_frame}' is missing")

        visible_wrist = [frame for frame in self.args.wrist_candidates if frame in frames]
        if visible_wrist:
            hints.append(f"wrist candidate frame(s) visible: {visible_wrist}")
        else:
            hints.append(
                f"wrist frames {self.args.wrist_candidates} are missing; "
                "check the right_wrist tracker serial/power/SteamVR tracking"
            )

        if f"tianji_{self.args.side}" in frames and f"{self.args.side}_wrist" not in frames:
            hints.append(
                f"'tianji_{self.args.side}' static TF exists, but its parent "
                f"'{self.args.side}_wrist' is not being published"
            )

        return "\nDiagnostics:\n- " + "\n- ".join(hints)

    def wait_for_tf(self) -> np.ndarray:
        deadline = time.monotonic() + self.args.tf_timeout
        last_fallback: tuple[str, str, np.ndarray] | None = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            for parent_frame in self.args.parent_candidates:
                for wrist_frame in self.args.wrist_candidates:
                    rel = self.relative_rotation_for(parent_frame, wrist_frame)
                    if rel is not None:
                        if (
                            self.args.prefer_primary_tf
                            and not self.args.tf_pair_locked
                            and (parent_frame, wrist_frame)
                            != (self.args.parent_candidates[0], self.args.wrist_candidates[0])
                        ):
                            last_fallback = (parent_frame, wrist_frame, rel)
                            continue
                        self.args.chest_frame = parent_frame
                        self.args.wrist_frame = wrist_frame
                        if self.args.lock_resolved_tf:
                            self.args.parent_candidates = [parent_frame]
                            self.args.wrist_candidates = [wrist_frame]
                            self.args.tf_pair_locked = True
                        print(
                            f"Resolved TF: {parent_frame}->{self.args.arm_frame}, "
                            f"{parent_frame}->{wrist_frame}"
                        )
                        return rel
        if last_fallback is not None:
            parent_frame, wrist_frame, rel = last_fallback
            self.args.chest_frame = parent_frame
            self.args.wrist_frame = wrist_frame
            if self.args.lock_resolved_tf:
                self.args.parent_candidates = [parent_frame]
                self.args.wrist_candidates = [wrist_frame]
                self.args.tf_pair_locked = True
            print(
                f"Resolved TF fallback: {parent_frame}->{self.args.arm_frame}, "
                f"{parent_frame}->{wrist_frame}"
            )
            return rel
        raise RuntimeError(
            f"Timed out waiting for TF {self.args.chest_frame}->{self.args.arm_frame} "
            f"and {self.args.chest_frame}->{self.args.wrist_frame}\n\n"
            "Visible TF frames:\n"
            f"{self._tf_summary()}"
            f"{self._missing_tf_hint()}"
        )

    def sample_relative_rotations(self, duration_sec: float) -> list[np.ndarray]:
        samples: list[np.ndarray] = []
        period = 1.0 / max(self.args.sample_rate_hz, 1.0)
        end_time = time.monotonic() + duration_sec
        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=0.0)
            rel = self.relative_rotation()
            if rel is not None:
                samples.append(rel)
            time.sleep(period)
        return samples


def _fit_axis(zero_rel: np.ndarray, samples: list[np.ndarray], min_angle_deg: float) -> np.ndarray:
    rotvecs = []
    max_angle = 0.0
    zero_inv = zero_rel.T
    for rel in samples:
        delta = zero_inv @ rel
        rotvec = np.degrees(R.from_matrix(delta).as_rotvec())
        angle = float(np.linalg.norm(rotvec))
        max_angle = max(max_angle, angle)
        if angle >= min_angle_deg:
            rotvecs.append(rotvec)
    if len(rotvecs) < 8:
        raise RuntimeError(
            f"Not enough usable samples ({len(rotvecs)}). Max observed motion was {max_angle:.2f} deg; "
            "press Enter first, then move during the recording window. If you did move, lower "
            "--min-angle-deg or move that axis farther."
        )
    data = np.asarray(rotvecs, dtype=np.float64)
    _, _, vt = np.linalg.svd(data, full_matrices=False)
    axis = vt[0]
    if float(np.sum(data @ axis)) < 0.0:
        axis = -axis
    axis /= np.linalg.norm(axis)
    return axis


def _project_axis(raw_axis: np.ndarray, normal_axis: np.ndarray, name: str) -> np.ndarray | None:
    projected = raw_axis - float(np.dot(raw_axis, normal_axis)) * normal_axis
    norm = float(np.linalg.norm(projected))
    if norm < 0.15:
        print(
            f"Warning: {name} motion is mostly along the forearm roll axis; "
            "its perpendicular component is weak."
        )
        return None
    return projected / norm


def _basis_from_forearm_and_plane_axes(
    z_axis: np.ndarray,
    x_raw: np.ndarray | None,
    y_raw: np.ndarray | None,
) -> np.ndarray:
    z_axis = z_axis / np.linalg.norm(z_axis)
    x_axis = _project_axis(x_raw, z_axis, "PITCH/x") if x_raw is not None else None
    y_axis = _project_axis(y_raw, z_axis, "YAW/y") if y_raw is not None else None

    if x_axis is None and y_axis is None:
        raise RuntimeError(
            "Both pitch and yaw motions were parallel to the forearm axis. "
            "Cannot determine the wrist plane."
        )
    if x_axis is None:
        y_axis = y_axis / np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, z_axis)
        x_axis /= np.linalg.norm(x_axis)
    if y_axis is None:
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        y_axis /= np.linalg.norm(y_axis)

    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = y_axis - float(np.dot(y_axis, x_axis)) * x_axis
    y_axis = y_axis - float(np.dot(y_axis, z_axis)) * z_axis
    y_norm = float(np.linalg.norm(y_axis))
    if y_norm < 0.15:
        y_axis = np.cross(z_axis, x_axis)
    else:
        y_axis /= y_norm

    if float(np.dot(np.cross(x_axis, y_axis), z_axis)) < 0.0:
        y_axis = -y_axis

    basis = np.stack([x_axis, y_axis, z_axis], axis=1)
    return basis


def _orthonormalize_basis(axis_by_name: dict[str, np.ndarray]) -> np.ndarray:
    basis = np.stack([axis_by_name["x"], axis_by_name["y"], axis_by_name["z"]], axis=1)
    u, _, vt = np.linalg.svd(basis)
    ortho = u @ vt
    if np.linalg.det(ortho) < 0.0:
        ortho[:, 1] *= -1.0
    return ortho


def _validate_axis_separation(axis_by_name: dict[str, np.ndarray], max_abs_dot: float) -> None:
    names = ("x", "y", "z")
    problems = []
    print("\nAxis separation quality, abs(dot) should be near 0:")
    for i, a_name in enumerate(names):
        for b_name in names[i + 1:]:
            dot = float(abs(np.dot(axis_by_name[a_name], axis_by_name[b_name])))
            print(f"  |{a_name}.{b_name}| = {dot:.3f}")
            if dot > max_abs_dot:
                problems.append((a_name, b_name, dot))
    if problems:
        detail = ", ".join(f"{a}/{b}={dot:.3f}" for a, b, dot in problems)
        raise RuntimeError(
            "Calibration motions are not separated enough; fitted axes are nearly parallel "
            f"({detail}). This means those recorded motions looked like the same rotation "
            "to the wrist tracker. Return to neutral before each prompt and move only the "
            "requested wrist axis while keeping the forearm tracker still. For PITCH, bend "
            "the hand up/down at the wrist; do not twist the forearm. For ROLL, twist around "
            "the forearm axis."
        )


def _print_yaml_block(basis: np.ndarray) -> None:
    print("\nwrist_orientation_axis_basis:")
    for idx, name in enumerate(("x", "y", "z")):
        vals = ", ".join(f"{v:.8f}" for v in basis[:, idx])
        print(f"  {name}: [{vals}]")


def _parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=["right", "left"], default="right")
    parser.add_argument("--chest-frame", default=None)
    parser.add_argument("--arm-frame", default=None)
    parser.add_argument("--wrist-frame", default=None)
    parser.add_argument("--duration-sec", type=float, default=5.0)
    parser.add_argument("--sample-rate-hz", type=float, default=80.0)
    parser.add_argument("--min-angle-deg", type=float, default=3.0)
    parser.add_argument("--max-axis-dot", type=float, default=0.75)
    parser.add_argument("--tf-timeout", type=float, default=5.0)
    parser.add_argument(
        "--no-lock-resolved-tf",
        action="store_true",
        help="Allow each calibration phase to re-resolve TF frames. Normally this should stay locked.",
    )
    parser.add_argument(
        "--no-prefer-primary-tf",
        action="store_true",
        help="Do not wait for the primary frame pair before falling back to world/right_wrist.",
    )
    return parser.parse_args(argv)


def main(argv: list[str]) -> int:
    args = _parse_args(argv)
    explicit_chest_frame = args.chest_frame is not None
    explicit_wrist_frame = args.wrist_frame is not None
    args.chest_frame = args.chest_frame or f"{args.side}_chest"
    args.arm_frame = args.arm_frame or f"{args.side}_arm"
    args.wrist_frame = args.wrist_frame or f"tianji_{args.side}"
    args.parent_candidates = (
        [args.chest_frame]
        if explicit_chest_frame
        else [f"{args.side}_chest", "world", "chest"]
    )
    args.wrist_candidates = (
        [args.wrist_frame]
        if explicit_wrist_frame
        else [f"tianji_{args.side}", f"{args.side}_wrist"]
    )
    args.lock_resolved_tf = not args.no_lock_resolved_tf
    args.prefer_primary_tf = not args.no_prefer_primary_tf
    args.tf_pair_locked = False

    rclpy.init(args=None)
    node = WristAxisCalibrator(args)
    try:
        print(
            f"Looking for TF: {args.parent_candidates} -> {args.arm_frame}, "
            f"{args.parent_candidates} -> {args.wrist_candidates}"
        )
        if args.lock_resolved_tf:
            print("The first resolved TF pair will be locked for all calibration motions.")
        print("保持小臂和手腕中立位，然后按 Enter。")
        input()
        zero_rel = node.wait_for_tf()

        z_axis = node.forearm_axis_in_delta_frame(zero_rel)
        print(
            "Forearm roll axis from tracker geometry "
            f"(joint5/z): [{z_axis[0]:.4f}, {z_axis[1]:.4f}, {z_axis[2]:.4f}]"
        )

        axis_by_name: dict[str, np.ndarray] = {"z": z_axis}
        motions = [
            ("x", "PITCH / joint6：手腕上下屈伸，像手掌点头；不要旋转小臂"),
            ("y", "YAW / joint7：手腕左右摆，像招手；不要拧手腕"),
        ]
        for axis_name, prompt in motions:
            print(f"\n{prompt}")
            print("先回到中立位，小臂 tracker 尽量不动，然后按 Enter 采这一轴的 zero。")
            input()
            zero_rel = node.wait_for_tf()
            print(f"再按 Enter 开始录制；看到 Recording now... 后，只做这一轴动作 {args.duration_sec:.1f}s。")
            input()
            print("Recording now...")
            samples = node.sample_relative_rotations(args.duration_sec)
            axis = _fit_axis(zero_rel, samples, args.min_angle_deg)
            axis_by_name[axis_name] = axis
            print(f"fitted raw {axis_name}-axis: [{axis[0]:.4f}, {axis[1]:.4f}, {axis[2]:.4f}]")

        try:
            _validate_axis_separation(axis_by_name, args.max_axis_dot)
            basis = _orthonormalize_basis(axis_by_name)
        except RuntimeError as exc:
            print(f"\nMotion-axis fit warning: {exc}")
            print("Falling back to forearm-geometry constrained basis.")
            basis = _basis_from_forearm_and_plane_axes(
                axis_by_name["z"],
                axis_by_name.get("x"),
                axis_by_name.get("y"),
            )
        print("\nPaste this block into tianji_output.yaml:")
        _print_yaml_block(basis)
        print("\nIf a joint moves in the opposite direction, flip the sign of that axis row in the YAML block.")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
