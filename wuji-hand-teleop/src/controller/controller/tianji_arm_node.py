"""Tianji arm controller node

Two control modes:
1. TELEOP mode: TF → IK → robot; publishes /tianji_arm/*/joint_command (output only)
2. INFERENCE mode: subscribes /tianji_arm/*/joint_command_in → robot (no self-loop)

Mode switching service: /tianji_arm/switch_mode
"""
from __future__ import annotations

import argparse
import contextlib
import os
import select
import sys
import termios
import threading
import time
import tty
from pathlib import Path
from typing import Optional, Set

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformListener
import tf2_ros
from scipy.spatial.transform import Rotation as R, Slerp
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool, Trigger

from tianji_output import TianjiArmController
from .common import (
    ControlMode,
    ROS2LoggerAdapter,
    get_default_qos,
    load_yaml_config,
    get_package_config_path,
)

# Topic names
LEFT_ARM_CMD_TOPIC = "/tianji_arm/left/joint_command"
RIGHT_ARM_CMD_TOPIC = "/tianji_arm/right/joint_command"
LEFT_ARM_CMD_IN_TOPIC = "/tianji_arm/left/joint_command_in"
RIGHT_ARM_CMD_IN_TOPIC = "/tianji_arm/right/joint_command_in"
LEFT_ARM_STATE_TOPIC = "/tianji_arm/left/joint_state"
RIGHT_ARM_STATE_TOPIC = "/tianji_arm/right/joint_state"
TELEOP_START_TF_WAIT_SEC = 5.0


class TianjiArmControllerNode(Node):
    """Tianji arm controller node"""

    def __init__(
        self,
        robot_ip: str = '192.168.1.190',
        dry_run: bool = False,
        read_only: bool = False,
        feedback_handshake: bool = False,
        read_only_connect_timeout: float = 3.0,
        tracker_mode: str = "absolute",
        tracker_position_scale: float = 1.0,
        tracker_position_scale_x: float = 1.0,
        tracker_position_scale_y: float = 1.0,
        tracker_position_scale_z: float = 1.0,
        tracker_position_limit_m: float = 0.0,
        tracker_position_min_x_m: Optional[float] = None,
        tracker_position_max_x_m: Optional[float] = None,
        tracker_position_min_y_m: Optional[float] = None,
        tracker_position_max_y_m: Optional[float] = None,
        tracker_position_min_z_m: Optional[float] = None,
        tracker_position_max_z_m: Optional[float] = None,
        control_rate_hz: float = 30.0,
        tracker_orientation_mode: str = "wrist_only",
        tracker_orientation_blend: float = 1.0,
        tracker_orientation_filter_alpha: float = 0.35,
        tracker_orientation_max_step_deg: float = 25.0,
        tracker_orientation_input_mode: str = "mapped_tf",
        tracker_orientation_source_frame_right: str = "right_wrist",
        tracker_orientation_source_frame_left: str = "left_wrist",
        tracker_orientation_map_mode: str = "identity",
        tracker_orientation_map_matrix: Optional[list] = None,
        tracker_orientation_map_mirror_left: bool = False,
        tracker_orientation_debug: bool = False,
        tracker_wrist_local_enable: bool = False,
        tracker_wrist_local_hold_position: bool = False,
        tracker_wrist_local_hold_position_mode: str = "last_target",
        tracker_wrist_local_debug: bool = False,
        tracker_wrist_local_pos_threshold_m: float = 0.025,
        tracker_wrist_local_rot_threshold_deg: float = 5.0,
        tracker_wrist_local_position_blend_enable: bool = False,
        tracker_wrist_local_weight_filter_alpha: float = 0.85,
        tracker_wrist_local_pos_start_m: float = 0.015,
        tracker_wrist_local_pos_full_m: float = 0.055,
        tracker_wrist_local_rot_start_deg: float = 3.0,
        tracker_wrist_local_rot_full_deg: float = 12.0,
        wrist_orientation_scale: float = 1.0,
        wrist_orientation_scale_x: float = 1.0,
        wrist_orientation_scale_y: float = 1.0,
        wrist_orientation_scale_z: float = 1.0,
        wrist_orientation_axis_order: str = "zxy",
        wrist_orientation_decompose: str = "rotvec",
        wrist_orientation_axis_basis: Optional[dict] = None,
        wrist_orientation_max_deg: float = 60.0,
        ik_reference_mode: str = "last_success",
        ik_subprocess_isolate: bool = False,
        ik_subprocess_timeout_sec: float = 2.0,
        ik_subprocess_ready_timeout_sec: float = 45.0,
        ik_subprocess_max_rate_hz: float = 25.0,
        ik_subprocess_max_branch_delta_deg: float = 25.0,
        tracker_start_delay_sec: float = 0.0,
        use_pinocchio_ik: bool = False,
        pinocchio_urdf_path: str = "",
        pinocchio_right_ee_frame: str = "tool0",
        pinocchio_left_ee_frame: str = "tool0",
        motor_unit: str = "deg",
        workspace_min_x_m: float = 0.20,
        workspace_max_x_m: float = 0.75,
        workspace_min_y_m: float = -0.45,
        workspace_max_y_m: float = 0.45,
        workspace_min_z_m: float = 0.05,
        workspace_max_z_m: float = 0.70,
        workspace_min_radius_m: float = 0.22,
        workspace_max_radius_m: float = 0.80,
        ik_position_weight: float = 1.0,
        ik_orientation_weight: float = 0.25,
        ik_orientation_weight_near_singularity: float = 0.05,
        ik_base_damping: float = 1e-4,
        ik_max_damping: float = 5e-2,
        ik_max_iters: int = 30,
        ik_dt: float = 0.5,
        ik_pos_eps_m: float = 0.005,
        ik_ori_eps_rad: float = 0.08,
        ik_max_dq_step_deg: float = 3.0,
        singularity_sigma_min_warn: float = 0.04,
        singularity_sigma_min_critical: float = 0.015,
        singularity_condition_warn: float = 80.0,
        singularity_condition_critical: float = 200.0,
        max_joint_step_deg: float = 8.0,
        pinocchio_j2_axis_constraint_enable: bool = False,
        pinocchio_j2_axis_constraint_hard: bool = False,
        pinocchio_j2_axis_constraint_weight: float = 0.0,
        pinocchio_j2_axis_constraint_gain: float = 1.0,
        pinocchio_j2_axis_constraint_joint_index: int = 1,
        pinocchio_j2_axis_constraint_max_delta_deg: float = 60.0,
        pinocchio_j2_axis_constraint_max_step_deg: float = 8.0,
        ik_frame_scan_debug: bool = False,
        keyboard_teleop_gate: bool = False,
        keyboard_start_key: str = "B",
        keyboard_stop_key: str = "E",
        keyboard_start_align_sec: float = 3.0,
        keyboard_zero_warmup_cycles: int = 0,
        teleop_ik_grace_cycles: int = 0,
        keyboard_align_max_ik_delta_deg: float = 20.0,
        teleop_ik_max_step_deg: float = 10.0,
        init_move_sides: str = "both",
        init_move_duration_sec: float = 3.0,
        teleop_active_sides: str = "right",
    ):
        super().__init__("tianji_arm_controller")

        self._mode = ControlMode.INFERENCE
        self._logger_adapter = ROS2LoggerAdapter(self.get_logger())
        self._log_counter = 0
        self._dry_run = dry_run
        self._read_only = read_only
        self._feedback_handshake = feedback_handshake
        self._read_only_connect_timeout = read_only_connect_timeout
        self._tracker_mode = tracker_mode
        self._tracker_position_scale = float(tracker_position_scale)
        self._tracker_position_scale_vec = np.array(
            [
                self._tracker_position_scale * float(tracker_position_scale_x),
                self._tracker_position_scale * float(tracker_position_scale_y),
                self._tracker_position_scale * float(tracker_position_scale_z),
            ],
            dtype=np.float64,
        )
        self._tracker_position_limit_m = max(float(tracker_position_limit_m), 0.0)
        self._tracker_position_min_m = np.array(
            [
                -np.inf if tracker_position_min_x_m is None else float(tracker_position_min_x_m),
                -np.inf if tracker_position_min_y_m is None else float(tracker_position_min_y_m),
                -np.inf if tracker_position_min_z_m is None else float(tracker_position_min_z_m),
            ],
            dtype=np.float64,
        )
        self._tracker_position_max_m = np.array(
            [
                np.inf if tracker_position_max_x_m is None else float(tracker_position_max_x_m),
                np.inf if tracker_position_max_y_m is None else float(tracker_position_max_y_m),
                np.inf if tracker_position_max_z_m is None else float(tracker_position_max_z_m),
            ],
            dtype=np.float64,
        )
        self._control_rate_hz = max(float(control_rate_hz), 1.0)
        self._tracker_orientation_mode = tracker_orientation_mode
        self._tracker_orientation_blend = min(max(float(tracker_orientation_blend), 0.0), 1.0)
        self._tracker_orientation_filter_alpha = min(max(float(tracker_orientation_filter_alpha), 0.0), 1.0)
        self._tracker_orientation_max_step_deg = max(float(tracker_orientation_max_step_deg), 0.0)
        self._tracker_orientation_input_mode = str(tracker_orientation_input_mode).lower()
        if self._tracker_orientation_input_mode not in ("mapped_tf", "raw_wrist"):
            raise ValueError("tracker_orientation_input_mode must be 'mapped_tf' or 'raw_wrist'")
        self._tracker_orientation_source_frames = {
            "left": str(tracker_orientation_source_frame_left),
            "right": str(tracker_orientation_source_frame_right),
        }
        self._tracker_orientation_map_mode = str(tracker_orientation_map_mode).lower()
        self._tracker_orientation_debug = bool(tracker_orientation_debug)
        self._tracker_wrist_local_enable = bool(tracker_wrist_local_enable)
        self._tracker_wrist_local_hold_position = bool(tracker_wrist_local_hold_position)
        self._tracker_wrist_local_hold_position_mode = str(tracker_wrist_local_hold_position_mode).lower()
        if self._tracker_wrist_local_hold_position_mode not in ("last_target", "robot_zero"):
            raise ValueError("tracker_wrist_local_hold_position_mode must be 'last_target' or 'robot_zero'")
        self._tracker_wrist_local_debug = bool(tracker_wrist_local_debug)
        self._tracker_wrist_local_pos_threshold_m = float(tracker_wrist_local_pos_threshold_m)
        self._tracker_wrist_local_rot_threshold_deg = float(tracker_wrist_local_rot_threshold_deg)
        self._tracker_wrist_local_position_blend_enable = bool(
            tracker_wrist_local_position_blend_enable
        )
        self._tracker_wrist_local_weight_filter_alpha = float(
            tracker_wrist_local_weight_filter_alpha
        )
        self._tracker_wrist_local_pos_start_m = float(tracker_wrist_local_pos_start_m)
        self._tracker_wrist_local_pos_full_m = float(tracker_wrist_local_pos_full_m)
        self._tracker_wrist_local_rot_start_deg = float(tracker_wrist_local_rot_start_deg)
        self._tracker_wrist_local_rot_full_deg = float(tracker_wrist_local_rot_full_deg)
        if self._tracker_wrist_local_pos_full_m <= self._tracker_wrist_local_pos_start_m:
            raise ValueError(
                "tracker_wrist_local_pos_full_m must be greater than tracker_wrist_local_pos_start_m"
            )
        if self._tracker_wrist_local_rot_full_deg <= self._tracker_wrist_local_rot_start_deg:
            raise ValueError(
                "tracker_wrist_local_rot_full_deg must be greater than tracker_wrist_local_rot_start_deg"
            )
        if not (0.0 <= self._tracker_wrist_local_weight_filter_alpha <= 0.99):
            raise ValueError(
                "tracker_wrist_local_weight_filter_alpha must be in [0.0, 0.99]"
            )
        self._wrist_orientation_scale = float(wrist_orientation_scale)
        self._wrist_orientation_axis_scale = {
            "x": float(wrist_orientation_scale_x),
            "y": float(wrist_orientation_scale_y),
            "z": float(wrist_orientation_scale_z),
        }
        self._wrist_orientation_axis_order = self._parse_wrist_axis_order(wrist_orientation_axis_order)
        self._wrist_orientation_decompose = self._parse_wrist_decompose(wrist_orientation_decompose)
        self._wrist_orientation_axis_basis = self._parse_wrist_axis_basis(wrist_orientation_axis_basis)
        self._wrist_orientation_max_deg = max(float(wrist_orientation_max_deg), 0.0)
        self._ik_reference_mode = ik_reference_mode
        self._ik_subprocess_isolate = bool(ik_subprocess_isolate)
        self._ik_subprocess_timeout_sec = max(float(ik_subprocess_timeout_sec), 0.1)
        self._ik_subprocess_ready_timeout_sec = max(float(ik_subprocess_ready_timeout_sec), 1.0)
        self._ik_subprocess_max_rate_hz = max(float(ik_subprocess_max_rate_hz), 0.0)
        self._ik_subprocess_max_branch_delta_deg = max(
            float(ik_subprocess_max_branch_delta_deg), 0.0
        )
        self._tracker_start_delay_sec = max(float(tracker_start_delay_sec), 0.0)
        self._use_pinocchio_ik = bool(use_pinocchio_ik)
        self._pinocchio_urdf_path = str(pinocchio_urdf_path or "")
        self._pinocchio_ee_frames = {
            "left": str(pinocchio_left_ee_frame or "tool0"),
            "right": str(pinocchio_right_ee_frame or "tool0"),
        }
        self._motor_unit = str(motor_unit).lower()
        self._workspace_min_m = np.array([workspace_min_x_m, workspace_min_y_m, workspace_min_z_m], dtype=np.float64)
        self._workspace_max_m = np.array([workspace_max_x_m, workspace_max_y_m, workspace_max_z_m], dtype=np.float64)
        self._workspace_min_radius_m = max(float(workspace_min_radius_m), 0.0)
        self._workspace_max_radius_m = max(float(workspace_max_radius_m), 0.0)
        self._ik_params = {
            "max_iters": int(ik_max_iters),
            "dt": float(ik_dt),
            "pos_eps": float(ik_pos_eps_m),
            "ori_eps": float(ik_ori_eps_rad),
            "position_weight": float(ik_position_weight),
            "orientation_weight": float(ik_orientation_weight),
            "orientation_weight_near_singularity": float(ik_orientation_weight_near_singularity),
            "base_damping": float(ik_base_damping),
            "max_damping": float(ik_max_damping),
            "sigma_min_warn": float(singularity_sigma_min_warn),
            "sigma_min_critical": float(singularity_sigma_min_critical),
            "condition_warn": float(singularity_condition_warn),
            "condition_critical": float(singularity_condition_critical),
            "max_dq_step_deg": float(ik_max_dq_step_deg),
        }
        self._singularity_sigma_min_warn = float(singularity_sigma_min_warn)
        self._singularity_sigma_min_critical = float(singularity_sigma_min_critical)
        self._singularity_condition_warn = float(singularity_condition_warn)
        self._singularity_condition_critical = float(singularity_condition_critical)
        self._max_joint_step_deg = max(float(max_joint_step_deg), 0.0)
        self._pin_j2_axis_constraint_enable = bool(pinocchio_j2_axis_constraint_enable)
        self._pin_j2_axis_constraint_hard = bool(pinocchio_j2_axis_constraint_hard)
        self._pin_j2_axis_constraint_weight = max(float(pinocchio_j2_axis_constraint_weight), 0.0)
        self._pin_j2_axis_constraint_gain = float(pinocchio_j2_axis_constraint_gain)
        self._pin_j2_axis_constraint_joint_index = int(pinocchio_j2_axis_constraint_joint_index)
        self._pin_j2_axis_constraint_max_delta_deg = max(
            float(pinocchio_j2_axis_constraint_max_delta_deg), 0.0
        )
        self._pin_j2_axis_constraint_max_step_deg = max(
            float(pinocchio_j2_axis_constraint_max_step_deg), 0.0
        )
        if self._pin_j2_axis_constraint_joint_index < 0:
            raise ValueError("pinocchio_j2_axis_constraint_joint_index must be >= 0")
        self._ik_frame_scan_debug = bool(ik_frame_scan_debug)
        self._keyboard_teleop_gate = bool(keyboard_teleop_gate)
        self._keyboard_start_key = str(keyboard_start_key).strip().upper() or "B"
        self._keyboard_stop_key = str(keyboard_stop_key).strip().upper() or "E"
        self._keyboard_start_align_sec = max(float(keyboard_start_align_sec), 0.0)
        self._keyboard_zero_warmup_cycles = max(int(keyboard_zero_warmup_cycles), 0)
        self._teleop_ik_grace_cycles = max(int(teleop_ik_grace_cycles), 0)
        self._keyboard_align_max_ik_delta_deg = max(float(keyboard_align_max_ik_delta_deg), 0.0)
        self._teleop_ik_max_step_deg = max(float(teleop_ik_max_step_deg), 0.0)
        self._teleop_zero_warmup_remaining = 0
        self._teleop_ik_grace_remaining = 0
        init_move_sides = str(init_move_sides).lower()
        if init_move_sides not in ("both", "left", "right"):
            raise ValueError("init_move_sides must be 'both', 'left', or 'right'")
        self._init_move_sides = init_move_sides
        self._init_move_duration_sec = max(float(init_move_duration_sec), 0.1)
        self._teleop_active_sides = self._parse_teleop_active_sides(teleop_active_sides)
        self._teleop_armed = False
        self._align_in_progress = False
        self._keyboard_lock = threading.Lock()
        self._pending_keyboard_start = False
        self._pending_keyboard_stop = False
        self._keyboard_stop_event = threading.Event()
        self._keyboard_thread: Optional[threading.Thread] = None
        self._keyboard_fd: Optional[int] = None
        self._start_time = time.monotonic()
        self._logged_tracker_delay = False
        self._tracker_zero = {"left": None, "right": None}
        self._tracker_ori_zero = {"left": None, "right": None}
        self._robot_zero = {"left": None, "right": None}
        self._filtered_orientation = {"left": None, "right": None}
        self._wrist_orientation_delta = {"left": None, "right": None}
        self._wrist_joint_zero = {"left": None, "right": None}
        self._wrist_relative_zero = {"left": None, "right": None}
        self._last_wrist_debug_log_time = {"left": 0.0, "right": 0.0}
        self._last_workspace_clamp_log_time = {"left": 0.0, "right": 0.0}
        self._last_joint_step_log_time = {"left": 0.0, "right": 0.0}
        self._last_ik_diag_log_time = {"left": 0.0, "right": 0.0}
        self._last_ik_singularity_log_time = {"left": 0.0, "right": 0.0}
        self._last_target_frame_log_time = {"left": 0.0, "right": 0.0}
        self._last_tracker_ori_log_time = {"left": 0.0, "right": 0.0}
        self._last_tracker_ori_source_log_time = {"left": 0.0, "right": 0.0}
        self._wrist_local_zero = {"left": None, "right": None}
        self._last_wrist_local_log_time = {"left": 0.0, "right": 0.0}
        self._last_target_pose = {"left": None, "right": None}
        self._last_wrist_local_hold_log_time = {"left": 0.0, "right": 0.0}
        self._last_wrist_local_state = {"left": False, "right": False}
        self._wrist_local_weight = {"left": 0.0, "right": 0.0}
        self._last_wrist_local_blend_log_time = {"left": 0.0, "right": 0.0}
        self._arm_axis_zero_xy = {"left": None, "right": None}
        self._pin_j2_zero = {"left": None, "right": None}
        self._pin_j2_target_last = {"left": None, "right": None}
        self._last_pin_j2_axis_log_time = {"left": 0.0, "right": 0.0}
        self._last_command_joints = {"left": None, "right": None}
        self._last_success_joints = {"left": None, "right": None}
        self._last_step_limit_hold_joints = {"left": None, "right": None}
        self._pin_ik = {"left": None, "right": None}
        self._pin_target_offset = {"left": None, "right": None}
        self._pin_fk_alignment_checked = {"left": False, "right": False}
        self._pinocchio_urdf_resolved = None
        self._tracker_orientation_map_mirror_left = bool(tracker_orientation_map_mirror_left)
        self._tracker_orientation_map = self._parse_tracker_orientation_map(
            self._tracker_orientation_map_mode,
            tracker_orientation_map_matrix,
        )
        mirror_y = np.diag([1.0, -1.0, 1.0])
        left_map = (
            mirror_y @ self._tracker_orientation_map @ mirror_y
            if self._tracker_orientation_map_mirror_left
            else self._tracker_orientation_map
        )
        self._tracker_orientation_maps = {
            "right": self._tracker_orientation_map,
            "left": left_map,
        }

        # Initialize controller
        if self._read_only:
            self.get_logger().warn("READ ONLY enabled: publishing robot feedback without sending commands")
            if self._feedback_handshake:
                self.get_logger().warn("READ ONLY feedback handshake enabled for robot joint feedback")
        elif self._dry_run:
            self.get_logger().warn("DRY RUN enabled: printing arm commands without driving robot")
        else:
            self.get_logger().info(f"Connecting to robot {robot_ip}...")
        self.controller = TianjiArmController(
            robot_ip=robot_ip,
            logger=self._logger_adapter,
            dry_run=self._dry_run,
            read_only=self._read_only,
            feedback_handshake=self._feedback_handshake,
            read_only_connect_timeout=self._read_only_connect_timeout,
            prefer_last_ik_reference=self._ik_reference_mode == "last_success",
            ik_subprocess_isolate=self._ik_subprocess_isolate,
            ik_subprocess_timeout_sec=self._ik_subprocess_timeout_sec,
            ik_subprocess_ready_timeout_sec=self._ik_subprocess_ready_timeout_sec,
            ik_subprocess_max_rate_hz=self._ik_subprocess_max_rate_hz,
            ik_subprocess_max_branch_delta_deg=self._ik_subprocess_max_branch_delta_deg,
        )
        if not self._read_only:
            self.controller.set_impedance_mode(mode='joint')

        if self._use_pinocchio_ik:
            self._init_pinocchio_ik()

        # Move to initial position (session runner arms teleop via start_teleop service).
        if self._keyboard_teleop_gate:
            if self._read_only:
                self.get_logger().info("Read only: keeping robot at its current physical joint angles")
            elif self._dry_run:
                self.get_logger().info("Dry run: using configured initial joints as IK reference")
                self.controller.move_to_init(wait=False, timeout=0)
            else:
                self.get_logger().info(
                    f"Moving arm to initial position (sides={self._init_move_sides}, "
                    f"{self._init_move_duration_sec:.1f}s)..."
                )
                self.controller.move_to_init(
                    wait=True,
                    timeout=1,
                    duration=self._init_move_duration_sec,
                    sides=self._init_move_sides,
                )
        else:
            self.get_logger().info(
                "Session teleop gate: holding current pose until /tianji_arm/start_teleop"
            )
        if self._keyboard_teleop_gate:
            self.get_logger().info(
                f"Keyboard teleop gate: press {self._keyboard_start_key} to start "
                f"(align {self._keyboard_start_align_sec:.1f}s), "
                f"{self._keyboard_stop_key} to stop"
            )
            self._start_keyboard_listener()
        self.get_logger().info("Controller initialization complete")
        self.get_logger().info(f"Tracker pose mode: {self._tracker_mode}")
        self.get_logger().info(
            f"Control rate: {self._control_rate_hz:.1f} Hz, "
            f"tracker position scale: {self._tracker_position_scale:.2f}, "
            f"tracker axis scale xyz: {self._tracker_position_scale_vec.tolist()}, "
            f"tracker position limit: {self._tracker_position_limit_m:.2f}m, "
            f"tracker position min xyz: {self._tracker_position_min_m.tolist()}, "
            f"tracker position max xyz: {self._tracker_position_max_m.tolist()}, "
            f"tracker orientation mode: {self._tracker_orientation_mode}, "
            f"tracker orientation blend: {self._tracker_orientation_blend:.2f}, "
            f"tracker orientation filter alpha: {self._tracker_orientation_filter_alpha:.2f}, "
            f"tracker orientation max step: {self._tracker_orientation_max_step_deg:.1f}deg, "
            f"tracker orientation input mode: {self._tracker_orientation_input_mode}, "
            f"tracker orientation source frames: {self._tracker_orientation_source_frames}, "
            f"tracker orientation map mode: {self._tracker_orientation_map_mode}, "
            f"tracker orientation debug: {self._tracker_orientation_debug}, "
            f"tracker wrist-local enable: {self._tracker_wrist_local_enable}, "
            f"tracker wrist-local hold position: {self._tracker_wrist_local_hold_position}, "
            f"tracker wrist-local hold mode: {self._tracker_wrist_local_hold_position_mode}, "
            f"tracker wrist-local debug: {self._tracker_wrist_local_debug}, "
            f"wrist-local pos threshold: {self._tracker_wrist_local_pos_threshold_m:.3f}m, "
            f"wrist-local rot threshold: {self._tracker_wrist_local_rot_threshold_deg:.1f}deg, "
            f"tracker wrist-local position blend enable: "
            f"{self._tracker_wrist_local_position_blend_enable}, "
            f"wrist-local weight filter alpha: "
            f"{self._tracker_wrist_local_weight_filter_alpha:.2f}, "
            f"wrist-local pos blend range: "
            f"start={self._tracker_wrist_local_pos_start_m:.3f}m "
            f"full={self._tracker_wrist_local_pos_full_m:.3f}m, "
            f"wrist-local rot blend range: "
            f"start={self._tracker_wrist_local_rot_start_deg:.1f}deg "
            f"full={self._tracker_wrist_local_rot_full_deg:.1f}deg, "
            f"wrist orientation axis order: {wrist_orientation_axis_order}, "
            f"wrist orientation scale: {self._wrist_orientation_scale:.2f}, "
            f"wrist orientation axis scale xyz: "
            f"{[self._wrist_orientation_axis_scale[axis] for axis in ('x', 'y', 'z')]}, "
            f"wrist orientation decompose: {self._wrist_orientation_decompose}, "
            f"wrist orientation calibrated basis: {self._wrist_orientation_axis_basis is not None}, "
            f"wrist orientation max: {self._wrist_orientation_max_deg:.1f}deg, "
            f"IK reference mode: {self._ik_reference_mode}, "
            f"IK subprocess isolate: {self._ik_subprocess_isolate}, "
            f"IK subprocess timeout: {self._ik_subprocess_timeout_sec:.2f}s, "
            f"IK subprocess max rate: {self._ik_subprocess_max_rate_hz:.1f} Hz, "
            f"IK subprocess branch reject: {self._ik_subprocess_max_branch_delta_deg:.1f}deg, "
            f"tracker start delay: {self._tracker_start_delay_sec:.1f}s, "
            f"use pinocchio IK: {self._use_pinocchio_ik}, "
            f"pin j2 axis constraint enable: {self._pin_j2_axis_constraint_enable}, "
            f"pin j2 axis hard: {self._pin_j2_axis_constraint_hard}, "
            f"pin j2 axis weight: {self._pin_j2_axis_constraint_weight:.3f}, "
            f"pin j2 axis gain: {self._pin_j2_axis_constraint_gain:.3f}, "
            f"pin j2 joint index: {self._pin_j2_axis_constraint_joint_index}, "
            f"pin j2 max delta: {self._pin_j2_axis_constraint_max_delta_deg:.1f}deg, "
            f"pin j2 max step: {self._pin_j2_axis_constraint_max_step_deg:.1f}deg, "
            f"keyboard teleop gate: {self._keyboard_teleop_gate}, "
            f"init move sides: {self._init_move_sides}, "
            f"teleop active sides: {sorted(self._teleop_active_sides)}"
        )

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Pose and joint cache
        self.left_pose = None
        self.right_pose = None
        self.left_pose_mat = None
        self.right_pose_mat = None
        self.left_pose_mat_m = None
        self.right_pose_mat_m = None
        self.left_arm_mat = None
        self.right_arm_mat = None
        self.left_y_axis = None
        self.right_y_axis = None
        self.left_inference_joints = None
        self.right_inference_joints = None

        qos = get_default_qos()
        self._service_callback_group = ReentrantCallbackGroup()
        self._control_callback_group = MutuallyExclusiveCallbackGroup()

        # Publishers
        self.left_cmd_pub = self.create_publisher(JointState, LEFT_ARM_CMD_TOPIC, qos)
        self.right_cmd_pub = self.create_publisher(JointState, RIGHT_ARM_CMD_TOPIC, qos)
        self.left_state_pub = self.create_publisher(JointState, LEFT_ARM_STATE_TOPIC, qos)
        self.right_state_pub = self.create_publisher(JointState, RIGHT_ARM_STATE_TOPIC, qos)

        # zsp_para and pose publishers
        self.left_zsp_para_pub = self.create_publisher(Float64MultiArray, '/tianji_arm/left/left_zsp_para', qos)
        self.right_zsp_para_pub = self.create_publisher(Float64MultiArray, '/tianji_arm/right/right_zsp_para', qos)
        self.left_ee_pose_pub = self.create_publisher(Float64MultiArray, '/tianji_arm/left/left_ee_pose', qos)
        self.right_ee_pose_pub = self.create_publisher(Float64MultiArray, '/tianji_arm/right/right_ee_pose', qos)

        # Inference input subscriptions are mode-gated: TELEOP does not subscribe.
        self.left_cmd_sub = None
        self.right_cmd_sub = None
        self._set_inference_subscriptions(self._mode == ControlMode.INFERENCE)

        # Services
        self.create_service(
            SetBool,
            '/tianji_arm/switch_mode',
            self._switch_mode_callback,
            callback_group=self._service_callback_group,
        )
        self.create_service(
            Trigger,
            '/tianji_arm/get_mode',
            self._get_mode_callback,
            callback_group=self._service_callback_group,
        )
        self.create_service(
            Trigger,
            '/tianji_arm/reset_tracker_zero',
            self._reset_tracker_zero_callback,
            callback_group=self._service_callback_group,
        )
        self.create_service(
            Trigger,
            '/tianji_arm/start_teleop',
            self._start_teleop_callback,
            callback_group=self._service_callback_group,
        )
        self.create_service(
            Trigger,
            '/tianji_arm/stop_teleop',
            self._stop_teleop_callback,
            callback_group=self._service_callback_group,
        )

        self.create_timer(
            1.0 / self._control_rate_hz,
            self._control_loop,
            callback_group=self._control_callback_group,
        )

        self.get_logger().info(
            f"Initialization complete, mode: {self._mode.value.upper()}. "
            f"TELEOP publishes {LEFT_ARM_CMD_TOPIC}, {RIGHT_ARM_CMD_TOPIC}; "
            f"INFERENCE listens on {LEFT_ARM_CMD_IN_TOPIC}, {RIGHT_ARM_CMD_IN_TOPIC}"
        )

    def _init_pinocchio_ik(self) -> None:
        self._pinocchio_urdf_resolved = self._resolve_pinocchio_urdf_path()
        self.get_logger().info(
            f"Using Pinocchio IK URDF: {self._pinocchio_urdf_resolved}. "
            "Solvers are initialized lazily per active side."
        )

    def _get_pinocchio_solver(self, side: str):
        solver = self._pin_ik.get(side)
        if solver is not None:
            return solver
        if self._pinocchio_urdf_resolved is None:
            self._init_pinocchio_ik()

        from .pinocchio_ik_solver import PinocchioIKSolver

        try:
            solver = PinocchioIKSolver(
                urdf_path=str(self._pinocchio_urdf_resolved),
                ee_frame_name=self._pinocchio_ee_frames[side],
                motor_unit=self._motor_unit,
                logger=self._logger_adapter,
                **self._ik_params,
            )
        except Exception as exc:
            raise RuntimeError(
                f"Failed to initialize Pinocchio IK for {side} with "
                f"ee_frame={self._pinocchio_ee_frames[side]!r}, "
                f"urdf={self._pinocchio_urdf_resolved}: {exc}"
            ) from exc
        self._pin_ik[side] = solver
        self._check_pinocchio_fk_alignment(side, solver)
        return solver

    def _resolve_pinocchio_urdf_path(self) -> Path:
        if self._pinocchio_urdf_path:
            path = Path(self._pinocchio_urdf_path).expanduser()
            if path.exists():
                return path.resolve()
            raise FileNotFoundError(f"Configured pinocchio_urdf_path does not exist: {path}")

        try:
            from ament_index_python.packages import get_package_share_directory
            package_path = Path(get_package_share_directory("tianji_urdf"))
            candidate = package_path / "urdf" / "right.urdf"
            if candidate.exists():
                return candidate.resolve()
        except Exception as exc:
            self.get_logger().warn(f"Could not resolve tianji_urdf package path: {exc}")

        local_candidate = Path(__file__).resolve().parents[2] / "tianji_urdf" / "urdf" / "right.urdf"
        if local_candidate.exists():
            return local_candidate.resolve()
        raise FileNotFoundError(
            "pinocchio_urdf_path is empty and tianji_urdf/urdf/right.urdf was not found"
        )

    @property
    def mode(self) -> ControlMode:
        return self._mode

    # -------------------- Service Callbacks --------------------

    def _switch_mode_callback(self, request: SetBool.Request, response: SetBool.Response):
        new_mode = ControlMode.INFERENCE if request.data else ControlMode.TELEOP
        if self._mode != new_mode:
            self._mode = new_mode
            self._set_inference_subscriptions(new_mode == ControlMode.INFERENCE)
            if new_mode == ControlMode.INFERENCE:
                self.get_logger().info(
                    f"Switched to {new_mode.value} mode: "
                    f"publish joint targets to {LEFT_ARM_CMD_IN_TOPIC}, {RIGHT_ARM_CMD_IN_TOPIC}"
                )
            else:
                self.get_logger().info(
                    f"Switched to {new_mode.value} mode: "
                    f"IK output on {LEFT_ARM_CMD_TOPIC}, {RIGHT_ARM_CMD_TOPIC}"
                )
        response.success = True
        response.message = f"Current mode: {new_mode.value}"
        return response

    def _set_inference_subscriptions(self, enabled: bool) -> None:
        """Create/destroy inference command subscribers based on controller mode."""
        qos = get_default_qos()
        if enabled:
            if self.left_cmd_sub is None:
                self.left_cmd_sub = self.create_subscription(
                    JointState, LEFT_ARM_CMD_IN_TOPIC, self._left_cmd_callback, qos
                )
            if self.right_cmd_sub is None:
                self.right_cmd_sub = self.create_subscription(
                    JointState, RIGHT_ARM_CMD_IN_TOPIC, self._right_cmd_callback, qos
                )
            return

        if self.left_cmd_sub is not None:
            self.destroy_subscription(self.left_cmd_sub)
            self.left_cmd_sub = None
        if self.right_cmd_sub is not None:
            self.destroy_subscription(self.right_cmd_sub)
            self.right_cmd_sub = None

    def _get_mode_callback(self, request: Trigger.Request, response: Trigger.Response):
        response.success = True
        response.message = self._mode.value
        return response

    def _reset_tracker_zero_callback(self, request: Trigger.Request, response: Trigger.Response):
        self._reset_tracker_state()
        response.success = True
        response.message = "Tracker zero will be reinitialized from the next valid TF"
        self.get_logger().info(response.message)
        return response

    def _start_teleop_callback(self, request: Trigger.Request, response: Trigger.Response):
        success, message = self._execute_keyboard_start()
        response.success = success
        response.message = message
        if success:
            self.get_logger().info(response.message)
        else:
            self.get_logger().warn(response.message)
        return response

    def _stop_teleop_callback(self, request: Trigger.Request, response: Trigger.Response):
        with self._keyboard_lock:
            self._pending_keyboard_stop = True
        response.success = True
        response.message = f"Teleop stop queued (same as {self._keyboard_stop_key})"
        self.get_logger().info(response.message)
        return response

    @staticmethod
    def _parse_teleop_active_sides(value: str) -> Set[str]:
        raw = str(value).strip().lower()
        if raw in ("both", "all"):
            return {"left", "right"}
        if raw in ("right", "left"):
            return {raw}
        raise ValueError("teleop_active_sides must be 'both', 'left', or 'right'")

    def _reset_tracker_state(self) -> None:
        self._tracker_zero = {"left": None, "right": None}
        self._tracker_ori_zero = {"left": None, "right": None}
        self._robot_zero = {"left": None, "right": None}
        self._filtered_orientation = {"left": None, "right": None}
        self._wrist_orientation_delta = {"left": None, "right": None}
        self._wrist_joint_zero = {"left": None, "right": None}
        self._wrist_relative_zero = {"left": None, "right": None}
        self._last_wrist_debug_log_time = {"left": 0.0, "right": 0.0}
        self._last_workspace_clamp_log_time = {"left": 0.0, "right": 0.0}
        self._last_joint_step_log_time = {"left": 0.0, "right": 0.0}
        self._last_ik_diag_log_time = {"left": 0.0, "right": 0.0}
        self._last_ik_singularity_log_time = {"left": 0.0, "right": 0.0}
        self._last_target_frame_log_time = {"left": 0.0, "right": 0.0}
        self._last_tracker_ori_log_time = {"left": 0.0, "right": 0.0}
        self._last_tracker_ori_source_log_time = {"left": 0.0, "right": 0.0}
        self._wrist_local_zero = {"left": None, "right": None}
        self._last_wrist_local_log_time = {"left": 0.0, "right": 0.0}
        self._last_target_pose = {"left": None, "right": None}
        self._last_wrist_local_hold_log_time = {"left": 0.0, "right": 0.0}
        self._last_wrist_local_state = {"left": False, "right": False}
        self._wrist_local_weight = {"left": 0.0, "right": 0.0}
        self._last_wrist_local_blend_log_time = {"left": 0.0, "right": 0.0}
        self._arm_axis_zero_xy = {"left": None, "right": None}
        self._pin_j2_zero = {"left": None, "right": None}
        self._pin_j2_target_last = {"left": None, "right": None}
        self._last_pin_j2_axis_log_time = {"left": 0.0, "right": 0.0}
        self._pin_target_offset = {"left": None, "right": None}
        self._last_step_limit_hold_joints = {"left": None, "right": None}
        self._pin_fk_alignment_checked = {"left": False, "right": False}
        self.left_pose = None
        self.right_pose = None
        self.left_pose_mat = None
        self.right_pose_mat = None
        self.left_pose_mat_m = None
        self.right_pose_mat_m = None

    def _resolve_keyboard_fd(self) -> Optional[int]:
        """Prefer /dev/tty so B/E work under ros2 launch (node stdin is often not a TTY)."""
        for path in ("/dev/tty",):
            try:
                fd = os.open(path, os.O_RDONLY)
                if os.isatty(fd):
                    return fd
                os.close(fd)
            except OSError:
                continue
        if sys.stdin.isatty():
            return sys.stdin.fileno()
        return None

    def _start_keyboard_listener(self) -> None:
        keyboard_fd = self._resolve_keyboard_fd()
        if keyboard_fd is None:
            self.get_logger().warn(
                "keyboard_teleop_gate: no TTY for B/E. Teleop stays disarmed until you call "
                "'ros2 service call /tianji_arm/start_teleop std_srvs/srv/Trigger' "
                "(stop: /tianji_arm/stop_teleop). Stand at INIT pose before start."
            )
            return
        self._keyboard_fd = keyboard_fd
        self.get_logger().info(
            f"Keyboard teleop gate listening on /dev/tty: "
            f"{self._keyboard_start_key}=start, {self._keyboard_stop_key}=stop"
        )
        self._keyboard_thread = threading.Thread(
            target=self._keyboard_listener_loop,
            name="tianji_keyboard_gate",
            daemon=True,
        )
        self._keyboard_thread.start()

    def _keyboard_listener_loop(self) -> None:
        fd = self._keyboard_fd
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while not self._keyboard_stop_event.is_set():
                ready, _, _ = select.select([fd], [], [], 0.2)
                if not ready:
                    continue
                value = os.read(fd, 1).decode("utf-8", errors="ignore")
                if not value:
                    continue
                key = value.upper()
                with self._keyboard_lock:
                    if key == self._keyboard_start_key:
                        self._pending_keyboard_start = True
                    elif key == self._keyboard_stop_key:
                        self._pending_keyboard_stop = True
        except Exception as exc:
            self.get_logger().error(f"Keyboard listener failed: {exc}")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            if fd != sys.stdin.fileno():
                try:
                    os.close(fd)
                except OSError:
                    pass

    def _sdk_cmd_sides(self) -> str:
        if self._teleop_active_sides == {"right"}:
            return "right"
        if self._teleop_active_sides == {"left"}:
            return "left"
        return "both"

    def _handle_pending_keyboard(self) -> None:
        start = False
        stop = False
        with self._keyboard_lock:
            if self._pending_keyboard_start:
                self._pending_keyboard_start = False
                start = True
            if self._pending_keyboard_stop:
                self._pending_keyboard_stop = False
                stop = True
        if stop:
            self._teleop_armed = False
            self._teleop_zero_warmup_remaining = 0
            self._teleop_ik_grace_remaining = 0
            self.get_logger().info(
                f"Teleop stopped ({self._keyboard_stop_key}). "
                f"Press {self._keyboard_start_key} to start again."
            )
        if start:
            self._execute_keyboard_start()

    @staticmethod
    def _joint_max_delta_deg(a: list, b: list) -> float:
        return max(abs(float(x) - float(y)) for x, y in zip(a, b))

    @staticmethod
    def _slew_joints_deg(ref: list, target: list, max_step_deg: float) -> list:
        """Move each joint toward target by at most max_step_deg (avoids one-shot SDK jumps)."""
        step = max(float(max_step_deg), 0.0)
        if step <= 0.0:
            return list(target)
        out: list[float] = []
        for r, t in zip(ref, target):
            delta = float(t) - float(r)
            if abs(delta) > step:
                delta = step if delta > 0.0 else -step
            out.append(float(r) + delta)
        return out

    def _align_side_target(
        self,
        side: str,
        current: list,
        ik_joints: Optional[list],
    ) -> tuple[list, bool]:
        align_target = list(current)
        use_ik_align = False
        if ik_joints is None or side not in self._teleop_active_sides:
            return align_target, use_ik_align
        delta = self._joint_max_delta_deg(current, ik_joints)
        self.get_logger().info(
            f"First IK {side} (deg): {[round(float(v), 2) for v in ik_joints]}, "
            f"max_delta_from_current={delta:.1f}"
        )
        if (
            self._keyboard_align_max_ik_delta_deg <= 0.0
            or delta <= self._keyboard_align_max_ik_delta_deg
        ):
            align_target = list(ik_joints)
            use_ik_align = True
        else:
            self.get_logger().warn(
                f"First IK {side} differs from current by {delta:.1f}deg "
                f"(limit {self._keyboard_align_max_ik_delta_deg:.1f}) at tracker zero. "
                "Holding calib joints for align (not bad IK branch). Fix static TF / wrist map."
            )
        return align_target, use_ik_align

    def _smooth_align_publish(
        self,
        left_target: list,
        right_target: list,
        start_left: list,
        start_right: list,
        duration: float,
        dt: float = 0.03,
    ) -> None:
        cmd_sides = self._sdk_cmd_sides()
        num_points = max(int(duration / dt), 1)
        for i in range(num_points + 1):
            t = i / num_points
            s = 10 * (t ** 3) - 15 * (t ** 4) + 6 * (t ** 5)
            interp_left = [
                start_left[j] + s * (left_target[j] - start_left[j]) for j in range(7)
            ]
            interp_right = [
                start_right[j] + s * (right_target[j] - start_right[j]) for j in range(7)
            ]
            left_cmd = interp_left if cmd_sides in ("both", "left") else None
            right_cmd = interp_right if cmd_sides in ("both", "right") else None
            if self._dry_run:
                self._publish_command(left_cmd, right_cmd)
            else:
                self.controller.move_to_joints_direct(
                    left_joints=left_cmd,
                    right_joints=right_cmd,
                )
            time.sleep(dt)

    def _execute_keyboard_start(self) -> tuple[bool, str]:
        """B: stand at INIT/calib → 3s joint move to first IK frame → reset tracker zero → teleop."""
        self.get_logger().info(
            f"Keyboard start ({self._keyboard_start_key}): "
            f"{self._keyboard_start_align_sec:.1f}s align to first IK frame (stand still at calib)."
        )
        self._align_in_progress = True
        try:
            self._reset_tracker_state()
            self.get_logger().info("[start_teleop] stage=reset_tracker_state_done")
            if not self._wait_for_tracker_tf_ready():
                message = (
                    f"Tracker TF not ready within {TELEOP_START_TF_WAIT_SEC:.1f}s; try B again"
                )
                self.get_logger().warn(message)
                return False, message
            self.get_logger().info("[start_teleop] stage=tracker_tf_ready")

            self.get_logger().info("[start_teleop] stage=get_current_joints_begin")
            left_cur, right_cur = self.controller.get_current_joints()
            self.get_logger().info(
                "[start_teleop] stage=get_current_joints_done "
                f"left_cur={[round(v, 3) for v in left_cur]} "
                f"right_cur={[round(v, 3) for v in right_cur]}"
            )
            self.get_logger().info("[start_teleop] stage=compute_teleop_ik_begin")
            left_ik, right_ik = self._compute_teleop_ik_joints()
            self.get_logger().info(
                "[start_teleop] stage=compute_teleop_ik_done "
                f"left_ik={None if left_ik is None else [round(v, 3) for v in left_ik]} "
                f"right_ik={None if right_ik is None else [round(v, 3) for v in right_ik]}"
            )

            self.get_logger().info("[start_teleop] stage=align_side_target_left_begin")
            align_left, left_use_ik = self._align_side_target("left", list(left_cur), left_ik)
            self.get_logger().info(
                "[start_teleop] stage=align_side_target_left_done "
                f"use_ik={left_use_ik} "
                f"align_left={[round(v, 3) for v in align_left]}"
            )
            self.get_logger().info("[start_teleop] stage=align_side_target_right_begin")
            align_right, right_use_ik = self._align_side_target("right", list(right_cur), right_ik)
            self.get_logger().info(
                "[start_teleop] stage=align_side_target_right_done "
                f"use_ik={right_use_ik} "
                f"align_right={[round(v, 3) for v in align_right]}"
            )
            use_ik_align = left_use_ik or right_use_ik

            if self._keyboard_start_align_sec > 0.0:
                if use_ik_align:
                    self.get_logger().info(
                        f"Aligning {self._keyboard_start_align_sec:.1f}s to first IK frame..."
                    )
                else:
                    self.get_logger().info(
                        f"Holding calib joints {self._keyboard_start_align_sec:.1f}s..."
                    )
                self.get_logger().info("[start_teleop] stage=smooth_align_publish_begin")
                self._smooth_align_publish(
                    left_target=align_left,
                    right_target=align_right,
                    start_left=list(left_cur),
                    start_right=list(right_cur),
                    duration=self._keyboard_start_align_sec,
                )
                self.get_logger().info("[start_teleop] stage=smooth_align_publish_done")

            self._reset_tracker_state()
            self.get_logger().info("[start_teleop] stage=post_align_reset_tracker_state_done")
            self.controller._last_left_ik_joints = None
            self.controller._last_right_ik_joints = None
            self._last_command_joints = {"left": None, "right": None}
            if "left" in self._teleop_active_sides:
                self._last_command_joints["left"] = list(align_left)
            if "right" in self._teleop_active_sides:
                self._last_command_joints["right"] = list(align_right)
            self._teleop_armed = True
            self._teleop_zero_warmup_remaining = 0
            self._teleop_ik_grace_remaining = 0
            self._start_time = time.monotonic()
            self._logged_tracker_delay = False
            message = "Teleop armed. Tracker zero binds on next TF; then IK teleop."
            self.get_logger().info(message)
            return True, message
        finally:
            self._align_in_progress = False

    def _update_tracker_poses_from_tf(self) -> bool:
        self.left_arm_mat = None
        self.right_arm_mat = None
        self.left_y_axis = None
        self.right_y_axis = None
        ok = False
        if "left" in self._teleop_active_sides:
            left_arm_tf = self._lookup_transform("left_chest", "left_arm")
            if left_arm_tf is not None:
                self.left_arm_mat = left_arm_tf
                self.left_y_axis = left_arm_tf[:3, 1]
            left_tf = self._lookup_transform("left_chest", "tianji_left")
            if left_tf is not None:
                left_ori_tf = self._lookup_orientation_source_tf("left", "left_chest", "tianji_left")
                self.left_pose = self._resolve_tracker_pose(
                    "left", left_tf, orientation_tf=left_ori_tf
                )
                if self.left_pose is not None:
                    ok = True
        if "right" in self._teleop_active_sides:
            right_arm_tf = self._lookup_transform("right_chest", "right_arm")
            if right_arm_tf is not None:
                self.right_arm_mat = right_arm_tf
                self.right_y_axis = right_arm_tf[:3, 1]
            right_tf = self._lookup_transform("right_chest", "tianji_right")
            if right_tf is not None:
                right_ori_tf = self._lookup_orientation_source_tf(
                    "right", "right_chest", "tianji_right"
                )
                self.right_pose = self._resolve_tracker_pose(
                    "right", right_tf, orientation_tf=right_ori_tf
                )
                if self.right_pose is not None:
                    ok = True
        return ok

    def _wait_for_tracker_tf_ready(
        self,
        timeout_sec: float = TELEOP_START_TF_WAIT_SEC,
        poll_sec: float = 0.05,
    ) -> bool:
        deadline = time.monotonic() + max(float(timeout_sec), 0.0)
        while True:
            if self._update_tracker_poses_from_tf():
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(max(float(poll_sec), 0.01))

    def _tracker_zero_ready(self, side: str) -> bool:
        return (
            self._tracker_zero[side] is not None
            and self._robot_zero[side] is not None
        )

    def _compute_teleop_ik_joints(self) -> tuple[Optional[list], Optional[list]]:
        left_mat = self.left_pose_mat if "left" in self._teleop_active_sides else None
        right_mat = self.right_pose_mat if "right" in self._teleop_active_sides else None
        if self._use_pinocchio_ik and self._tracker_orientation_mode in ("full_pose", "position_only"):
            l_joints = self._solve_pinocchio_side("left", self.left_pose_mat_m)
            r_joints = self._solve_pinocchio_side("right", self.right_pose_mat_m)
            return l_joints, r_joints
        if self._tracker_mode == "incremental":
            _, _, l_joints, r_joints = self.controller.move_to_matrix_direct(
                left_mat=left_mat,
                right_mat=right_mat,
                left_target_log=self.left_pose,
                right_target_log=self.right_pose,
                joint_postprocessor=self._postprocess_ik_joints,
                send_cmd=False,
            )
            return l_joints, r_joints
        _, _, l_joints, r_joints = self.controller.move_to_pose_direct(
            left_pose=self.left_pose if "left" in self._teleop_active_sides else None,
            right_pose=self.right_pose if "right" in self._teleop_active_sides else None,
            unit="m",
            joint_postprocessor=self._postprocess_ik_joints,
            send_cmd=False,
        )
        return l_joints, r_joints

    # -------------------- Subscription Callbacks --------------------

    def _left_cmd_callback(self, msg: JointState):
        if self._mode == ControlMode.INFERENCE and msg.position:
            self.left_inference_joints = list(msg.position)

    def _right_cmd_callback(self, msg: JointState):
        if self._mode == ControlMode.INFERENCE and msg.position:
            self.right_inference_joints = list(msg.position)

    # -------------------- Control Loop --------------------

    def _control_loop(self):
        self._publish_state()
        if self._keyboard_teleop_gate or self._pending_keyboard_start or self._pending_keyboard_stop:
            self._handle_pending_keyboard()

        if self._mode == ControlMode.TELEOP:
            self._teleop_control()
        else:
            self._inference_control()

    def _teleop_control(self):
        """Teleoperation control: TF → IK → robot"""
        if self._align_in_progress:
            return
        if not self._teleop_armed:
            return
        elapsed = time.monotonic() - self._start_time
        if elapsed < self._tracker_start_delay_sec:
            if not self._logged_tracker_delay:
                self.get_logger().info(
                    f"Waiting {self._tracker_start_delay_sec:.1f}s before tracker control starts"
                )
                self._logged_tracker_delay = True
            return

        if not self._update_tracker_poses_from_tf():
            return

        # Log (every 3 seconds)
        self._log_counter += 1
        if self._log_counter >= 300:
            self._log_counter = 0
            self.get_logger().info(
                f"TF: left={'OK' if self.left_pose is not None else 'None'}, "
                f"right={'OK' if self.right_pose is not None else 'None'}")

        use_pin = (
            self._use_pinocchio_ik
            and self._tracker_orientation_mode in ("full_pose", "position_only")
        )
        active_pose = (
            (
                "left" in self._teleop_active_sides
                and self.left_pose is not None
                and (
                    self.left_pose_mat is not None
                    or (use_pin and self.left_pose_mat_m is not None)
                )
                and self._tracker_zero_ready("left")
            )
            or (
                "right" in self._teleop_active_sides
                and self.right_pose is not None
                and (
                    self.right_pose_mat is not None
                    or (use_pin and self.right_pose_mat_m is not None)
                )
                and self._tracker_zero_ready("right")
            )
        )
        if active_pose:
            # Live zsp from right_arm y-axis can crash libKine in the IK subprocess; keep yaml default.
            if self.controller._ik_subprocess_client is None:
                if self.left_y_axis is not None:
                    self.controller.left_zsp_para = [*self.left_y_axis, 0, 0, 0]
                if self.right_y_axis is not None:
                    self.controller.right_zsp_para = [*self.right_y_axis, 0, 0, 0]

            l_joints, r_joints = self._compute_teleop_ik_joints()
            if use_pin:
                l_joints, r_joints = self._limit_teleop_joint_commands(l_joints, r_joints)
            for side, joints in (("left", l_joints), ("right", r_joints)):
                if joints is not None and side in self._teleop_active_sides:
                    ref = self._last_command_joints.get(side)
                    if ref is not None and self._teleop_ik_max_step_deg > 0.0:
                        raw_delta = self._joint_max_delta_deg(ref, joints)
                        if raw_delta > self._teleop_ik_max_step_deg:
                            joints = self._slew_joints_deg(ref, joints, self._teleop_ik_max_step_deg)
                    self._last_command_joints[side] = list(joints)
                elif joints is None and side in self._teleop_active_sides:
                    hold = self._last_command_joints.get(side)
                    if hold is not None:
                        joints = list(hold)
                if side == "left":
                    l_joints = joints
                else:
                    r_joints = joints
            self._publish_command(l_joints, r_joints)

        # Publish null-space parameters and end-effector poses
        if (
            self.left_pose is not None
            or self.right_pose is not None
            or self.controller.left_zsp_para is not None
            or self.controller.right_zsp_para is not None
        ):
            self._publish_zsp_para_and_pose()

    def _inference_control(self):
        """Inference control: joint_command_in → joint_command relay"""
        left, right = self.left_inference_joints, self.right_inference_joints
        if left is not None or right is not None:
            self._publish_command(left, right)

    def _solve_pinocchio_side(self, side: str, target_mat_m: Optional[np.ndarray]) -> Optional[list]:
        if target_mat_m is None:
            return None
        try:
            solver = self._get_pinocchio_solver(side)
        except Exception as exc:
            self.get_logger().warn(f"[IK_DIAG] side={side} Pinocchio solver init failed: {exc}; holding")
            return self._last_command_joints.get(side)

        seed = self._last_success_joints.get(side)
        if seed is None:
            seed = self._current_joints(side)
        if seed is None:
            self.get_logger().warn(f"[IK_DIAG] side={side} no joint seed available; holding")
            return self._last_command_joints.get(side)

        target_for_pin = self._target_for_pinocchio_frame(side, solver, target_mat_m, seed)
        j2_target = self._build_pinocchio_j2_axis_constraint(side, seed)
        joint_target_motor = None if (j2_target is None or self._pin_j2_axis_constraint_hard) else (
            int(self._pin_j2_axis_constraint_joint_index),
            float(j2_target),
            float(self._pin_j2_axis_constraint_weight),
        )
        joint_lock_motor = None if (j2_target is None or not self._pin_j2_axis_constraint_hard) else (
            int(self._pin_j2_axis_constraint_joint_index),
            float(j2_target),
        )

        try:
            result = solver.solve(
                target_for_pin,
                seed,
                side=side,
                joint_target_motor=joint_target_motor,
                joint_lock_motor=joint_lock_motor,
            )
        except Exception as exc:
            self.get_logger().warn(f"[IK_DIAG] side={side} exception={exc}; holding last command")
            return self._last_command_joints.get(side)

        self._log_ik_result(side, result, target_for_pin)
        if not result.success or not np.all(np.isfinite(result.q_motor)):
            return self._last_command_joints.get(side)

        q = self._apply_joint_step_limit(side, result.q_motor)
        self._last_success_joints[side] = list(q)
        self._last_command_joints[side] = list(q)
        self._last_step_limit_hold_joints[side] = None
        return q

    @staticmethod
    def _normalize_xy(vec: np.ndarray) -> Optional[np.ndarray]:
        v = np.array(vec, dtype=np.float64).reshape(3)
        v[2] = 0.0
        norm = float(np.linalg.norm(v[:2]))
        if norm < 1e-6:
            return None
        return v / norm

    @staticmethod
    def _wrap_angle_rad(angle: float) -> float:
        return float((angle + np.pi) % (2.0 * np.pi) - np.pi)

    def _build_pinocchio_j2_axis_constraint(
        self,
        side: str,
        seed_joints: list[float],
    ) -> Optional[float]:
        if (
            not self._pin_j2_axis_constraint_enable
            or (
                not self._pin_j2_axis_constraint_hard
                and self._pin_j2_axis_constraint_weight <= 0.0
            )
        ):
            return None

        axis_xy = self._constraint_reference_xy(side)
        if axis_xy is None:
            return None

        if self._arm_axis_zero_xy[side] is None:
            self._arm_axis_zero_xy[side] = np.array(axis_xy, dtype=np.float64)
            if len(seed_joints) > self._pin_j2_axis_constraint_joint_index:
                self._pin_j2_zero[side] = float(seed_joints[self._pin_j2_axis_constraint_joint_index])
                self._pin_j2_target_last[side] = float(self._pin_j2_zero[side])
                self.get_logger().info(
                    f"[J_AXIS_CONSTRAINT] side={side} initialized axis-zero on chest XY plane, "
                    f"joint=j{self._pin_j2_axis_constraint_joint_index + 1} "
                    f"joint_zero_deg={self._pin_j2_zero[side]:.2f}"
                )
            else:
                return None

        axis_zero = self._arm_axis_zero_xy[side]
        j2_zero = self._pin_j2_zero[side]
        if axis_zero is None or j2_zero is None:
            return None

        now_angle = float(np.arctan2(axis_xy[1], axis_xy[0]))
        zero_angle = float(np.arctan2(axis_zero[1], axis_zero[0]))
        delta_deg = np.degrees(self._wrap_angle_rad(now_angle - zero_angle))
        delta_deg *= self._pin_j2_axis_constraint_gain
        if self._pin_j2_axis_constraint_max_delta_deg > 0.0:
            delta_deg = float(
                np.clip(
                    delta_deg,
                    -self._pin_j2_axis_constraint_max_delta_deg,
                    self._pin_j2_axis_constraint_max_delta_deg,
                )
            )
        target_deg = float(j2_zero + delta_deg)

        prev_target = self._pin_j2_target_last.get(side)
        if (
            prev_target is not None
            and self._pin_j2_axis_constraint_max_step_deg > 0.0
        ):
            step = float(target_deg - prev_target)
            step = float(
                np.clip(
                    step,
                    -self._pin_j2_axis_constraint_max_step_deg,
                    self._pin_j2_axis_constraint_max_step_deg,
                )
            )
            target_deg = float(prev_target + step)
        self._pin_j2_target_last[side] = target_deg

        now = time.monotonic()
        if now - self._last_pin_j2_axis_log_time[side] >= 1.0:
            self._last_pin_j2_axis_log_time[side] = now
            axis_fmt = [round(float(v), 4) for v in axis_xy.tolist()]
            self.get_logger().info(
                f"[J_AXIS_CONSTRAINT] side={side} joint=j{self._pin_j2_axis_constraint_joint_index + 1} "
                f"axis_xy={axis_fmt} delta_deg={delta_deg:.2f} target_deg={target_deg:.2f} "
                f"mode={'hard' if self._pin_j2_axis_constraint_hard else 'soft'} "
                f"weight={self._pin_j2_axis_constraint_weight:.3f}"
            )

        return float(target_deg)

    def _constraint_reference_xy(self, side: str) -> Optional[np.ndarray]:
        """Build chest-XY reference vector for joint axis constraint.

        - For j7 (index 6): use right_wrist relative to right_arm, represented in right_arm frame.
        - Otherwise: keep legacy right_arm local y-axis in chest frame.
        """
        if self._pin_j2_axis_constraint_joint_index == 6:
            arm_tf = self.left_arm_mat if side == "left" else self.right_arm_mat
            if arm_tf is None:
                return None
            chest_frame = "left_chest" if side == "left" else "right_chest"
            wrist_frame = "left_wrist" if side == "left" else "right_wrist"
            wrist_tf = self._lookup_transform(chest_frame, wrist_frame)
            if wrist_tf is None:
                return None
            rel_chest = np.array(wrist_tf[:3, 3] - arm_tf[:3, 3], dtype=np.float64)
            rel_arm = np.array(arm_tf[:3, :3], dtype=np.float64).T @ rel_chest
            return self._normalize_xy(rel_arm)

        axis = self.left_y_axis if side == "left" else self.right_y_axis
        if axis is None:
            return None
        return self._normalize_xy(np.array(axis, dtype=np.float64))

    def _target_for_pinocchio_frame(
        self,
        side: str,
        solver,
        sdk_target_mat_m: np.ndarray,
        q_seed: list[float],
    ) -> np.ndarray:
        del solver, q_seed
        target = np.array(sdk_target_mat_m, dtype=np.float64)
        offset = self._pin_target_offset.get(side)
        if offset is None:
            return target

        offset_xyz = [round(float(v), 4) for v in offset[:3, 3]]
        offset_rpy = [
            round(float(v), 2)
            for v in R.from_matrix(offset[:3, :3]).as_euler("xyz", degrees=True)
        ]
        now = time.monotonic()
        if now - self._last_target_frame_log_time[side] >= 1.0:
            self._last_target_frame_log_time[side] = now
            self.get_logger().info(
                f"[IK_TARGET_FRAME] side={side} applying sdk_to_pin offset "
                f"xyz={offset_xyz} rpy={offset_rpy}"
            )
        return target @ offset

    def _check_pinocchio_fk_alignment(self, side: str, solver) -> None:
        if self._pin_fk_alignment_checked.get(side):
            return
        self._pin_fk_alignment_checked[side] = True

        q_current = self._current_joints(side)
        sdk_fk = self._current_robot_fk_matrix(side)
        if q_current is None or sdk_fk is None:
            self.get_logger().warn(
                f"[IK_FRAME_ALIGN] side={side} cannot compare SDK FK and Pinocchio FK: "
                "current joints or SDK FK unavailable"
            )
            return

        pin_fk = solver.frame_matrix(q_current)
        offset = np.linalg.inv(sdk_fk) @ pin_fk
        sdk_xyz = [round(float(v), 4) for v in sdk_fk[:3, 3]]
        pin_xyz = [round(float(v), 4) for v in pin_fk[:3, 3]]
        offset_xyz = [round(float(v), 4) for v in offset[:3, 3]]
        sdk_rpy = [
            round(float(v), 2)
            for v in R.from_matrix(sdk_fk[:3, :3]).as_euler("xyz", degrees=True)
        ]
        pin_rpy = [
            round(float(v), 2)
            for v in R.from_matrix(pin_fk[:3, :3]).as_euler("xyz", degrees=True)
        ]
        offset_rpy = [
            round(float(v), 2)
            for v in R.from_matrix(offset[:3, :3]).as_euler("xyz", degrees=True)
        ]
        self.get_logger().info(
            f"[IK_FRAME_ALIGN] side={side} "
            f"sdk_fk_xyz_m={sdk_xyz} sdk_fk_rpy_deg={sdk_rpy} "
            f"pin_fk_xyz_m={pin_xyz} pin_fk_rpy_deg={pin_rpy} "
            f"sdk_to_pin_offset_xyz_m={offset_xyz} sdk_to_pin_offset_rpy_deg={offset_rpy}"
        )

        offset_translation_m = float(np.linalg.norm(offset[:3, 3]))
        offset_rotation_deg = float(np.degrees(R.from_matrix(offset[:3, :3]).magnitude()))
        offset_exceeds_safety_threshold = offset_translation_m > 0.15 or offset_rotation_deg > 150.0
        if offset_exceeds_safety_threshold:
            self._pin_target_offset[side] = None
            self.get_logger().warn(
                f"[IK_FRAME_ALIGN] side={side} not applying sdk_to_pin offset: "
                f"offset_translation_m={offset_translation_m:.4f} "
                f"offset_rotation_deg={offset_rotation_deg:.2f} exceeds safety threshold"
            )
        else:
            self._pin_target_offset[side] = np.array(offset, dtype=np.float64)

        if offset_translation_m > 0.05 or offset_rotation_deg > 10.0:
            self.get_logger().warn(
                f"[IK_FRAME_ALIGN] side={side} SDK FK and Pinocchio FK mismatch: "
                f"offset_translation_m={offset_translation_m:.4f} "
                f"offset_rotation_deg={offset_rotation_deg:.2f}. "
                "URDF / ee_frame / joint order / joint unit may be wrong."
            )
        if self._ik_frame_scan_debug or offset_exceeds_safety_threshold:
            self._diagnose_pinocchio_frames(side, solver, q_current, sdk_fk)

    def _diagnose_pinocchio_frames(self, side: str, solver, q_current: list[float], sdk_fk: np.ndarray) -> None:
        try:
            import pinocchio as pin
        except ImportError as exc:
            self.get_logger().warn(f"[IK_FRAME_SCAN] side={side} cannot import pinocchio: {exc}")
            return

        q_pin = solver.motor_to_pin(q_current)
        solver._clamp_q_in_place(q_pin)
        pin.forwardKinematics(solver.model, solver.data, q_pin)
        pin.updateFramePlacements(solver.model, solver.data)

        scan = []
        sdk_inv = np.linalg.inv(sdk_fk)
        for frame_id, frame in enumerate(solver.model.frames):
            pin_frame = solver.data.oMf[frame_id]
            frame_matrix = np.eye(4, dtype=np.float64)
            frame_matrix[:3, :3] = pin_frame.rotation
            frame_matrix[:3, 3] = pin_frame.translation
            offset = sdk_inv @ frame_matrix
            translation_error_m = float(np.linalg.norm(offset[:3, 3]))
            rotation_error_deg = float(np.degrees(R.from_matrix(offset[:3, :3]).magnitude()))
            score = translation_error_m + 0.002 * rotation_error_deg
            frame_xyz = [round(float(v), 4) for v in frame_matrix[:3, 3]]
            frame_rpy = [
                round(float(v), 2)
                for v in R.from_matrix(frame_matrix[:3, :3]).as_euler("xyz", degrees=True)
            ]
            scan.append(
                {
                    "frame_name": frame.name,
                    "translation_error_m": translation_error_m,
                    "rotation_error_deg": rotation_error_deg,
                    "frame_xyz": frame_xyz,
                    "frame_rpy": frame_rpy,
                    "score": score,
                }
            )

        scan.sort(key=lambda item: item["score"])
        for rank, item in enumerate(scan[:10], start=1):
            self.get_logger().info(
                f"[IK_FRAME_SCAN] side={side} rank={rank} frame={item['frame_name']} "
                f"trans_err_m={item['translation_error_m']:.4f} "
                f"rot_err_deg={item['rotation_error_deg']:.2f} "
                f"pin_xyz_m={item['frame_xyz']} pin_rpy_deg={item['frame_rpy']}"
            )

        if not scan:
            self.get_logger().warn(f"[IK_FRAME_SCAN] side={side} no Pinocchio frames found")
            return

        best = scan[0]
        self.get_logger().info(
            f"[IK_FRAME_SCAN_BEST] side={side} best_frame={best['frame_name']} "
            f"trans_err_m={best['translation_error_m']:.4f} "
            f"rot_err_deg={best['rotation_error_deg']:.2f}"
        )

        configured_frame = self._pinocchio_ee_frames.get(side)
        if best["translation_error_m"] < 0.02 and best["rotation_error_deg"] < 5.0:
            config_key = f"pinocchio_{side}_ee_frame"
            self.get_logger().warn(
                f"[IK_FRAME_SCAN_BEST] side={side} configured {config_key}={configured_frame!r} "
                f"may need to be changed to best_frame={best['frame_name']!r}"
            )

        any_close = any(
            item["translation_error_m"] <= 0.05 and item["rotation_error_deg"] <= 20.0
            for item in scan
        )
        if not any_close:
            q_fmt = [round(float(v), 4) for v in q_current]
            self.get_logger().warn(
                f"[IK_FRAME_SCAN] side={side} No Pinocchio frame matches SDK FK. "
                f"Joint order/sign/zero convention may differ from SDK. q_current={q_fmt}"
            )

    def _current_joints(self, side: str) -> Optional[list]:
        try:
            left_joints, right_joints = self.controller.get_current_joints()
            joints = left_joints if side == "left" else right_joints
            if joints is None:
                return None
            return [float(v) for v in joints]
        except Exception as exc:
            self.get_logger().warn(f"Failed to read {side} joint seed: {exc}")
            return None

    def _apply_joint_step_limit(self, side: str, q_new: list[float]) -> list[float]:
        if self._max_joint_step_deg <= 0.0:
            return [float(v) for v in q_new]
        q_ref = self._last_command_joints.get(side)
        if q_ref is None:
            q_ref = self._current_joints(side)
        if q_ref is None:
            return [float(v) for v in q_new]

        q_ik_arr = np.array(q_new, dtype=np.float64)
        q_ref_arr = np.array(q_ref, dtype=np.float64)
        if self._motor_unit == "rad":
            limit = np.deg2rad(self._max_joint_step_deg)
            step_deg = np.abs(np.rad2deg(q_ik_arr - q_ref_arr))
        else:
            limit = self._max_joint_step_deg
            step_deg = np.abs(q_ik_arr - q_ref_arr)

        q_cmd_arr = q_ref_arr + np.clip(q_ik_arr - q_ref_arr, -limit, limit)
        max_step = float(np.max(step_deg))
        if max_step <= self._max_joint_step_deg:
            return [float(v) for v in q_ik_arr]

        now = time.monotonic()
        if now - self._last_joint_step_log_time[side] >= 1.0:
            self._last_joint_step_log_time[side] = now
            joint_idx = int(np.argmax(step_deg)) + 1
            self.get_logger().warn(
                f"[JOINT_STEP_LIMIT] side={side} max_step_deg={max_step:.2f} "
                f"joint={joint_idx} limit={self._max_joint_step_deg:.2f} sending limited command"
            )
        return [float(v) for v in q_cmd_arr]

    def _log_ik_result(self, side: str, result, target_mat_m: np.ndarray) -> None:
        now = time.monotonic()
        if now - self._last_ik_diag_log_time[side] >= 1.0:
            self._last_ik_diag_log_time[side] = now
            target_xyz = [round(float(v), 4) for v in target_mat_m[:3, 3]]
            target_rpy = [
                round(float(v), 2)
                for v in R.from_matrix(target_mat_m[:3, :3]).as_euler("xyz", degrees=True)
            ]
            self.get_logger().info(
                f"[IK_DIAG] side={side} success={result.success} "
                f"target_xyz_m={target_xyz} target_rpy_deg={target_rpy} "
                f"pos_err={result.position_error:.4f} ori_err={result.orientation_error:.4f} "
                f"sigma_min={result.sigma_min:.5f} cond={result.condition_number:.1f} "
                f"damping={result.damping:.5f} ori_weight={result.orientation_weight:.3f} "
                f"iter={result.iterations} reason={result.reason}"
            )

        warn = (
            result.sigma_min < self._singularity_sigma_min_warn
            or result.condition_number > self._singularity_condition_warn
        )
        critical = (
            result.sigma_min < self._singularity_sigma_min_critical
            or result.condition_number > self._singularity_condition_critical
        )
        if warn and now - self._last_ik_singularity_log_time[side] >= 1.0:
            self._last_ik_singularity_log_time[side] = now
            if critical:
                self.get_logger().warn(
                    f"[IK_SINGULARITY_CRITICAL] side={side} sigma_min={result.sigma_min:.5f} "
                    f"cond={result.condition_number:.1f} damping={result.damping:.5f} "
                    f"ori_weight={result.orientation_weight:.3f} fallback/holding={not result.success}"
                )
            else:
                self.get_logger().warn(
                    f"[IK_SINGULARITY_WARN] side={side} sigma_min={result.sigma_min:.5f} "
                    f"cond={result.condition_number:.1f} damping={result.damping:.5f} "
                    f"ori_weight={result.orientation_weight:.3f}"
                )

    # -------------------- Publishing --------------------

    def _publish_command(self, left: Optional[list], right: Optional[list]):
        stamp = self.get_clock().now().to_msg()
        if left is not None:
            msg = JointState()
            msg.header.stamp = stamp
            msg.header.frame_id = "left_base_cmd"
            msg.name = [f'left_joint_{i+1}' for i in range(7)]
            msg.position = list(left)
            self.left_cmd_pub.publish(msg)
        if right is not None:
            msg = JointState()
            msg.header.stamp = stamp
            msg.header.frame_id = "right_base_cmd"
            msg.name = [f'right_joint_{i+1}' for i in range(7)]
            msg.position = list(right)
            self.right_cmd_pub.publish(msg)

    def _publish_state(self):
        try:
            left_joints, right_joints = self.controller.get_current_joints()
            stamp = self.get_clock().now().to_msg()

            if left_joints is not None:
                msg = JointState()
                msg.header.stamp = stamp
                msg.header.frame_id = "left_base_state"
                msg.name = [f'left_joint_{i+1}' for i in range(7)]
                msg.position = list(left_joints)
                self.left_state_pub.publish(msg)

            if right_joints is not None:
                msg = JointState()
                msg.header.stamp = stamp
                msg.header.frame_id = "right_base_state"
                msg.name = [f'right_joint_{i+1}' for i in range(7)]
                msg.position = list(right_joints)
                self.right_state_pub.publish(msg)
        except Exception as e:
            self.get_logger().debug(f"State publishing error: {e}")

    def _publish_zsp_para_and_pose(self):
        """Publish null-space parameters and end-effector poses"""
        # This line doesn't actually use stamp; can be removed if msg doesn't need header, or add header to pose
        # stamp = self.get_clock().now().to_msg() 

        try:
            # --- Publish left_zsp_para ---
            # try-except added to prevent errors when controller is destroyed while reading attributes
            if hasattr(self.controller, 'left_zsp_para') and self.controller.left_zsp_para is not None:
                raw_data = self.controller.left_zsp_para
                # Ensure data is non-empty
                if len(raw_data) > 0:
                    msg = Float64MultiArray()
                    # Force convert to list with float elements for compatibility
                    msg.data = [float(x) for x in raw_data]
                    self.left_zsp_para_pub.publish(msg)

            # --- Publish right_zsp_para ---
            if hasattr(self.controller, 'right_zsp_para') and self.controller.right_zsp_para is not None:
                raw_data = self.controller.right_zsp_para
                if len(raw_data) > 0:
                    msg = Float64MultiArray()
                    msg.data = [float(x) for x in raw_data]
                    self.right_zsp_para_pub.publish(msg)

            # --- Publish left_pose ---
            if self.left_pose is not None:
                msg = Float64MultiArray()
                # Similarly, ensure pose data is also a pure float list
                msg.data = [float(x) for x in self.left_pose]
                self.left_ee_pose_pub.publish(msg)

            # --- Publish right_pose ---
            if self.right_pose is not None:
                msg = Float64MultiArray()
                msg.data = [float(x) for x in self.right_pose]
                self.right_ee_pose_pub.publish(msg)

        except Exception as e:
            # If any unexpected error occurs during shutdown (e.g., C++ object already released), only log without crashing the node
            self.get_logger().warn(f"Failed to publish debug info during shutdown: {e}")

    # -------------------- TF Utilities --------------------

    def _lookup_transform(self, from_frame: str, to_frame: str) -> Optional[np.ndarray]:
        try:
            tf = self.tf_buffer.lookup_transform(from_frame, to_frame, rclpy.time.Time())
            return self._transform_to_matrix(tf)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None

    def _lookup_orientation_source_tf(
        self,
        side: str,
        chest_frame: str,
        mapped_tf_frame: str,
    ) -> Optional[np.ndarray]:
        if (
            self._tracker_orientation_mode != "full_pose"
            or self._tracker_orientation_input_mode != "raw_wrist"
        ):
            self._log_tracker_orientation_source(side, mapped_tf_frame, mapped_tf_frame, fallback=False)
            return None

        source_frame = self._tracker_orientation_source_frames[side]
        orientation_tf = self._lookup_transform(chest_frame, source_frame)
        if orientation_tf is None:
            self._log_tracker_orientation_source(side, mapped_tf_frame, source_frame, fallback=True)
            return None

        self._log_tracker_orientation_source(side, mapped_tf_frame, source_frame, fallback=False)
        return orientation_tf

    def _log_tracker_orientation_source(
        self,
        side: str,
        position_frame: str,
        orientation_frame: str,
        fallback: bool,
    ) -> None:
        now = time.monotonic()
        if now - self._last_tracker_ori_source_log_time[side] < 1.0:
            return
        self._last_tracker_ori_source_log_time[side] = now
        if fallback:
            self.get_logger().warn(
                f"[TRACKER_ORI_SOURCE] side={side} raw orientation TF {orientation_frame} unavailable; "
                f"falling back to mapped {position_frame}"
            )
            orientation_frame = position_frame
        self.get_logger().info(
            f"[TRACKER_ORI_SOURCE] side={side} input_mode={self._tracker_orientation_input_mode} "
            f"position_tf={position_frame} orientation_tf={orientation_frame} "
            f"r_map_mode={self._tracker_orientation_map_mode}"
        )

    @staticmethod
    def _transform_to_matrix(transform) -> np.ndarray:
        t = transform.transform
        quat = [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]
        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = R.from_quat(quat).as_matrix()
        T[:3, 3] = [t.translation.x, t.translation.y, t.translation.z]
        return T

    @staticmethod
    def _matrix_to_pose(matrix: np.ndarray) -> np.ndarray:
        """4x4 matrix → [x, y, z, RX, RY, RZ] (degrees)"""
        xyz = matrix[:3, 3]
        rpy = R.from_matrix(matrix[:3, :3]).as_euler('ZYX', degrees=True)[::-1]
        return np.array([xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]])

    def _blend_rotation(self, start_rot: np.ndarray, end_rot: np.ndarray) -> np.ndarray:
        blend = self._tracker_orientation_blend
        if blend <= 0.0:
            return np.array(start_rot, dtype=np.float64)
        if blend >= 1.0:
            return np.array(end_rot, dtype=np.float64)
        rotations = R.from_matrix(np.stack([start_rot, end_rot], axis=0))
        return Slerp([0.0, 1.0], rotations)([blend]).as_matrix()[0]

    def _stabilize_orientation(self, side: str, target_rot: np.ndarray) -> np.ndarray:
        """Low-pass and rate-limit tracker orientation to avoid IK branch jumps."""
        target_rot = np.array(target_rot, dtype=np.float64)
        previous = self._filtered_orientation[side]
        if previous is None:
            self._filtered_orientation[side] = target_rot
            return target_rot

        previous_r = R.from_matrix(previous)
        target_r = R.from_matrix(target_rot)
        delta_r = previous_r.inv() * target_r
        delta_angle_deg = np.degrees(delta_r.magnitude())

        step_ratio = 1.0
        if self._tracker_orientation_max_step_deg > 0.0 and delta_angle_deg > self._tracker_orientation_max_step_deg:
            step_ratio = self._tracker_orientation_max_step_deg / delta_angle_deg
        step_ratio *= self._tracker_orientation_filter_alpha

        if step_ratio <= 0.0:
            filtered = previous
        elif step_ratio >= 1.0:
            filtered = target_rot
        else:
            rotations = R.from_matrix(np.stack([previous, target_rot], axis=0))
            filtered = Slerp([0.0, 1.0], rotations)([step_ratio]).as_matrix()[0]
        self._filtered_orientation[side] = filtered
        return filtered

    @staticmethod
    def _yaw_only_rotation(base_rot: np.ndarray, tracker_rot: np.ndarray) -> np.ndarray:
        """Keep the easy-to-reach base roll/pitch, but follow tracker heading."""
        base_zyx = R.from_matrix(base_rot).as_euler("ZYX")
        tracker_zyx = R.from_matrix(tracker_rot).as_euler("ZYX")
        return R.from_euler("ZYX", [tracker_zyx[0], base_zyx[1], base_zyx[2]]).as_matrix()

    def _parse_tracker_orientation_map(self, mode: str, matrix_cfg) -> np.ndarray:
        mode = str(mode).lower()

        if mode == "identity":
            r_map = np.eye(3, dtype=np.float64)
        elif mode == "matrix":
            if matrix_cfg is None:
                raise ValueError(
                    "tracker_orientation_map_mode='matrix' requires tracker_orientation_map_matrix"
                )
            r_map = np.array(matrix_cfg, dtype=np.float64)
            if r_map.shape != (3, 3):
                raise ValueError(
                    f"tracker_orientation_map_matrix must be 3x3, got shape={r_map.shape}"
                )
        else:
            raise ValueError(
                f"Unsupported tracker_orientation_map_mode={mode!r}; expected 'identity' or 'matrix'"
            )

        # Orthonormalize slightly to tolerate YAML numeric noise.
        u, _, vt = np.linalg.svd(r_map)
        r_map = u @ vt

        det = float(np.linalg.det(r_map))
        if det < 0.0:
            # Prevent reflection matrix. A valid rotation mapping must have det +1.
            u[:, -1] *= -1.0
            r_map = u @ vt
            det = float(np.linalg.det(r_map))

        ortho_err = float(np.linalg.norm(r_map.T @ r_map - np.eye(3)))
        if abs(det - 1.0) > 1e-3 or ortho_err > 1e-3:
            raise ValueError(
                f"Invalid tracker orientation map: det={det:.6f}, ortho_err={ortho_err:.6e}"
            )

        self.get_logger().info(
            "[TRACKER_ORI_MAP] "
            f"mode={mode} det={det:.6f} ortho_err={ortho_err:.3e} "
            f"matrix={np.round(r_map, 4).tolist()}"
        )
        return r_map

    def _map_tracker_delta_rotation(self, delta_rot: np.ndarray, side: str) -> np.ndarray:
        r_map = self._tracker_orientation_maps[side]
        return r_map @ delta_rot @ r_map.T

    def _log_tracker_orientation_debug(
        self,
        side: str,
        raw_delta_rot: np.ndarray,
        mapped_delta_rot: np.ndarray,
        blended_rot: np.ndarray,
        stabilized_rot: Optional[np.ndarray] = None,
    ) -> None:
        if not self._tracker_orientation_debug:
            return

        now = time.monotonic()
        if now - self._last_tracker_ori_log_time[side] < 1.0:
            return
        self._last_tracker_ori_log_time[side] = now

        raw_rotvec_deg = np.degrees(R.from_matrix(raw_delta_rot).as_rotvec())
        mapped_rotvec_deg = np.degrees(R.from_matrix(mapped_delta_rot).as_rotvec())

        robot_zero_r = self._robot_zero[side][:3, :3]
        blended_delta_rot = robot_zero_r.T @ blended_rot
        blended_rotvec_deg = np.degrees(R.from_matrix(blended_delta_rot).as_rotvec())
        stabilized_fields = ""
        if stabilized_rot is not None:
            stabilized_delta_rot = robot_zero_r.T @ stabilized_rot
            stabilized_rotvec_deg = np.degrees(R.from_matrix(stabilized_delta_rot).as_rotvec())
            stabilized_fields = (
                f" stabilized_rotvec_deg={[round(float(v), 2) for v in stabilized_rotvec_deg]}"
                f" stabilized_delta_deg={np.degrees(R.from_matrix(stabilized_delta_rot).magnitude()):.2f}"
            )

        self.get_logger().info(
            f"[TRACKER_ORI] side={side} "
            f"raw_rotvec_deg={[round(float(v), 2) for v in raw_rotvec_deg]} "
            f"mapped_rotvec_deg={[round(float(v), 2) for v in mapped_rotvec_deg]} "
            f"blended_rotvec_deg={[round(float(v), 2) for v in blended_rotvec_deg]} "
            f"raw_delta_deg={np.degrees(R.from_matrix(raw_delta_rot).magnitude()):.2f} "
            f"mapped_delta_deg={np.degrees(R.from_matrix(mapped_delta_rot).magnitude()):.2f} "
            f"blended_delta_deg={np.degrees(R.from_matrix(blended_delta_rot).magnitude()):.2f} "
            f"blend={self._tracker_orientation_blend:.2f}"
            f"{stabilized_fields}"
        )

    def _empty_wrist_local_state(self, side: str, available: bool = False) -> dict:
        return {
            "available": bool(available),
            "is_wrist_local": False,
            "local_pos_delta_m": [0.0, 0.0, 0.0],
            "local_pos_norm_m": 0.0,
            "local_rotvec_deg": [0.0, 0.0, 0.0],
            "local_rot_deg": 0.0,
            "global_pos_delta_m": [0.0, 0.0, 0.0],
            "global_pos_norm_m": 0.0,
            "arm_frame": f"{side}_arm",
            "wrist_frame": f"{side}_wrist",
        }

    def _compute_wrist_local_state(
        self,
        side: str,
        tracker_tf: np.ndarray,
    ) -> dict:
        arm_frame = f"{side}_arm"
        wrist_frame = f"{side}_wrist"
        wrist_local_tf = self._lookup_transform(arm_frame, wrist_frame)
        if wrist_local_tf is None:
            return self._empty_wrist_local_state(side, available=False)

        if self._wrist_local_zero[side] is None:
            self._wrist_local_zero[side] = np.array(wrist_local_tf, dtype=np.float64)
            self.get_logger().info(
                f"[WRIST_LOCAL] side={side} zero initialized from {arm_frame} -> {wrist_frame}"
            )

        zero = self._wrist_local_zero[side]
        current = np.array(wrist_local_tf, dtype=np.float64)
        local_pos_delta = current[:3, 3] - zero[:3, 3]
        local_pos_delta_norm = float(np.linalg.norm(local_pos_delta))

        local_rot_delta = zero[:3, :3].T @ current[:3, :3]
        local_rotation = R.from_matrix(local_rot_delta)
        local_rot_delta_deg = float(np.degrees(local_rotation.magnitude()))
        local_rotvec_deg = np.degrees(local_rotation.as_rotvec())

        if self._tracker_zero[side] is None:
            global_pos_delta = np.zeros(3, dtype=np.float64)
        else:
            global_pos_delta = (
                np.array(tracker_tf[:3, 3], dtype=np.float64)
                - self._tracker_zero[side][:3, 3]
            )
        global_pos_delta_norm = float(np.linalg.norm(global_pos_delta))

        is_wrist_local = (
            local_pos_delta_norm <= self._tracker_wrist_local_pos_threshold_m
            and local_rot_delta_deg >= self._tracker_wrist_local_rot_threshold_deg
        )
        self._last_wrist_local_state[side] = bool(is_wrist_local)

        return {
            "available": True,
            "is_wrist_local": bool(is_wrist_local),
            "local_pos_delta_m": [float(v) for v in local_pos_delta],
            "local_pos_norm_m": local_pos_delta_norm,
            "local_rotvec_deg": [float(v) for v in local_rotvec_deg],
            "local_rot_deg": local_rot_delta_deg,
            "global_pos_delta_m": [float(v) for v in global_pos_delta],
            "global_pos_norm_m": global_pos_delta_norm,
            "arm_frame": arm_frame,
            "wrist_frame": wrist_frame,
        }

    def _log_wrist_local_debug(self, side: str, state: dict) -> None:
        if not self._tracker_wrist_local_debug:
            return

        now = time.monotonic()
        if now - self._last_wrist_local_log_time[side] < 1.0:
            return
        self._last_wrist_local_log_time[side] = now

        if not state.get("available", False):
            self.get_logger().warn(
                f"[WRIST_LOCAL] side={side} missing TF "
                f"{state.get('arm_frame', f'{side}_arm')} -> {state.get('wrist_frame', f'{side}_wrist')}"
            )
            return

        self.get_logger().info(
            f"[WRIST_LOCAL] side={side} "
            f"local_pos_delta_m={[round(float(v), 4) for v in state['local_pos_delta_m']]} "
            f"local_pos_norm_m={float(state['local_pos_norm_m']):.4f} "
            f"local_rotvec_deg={[round(float(v), 2) for v in state['local_rotvec_deg']]} "
            f"local_rot_deg={float(state['local_rot_deg']):.2f} "
            f"global_pos_delta_m={[round(float(v), 4) for v in state['global_pos_delta_m']]} "
            f"global_pos_norm_m={float(state['global_pos_norm_m']):.4f} "
            f"is_wrist_local={str(bool(state['is_wrist_local'])).lower()}"
        )

    def _log_wrist_local_hold(self, side: str, target: np.ndarray, state: dict) -> None:
        now = time.monotonic()
        if now - self._last_wrist_local_hold_log_time[side] < 1.0:
            return
        self._last_wrist_local_hold_log_time[side] = now
        self.get_logger().info(
            f"[WRIST_LOCAL_HOLD] side={side} "
            f"mode={self._tracker_wrist_local_hold_position_mode} "
            f"local_pos_norm_m={float(state.get('local_pos_norm_m', 0.0)):.4f} "
            f"local_rot_deg={float(state.get('local_rot_deg', 0.0)):.2f} "
            f"global_pos_norm_m={float(state.get('global_pos_norm_m', 0.0)):.4f} "
            f"holding_position={[round(float(v), 4) for v in target[:3, 3]]}"
        )

    @staticmethod
    def _clamp01(value: float) -> float:
        return max(0.0, min(1.0, float(value)))

    def _compute_wrist_local_weight(self, side: str, state: dict) -> dict:
        available = bool(state.get("available", False))
        if not available:
            local_pos = float(state.get("local_pos_norm_m", 0.0))
            local_rot = float(state.get("local_rot_deg", 0.0))
            pos_score = 0.0
            rot_score = 0.0
            raw_weight = 0.0
        else:
            local_pos = float(state.get("local_pos_norm_m", 0.0))
            local_rot = float(state.get("local_rot_deg", 0.0))
            pos_score = self._clamp01(
                (self._tracker_wrist_local_pos_full_m - local_pos)
                / (self._tracker_wrist_local_pos_full_m - self._tracker_wrist_local_pos_start_m)
            )
            rot_score = self._clamp01(
                (local_rot - self._tracker_wrist_local_rot_start_deg)
                / (self._tracker_wrist_local_rot_full_deg - self._tracker_wrist_local_rot_start_deg)
            )
            raw_weight = pos_score * rot_score

        prev = float(self._wrist_local_weight.get(side, 0.0))
        alpha = self._tracker_wrist_local_weight_filter_alpha
        filtered_weight = alpha * prev + (1.0 - alpha) * raw_weight
        filtered_weight = self._clamp01(filtered_weight)
        self._wrist_local_weight[side] = filtered_weight
        return {
            "available": available,
            "raw_weight": float(raw_weight),
            "filtered_weight": float(filtered_weight),
            "pos_score": float(pos_score),
            "rot_score": float(rot_score),
            "local_pos_norm_m": float(local_pos),
            "local_rot_deg": float(local_rot),
        }

    def _log_wrist_local_blend(
        self,
        side: str,
        weight_info: dict,
        follow_pos: np.ndarray,
        hold_pos: np.ndarray,
        blended_pos: np.ndarray,
    ) -> None:
        now = time.monotonic()
        if now - self._last_wrist_local_blend_log_time[side] < 1.0:
            return
        self._last_wrist_local_blend_log_time[side] = now
        self.get_logger().info(
            f"[WRIST_LOCAL_BLEND] side={side} "
            f"weight={float(weight_info.get('filtered_weight', 0.0)):.3f} "
            f"raw_weight={float(weight_info.get('raw_weight', 0.0)):.3f} "
            f"pos_score={float(weight_info.get('pos_score', 0.0)):.3f} "
            f"rot_score={float(weight_info.get('rot_score', 0.0)):.3f} "
            f"local_pos_norm_m={float(weight_info.get('local_pos_norm_m', 0.0)):.4f} "
            f"local_rot_deg={float(weight_info.get('local_rot_deg', 0.0)):.2f} "
            f"follow_pos={[round(float(v), 4) for v in follow_pos]} "
            f"hold_pos={[round(float(v), 4) for v in hold_pos]} "
            f"blended_pos={[round(float(v), 4) for v in blended_pos]}"
        )

    def _update_wrist_orientation_delta(self, side: str, tracker_rot: np.ndarray) -> None:
        """Convert tracker relative rotation into small wrist-joint deltas.

        IK still receives the startup TCP orientation. The relative wrist tracker
        rotation is applied after IK to joints 5-7 so position remains the
        primary task.
        """
        if self._tracker_zero[side] is None or self._robot_zero[side] is None:
            return

        tracker_delta_rot = self._wrist_relative_delta_rot(side, tracker_rot)
        full_rot = self._robot_zero[side][:3, :3] @ tracker_delta_rot
        filtered_full_rot = self._stabilize_orientation(side, full_rot)
        filtered_delta_rot = self._robot_zero[side][:3, :3].T @ filtered_full_rot
        wrist_delta = self._decompose_wrist_delta(filtered_delta_rot)
        order = self._wrist_orientation_axis_order.lower()
        axis_scale = np.array(
            [self._wrist_orientation_axis_scale[axis] for axis in order],
            dtype=np.float64,
        )
        wrist_delta *= self._wrist_orientation_scale * axis_scale
        if self._wrist_orientation_max_deg > 0.0:
            wrist_delta = np.clip(
                wrist_delta,
                -self._wrist_orientation_max_deg,
                self._wrist_orientation_max_deg,
            )
        self._wrist_orientation_delta[side] = wrist_delta

    def _decompose_wrist_delta(self, delta_rot: np.ndarray) -> np.ndarray:
        order = self._wrist_orientation_axis_order.lower()
        rotation = R.from_matrix(delta_rot)
        if self._wrist_orientation_decompose == "euler":
            return rotation.as_euler(self._wrist_orientation_axis_order, degrees=True)

        rotvec_deg = np.degrees(rotation.as_rotvec())
        if self._wrist_orientation_axis_basis is not None:
            rotvec_deg = self._wrist_orientation_axis_basis.T @ rotvec_deg
        return np.array(
            [{"x": rotvec_deg[0], "y": rotvec_deg[1], "z": rotvec_deg[2]}[axis] for axis in order],
            dtype=np.float64,
        )

    def _wrist_relative_delta_rot(self, side: str, tracker_rot: np.ndarray) -> np.ndarray:
        arm_mat = self.left_arm_mat if side == "left" else self.right_arm_mat
        if arm_mat is None:
            return self._tracker_zero[side][:3, :3].T @ tracker_rot

        current_relative = arm_mat[:3, :3].T @ tracker_rot
        if self._wrist_relative_zero[side] is None:
            self._wrist_relative_zero[side] = current_relative
            self.get_logger().info(
                f"Wrist relative zero initialized for {side}: wrist rotation follows {side}_arm-relative motion"
            )
        return self._wrist_relative_zero[side].T @ current_relative

    @staticmethod
    def _parse_wrist_axis_order(axis_order: str) -> str:
        order = axis_order.lower()
        if sorted(order) != ["x", "y", "z"]:
            raise ValueError(
                f"wrist_orientation_axis_order must be a permutation of xyz, got {axis_order!r}"
            )
        return order.upper()

    @staticmethod
    def _parse_wrist_decompose(decompose: str) -> str:
        value = str(decompose).lower()
        if value not in ("rotvec", "euler"):
            raise ValueError(
                f"wrist_orientation_decompose must be 'rotvec' or 'euler', got {decompose!r}"
            )
        return value

    @staticmethod
    def _parse_wrist_axis_basis(axis_basis) -> Optional[np.ndarray]:
        if not axis_basis:
            return None
        axes = []
        for name in ("x", "y", "z"):
            values = axis_basis.get(name)
            if values is None or len(values) != 3:
                raise ValueError(f"wrist_orientation_axis_basis.{name} must contain 3 values")
            axis = np.array([float(v) for v in values], dtype=np.float64)
            norm = float(np.linalg.norm(axis))
            if norm < 1e-9:
                raise ValueError(f"wrist_orientation_axis_basis.{name} must be nonzero")
            axes.append(axis / norm)
        basis = np.stack(axes, axis=1)
        u, _, vt = np.linalg.svd(basis)
        return u @ vt

    def _limit_teleop_joint_commands(
        self,
        left: Optional[list],
        right: Optional[list],
    ) -> tuple[Optional[list], Optional[list]]:
        """Per-frame step cap (only used when use_pinocchio_ik; SDK path matches sim — no cap)."""
        if left is not None and "left" in self._teleop_active_sides:
            left = self._apply_joint_step_limit("left", list(left))
            self._last_command_joints["left"] = list(left)
        if right is not None and "right" in self._teleop_active_sides:
            right = self._apply_joint_step_limit("right", list(right))
            self._last_command_joints["right"] = list(right)
        return left, right

    def _postprocess_ik_joints(self, side: str, joints: list) -> list:
        if self._tracker_orientation_mode != "wrist_only":
            return joints
        wrist_delta = self._wrist_orientation_delta.get(side)
        if wrist_delta is None:
            return joints
        processed = list(joints)
        before = [float(processed[i]) for i in (4, 5, 6)]
        joint_zero = self._wrist_joint_zero.get(side)
        if joint_zero is None:
            joint_zero = before
            self._wrist_joint_zero[side] = list(joint_zero)
        for idx, zero, delta in zip((4, 5, 6), joint_zero, wrist_delta):
            processed[idx] = float(zero) + float(delta)
        self._log_wrist_debug(side, wrist_delta, before, [float(processed[i]) for i in (4, 5, 6)])
        return processed

    def _log_wrist_debug(self, side: str, wrist_delta: np.ndarray, before: list, after: list) -> None:
        now = time.monotonic()
        if now - self._last_wrist_debug_log_time[side] < 0.5:
            return
        self._last_wrist_debug_log_time[side] = now
        delta_fmt = [round(float(v), 2) for v in wrist_delta]
        before_fmt = [round(float(v), 2) for v in before]
        after_fmt = [round(float(v), 2) for v in after]
        self.get_logger().info(
            f"[{side.upper()}_WRIST_ONLY] delta567_deg={delta_fmt} "
            f"ik567_before={before_fmt} cmd567_after={after_fmt}"
        )

    def _resolve_tracker_pose(
        self,
        side: str,
        tracker_tf: np.ndarray,
        orientation_tf: Optional[np.ndarray] = None,
    ) -> Optional[np.ndarray]:
        """Convert tracker TF to a Tianji IK pose.

        absolute: use the tracker TF directly.
        incremental: first valid tracker TF is zero; tracker deltas are applied
        on top of the robot's current FK pose.
        Returns None when FK/subprocess is not ready (skip IK this cycle).
        """
        if self._use_pinocchio_ik and self._tracker_orientation_mode in ("full_pose", "position_only"):
            if self._tracker_mode != "incremental":
                self.get_logger().warn(
                    "Pinocchio full-pose IK expects tracker_mode='incremental'; using incremental target logic"
                )
            return self._resolve_pinocchio_tracker_pose(side, tracker_tf, orientation_tf=orientation_tf)

        if self._tracker_mode == "absolute":
            if side == "left":
                self.left_pose_mat = None
            else:
                self.right_pose_mat = None
            scaled_tf = np.array(tracker_tf, dtype=np.float64)
            scaled_tf[:3, 3] *= self._tracker_position_scale_vec
            limited_tf = self._apply_position_limit(self._apply_position_bounds(scaled_tf))
            if self._tracker_orientation_mode in ("position_only", "wrist_only", "yaw_only", "blended"):
                if self._robot_zero[side] is None:
                    self._robot_zero[side] = self._current_robot_fk_matrix(side)
                if self._robot_zero[side] is not None:
                    limited_tf = np.array(limited_tf, dtype=np.float64)
                    if self._tracker_orientation_mode == "wrist_only" and self._tracker_zero[side] is None:
                        self._tracker_zero[side] = np.array(tracker_tf, dtype=np.float64)
                        self._wrist_joint_zero[side] = self._current_wrist_joints(side)
                        self.get_logger().info(
                            f"Wrist orientation zero initialized for {side}: IK will stay position-only"
                        )
                    if self._tracker_orientation_mode == "wrist_only":
                        self._update_wrist_orientation_delta(side, tracker_tf[:3, :3])
                    if self._tracker_orientation_mode in ("position_only", "wrist_only"):
                        limited_tf[:3, :3] = self._robot_zero[side][:3, :3]
                    elif self._tracker_orientation_mode == "yaw_only":
                        limited_tf[:3, :3] = self._yaw_only_rotation(
                            self._robot_zero[side][:3, :3],
                            limited_tf[:3, :3],
                        )
                    else:
                        limited_tf[:3, :3] = self._blend_rotation(
                            self._robot_zero[side][:3, :3],
                            limited_tf[:3, :3],
                        )
            if self._tracker_orientation_mode in ("full", "full_pose", "blended"):
                limited_tf = np.array(limited_tf, dtype=np.float64)
                limited_tf[:3, :3] = self._stabilize_orientation(side, limited_tf[:3, :3])
            return self._matrix_to_pose(limited_tf)

        if self._tracker_zero[side] is None or self._robot_zero[side] is None:
            robot_zero = self._current_robot_fk_matrix(side)
            if robot_zero is None:
                if side == "left":
                    self.left_pose_mat = None
                else:
                    self.right_pose_mat = None
                return None
            self._tracker_zero[side] = np.array(tracker_tf, dtype=np.float64)
            self._robot_zero[side] = robot_zero
            if self._tracker_orientation_mode == "wrist_only":
                self._wrist_joint_zero[side] = self._current_wrist_joints(side)
            self.get_logger().info(
                f"Tracker zero initialized for {side}: current tracker pose maps to current Tianji FK"
            )

        target = np.array(self._robot_zero[side], dtype=np.float64)

        # Translation is controlled in the chest frame so it does not depend on
        # how the tracker happened to be rotated when it was strapped on.
        tracker_delta_pos = (
            tracker_tf[:3, 3] - self._tracker_zero[side][:3, 3]
        ) * self._tracker_position_scale_vec
        target[:3, 3] = self._robot_zero[side][:3, 3] + tracker_delta_pos
        target = self._apply_position_limit(self._apply_position_bounds(target))

        # Orientation modes:
        # - full: relative to startup calibration pose
        # - blended: partly follows tracker rotation from the startup TCP orientation
        # - yaw_only: only follows tracker heading
        # - position_only/wrist_only: keep the startup TCP orientation for IK
        if self._tracker_orientation_mode == "wrist_only":
            self._update_wrist_orientation_delta(side, tracker_tf[:3, :3])
        elif self._tracker_orientation_mode in ("full", "full_pose", "blended", "yaw_only"):
            tracker_delta_rot = self._tracker_zero[side][:3, :3].T @ tracker_tf[:3, :3]
            mapped_delta_rot = self._map_tracker_delta_rotation(tracker_delta_rot, side)
            full_rot = self._robot_zero[side][:3, :3] @ mapped_delta_rot
            if self._tracker_orientation_mode == "full":
                target[:3, :3] = full_rot
            elif self._tracker_orientation_mode == "full_pose":
                target[:3, :3] = self._blend_rotation(self._robot_zero[side][:3, :3], full_rot)
                self._log_tracker_orientation_debug(
                    side,
                    tracker_delta_rot,
                    mapped_delta_rot,
                    target[:3, :3],
                )
            elif self._tracker_orientation_mode == "yaw_only":
                target[:3, :3] = self._yaw_only_rotation(self._robot_zero[side][:3, :3], full_rot)
            else:
                target[:3, :3] = self._blend_rotation(self._robot_zero[side][:3, :3], full_rot)
            target[:3, :3] = self._stabilize_orientation(side, target[:3, :3])
        target_mm = np.array(target, dtype=np.float64)
        target_mm[:3, 3] *= 1000.0
        if side == "left":
            self.left_pose_mat = target_mm.tolist()
        else:
            self.right_pose_mat = target_mm.tolist()
        # IK uses right_pose_mat (mm); xyzabc pose is for logging — scipy is fine here.
        return self._matrix_to_pose(target)

    def _resolve_pinocchio_tracker_pose(
        self,
        side: str,
        tracker_tf: np.ndarray,
        orientation_tf: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        orientation_source_tf = orientation_tf if orientation_tf is not None else tracker_tf
        if self._tracker_zero[side] is None or self._robot_zero[side] is None:
            robot_zero = self._current_robot_fk_matrix(side)
            if robot_zero is None:
                self._set_pose_matrix_m(side, None)
                return self._matrix_to_pose(tracker_tf)
            self._tracker_zero[side] = np.array(tracker_tf, dtype=np.float64)
            self._tracker_ori_zero[side] = np.array(orientation_source_tf[:3, :3], dtype=np.float64)
            self._robot_zero[side] = robot_zero
            self.get_logger().info(
                f"Tracker zero initialized for {side}: Pinocchio IK target starts at current FK"
            )

        target = np.array(self._robot_zero[side], dtype=np.float64)
        tracker_delta_pos = (
            tracker_tf[:3, 3] - self._tracker_zero[side][:3, 3]
        ) * self._tracker_position_scale_vec
        target[:3, 3] = self._robot_zero[side][:3, 3] + tracker_delta_pos

        wrist_local_state = self._compute_wrist_local_state(side, tracker_tf)
        self._log_wrist_local_debug(side, wrist_local_state)
        follow_pos = np.array(target[:3, 3], dtype=np.float64)
        if self._last_target_pose[side] is not None:
            hold_pos = np.array(self._last_target_pose[side][:3, 3], dtype=np.float64)
        else:
            hold_pos = np.array(self._robot_zero[side][:3, 3], dtype=np.float64)
        if (
            self._tracker_wrist_local_enable
            and self._tracker_wrist_local_position_blend_enable
            and self._tracker_orientation_mode == "full_pose"
        ):
            weight_info = self._compute_wrist_local_weight(side, wrist_local_state)
            w = self._clamp01(float(weight_info.get("filtered_weight", 0.0)))
            blended_pos = (1.0 - w) * follow_pos + w * hold_pos
            target[:3, 3] = blended_pos
            self._log_wrist_local_blend(
                side,
                weight_info,
                follow_pos,
                hold_pos,
                blended_pos,
            )
        elif (
            self._tracker_wrist_local_enable
            and self._tracker_wrist_local_hold_position
            and self._tracker_orientation_mode == "full_pose"
            and wrist_local_state.get("available", False)
            and wrist_local_state.get("is_wrist_local", False)
        ):
            if self._tracker_wrist_local_hold_position_mode == "last_target":
                last_target = self._last_target_pose[side]
                if last_target is not None:
                    target[:3, 3] = last_target[:3, 3]
                else:
                    target[:3, 3] = self._robot_zero[side][:3, 3]
            elif self._tracker_wrist_local_hold_position_mode == "robot_zero":
                target[:3, 3] = self._robot_zero[side][:3, 3]
            self._log_wrist_local_hold(side, target, wrist_local_state)

        target = self._apply_workspace_clamp(side, target)

        if self._tracker_orientation_mode == "full_pose":
            if self._tracker_ori_zero[side] is None:
                self._tracker_ori_zero[side] = np.array(orientation_source_tf[:3, :3], dtype=np.float64)
            tracker_delta_rot = self._tracker_ori_zero[side].T @ orientation_source_tf[:3, :3]
            mapped_delta_rot = self._map_tracker_delta_rotation(tracker_delta_rot, side)
            raw_target_rot = self._robot_zero[side][:3, :3] @ mapped_delta_rot
            blended_rot = self._blend_rotation(
                self._robot_zero[side][:3, :3],
                raw_target_rot,
            )
            stabilized_rot = self._stabilize_orientation(side, blended_rot)
            self._log_tracker_orientation_debug(
                side,
                tracker_delta_rot,
                mapped_delta_rot,
                blended_rot,
                stabilized_rot=stabilized_rot,
            )
            target[:3, :3] = stabilized_rot
        else:
            target[:3, :3] = self._robot_zero[side][:3, :3]

        self._set_pose_matrix_m(side, target)
        self._last_target_pose[side] = np.array(target, dtype=np.float64)
        return self._matrix_to_pose(target)

    def _set_pose_matrix_m(self, side: str, matrix: Optional[np.ndarray]) -> None:
        if side == "left":
            self.left_pose_mat_m = None if matrix is None else np.array(matrix, dtype=np.float64)
        else:
            self.right_pose_mat_m = None if matrix is None else np.array(matrix, dtype=np.float64)

    def _apply_position_limit(self, matrix: np.ndarray) -> np.ndarray:
        """Clamp target translation radius in the current control frame."""
        if self._tracker_position_limit_m <= 0.0:
            return matrix
        limited = np.array(matrix, dtype=np.float64)
        pos = limited[:3, 3]
        radius = float(np.linalg.norm(pos))
        if radius > self._tracker_position_limit_m:
            limited[:3, 3] = pos * (self._tracker_position_limit_m / radius)
        return limited

    def _apply_position_bounds(self, matrix: np.ndarray) -> np.ndarray:
        """Clamp target translation by axis in the current control frame."""
        bounded = np.array(matrix, dtype=np.float64)
        bounded[:3, 3] = np.minimum(
            np.maximum(bounded[:3, 3], self._tracker_position_min_m),
            self._tracker_position_max_m,
        )
        return bounded

    def _apply_workspace_clamp(self, side: str, matrix: np.ndarray) -> np.ndarray:
        """Clamp Pinocchio target translation in robot-base meters."""
        clamped = np.array(matrix, dtype=np.float64)
        before = np.array(clamped[:3, 3], dtype=np.float64)
        pos = np.minimum(np.maximum(before, self._workspace_min_m), self._workspace_max_m)

        radius = float(np.linalg.norm(pos))
        if radius > 1e-9:
            if self._workspace_max_radius_m > 0.0 and radius > self._workspace_max_radius_m:
                pos = pos * (self._workspace_max_radius_m / radius)
            elif self._workspace_min_radius_m > 0.0 and radius < self._workspace_min_radius_m:
                pos = pos * (self._workspace_min_radius_m / radius)
        elif self._workspace_min_radius_m > 0.0:
            pos[0] = self._workspace_min_radius_m

        clamped[:3, 3] = pos
        if not np.allclose(before, pos, atol=1e-6):
            now = time.monotonic()
            if now - self._last_workspace_clamp_log_time[side] >= 1.0:
                self._last_workspace_clamp_log_time[side] = now
                self.get_logger().info(
                    f"[WORKSPACE_CLAMP] side={side} "
                    f"before={[round(float(v), 4) for v in before]} "
                    f"after={[round(float(v), 4) for v in pos]}"
                )
        return clamped

    def _kine_matrix_to_pose(self, side: str, matrix_m: np.ndarray) -> np.ndarray:
        matrix_mm = np.array(matrix_m, dtype=np.float64)
        matrix_mm[:3, 3] *= 1000.0
        matrix_list = matrix_mm.tolist()
        kine = self.controller.kine_left if side == "left" else self.controller.kine_right
        if kine is None:
            return self._matrix_to_pose(np.array(matrix_m, dtype=np.float64))
        pose = kine.mat4x4_to_xyzabc(matrix_list)
        if pose is False:
            raise RuntimeError(f"{side} matrix to xyzabc failed")
        pose_m = list(pose)
        for i in range(3):
            pose_m[i] *= 0.001
        return np.array(pose_m, dtype=np.float64)

    def _current_robot_fk_matrix(self, side: str) -> Optional[np.ndarray]:
        try:
            left_joints, right_joints = self.controller.get_current_joints()
            serial = 0 if side == "left" else 1
            joints = left_joints if side == "left" else right_joints
            if joints is None:
                return None
            fk = self.controller.fk_mat4x4(serial, list(joints))
            if fk is False:
                return None
            matrix = np.array(fk, dtype=np.float64)
            matrix[:3, 3] *= 0.001
            return matrix
        except Exception as e:
            self.get_logger().warn(f"Failed to initialize {side} tracker zero from FK: {e}")
            return None

    def _current_wrist_joints(self, side: str) -> Optional[list]:
        try:
            left_joints, right_joints = self.controller.get_current_joints()
            joints = left_joints if side == "left" else right_joints
            if joints is None:
                return None
            return [float(joints[i]) for i in (4, 5, 6)]
        except Exception as e:
            self.get_logger().warn(f"Failed to initialize {side} wrist joint zero: {e}")
            return None

    def shutdown(self):
        self.get_logger().info("Shutting down...")
        self._keyboard_stop_event.set()
        if self._keyboard_thread is not None:
            self._keyboard_thread.join(timeout=1.0)
        self.controller.disable_and_release()
        self.get_logger().info("Safely exited")


# -------------------- Entry Function --------------------

def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Tianji arm controller")
    parser.add_argument("-c", "--config", help="Configuration file path")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Compute and print commands without connecting to or driving the robot.",
    )
    parser.add_argument(
        "--read-only",
        action="store_true",
        help="Connect to the robot and publish feedback only; never send commands.",
    )
    parser.add_argument(
        "--feedback-handshake",
        action="store_true",
        help=(
            "In read-only mode, send one non-motion SDK command sequence "
            "to start robot feedback streaming."
        ),
    )
    parser.add_argument(
        "--read-only-connect-timeout",
        type=float,
        help=(
            "Seconds to wait for robot feedback connection in read-only mode "
            "before falling back to simulated feedback."
        ),
    )
    parser.add_argument(
        "--tracker-mode",
        choices=["absolute", "incremental"],
        help="absolute uses tracker pose directly; incremental applies tracker deltas to the robot start pose.",
    )
    parser.add_argument(
        "--tracker-position-scale",
        type=float,
        help="Scale incremental tracker translation before applying it to the robot start pose.",
    )
    parser.add_argument(
        "--tracker-position-scale-x",
        type=float,
        help="Additional scale for tracker X translation.",
    )
    parser.add_argument(
        "--tracker-position-scale-y",
        type=float,
        help="Additional scale for tracker Y translation.",
    )
    parser.add_argument(
        "--tracker-position-scale-z",
        type=float,
        help="Additional scale for tracker Z translation.",
    )
    parser.add_argument(
        "--tracker-position-limit-m",
        type=float,
        help="Clamp tracker target translation radius in meters; <=0 disables clamping.",
    )
    parser.add_argument("--tracker-position-min-x-m", type=float, help="Minimum tracker target X in meters.")
    parser.add_argument("--tracker-position-max-x-m", type=float, help="Maximum tracker target X in meters.")
    parser.add_argument("--tracker-position-min-y-m", type=float, help="Minimum tracker target Y in meters.")
    parser.add_argument("--tracker-position-max-y-m", type=float, help="Maximum tracker target Y in meters.")
    parser.add_argument("--tracker-position-min-z-m", type=float, help="Minimum tracker target Z in meters.")
    parser.add_argument("--tracker-position-max-z-m", type=float, help="Maximum tracker target Z in meters.")
    parser.add_argument(
        "--control-rate-hz",
        type=float,
        help="Controller loop frequency in Hz.",
    )
    parser.add_argument(
        "--tracker-orientation-mode",
        choices=["full", "full_pose", "position_only", "wrist_only", "yaw_only", "blended"],
        help=(
            "full/full_pose follows tracker rotation; yaw_only follows tracker heading only; "
            "blended partially follows it; position_only keeps the startup TCP orientation; "
            "wrist_only keeps IK position-only and maps tracker rotation onto joints 5-7."
        ),
    )
    parser.add_argument(
        "--tracker-orientation-blend",
        type=float,
        help="Blended orientation strength from 0.0 to 1.0; only used when tracker-orientation-mode=blended.",
    )
    parser.add_argument(
        "--tracker-orientation-filter-alpha",
        type=float,
        help="Orientation smoothing strength per control step; 1.0 disables smoothing lag.",
    )
    parser.add_argument(
        "--tracker-orientation-max-step-deg",
        type=float,
        help="Maximum tracker orientation step per control frame in degrees; <=0 disables rate limiting.",
    )
    parser.add_argument(
        "--wrist-orientation-scale",
        type=float,
        help="Overall scale for tracker orientation before adding it to wrist joints in wrist_only mode.",
    )
    parser.add_argument(
        "--wrist-orientation-scale-x",
        type=float,
        help="Additional wrist_only scale for tracker roll mapped to joint 5.",
    )
    parser.add_argument(
        "--wrist-orientation-scale-y",
        type=float,
        help="Additional wrist_only scale for tracker pitch mapped to joint 6.",
    )
    parser.add_argument(
        "--wrist-orientation-scale-z",
        type=float,
        help="Additional wrist_only scale for tracker yaw mapped to joint 7.",
    )
    parser.add_argument(
        "--wrist-orientation-axis-order",
        choices=["xyz", "xzy", "yxz", "yzx", "zxy", "zyx"],
        help="Axis order used to map tracker wrist rotation deltas onto joints 5, 6, 7.",
    )
    parser.add_argument(
        "--wrist-orientation-decompose",
        choices=["rotvec", "euler"],
        help="How to decompose wrist rotation; rotvec reduces Euler cross-axis coupling.",
    )
    parser.add_argument(
        "--wrist-orientation-max-deg",
        type=float,
        help="Clamp each wrist orientation joint delta in degrees; <=0 disables this clamp.",
    )
    parser.add_argument(
        "--ik-reference-mode",
        choices=["feedback", "last_success"],
        help="feedback uses live robot joints as IK reference; last_success uses the last valid IK solution.",
    )
    parser.add_argument(
        "--tracker-start-delay-sec",
        type=float,
        help="Seconds to wait after startup before capturing tracker zero and running IK.",
    )
    return parser.parse_args(argv)


def _finite_or_none(value):
    if value is None:
        return None
    value = float(value)
    if not np.isfinite(value):
        return None
    return value


def main(argv: Optional[list[str]] = None):
    program_name = sys.argv[0] if sys.argv else "tianji_arm_controller"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    cli_argv = [arg for arg in remove_ros_args(raw_argv)[1:] if arg]
    args = _parse_args(cli_argv)

    # Load configuration
    config_path = args.config or get_package_config_path("tianji_output", "tianji_output.yaml")
    config = load_yaml_config(config_path)

    rclpy.init(args=raw_argv)
    dry_run = bool(args.dry_run or config.get("dry_run", False))
    read_only = bool(args.read_only or config.get("read_only", False))
    feedback_handshake = bool(args.feedback_handshake or config.get("feedback_handshake", False))
    read_only_connect_timeout = float(
        args.read_only_connect_timeout
        if args.read_only_connect_timeout is not None
        else config.get("read_only_connect_timeout", 3.0)
    )
    tracker_mode = args.tracker_mode or config.get("tracker_mode", "absolute")
    tracker_position_scale = float(
        args.tracker_position_scale
        if args.tracker_position_scale is not None
        else config.get("tracker_position_scale", 1.0)
    )
    tracker_position_scale_x = float(
        args.tracker_position_scale_x
        if args.tracker_position_scale_x is not None
        else config.get("tracker_position_scale_x", 1.0)
    )
    tracker_position_scale_y = float(
        args.tracker_position_scale_y
        if args.tracker_position_scale_y is not None
        else config.get("tracker_position_scale_y", 1.0)
    )
    tracker_position_scale_z = float(
        args.tracker_position_scale_z
        if args.tracker_position_scale_z is not None
        else config.get("tracker_position_scale_z", 1.0)
    )
    tracker_position_limit_m = float(
        args.tracker_position_limit_m
        if args.tracker_position_limit_m is not None
        else config.get("tracker_position_limit_m", 0.0)
    )
    tracker_position_min_x_m = _finite_or_none(args.tracker_position_min_x_m)
    if tracker_position_min_x_m is None:
        tracker_position_min_x_m = _finite_or_none(config.get("tracker_position_min_x_m"))
    tracker_position_max_x_m = _finite_or_none(args.tracker_position_max_x_m)
    if tracker_position_max_x_m is None:
        tracker_position_max_x_m = _finite_or_none(config.get("tracker_position_max_x_m"))
    tracker_position_min_y_m = _finite_or_none(args.tracker_position_min_y_m)
    if tracker_position_min_y_m is None:
        tracker_position_min_y_m = _finite_or_none(config.get("tracker_position_min_y_m"))
    tracker_position_max_y_m = _finite_or_none(args.tracker_position_max_y_m)
    if tracker_position_max_y_m is None:
        tracker_position_max_y_m = _finite_or_none(config.get("tracker_position_max_y_m"))
    tracker_position_min_z_m = _finite_or_none(args.tracker_position_min_z_m)
    if tracker_position_min_z_m is None:
        tracker_position_min_z_m = _finite_or_none(config.get("tracker_position_min_z_m"))
    tracker_position_max_z_m = _finite_or_none(args.tracker_position_max_z_m)
    if tracker_position_max_z_m is None:
        tracker_position_max_z_m = _finite_or_none(config.get("tracker_position_max_z_m"))
    control_rate_hz = float(args.control_rate_hz or config.get("control_rate_hz", 30.0))
    tracker_orientation_mode = (
        args.tracker_orientation_mode
        or config.get("tracker_orientation_mode", "wrist_only")
    )
    tracker_orientation_blend = float(
        args.tracker_orientation_blend
        if args.tracker_orientation_blend is not None
        else config.get("tracker_orientation_blend", 1.0)
    )
    tracker_orientation_filter_alpha = float(
        args.tracker_orientation_filter_alpha
        if args.tracker_orientation_filter_alpha is not None
        else config.get("tracker_orientation_filter_alpha", 0.35)
    )
    tracker_orientation_max_step_deg = float(
        args.tracker_orientation_max_step_deg
        if args.tracker_orientation_max_step_deg is not None
        else config.get("tracker_orientation_max_step_deg", 25.0)
    )
    tracker_orientation_input_mode = str(config.get("tracker_orientation_input_mode", "mapped_tf"))
    tracker_orientation_source_frame_right = str(
        config.get("tracker_orientation_source_frame_right", "right_wrist")
    )
    tracker_orientation_source_frame_left = str(
        config.get("tracker_orientation_source_frame_left", "left_wrist")
    )
    tracker_orientation_map_mode = str(config.get("tracker_orientation_map_mode", "identity"))
    tracker_orientation_map_matrix = config.get("tracker_orientation_map_matrix")
    tracker_orientation_map_mirror_left = bool(config.get("tracker_orientation_map_mirror_left", False))
    tracker_orientation_debug = bool(config.get("tracker_orientation_debug", False))
    tracker_wrist_local_enable = bool(config.get("tracker_wrist_local_enable", False))
    tracker_wrist_local_hold_position = bool(config.get("tracker_wrist_local_hold_position", False))
    tracker_wrist_local_hold_position_mode = str(
        config.get("tracker_wrist_local_hold_position_mode", "last_target")
    )
    tracker_wrist_local_debug = bool(config.get("tracker_wrist_local_debug", False))
    tracker_wrist_local_pos_threshold_m = float(
        config.get("tracker_wrist_local_pos_threshold_m", 0.025)
    )
    tracker_wrist_local_rot_threshold_deg = float(
        config.get("tracker_wrist_local_rot_threshold_deg", 5.0)
    )
    tracker_wrist_local_position_blend_enable = bool(
        config.get("tracker_wrist_local_position_blend_enable", False)
    )
    tracker_wrist_local_weight_filter_alpha = float(
        config.get("tracker_wrist_local_weight_filter_alpha", 0.85)
    )
    tracker_wrist_local_pos_start_m = float(
        config.get("tracker_wrist_local_pos_start_m", 0.015)
    )
    tracker_wrist_local_pos_full_m = float(
        config.get("tracker_wrist_local_pos_full_m", 0.055)
    )
    tracker_wrist_local_rot_start_deg = float(
        config.get("tracker_wrist_local_rot_start_deg", 3.0)
    )
    tracker_wrist_local_rot_full_deg = float(
        config.get("tracker_wrist_local_rot_full_deg", 12.0)
    )
    wrist_orientation_scale = float(
        args.wrist_orientation_scale
        if args.wrist_orientation_scale is not None
        else config.get("wrist_orientation_scale", 1.0)
    )
    wrist_orientation_scale_x = float(
        args.wrist_orientation_scale_x
        if args.wrist_orientation_scale_x is not None
        else config.get("wrist_orientation_scale_x", 1.0)
    )
    wrist_orientation_scale_y = float(
        args.wrist_orientation_scale_y
        if args.wrist_orientation_scale_y is not None
        else config.get("wrist_orientation_scale_y", 1.0)
    )
    wrist_orientation_scale_z = float(
        args.wrist_orientation_scale_z
        if args.wrist_orientation_scale_z is not None
        else config.get("wrist_orientation_scale_z", 1.0)
    )
    wrist_orientation_axis_order = (
        args.wrist_orientation_axis_order
        or config.get("wrist_orientation_axis_order", "zxy")
    )
    wrist_orientation_decompose = (
        args.wrist_orientation_decompose
        or config.get("wrist_orientation_decompose", "rotvec")
    )
    wrist_orientation_axis_basis = config.get("wrist_orientation_axis_basis")
    wrist_orientation_max_deg = float(
        args.wrist_orientation_max_deg
        if args.wrist_orientation_max_deg is not None
        else config.get("wrist_orientation_max_deg", 60.0)
    )
    ik_reference_mode = args.ik_reference_mode or config.get("ik_reference_mode", "last_success")
    ik_subprocess_isolate = bool(config.get("ik_subprocess_isolate", False))
    ik_subprocess_timeout_sec = float(config.get("ik_subprocess_timeout_sec", 2.0))
    ik_subprocess_ready_timeout_sec = float(config.get("ik_subprocess_ready_timeout_sec", 45.0))
    ik_subprocess_max_rate_hz = float(config.get("ik_subprocess_max_rate_hz", 25.0))
    ik_subprocess_max_branch_delta_deg = float(
        config.get("ik_subprocess_max_branch_delta_deg", 25.0)
    )
    tracker_start_delay_sec = float(
        args.tracker_start_delay_sec
        if args.tracker_start_delay_sec is not None
        else config.get("tracker_start_delay_sec", 0.0)
    )
    use_pinocchio_ik = bool(config.get("use_pinocchio_ik", False))
    pinocchio_urdf_path = str(config.get("pinocchio_urdf_path", "") or "")
    pinocchio_right_ee_frame = str(config.get("pinocchio_right_ee_frame", "tool0") or "tool0")
    pinocchio_left_ee_frame = str(config.get("pinocchio_left_ee_frame", "tool0") or "tool0")
    ik_frame_scan_debug = bool(config.get("ik_frame_scan_debug", False))
    motor_unit = str(config.get("motor_unit", "deg") or "deg")
    workspace_min_x_m = float(config.get("workspace_min_x_m", 0.20))
    workspace_max_x_m = float(config.get("workspace_max_x_m", 0.75))
    workspace_min_y_m = float(config.get("workspace_min_y_m", -0.45))
    workspace_max_y_m = float(config.get("workspace_max_y_m", 0.45))
    workspace_min_z_m = float(config.get("workspace_min_z_m", 0.05))
    workspace_max_z_m = float(config.get("workspace_max_z_m", 0.70))
    workspace_min_radius_m = float(config.get("workspace_min_radius_m", 0.22))
    workspace_max_radius_m = float(config.get("workspace_max_radius_m", 0.80))
    ik_position_weight = float(config.get("ik_position_weight", 1.0))
    ik_orientation_weight = float(config.get("ik_orientation_weight", 0.25))
    ik_orientation_weight_near_singularity = float(
        config.get("ik_orientation_weight_near_singularity", 0.05)
    )
    ik_base_damping = float(config.get("ik_base_damping", 1e-4))
    ik_max_damping = float(config.get("ik_max_damping", 5e-2))
    ik_max_iters = int(config.get("ik_max_iters", 30))
    ik_dt = float(config.get("ik_dt", 0.5))
    ik_pos_eps_m = float(config.get("ik_pos_eps_m", 0.005))
    ik_ori_eps_rad = float(config.get("ik_ori_eps_rad", 0.08))
    ik_max_dq_step_deg = float(config.get("ik_max_dq_step_deg", 3.0))
    singularity_sigma_min_warn = float(config.get("singularity_sigma_min_warn", 0.04))
    singularity_sigma_min_critical = float(config.get("singularity_sigma_min_critical", 0.015))
    singularity_condition_warn = float(config.get("singularity_condition_warn", 80.0))
    singularity_condition_critical = float(config.get("singularity_condition_critical", 200.0))
    max_joint_step_deg = float(config.get("max_joint_step_deg", 8.0))
    pinocchio_j2_axis_constraint_enable = bool(
        config.get("pinocchio_j2_axis_constraint_enable", False)
    )
    pinocchio_j2_axis_constraint_hard = bool(
        config.get("pinocchio_j2_axis_constraint_hard", False)
    )
    pinocchio_j2_axis_constraint_weight = float(
        config.get("pinocchio_j2_axis_constraint_weight", 0.0)
    )
    pinocchio_j2_axis_constraint_gain = float(
        config.get("pinocchio_j2_axis_constraint_gain", 1.0)
    )
    pinocchio_j2_axis_constraint_joint_index = int(
        config.get("pinocchio_j2_axis_constraint_joint_index", 1)
    )
    pinocchio_j2_axis_constraint_max_delta_deg = float(
        config.get("pinocchio_j2_axis_constraint_max_delta_deg", 60.0)
    )
    pinocchio_j2_axis_constraint_max_step_deg = float(
        config.get("pinocchio_j2_axis_constraint_max_step_deg", 8.0)
    )
    keyboard_teleop_gate = bool(config.get("keyboard_teleop_gate", False))
    keyboard_start_key = str(config.get("keyboard_start_key", "B"))
    keyboard_stop_key = str(config.get("keyboard_stop_key", "E"))
    keyboard_start_align_sec = float(config.get("keyboard_start_align_sec", 3.0))
    keyboard_zero_warmup_cycles = int(config.get("keyboard_zero_warmup_cycles", 0))
    teleop_ik_grace_cycles = int(config.get("teleop_ik_grace_cycles", 0))
    keyboard_align_max_ik_delta_deg = float(config.get("keyboard_align_max_ik_delta_deg", 20.0))
    teleop_ik_max_step_deg = float(config.get("teleop_ik_max_step_deg", 10.0))
    init_move_sides = str(config.get("init_move_sides", "both"))
    init_move_duration_sec = float(config.get("init_move_duration_sec", 3.0))
    teleop_active_sides = str(config.get("teleop_active_sides", "right"))
    node = TianjiArmControllerNode(
        robot_ip=config.get("robot_ip", "192.168.1.190"),
        dry_run=dry_run,
        read_only=read_only,
        feedback_handshake=feedback_handshake,
        read_only_connect_timeout=read_only_connect_timeout,
        tracker_mode=tracker_mode,
        tracker_position_scale=tracker_position_scale,
        tracker_position_scale_x=tracker_position_scale_x,
        tracker_position_scale_y=tracker_position_scale_y,
        tracker_position_scale_z=tracker_position_scale_z,
        tracker_position_limit_m=tracker_position_limit_m,
        tracker_position_min_x_m=tracker_position_min_x_m,
        tracker_position_max_x_m=tracker_position_max_x_m,
        tracker_position_min_y_m=tracker_position_min_y_m,
        tracker_position_max_y_m=tracker_position_max_y_m,
        tracker_position_min_z_m=tracker_position_min_z_m,
        tracker_position_max_z_m=tracker_position_max_z_m,
        control_rate_hz=control_rate_hz,
        tracker_orientation_mode=tracker_orientation_mode,
        tracker_orientation_blend=tracker_orientation_blend,
        tracker_orientation_filter_alpha=tracker_orientation_filter_alpha,
        tracker_orientation_max_step_deg=tracker_orientation_max_step_deg,
        tracker_orientation_input_mode=tracker_orientation_input_mode,
        tracker_orientation_source_frame_right=tracker_orientation_source_frame_right,
        tracker_orientation_source_frame_left=tracker_orientation_source_frame_left,
        tracker_orientation_map_mode=tracker_orientation_map_mode,
        tracker_orientation_map_matrix=tracker_orientation_map_matrix,
        tracker_orientation_map_mirror_left=tracker_orientation_map_mirror_left,
        tracker_orientation_debug=tracker_orientation_debug,
        tracker_wrist_local_enable=tracker_wrist_local_enable,
        tracker_wrist_local_hold_position=tracker_wrist_local_hold_position,
        tracker_wrist_local_hold_position_mode=tracker_wrist_local_hold_position_mode,
        tracker_wrist_local_debug=tracker_wrist_local_debug,
        tracker_wrist_local_pos_threshold_m=tracker_wrist_local_pos_threshold_m,
        tracker_wrist_local_rot_threshold_deg=tracker_wrist_local_rot_threshold_deg,
        tracker_wrist_local_position_blend_enable=tracker_wrist_local_position_blend_enable,
        tracker_wrist_local_weight_filter_alpha=tracker_wrist_local_weight_filter_alpha,
        tracker_wrist_local_pos_start_m=tracker_wrist_local_pos_start_m,
        tracker_wrist_local_pos_full_m=tracker_wrist_local_pos_full_m,
        tracker_wrist_local_rot_start_deg=tracker_wrist_local_rot_start_deg,
        tracker_wrist_local_rot_full_deg=tracker_wrist_local_rot_full_deg,
        wrist_orientation_scale=wrist_orientation_scale,
        wrist_orientation_scale_x=wrist_orientation_scale_x,
        wrist_orientation_scale_y=wrist_orientation_scale_y,
        wrist_orientation_scale_z=wrist_orientation_scale_z,
        wrist_orientation_axis_order=wrist_orientation_axis_order,
        wrist_orientation_decompose=wrist_orientation_decompose,
        wrist_orientation_axis_basis=wrist_orientation_axis_basis,
        wrist_orientation_max_deg=wrist_orientation_max_deg,
        ik_reference_mode=ik_reference_mode,
        ik_subprocess_isolate=ik_subprocess_isolate,
        ik_subprocess_timeout_sec=ik_subprocess_timeout_sec,
        ik_subprocess_ready_timeout_sec=ik_subprocess_ready_timeout_sec,
        ik_subprocess_max_rate_hz=ik_subprocess_max_rate_hz,
        ik_subprocess_max_branch_delta_deg=ik_subprocess_max_branch_delta_deg,
        tracker_start_delay_sec=tracker_start_delay_sec,
        use_pinocchio_ik=use_pinocchio_ik,
        pinocchio_urdf_path=pinocchio_urdf_path,
        pinocchio_right_ee_frame=pinocchio_right_ee_frame,
        pinocchio_left_ee_frame=pinocchio_left_ee_frame,
        ik_frame_scan_debug=ik_frame_scan_debug,
        motor_unit=motor_unit,
        workspace_min_x_m=workspace_min_x_m,
        workspace_max_x_m=workspace_max_x_m,
        workspace_min_y_m=workspace_min_y_m,
        workspace_max_y_m=workspace_max_y_m,
        workspace_min_z_m=workspace_min_z_m,
        workspace_max_z_m=workspace_max_z_m,
        workspace_min_radius_m=workspace_min_radius_m,
        workspace_max_radius_m=workspace_max_radius_m,
        ik_position_weight=ik_position_weight,
        ik_orientation_weight=ik_orientation_weight,
        ik_orientation_weight_near_singularity=ik_orientation_weight_near_singularity,
        ik_base_damping=ik_base_damping,
        ik_max_damping=ik_max_damping,
        ik_max_iters=ik_max_iters,
        ik_dt=ik_dt,
        ik_pos_eps_m=ik_pos_eps_m,
        ik_ori_eps_rad=ik_ori_eps_rad,
        ik_max_dq_step_deg=ik_max_dq_step_deg,
        singularity_sigma_min_warn=singularity_sigma_min_warn,
        singularity_sigma_min_critical=singularity_sigma_min_critical,
        singularity_condition_warn=singularity_condition_warn,
        singularity_condition_critical=singularity_condition_critical,
        max_joint_step_deg=max_joint_step_deg,
        pinocchio_j2_axis_constraint_enable=pinocchio_j2_axis_constraint_enable,
        pinocchio_j2_axis_constraint_hard=pinocchio_j2_axis_constraint_hard,
        pinocchio_j2_axis_constraint_weight=pinocchio_j2_axis_constraint_weight,
        pinocchio_j2_axis_constraint_gain=pinocchio_j2_axis_constraint_gain,
        pinocchio_j2_axis_constraint_joint_index=pinocchio_j2_axis_constraint_joint_index,
        pinocchio_j2_axis_constraint_max_delta_deg=pinocchio_j2_axis_constraint_max_delta_deg,
        pinocchio_j2_axis_constraint_max_step_deg=pinocchio_j2_axis_constraint_max_step_deg,
        keyboard_teleop_gate=keyboard_teleop_gate,
        keyboard_start_key=keyboard_start_key,
        keyboard_stop_key=keyboard_stop_key,
        keyboard_start_align_sec=keyboard_start_align_sec,
        keyboard_zero_warmup_cycles=keyboard_zero_warmup_cycles,
        teleop_ik_grace_cycles=teleop_ik_grace_cycles,
        keyboard_align_max_ik_delta_deg=keyboard_align_max_ik_delta_deg,
        teleop_ik_max_step_deg=teleop_ik_max_step_deg,
        init_move_sides=init_move_sides,
        init_move_duration_sec=init_move_duration_sec,
        teleop_active_sides=teleop_active_sides,
    )

    executor = None
    try:
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if executor is not None:
            with contextlib.suppress(Exception):
                executor.shutdown()
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
