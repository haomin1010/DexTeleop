#!/usr/bin/env python3
"""
Tianji Arm Unified Controller

Integrates Cartesian space control and joint space control:
- Cartesian space control: via end-effector pose control (IK solving)
- Joint space control: direct joint angle control
"""
try:
    from tianji_output._internal.fx_robot import Marvin_Robot
    from tianji_output._internal.fx_kine import Marvin_Kine
    from tianji_output._internal.structure_data import DCSS
except ImportError:
    from ._internal.fx_robot import Marvin_Robot
    from ._internal.fx_kine import Marvin_Kine
    from ._internal.structure_data import DCSS
import time
import logging
import os
import queue
import threading
from contextlib import contextmanager
from ament_index_python.packages import get_package_share_directory


class TianjiArmController:
    """
    Tianji Arm Unified Controller

    Supports both Cartesian space control and joint space control.
    """

    # Teleop home pose (degrees). Right arm matches sim capture / operator T-pose.
    INIT_JOINTS_LEFT = [90.0, -90.0, -90.0, -90.0, 0.0, 0.0, 0.0]
    INIT_JOINTS_RIGHT = [-90.0, -90.0, 90.0, -90.0, 0.0, 0.0, 0.0]

    def __init__(
        self,
        robot_ip='192.168.8.166',
        config_path=None,
        logger=None,
        dry_run=False,
        read_only=False,
        feedback_handshake=False,
        read_only_connect_timeout=3.0,
        prefer_last_ik_reference=False,
        ik_subprocess_isolate: bool = False,
        ik_subprocess_timeout_sec: float = 2.0,
        ik_subprocess_ready_timeout_sec: float = 45.0,
        ik_subprocess_max_rate_hz: float = 25.0,
        ik_subprocess_max_branch_delta_deg: float = 25.0,
    ):
        """
        Initialize Tianji arm controller (initializes both left and right arms)

        Args:
            robot_ip: Robot IP address
            config_path: Kinematics configuration file path
                - None: Use default 'ccs_m6.MvKDCfg'(auto-searched from ROS2 package)
                - Relative path: search from ROS2 package config directory
                - Absolute path: use this path directly
            logger: External logger (optional, for integrating ROS2 logging system)
            dry_run: If True, compute IK and log commands without connecting to the robot.
            read_only: If True, connect and read feedback only; never send robot commands.
            feedback_handshake: In read-only mode, send one non-motion SDK command sequence
                to start robot feedback streaming.
            read_only_connect_timeout: Timeout in seconds before read-only mode falls back
                to local simulated feedback. Set <= 0 to wait indefinitely.
            ik_subprocess_isolate: Run libKine IK in a spawn child process (Marvin SDK stays
                in the main process).
            ik_subprocess_timeout_sec: Per-IK call timeout when subprocess isolation is on.
            ik_subprocess_ready_timeout_sec: Max wait for child libKine init before teleop.
        """
        # Logging: prefer externally provided logger
        if logger is not None:
            self.logger = logger
        else:
            self.logger = logging.getLogger('TianjiArmController')
            self.logger.setLevel(logging.INFO)
            if not self.logger.handlers:
                handler = logging.StreamHandler()
                handler.setFormatter(logging.Formatter('[%(name)s] %(message)s'))
                self.logger.addHandler(handler)

        # Parse configuration file path
        if config_path is None:
            config_filename = 'ccs_m6.MvKDCfg'
            package_share = get_package_share_directory('tianji_output')
            config_path = os.path.join(package_share, 'config', config_filename)

        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Configuration file does not exist: {config_path}")

        self.logger.debug(f"Loading configuration file: {config_path}")
        self._config_path = config_path
        self._ik_subprocess_isolate = bool(ik_subprocess_isolate)
        self._ik_subprocess_timeout_sec = max(float(ik_subprocess_timeout_sec), 0.1)
        self._ik_subprocess_ready_timeout_sec = max(float(ik_subprocess_ready_timeout_sec), 1.0)
        self._ik_subprocess_max_rate_hz = max(float(ik_subprocess_max_rate_hz), 0.0)
        self._ik_subprocess_max_branch_delta_deg = max(
            float(ik_subprocess_max_branch_delta_deg), 0.0
        )
        self._ik_subprocess_client = None

        # ---------------------- Initialize kinematics ----------------------
        if self._ik_subprocess_isolate:
            self.logger.debug(
                "IK isolate: libKine FK/IK in subprocess only; parent uses Marvin SDK"
            )
            # Do not call LOADMvCfg / libKine in the parent — only the child may touch it.
            self.kine_left = None
            self.kine_right = None
            self._joint_limits = None
        else:
            self.logger.debug("[Arm A] Initializing kinematics SDK...")
            self.kine_left = Marvin_Kine()
            config_result = self.kine_left.load_config(config_path=config_path)
            time.sleep(0.3)
            self.kine_left.initial_kine(
                robot_serial=0,
                robot_type=config_result['TYPE'][0],
                dh=config_result['DH'][0],
                pnva=config_result['PNVA'][0],
                j67=config_result['BD'][0]
            )

            self.logger.debug("[Arm B] Initializing kinematics SDK...")
            self.kine_right = Marvin_Kine()
            config_result = self.kine_right.load_config(config_path=config_path)
            time.sleep(0.3)
            self.kine_right.initial_kine(
                robot_serial=1,
                robot_type=config_result['TYPE'][1],
                dh=config_result['DH'][1],
                pnva=config_result['PNVA'][1],
                j67=config_result['BD'][1]
            )
            self._joint_limits = {
                'left': self._parse_joint_limits(config_result['PNVA'][0]),
                'right': self._parse_joint_limits(config_result['PNVA'][1]),
            }

        self.dry_run = dry_run
        self.read_only = read_only
        self.feedback_handshake = feedback_handshake
        self.read_only_connect_timeout = read_only_connect_timeout
        self.prefer_last_ik_reference = prefer_last_ik_reference
        if self._ik_subprocess_isolate and self.prefer_last_ik_reference:
            self.logger.info(
                "IK subprocess: seed IK from robot feedback (not last_success); "
                "avoids bad branch + libKine state corruption"
            )
            self.prefer_last_ik_reference = False
        self.robot = None
        self._dry_left_joints = list(self.INIT_JOINTS_LEFT)
        self._dry_right_joints = list(self.INIT_JOINTS_RIGHT)
        self._last_left_ik_joints = None
        self._last_right_ik_joints = None
        self._sdk_lock = threading.RLock()
        self._last_ik_failure_log_time = 0.0
        self._last_pose_log_time = 0.0
        self._last_joint_command_log_time = 0.0
        self._ik_reject_restart_cooldown_sec = 1.0
        self._last_ik_reject_restart_time = {"left": 0.0, "right": 0.0}
        self._last_ik_reject_log_time = {"left": 0.0, "right": 0.0}
        self._ik_failure_block_sec = 0.6
        self._ik_block_until = {"left": 0.0, "right": 0.0}

        if self.dry_run:
            self.logger.warning("DRY RUN enabled: robot connection and command sending are disabled")
        else:
            # ---------------------- Initialize robot connection ----------------------
            self.logger.debug("Initializing robot control...")
            self.robot, init = self._connect_robot(robot_ip)
            if self.read_only and self.robot is None:
                self.logger.warning(
                    "READ ONLY: robot feedback connection unavailable; "
                    "falling back to local simulated joint feedback for visualization."
                )
            elif init == 0:
                raise ConnectionError("Connection failed: port occupied!")
            elif init is None:
                raise ConnectionError("Connection failed!")

            if self.robot is None:
                init = 0

            if init == 0:
                if self.read_only:
                    self.logger.warning("READ ONLY: continuing without robot feedback")
                else:
                    raise ConnectionError("Connection failed: port occupied!")

            if self.robot is not None:
                time.sleep(0.5)
                if self.read_only:
                    self.logger.warning("READ ONLY enabled: no robot motion commands will be sent")
                    if self.feedback_handshake:
                        self._start_feedback_stream_read_only()
                else:
                    self.robot.clear_set()
                    self.robot.clear_error('A')
                    self.robot.clear_error('B')
                    self.robot.send_cmd()
                    time.sleep(0.5)

                if not self._verify_connection():
                    if self.read_only:
                        self.logger.warning(
                            "Robot feedback did not update; falling back to local simulated joint feedback. "
                            "Check robot IP/network if you need live physical joint angles."
                        )
                        self.robot = None
                    else:
                        raise ConnectionError("Robot connection failed!")

            # Save as shared instance
            TianjiArmController._shared_robot = self.robot
        TianjiArmController._shared_kine_left = self.kine_left
        TianjiArmController._shared_kine_right = self.kine_right
        TianjiArmController._initialized = True

        # ---------------------- IK parameters (can be modified in real-time)----------------------
        self.zsp_type = 1                           # Nullspace constraint type
        self.left_zsp_para = [0, -1, -1, 0, 0, 0]   # Left arm nullspace reference plane parameters
        self.right_zsp_para = [0, 1, -1, 0, 0, 0]   # Right arm nullspace reference plane parameters
        self.zsp_angle = 0.0                        # Nullspace arm angle rotation angle
        self.dgr = [5.0, 5.0, 5.0]                  # Singularity tolerance angle range

        if self._ik_subprocess_isolate:
            from .ik_subprocess_worker import IkSubprocessClient

            self._ik_subprocess_client = IkSubprocessClient(
                config_path=self._config_path,
                logger=self.logger,
                timeout_sec=self._ik_subprocess_timeout_sec,
                ready_timeout_sec=self._ik_subprocess_ready_timeout_sec,
                max_rate_hz=self._ik_subprocess_max_rate_hz,
                on_worker_restart=self._on_ik_subprocess_restart,
            )
            if not self._ik_subprocess_client._ready:
                raise RuntimeError(
                    "IK subprocess worker failed to initialize; "
                    "check libKine.so / ccs_m6.MvKDCfg in container"
                )
            pnva = self._ik_subprocess_client.pnva
            if pnva is None:
                raise RuntimeError("IK subprocess worker ready but PNVA limits missing")
            self._joint_limits = {
                "left": self._parse_joint_limits(pnva[0]),
                "right": self._parse_joint_limits(pnva[1]),
            }
            self.logger.info(
                "IK subprocess isolation ON: libKine FK/IK in child, Marvin SDK in parent"
            )

        # Set tool parameters (wuji hand)
        if not self.dry_run and not self.read_only:
            self._set_tool_params()

        self.logger.info("Dual-arm controller initialization complete")

    def _connect_robot(self, robot_ip):
        if not self.read_only or self.read_only_connect_timeout <= 0:
            robot = Marvin_Robot()
            return robot, robot.connect(robot_ip)

        result_queue = queue.Queue(maxsize=1)

        def _worker():
            try:
                robot = Marvin_Robot()
                result_queue.put((robot, robot.connect(robot_ip), None))
            except Exception as exc:
                result_queue.put((None, None, exc))

        thread = threading.Thread(target=_worker, name="tianji_read_only_connect", daemon=True)
        thread.start()
        try:
            robot, init, exc = result_queue.get(timeout=float(self.read_only_connect_timeout))
        except queue.Empty:
            self.logger.warning(
                f"READ ONLY: robot connect timed out after {self.read_only_connect_timeout:.1f}s"
            )
            return None, 0
        if exc is not None:
            raise exc
        return robot, init

    def _start_feedback_stream_read_only(self):
        """Ask the SDK to start feedback without sending any joint or pose targets."""
        self.logger.warning(
            "READ ONLY feedback handshake enabled: sending clear_error + one SDK send_cmd "
            "to start feedback streaming; no joint/pose target is sent."
        )
        self.robot.clear_set()
        self.robot.clear_error('A')
        self.robot.clear_error('B')
        self.robot.send_cmd()
        time.sleep(0.5)

    def _set_tool_params(self):
        """
        Set tool parameters (kinematics + dynamics)

        Kinematics parameters [X, Y, Z, A, B, C]:
            X, Y, Z: Tool center point offset relative to flange (mm)
            A, B, C: Tool orientation offset relative to flange (degrees)

        Dynamics parameters [M, mx, my, mz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]:
            M: Tool mass (kg)
            mx, my, mz: Center of mass position relative to flange (mm)
            Ixx~Izz: inertia tensor (kg·mm²)
        """
        tool_kine = [0, 0, 120, 0, 0, 0]  # Tool center point 120mm from flange
        tool_dyn = [0.874014, -24.586772, -39.924666, 232.723684, 0.003369, 0.0, 0.0, 0.001, 0.0, 0.009483]

        self.logger.debug(f"Setting tool parameters: kine={tool_kine}, dyn={tool_dyn}")

        self.robot.clear_set()
        self.robot.set_tool(arm='A', kineParams=tool_kine, dynamicParams=tool_dyn)
        self.robot.set_tool(arm='B', kineParams=tool_kine, dynamicParams=tool_dyn)
        self.robot.send_cmd()
        time.sleep(0.3)

    def _verify_connection(self):
        """Verify robot connection"""
        dcss = DCSS()
        motion_tag = 0
        frame_update = None
        for i in range(5):
            sub_data = self.robot.subscribe(dcss)
            serial = sub_data['outputs'][0]['frame_serial']
            if serial != 0 and frame_update != serial:
                motion_tag += 1
                frame_update = serial
            time.sleep(0.1)
        return motion_tag > 0

    # ==================== State Retrieval Methods ====================

    def get_current_joints(self):
        """
        Get current dual-arm joint angles

        Returns:
            tuple: (left_joints, right_joints) each is a list of 7 joint angles (degrees)
        """
        with self._sdk_lock:
            dcss = DCSS()
            if self.robot is None:
                return list(self._dry_left_joints), list(self._dry_right_joints)
            sub_data = self.robot.subscribe(dcss)
            left_joints = sub_data["outputs"][0]["fb_joint_pos"]
            right_joints = sub_data["outputs"][1]["fb_joint_pos"]
            return left_joints, right_joints

    def _on_ik_subprocess_restart(self) -> None:
        self._last_left_ik_joints = None
        self._last_right_ik_joints = None

    @staticmethod
    def _max_joint_delta_deg(a: list, b: list) -> float:
        return max(abs(float(x) - float(y)) for x, y in zip(a, b))

    def _ik_branch_delta_ok(self, side: str, candidate: list) -> bool:
        if (
            self._ik_subprocess_client is None
            or self._ik_subprocess_max_branch_delta_deg <= 0.0
        ):
            return True
        fb_left, fb_right = self.get_current_joints()
        fb = fb_right if side == "right" else fb_left
        if fb is None:
            return True
        delta = self._max_joint_delta_deg(fb, candidate)
        if delta <= self._ik_subprocess_max_branch_delta_deg:
            return True
        now = time.monotonic()
        if now - self._last_ik_reject_log_time[side] >= 1.0:
            self._last_ik_reject_log_time[side] = now
            self.logger.warning(
                f"[{side.upper()}_IK_REJECT] branch delta {delta:.1f}deg > "
                f"{self._ik_subprocess_max_branch_delta_deg:.1f} vs feedback; "
                "holding command (check static TF / wrist map)"
            )
        self._ik_block_until[side] = now + self._ik_failure_block_sec
        return False

    def _ik_side_blocked(self, side: str) -> bool:
        return time.monotonic() < self._ik_block_until[side]

    def _get_ik_reference_joints(self):
        ref_left, ref_right = self.get_current_joints()
        if self.prefer_last_ik_reference:
            if self._last_left_ik_joints is not None:
                ref_left = list(self._last_left_ik_joints)
            if self._last_right_ik_joints is not None:
                ref_right = list(self._last_right_ik_joints)
        return ref_left, ref_right

    def get_current_joint_velocities(self):
        """
        Get current dual-arm joint velocities

        Returns:
            tuple: (left_velocities, right_velocities) each is a list of 7 joint velocities (degrees/second)
        """
        dcss = DCSS()
        sub_data = self.robot.subscribe(dcss)
        left_vel = sub_data["outputs"][0]["fb_joint_vel"]
        right_vel = sub_data["outputs"][1]["fb_joint_vel"]
        return left_vel, right_vel

    def get_current_joint_torques(self):
        """
        Get current dual-arm joint torques

        Returns:
            tuple: (left_torques, right_torques) each is a list of 7 joint torques (Nm)
        """
        dcss = DCSS()
        sub_data = self.robot.subscribe(dcss)
        left_torque = sub_data["outputs"][0]["fb_joint_tor"]
        right_torque = sub_data["outputs"][1]["fb_joint_tor"]
        return left_torque, right_torque

    def get_full_state(self):
        """
        Get full robot state

        Returns:
            dict: Contains dual-arm joint position, velocity, torque and other information
        """
        dcss = DCSS()
        sub_data = self.robot.subscribe(dcss)
        return {
            'left': {
                'joints': sub_data["outputs"][0]["fb_joint_pos"],
                'velocities': sub_data["outputs"][0]["fb_joint_vel"],
                'torques': sub_data["outputs"][0]["fb_joint_tor"],
            },
            'right': {
                'joints': sub_data["outputs"][1]["fb_joint_pos"],
                'velocities': sub_data["outputs"][1]["fb_joint_vel"],
                'torques': sub_data["outputs"][1]["fb_joint_tor"],
            }
        }

    # ==================== Impedance Mode Setup ====================

    def set_impedance_mode(self, mode='joint', K=None, D=None):
        """
        Set dual-arm impedance mode

        Args:
            mode: 'joint' joint impedance or 'cart' Cartesian impedance
            K: Stiffness parameter list (7 elements)
            D: Damping parameter list (7 elements)
        """
        if self.dry_run or self.read_only or self.robot is None:
            self.logger.info(f"Skip setting {mode} impedance mode")
            return

        self.robot.clear_set()
        self.robot.set_state(arm='A', state=3)
        self.robot.set_state(arm='B', state=3)
        self.robot.set_vel_acc(arm='A', velRatio=60, AccRatio=60)
        self.robot.set_vel_acc(arm='B', velRatio=60, AccRatio=60)
        self.robot.send_cmd()
        time.sleep(0.5)

        if mode == 'cart':
            K = K or [8000, 8000, 8000, 100, 100, 100, 20]
            D = D or [0.3, 0.3, 0.3, 0.4, 0.4, 0.4, 0.4]

            self.robot.clear_set()
            self.robot.set_cart_kd_params(arm='A', K=K, D=D, type=2)
            self.robot.set_cart_kd_params(arm='B', K=K, D=D, type=2)
            time.sleep(0.5)
            self.robot.set_impedance_type(arm='A', type=2)
            self.robot.set_impedance_type(arm='B', type=2)
            self.robot.send_cmd()
            time.sleep(0.5)

            self.logger.info(f"Dual-arm Cartesian impedance mode K={K}")

        elif mode == 'joint':
            K = K or [2, 2, 2, 1.6, 1, 1, 1]
            D = D or [0.3, 0.3, 0.3, 0.2, 0.2, 0.2, 0.2]

            self.robot.clear_set()
            self.robot.set_joint_kd_params(arm='A', K=K, D=D)
            self.robot.set_joint_kd_params(arm='B', K=K, D=D)
            self.robot.send_cmd()
            time.sleep(0.5)

            self.robot.clear_set()
            self.robot.set_impedance_type(arm='A', type=1)
            self.robot.set_impedance_type(arm='B', type=1)
            self.robot.send_cmd()
            time.sleep(0.5)

            self.logger.info(f"Dual-arm joint impedance mode K={K}")

    # ==================== Cartesian Space Control Methods ====================

    def move_to_pose_direct(
        self,
        left_pose=None,
        right_pose=None,
        unit='mm',
        joint_postprocessor=None,
        send_cmd: bool = True,
    ):
        """
        Cartesian space control: simultaneous dual-arm IK solving and joint command sending (non-blocking, for real-time tracking)

        Args:
            left_pose: [X, Y, Z, RX, RY, RZ] Left arm target pose, None means do not control left arm
            right_pose: [X, Y, Z, RX, RY, RZ] Right arm target pose, None means do not control right arm
            unit: 'mm' (millimeters) or 'm' (meters)
            joint_postprocessor: optional callable(side, joints) -> joints,
                applied after successful IK and before sending commands.

        Returns:
            tuple: (left_success, right_success, left_joints, right_joints)
        """
        # Convert units to mm
        left_mm = None
        right_mm = None
        if left_pose is not None:
            left_mm = list(left_pose)
            if unit == 'm':
                for i in range(3):
                    left_mm[i] *= 1000
        if right_pose is not None:
            right_mm = list(right_pose)
            if unit == 'm':
                for i in range(3):
                    right_mm[i] *= 1000

        # Get current joints, or the last successful IK solution, as IK reference.
        ref_left, ref_right = self._get_ik_reference_joints()

        left_success = False
        right_success = False
        left_joints = None
        right_joints = None

        # Left arm IK solving
        if left_mm is not None:
            try:
                left_mat = self.kine_left.xyzabc_to_mat4x4(left_mm)
                left_ik = self._call_ik_quietly(
                    self.kine_left,
                    robot_serial=0,
                    pose_mat=left_mat,
                    ref_joints=ref_left,
                    zsp_type=self.zsp_type,
                    zsp_para=self.left_zsp_para,
                    zsp_angle=self.zsp_angle,
                    dgr=self.dgr
                )
                if left_ik is not False:
                    candidate = left_ik.m_Output_RetJoint.to_list()
                    if self._joints_within_limits('left', candidate):
                        left_joints = candidate
                        self._last_left_ik_joints = list(candidate)
                        left_success = True
                    else:
                        self._log_ik_failure_detail(
                            "left",
                            left_mm,
                            ref_left,
                            out_range=bool(left_ik.m_Output_IsOutRange),
                            joint_exceed=bool(left_ik.m_Output_IsJntExd),
                            joint_exceed_tags=list(left_ik.m_Output_JntExdTags),
                            candidate_joints=candidate,
                        )
            except Exception as e:
                self.logger.debug(f"Left arm IK solving exception: {e}")

        # Right arm IK solving
        if right_mm is not None:
            try:
                right_mat = self.kine_right.xyzabc_to_mat4x4(right_mm)
                right_ik = self._call_ik_quietly(
                    self.kine_right,
                    robot_serial=1,
                    pose_mat=right_mat,
                    ref_joints=ref_right,
                    zsp_type=self.zsp_type,
                    zsp_para=self.right_zsp_para,
                    zsp_angle=self.zsp_angle,
                    dgr=self.dgr
                )
                if right_ik is not False:
                    candidate = right_ik.m_Output_RetJoint.to_list()
                    if self._joints_within_limits('right', candidate):
                        right_joints = candidate
                        self._last_right_ik_joints = list(candidate)
                        right_success = True
                    else:
                        self._log_ik_failure_detail(
                            "right",
                            right_mm,
                            ref_right,
                            out_range=bool(right_ik.m_Output_IsOutRange),
                            joint_exceed=bool(right_ik.m_Output_IsJntExd),
                            joint_exceed_tags=list(right_ik.m_Output_JntExdTags),
                            candidate_joints=candidate,
                        )
                else:
                    self._log_ik_failure_detail("right", right_mm, ref_right, sdk_error=True)
            except Exception as e:
                self.logger.debug(f"Right arm IK solving exception: {e}")
                self._log_ik_failure_detail("right", right_mm, ref_right, exception=e)

        if joint_postprocessor is not None:
            if left_joints is not None:
                left_joints = self._postprocess_ik_joints("left", left_joints, joint_postprocessor)
            if right_joints is not None:
                right_joints = self._postprocess_ik_joints("right", right_joints, joint_postprocessor)

        # IK debug output
        if left_joints is not None:
            left_joints_str = ', '.join([f'{j:7.2f}' for j in left_joints])
            self.logger.debug(f"[LEFT_IK]  joints: [{left_joints_str}]")
        else:
            self.logger.debug("[LEFT_IK]  FAILED!")

        if right_joints is not None:
            right_joints_str = ', '.join([f'{j:7.2f}' for j in right_joints])
            self.logger.debug(f"[RIGHT_IK] joints: [{right_joints_str}]")
        else:
            self.logger.debug("[RIGHT_IK] FAILED!")

        # Send dual-arm joint commands
        if self.dry_run or self.read_only:
            if left_joints is not None:
                self._dry_left_joints = list(left_joints)
            if right_joints is not None:
                self._dry_right_joints = list(right_joints)
            now = time.monotonic()
            if now - self._last_pose_log_time >= 1.0:
                self._last_pose_log_time = now
                tag = "READ_ONLY_POSE" if self.read_only else "DRY_RUN_POSE"
                right_target = None
                if right_mm is not None:
                    right_target = [round(float(v), 3) for v in right_mm]
                self.logger.info(
                    f"[{tag}] left_success={left_success} right_success={right_success} "
                    f"right_target_xyzabc_mm_deg={right_target} "
                    f"left_joints={left_joints} right_joints={right_joints}"
                )
        elif send_cmd:
            self.robot.clear_set()
            if left_joints is not None:
                self.robot.set_joint_cmd_pose(arm='A', joints=left_joints)
            if right_joints is not None:
                self.robot.set_joint_cmd_pose(arm='B', joints=right_joints)
            self.robot.send_cmd()

        return left_success, right_success, left_joints, right_joints

    def move_to_matrix_direct(
        self,
        left_mat=None,
        right_mat=None,
        left_target_log=None,
        right_target_log=None,
        joint_postprocessor=None,
        send_cmd: bool = True,
    ):
        """Cartesian control using 4x4 target matrices directly.

        The matrices must use SDK units: millimeters for translation. This path
        avoids converting through XYZABC Euler angles before IK, which is
        fragile near wrist orientations around 90 degrees.
        """
        ref_left, ref_right = self._get_ik_reference_joints()

        left_success = False
        right_success = False
        left_joints = None
        right_joints = None
        left_log = self._target_log_or_default(left_target_log)
        right_log = self._target_log_or_default(right_target_log)

        if left_mat is not None and not self._ik_side_blocked("left"):
            try:
                left_ik = self._call_ik_quietly(
                    self.kine_left,
                    robot_serial=0,
                    pose_mat=left_mat,
                    ref_joints=ref_left,
                    zsp_type=self.zsp_type,
                    zsp_para=self.left_zsp_para,
                    zsp_angle=self.zsp_angle,
                    dgr=self.dgr,
                )
                if left_ik is not False:
                    candidate = left_ik.m_Output_RetJoint.to_list()
                    if self._joints_within_limits('left', candidate):
                        if self._ik_branch_delta_ok("left", candidate):
                            left_joints = candidate
                            self._last_left_ik_joints = list(candidate)
                            left_success = True
                    else:
                        self._log_ik_failure_detail(
                            "left",
                            left_log,
                            ref_left,
                            out_range=bool(left_ik.m_Output_IsOutRange),
                            joint_exceed=bool(left_ik.m_Output_IsJntExd),
                            joint_exceed_tags=list(left_ik.m_Output_JntExdTags),
                            candidate_joints=candidate,
                        )
                else:
                    self._ik_block_until["left"] = time.monotonic() + self._ik_failure_block_sec
                    self._log_ik_failure_detail(
                        "left",
                        left_log,
                        ref_left,
                        sdk_error=True,
                    )
            except Exception as e:
                self.logger.debug(f"Left arm matrix IK solving exception: {e}")

        if right_mat is not None and not self._ik_side_blocked("right"):
            try:
                right_ik = self._call_ik_quietly(
                    self.kine_right,
                    robot_serial=1,
                    pose_mat=right_mat,
                    ref_joints=ref_right,
                    zsp_type=self.zsp_type,
                    zsp_para=self.right_zsp_para,
                    zsp_angle=self.zsp_angle,
                    dgr=self.dgr,
                )
                if right_ik is not False:
                    candidate = right_ik.m_Output_RetJoint.to_list()
                    if self._joints_within_limits('right', candidate):
                        if self._ik_branch_delta_ok("right", candidate):
                            right_joints = candidate
                            self._last_right_ik_joints = list(candidate)
                            right_success = True
                    else:
                        self._log_ik_failure_detail(
                            "right",
                            right_log,
                            ref_right,
                            out_range=bool(right_ik.m_Output_IsOutRange),
                            joint_exceed=bool(right_ik.m_Output_IsJntExd),
                            joint_exceed_tags=list(right_ik.m_Output_JntExdTags),
                            candidate_joints=candidate,
                        )
                else:
                    self._ik_block_until["right"] = time.monotonic() + self._ik_failure_block_sec
                    self._log_ik_failure_detail(
                        "right",
                        right_log,
                        ref_right,
                        sdk_error=True,
                    )
            except Exception as e:
                self.logger.debug(f"Right arm matrix IK solving exception: {e}")
                self._log_ik_failure_detail(
                    "right",
                    right_log,
                    ref_right,
                    exception=e,
                )

        if joint_postprocessor is not None:
            if left_joints is not None:
                left_joints = self._postprocess_ik_joints("left", left_joints, joint_postprocessor)
            if right_joints is not None:
                right_joints = self._postprocess_ik_joints("right", right_joints, joint_postprocessor)

        if self.dry_run or self.read_only:
            if left_joints is not None:
                self._dry_left_joints = list(left_joints)
            if right_joints is not None:
                self._dry_right_joints = list(right_joints)
            now = time.monotonic()
            if now - self._last_pose_log_time >= 1.0:
                self._last_pose_log_time = now
                tag = "READ_ONLY_POSE" if self.read_only else "DRY_RUN_POSE"
                right_target = self._format_target_log(right_target_log)
                self.logger.info(
                    f"[{tag}] left_success={left_success} right_success={right_success} "
                    f"right_target_xyzabc_mm_deg={right_target} "
                    f"left_joints={left_joints} right_joints={right_joints}"
                )
        elif send_cmd:
            with self._sdk_lock:
                self.robot.clear_set()
                if left_joints is not None:
                    self.robot.set_joint_cmd_pose(arm='A', joints=left_joints)
                if right_joints is not None:
                    self.robot.set_joint_cmd_pose(arm='B', joints=right_joints)
                self.robot.send_cmd()

        return left_success, right_success, left_joints, right_joints

    @staticmethod
    def _target_log_or_default(target):
        return [0, 0, 0, 0, 0, 0] if target is None else target

    @staticmethod
    def _format_target_log(target):
        if target is None:
            return None
        values = [float(v) for v in target]
        if len(values) >= 3 and max(abs(values[0]), abs(values[1]), abs(values[2])) < 10.0:
            values[:3] = [v * 1000.0 for v in values[:3]]
        return [round(v, 3) for v in values]

    def _postprocess_ik_joints(self, side, joints, joint_postprocessor):
        processed = list(joint_postprocessor(side, list(joints)))
        if len(processed) != len(joints):
            raise ValueError(
                f"{side} joint postprocessor returned {len(processed)} joints, expected {len(joints)}"
            )
        return self._clamp_joints_to_limits(side, processed)

    def _log_ik_failure_detail(
        self,
        side,
        pose_mm,
        ref_joints,
        out_range=False,
        joint_exceed=False,
        joint_exceed_tags=None,
        candidate_joints=None,
        sdk_error=False,
        exception=None,
    ):
        now = time.monotonic()
        if now - self._last_ik_failure_log_time < 1.0:
            return
        self._last_ik_failure_log_time = now

        pose_str = ", ".join(f"{float(v):.3f}" for v in pose_mm)
        ref_str = ", ".join(f"{float(v):.3f}" for v in ref_joints)
        reason = []
        if sdk_error:
            reason.append("sdk_error")
        if out_range:
            reason.append("out_range")
        if joint_exceed:
            reason.append(f"joint_exceed tags={joint_exceed_tags}")
        if exception is not None:
            reason.append(f"exception={exception}")
        if not reason:
            reason.append("unknown")
        candidate = ""
        if candidate_joints is not None:
            candidate_str = ", ".join(f"{float(v):.3f}" for v in candidate_joints)
            candidate = f" candidate_joints_deg=[{candidate_str}]"

        self.logger.warning(
            f"[{side.upper()}_IK_FAIL] reason={'; '.join(reason)} "
            f"target_xyzabc_mm_deg=[{pose_str}] ref_joints_deg=[{ref_str}]{candidate}"
        )

    @staticmethod
    def _parse_joint_limits(pnva):
        # PNVA rows are [positive_limit_deg, negative_limit_deg, velocity, acceleration].
        return [(float(row[1]), float(row[0])) for row in pnva]

    def _joints_within_limits(self, side, joints, margin_deg=1.0):
        limits = self._joint_limits[side]
        for joint, (lower, upper) in zip(joints, limits):
            if joint < lower - margin_deg or joint > upper + margin_deg:
                return False
        return True

    def _clamp_joints_to_limits(self, side, joints, margin_deg=1.0):
        limits = self._joint_limits[side]
        clamped = []
        for joint, (lower, upper) in zip(joints, limits):
            clamped.append(min(max(float(joint), lower + margin_deg), upper - margin_deg))
        return clamped

    def fk_mat4x4(self, robot_serial: int, joints: list):
        """Forward kinematics 4x4 matrix (mm in translation). Uses subprocess when isolated."""
        if self._ik_subprocess_client is not None:
            return self._ik_subprocess_client.fk(robot_serial, joints)
        kine = self.kine_left if robot_serial == 0 else self.kine_right
        if kine is None:
            return False
        return kine.fk(robot_serial, joints)

    def _call_ik_quietly(self, kine, **kwargs):
        if self._ik_subprocess_client is not None:
            serial = int(kwargs["robot_serial"])
            return self._ik_subprocess_client.solve(
                robot_serial=serial,
                pose_mat=kwargs["pose_mat"],
                ref_joints=kwargs["ref_joints"],
                zsp_type=kwargs["zsp_type"],
                zsp_para=kwargs["zsp_para"],
                zsp_angle=kwargs["zsp_angle"],
                dgr=kwargs["dgr"],
            )
        with self._sdk_lock:
            with self._suppress_native_output():
                return kine.ik(**kwargs)

    @staticmethod
    @contextmanager
    def _suppress_native_output():
        """Silence noisy native SDK stdout/stderr during real-time IK calls."""
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        stdout_fd = os.dup(1)
        stderr_fd = os.dup(2)
        try:
            os.dup2(devnull_fd, 1)
            os.dup2(devnull_fd, 2)
            yield
        finally:
            os.dup2(stdout_fd, 1)
            os.dup2(stderr_fd, 2)
            os.close(stdout_fd)
            os.close(stderr_fd)
            os.close(devnull_fd)

    # ==================== Joint Space Control Methods ====================

    def move_to_joints_direct(self, left_joints=None, right_joints=None):
        """
        Joint space control: simultaneous dual-arm joint angle command sending (non-blocking, for real-time tracking)

        Args:
            left_joints: [j1, j2, j3, j4, j5, j6, j7] left arm target joint angles (degrees), None means do not control left arm
            right_joints: [j1, j2, j3, j4, j5, j6, j7] right arm target joint angles (degrees), None means do not control right arm

        Returns:
            tuple: (left_success, right_success)
        """
        left_success = left_joints is not None
        right_success = right_joints is not None

        if self.dry_run or self.read_only:
            if left_joints is not None:
                self._dry_left_joints = list(left_joints)
            if right_joints is not None:
                self._dry_right_joints = list(right_joints)
            tag = "READ_ONLY_JOINTS" if self.read_only else "DRY_RUN_JOINTS"
            now = time.monotonic()
            if now - self._last_joint_command_log_time >= 1.0:
                self._last_joint_command_log_time = now
                self.logger.info(f"[{tag}] left={left_joints} right={right_joints}")
            return left_success, right_success

        with self._sdk_lock:
            self.robot.clear_set()
            if left_joints is not None:
                self.robot.set_joint_cmd_pose(arm='A', joints=list(left_joints))
                left_joints_str = ', '.join([f'{j:7.2f}' for j in left_joints])
                self.logger.debug(f"[LEFT]  joints: [{left_joints_str}]")
            if right_joints is not None:
                self.robot.set_joint_cmd_pose(arm='B', joints=list(right_joints))
                right_joints_str = ', '.join([f'{j:7.2f}' for j in right_joints])
                self.logger.debug(f"[RIGHT] joints: [{right_joints_str}]")
            self.robot.send_cmd()

        return left_success, right_success

    def move_to_joints_smooth(
        self,
        left_target=None,
        right_target=None,
        duration=3.0,
        dt=0.01,
        cmd_sides: str = "both",
        start_left=None,
        start_right=None,
    ):
        """
        Smooth dual-arm movement to target joint angles (using quintic polynomial interpolation)

        Args:
            left_target: [j1, j2, j3, j4, j5, j6, j7] left arm target joint angles (degrees), None means do not control left arm
            right_target: [j1, j2, j3, j4, j5, j6, j7] right arm target joint angles (degrees), None means do not control right arm
            duration: Total trajectory duration (seconds), larger = slower and smoother
            dt: Interpolation time step (seconds)
            cmd_sides: "both", "left", or "right" — only those arms receive SDK commands
            start_left/start_right: optional known start joints (avoids SDK read right after IK)

        Returns:
            bool: Whether successful
        """
        cmd_sides = str(cmd_sides).lower()
        if cmd_sides not in ("both", "left", "right"):
            raise ValueError(f"cmd_sides must be 'both', 'left', or 'right', got {cmd_sides!r}")
        if start_left is None or start_right is None:
            left_joints, right_joints = self.get_current_joints()
            if start_left is None:
                start_left = list(left_joints)
            if start_right is None:
                start_right = list(right_joints)
        else:
            start_left = list(start_left)
            start_right = list(start_right)

        if left_target is None:
            left_target = start_left
        if right_target is None:
            right_target = start_right

        num_points = max(int(duration / dt), 1)

        self.logger.debug(f"Smoothly move to target position({duration}s compliant trajectory)...")

        for i in range(num_points + 1):
            t = i / num_points
            s = 10 * (t ** 3) - 15 * (t ** 4) + 6 * (t ** 5)

            target_left = [
                start_left[j] + s * (left_target[j] - start_left[j])
                for j in range(7)
            ]
            target_right = [
                start_right[j] + s * (right_target[j] - start_right[j])
                for j in range(7)
            ]

            left_cmd = target_left if cmd_sides in ("both", "left") else None
            right_cmd = target_right if cmd_sides in ("both", "right") else None
            if self.dry_run or self.read_only:
                self.move_to_joints_direct(left_joints=left_cmd, right_joints=right_cmd)
            else:
                with self._sdk_lock:
                    self.robot.clear_set()
                    if left_cmd is not None:
                        self.robot.set_joint_cmd_pose(arm='A', joints=left_cmd)
                    if right_cmd is not None:
                        self.robot.set_joint_cmd_pose(arm='B', joints=right_cmd)
                    self.robot.send_cmd()

            time.sleep(dt)

        return True

    # ==================== Initial Pose and Release ====================

    def move_to_init(self, wait=True, timeout=1, duration=3.0, dt=0.01, sides="both"):
        """
        Move arm(s) to initial pose (joint-space quintic interpolation).

        Args:
            wait: Whether to wait for motion completion
            timeout: Additional wait time after reaching (seconds)
            duration: Total trajectory duration (seconds), larger = slower and smoother
            dt: Interpolation time step (seconds)
            sides: "both", "left", or "right". The other side holds its current joints.

        Returns:
            bool: Whether successful
        """
        sides = str(sides).lower()
        if sides not in ("both", "left", "right"):
            raise ValueError(f"sides must be 'both', 'left', or 'right', got {sides!r}")

        self.logger.debug(f"Moving to initial pose sides={sides} ({duration}s)...")

        if self.dry_run or self.read_only:
            if self.read_only:
                self.logger.info("[READ_ONLY_INIT] keeping current physical joint angles")
                return True
            self._dry_left_joints = list(self.INIT_JOINTS_LEFT)
            self._dry_right_joints = list(self.INIT_JOINTS_RIGHT)
            self.logger.info(
                f"[DRY_RUN_INIT] left={self._dry_left_joints} right={self._dry_right_joints}"
            )
            return True

        left_current, right_current = self.get_current_joints()
        if sides == "both":
            left_target = list(self.INIT_JOINTS_LEFT)
            right_target = list(self.INIT_JOINTS_RIGHT)
        elif sides == "right":
            left_target = list(left_current)
            right_target = list(self.INIT_JOINTS_RIGHT)
        else:
            left_target = list(self.INIT_JOINTS_LEFT)
            right_target = list(right_current)

        self.move_to_joints_smooth(
            left_target=left_target,
            right_target=right_target,
            duration=duration,
            dt=dt,
            cmd_sides=sides,
        )

        if wait:
            time.sleep(timeout)

        final_left, final_right = self.get_current_joints()
        left_errors = [abs(final_left[i] - left_target[i]) for i in range(7)]
        right_errors = [abs(final_right[i] - right_target[i]) for i in range(7)]
        max_left_error = max(left_errors)
        max_right_error = max(right_errors)

        success = True
        if max_left_error < 5.0:
            self.logger.debug("[Arm A] Reached initial pose")
        else:
            self.logger.warning(f"[Arm A] Large initial pose error ({max_left_error:.1f}°)")
            success = False

        if max_right_error < 5.0:
            self.logger.debug("[Arm B] Reached initial pose")
        else:
            self.logger.warning(f"[Arm B] Large initial pose error ({max_right_error:.1f}°)")
            success = False

        return success

    def disable_and_release(self):
        """Disable and release both arms"""
        if self._ik_subprocess_client is not None:
            self._ik_subprocess_client.shutdown()
            self._ik_subprocess_client = None
        if self.dry_run or self.robot is None:
            self.logger.info("DRY RUN: no robot connection to release")
            return
        if self.read_only:
            self.logger.info("READ ONLY: releasing robot connection without disabling arms")
            self.robot.release_robot()
            return
        self.logger.info("Disabling both arms...")
        self.robot.clear_set()
        self.robot.set_state(arm='A', state=0)
        self.robot.set_state(arm='B', state=0)
        self.robot.send_cmd()
        time.sleep(2)

        self.logger.debug("Releasing connection...")
        self.robot.release_robot()
        self.logger.info("Safely exited")
