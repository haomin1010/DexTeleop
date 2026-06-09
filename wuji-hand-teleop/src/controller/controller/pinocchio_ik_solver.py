"""Pinocchio full-pose IK for the Tianji 7-axis arm."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

try:
    import pinocchio as pin
except ImportError as exc:  # pragma: no cover - handled by node startup logging.
    pin = None
    _PINOCCHIO_IMPORT_ERROR = exc
else:
    _PINOCCHIO_IMPORT_ERROR = None


@dataclass
class IKResult:
    success: bool
    q_motor: list[float]
    q_pin: np.ndarray
    position_error: float
    orientation_error: float
    sigma_min: float
    condition_number: float
    damping: float
    orientation_weight: float
    iterations: int
    reason: str


class PinocchioIKSolver:
    def __init__(
        self,
        urdf_path: str,
        ee_frame_name: str,
        joint_names: Optional[list[str]] = None,
        motor_unit: str = "deg",
        max_iters: int = 30,
        dt: float = 0.5,
        pos_eps: float = 0.005,
        ori_eps: float = 0.08,
        position_weight: float = 1.0,
        orientation_weight: float = 0.25,
        orientation_weight_near_singularity: float = 0.05,
        base_damping: float = 1e-4,
        max_damping: float = 5e-2,
        sigma_min_warn: float = 0.04,
        sigma_min_critical: float = 0.015,
        condition_warn: float = 80.0,
        condition_critical: float = 200.0,
        joint_limit_avoidance_enable: bool = False,
        joint_limit_avoidance_gain: float = 0.02,
        max_dq_step_deg: float = 3.0,
        posture_weight: float = 0.0,
        logger=None,
    ):
        if pin is None:
            raise ImportError(f"pinocchio is not available: {_PINOCCHIO_IMPORT_ERROR}")

        self.urdf_path = str(urdf_path)
        self.ee_frame_name = str(ee_frame_name)
        self.joint_names = joint_names
        self.motor_unit = str(motor_unit).lower()
        if self.motor_unit not in ("deg", "rad"):
            raise ValueError(f"motor_unit must be 'deg' or 'rad', got {motor_unit!r}")

        self.max_iters = int(max_iters)
        self.dt = float(dt)
        self.pos_eps = float(pos_eps)
        self.ori_eps = float(ori_eps)
        self.position_weight = float(position_weight)
        self.orientation_weight = float(orientation_weight)
        self.orientation_weight_near_singularity = float(orientation_weight_near_singularity)
        self.base_damping = float(base_damping)
        self.max_damping = float(max_damping)
        self.sigma_min_warn = float(sigma_min_warn)
        self.sigma_min_critical = float(sigma_min_critical)
        self.condition_warn = float(condition_warn)
        self.condition_critical = float(condition_critical)
        self.joint_limit_avoidance_enable = bool(joint_limit_avoidance_enable)
        self.joint_limit_avoidance_gain = float(joint_limit_avoidance_gain)
        self.max_dq_step_rad = np.deg2rad(float(max_dq_step_deg))
        self.posture_weight = float(posture_weight)
        self.logger = logger

        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        self.ee_frame_id = self.model.getFrameId(self.ee_frame_name)
        if self.ee_frame_id >= len(self.model.frames):
            frames = ", ".join(frame.name for frame in self.model.frames)
            raise ValueError(
                f"EE frame {self.ee_frame_name!r} not found in {self.urdf_path}. "
                f"Available frames: {frames}"
            )
        if self.model.nq != 7:
            self._warn(
                f"Pinocchio model nq={self.model.nq}, expected 7. "
                f"model.names={list(self.model.names)}"
            )
        self._debug_model()

    def solve(
        self,
        target_matrix_m: np.ndarray,
        q_seed_motor: list[float],
        side: str = "right",
        orientation_scales: Optional[tuple[float, ...]] = None,
        joint_target_motor: Optional[tuple[int, float, float]] = None,
        joint_lock_motor: Optional[tuple[int, float]] = None,
    ) -> IKResult:
        target_matrix_m = np.array(target_matrix_m, dtype=np.float64)
        q_seed = self.motor_to_pin(q_seed_motor)
        self._clamp_q_in_place(q_seed)
        if orientation_scales is None:
            orientation_scales = (1.0,)
        joint_target_pin = self._joint_target_to_pin(joint_target_motor)
        joint_lock_pin = self._joint_lock_to_pin(joint_lock_motor)

        pin.forwardKinematics(self.model, self.data, q_seed)
        pin.updateFramePlacements(self.model, self.data)
        reference_rot = np.array(self.data.oMf[self.ee_frame_id].rotation, dtype=np.float64)
        target_rot = np.array(target_matrix_m[:3, :3], dtype=np.float64)
        target_pos = np.array(target_matrix_m[:3, 3], dtype=np.float64)

        best: Optional[IKResult] = None
        reasons = []
        for ori_scale in orientation_scales:
            blended_target = np.array(target_matrix_m, dtype=np.float64)
            blended_target[:3, 3] = target_pos
            blended_target[:3, :3] = self._blend_rotation(reference_rot, target_rot, ori_scale)
            result = self._solve_once(
                blended_target,
                q_seed,
                ori_scale,
                q_seed,
                joint_target_pin=joint_target_pin,
                joint_lock_pin=joint_lock_pin,
            )
            result.reason = (
                f"success_ori_scale_{ori_scale:.1f}"
                if result.success
                else f"fail_ori_scale_{ori_scale:.1f}:{result.reason}"
            )
            if result.success:
                return result
            reasons.append(result.reason)
            if best is None or result.position_error < best.position_error:
                best = result

        if best is None:
            return self._failed_result(q_seed, "no_attempts")
        best.success = False
        best.reason = "all_fallbacks_failed:" + ";".join(reasons)
        return best

    def motor_to_pin(self, q_motor: list[float]) -> np.ndarray:
        q = np.array(q_motor, dtype=np.float64)
        if q.size != self.model.nq:
            raise ValueError(f"Expected {self.model.nq} joints, got {q.size}")
        if self.motor_unit == "deg":
            q = np.deg2rad(q)
        return q

    def pin_to_motor(self, q_pin: np.ndarray) -> list[float]:
        q = np.array(q_pin, dtype=np.float64)
        if self.motor_unit == "deg":
            q = np.rad2deg(q)
        return [float(v) for v in q]

    def frame_matrix(self, q_motor: list[float]) -> np.ndarray:
        q = self.motor_to_pin(q_motor)
        self._clamp_q_in_place(q)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        frame = self.data.oMf[self.ee_frame_id]
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = frame.rotation
        matrix[:3, 3] = frame.translation
        return matrix

    def debug_description(self) -> str:
        frames = [frame.name for frame in self.model.frames]
        return (
            f"names={list(self.model.names)}, frames={frames}, "
            f"lower={self.model.lowerPositionLimit.tolist()}, "
            f"upper={self.model.upperPositionLimit.tolist()}"
        )

    def _solve_once(
        self,
        target_matrix_m: np.ndarray,
        q_seed: np.ndarray,
        ori_scale: float,
        q_posture: np.ndarray,
        joint_target_pin: Optional[tuple[int, float, float]] = None,
        joint_lock_pin: Optional[tuple[int, float]] = None,
    ) -> IKResult:
        q = np.array(q_seed, dtype=np.float64)
        target = pin.SE3(target_matrix_m[:3, :3], target_matrix_m[:3, 3])
        ori_weight_limit = self.orientation_weight_near_singularity + 1e-6
        last_diag = self._diag_defaults()
        if joint_lock_pin is not None:
            lock_index, lock_target = joint_lock_pin
            q[lock_index] = lock_target
            self._clamp_q_in_place(q)

        for iteration in range(1, self.max_iters + 1):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            current = self.data.oMf[self.ee_frame_id]

            iMd = current.inverse() * target
            err6 = pin.log6(iMd).vector
            e_pos = err6[0:3]
            e_ori = err6[3:6]
            pos_err = float(np.linalg.norm(e_pos))
            ori_err = float(np.linalg.norm(e_ori))

            J6 = pin.computeFrameJacobian(
                self.model,
                self.data,
                q,
                self.ee_frame_id,
                pin.ReferenceFrame.LOCAL,
            )
            sigma_min, condition_number = self._jacobian_diagnostics(J6)
            damping = self._adaptive_damping(sigma_min)
            ori_w = self._adaptive_orientation_weight(sigma_min) * float(ori_scale)

            last_diag = {
                "position_error": pos_err,
                "orientation_error": ori_err,
                "sigma_min": sigma_min,
                "condition_number": condition_number,
                "damping": damping,
                "orientation_weight": ori_w,
                "iterations": iteration,
            }
            if pos_err < self.pos_eps and (ori_err < self.ori_eps or ori_w <= ori_weight_limit):
                return IKResult(True, self.pin_to_motor(q), q, reason="converged", **last_diag)

            J_weighted = np.array(J6, dtype=np.float64)
            J_weighted[0:3, :] *= self.position_weight
            J_weighted[3:6, :] *= ori_w
            if joint_lock_pin is not None:
                J_weighted[:, joint_lock_pin[0]] = 0.0
            err_weighted = np.concatenate([self.position_weight * e_pos, ori_w * e_ori]).astype(np.float64)
            if self.posture_weight > 0.0:
                J_weighted = np.vstack([J_weighted, self.posture_weight * np.eye(self.model.nv)])
                err_weighted = np.concatenate([err_weighted, self.posture_weight * (q_posture - q)])
            if joint_target_pin is not None:
                joint_index, target_pin, target_weight = joint_target_pin
                if target_weight > 0.0:
                    target_row = np.zeros((1, self.model.nv), dtype=np.float64)
                    target_row[0, joint_index] = np.sqrt(target_weight)
                    target_err = np.sqrt(target_weight) * (target_pin - q[joint_index])
                    J_weighted = np.vstack([J_weighted, target_row])
                    err_weighted = np.concatenate([err_weighted, [target_err]])
            try:
                jj_t = J_weighted @ J_weighted.T
                dq = J_weighted.T @ np.linalg.solve(
                    jj_t + (damping ** 2) * np.eye(J_weighted.shape[0]),
                    err_weighted,
                )
            except np.linalg.LinAlgError:
                return IKResult(False, self.pin_to_motor(q), q, reason="linear_solve_failed", **last_diag)

            if self.joint_limit_avoidance_enable:
                dq += self._joint_limit_avoidance(q)
            if joint_lock_pin is not None:
                dq[joint_lock_pin[0]] = 0.0
            dq = np.clip(dq, -self.max_dq_step_rad, self.max_dq_step_rad)
            q = pin.integrate(self.model, q, self.dt * dq)
            if joint_lock_pin is not None:
                q[joint_lock_pin[0]] = joint_lock_pin[1]
            self._clamp_q_in_place(q)

        return IKResult(False, self.pin_to_motor(q), q, reason="max_iters", **last_diag)

    def _jacobian_diagnostics(self, J6: np.ndarray) -> tuple[float, float]:
        s = np.linalg.svd(J6, compute_uv=False)
        if s.size == 0:
            return 0.0, float("inf")
        sigma_max = float(np.max(s))
        sigma_min = float(np.min(s))
        condition_number = sigma_max / max(sigma_min, 1e-9)
        return sigma_min, condition_number

    def _adaptive_damping(self, sigma_min: float) -> float:
        if sigma_min >= self.sigma_min_warn:
            return self.base_damping
        if sigma_min <= self.sigma_min_critical:
            return self.max_damping
        ratio = (self.sigma_min_warn - sigma_min) / (self.sigma_min_warn - self.sigma_min_critical)
        return self.base_damping + ratio * (self.max_damping - self.base_damping)

    def _joint_target_to_pin(
        self,
        joint_target_motor: Optional[tuple[int, float, float]],
    ) -> Optional[tuple[int, float, float]]:
        if joint_target_motor is None:
            return None
        if len(joint_target_motor) != 3:
            raise ValueError("joint_target_motor must be (joint_index, target_motor, weight)")
        joint_index = int(joint_target_motor[0])
        if joint_index < 0 or joint_index >= self.model.nq:
            raise ValueError(
                f"joint_target_motor index out of range: {joint_index} (nq={self.model.nq})"
            )
        target_motor = float(joint_target_motor[1])
        weight = max(float(joint_target_motor[2]), 0.0)
        if self.motor_unit == "deg":
            target_pin = float(np.deg2rad(target_motor))
        else:
            target_pin = target_motor
        return (joint_index, target_pin, weight)

    def _joint_lock_to_pin(
        self,
        joint_lock_motor: Optional[tuple[int, float]],
    ) -> Optional[tuple[int, float]]:
        if joint_lock_motor is None:
            return None
        if len(joint_lock_motor) != 2:
            raise ValueError("joint_lock_motor must be (joint_index, target_motor)")
        joint_index = int(joint_lock_motor[0])
        if joint_index < 0 or joint_index >= self.model.nq:
            raise ValueError(
                f"joint_lock_motor index out of range: {joint_index} (nq={self.model.nq})"
            )
        target_motor = float(joint_lock_motor[1])
        if self.motor_unit == "deg":
            target_pin = float(np.deg2rad(target_motor))
        else:
            target_pin = target_motor
        return (joint_index, target_pin)

    def _adaptive_orientation_weight(self, sigma_min: float) -> float:
        if sigma_min >= self.sigma_min_warn:
            return self.orientation_weight
        if sigma_min <= self.sigma_min_critical:
            return self.orientation_weight_near_singularity
        ratio = (self.sigma_min_warn - sigma_min) / (self.sigma_min_warn - self.sigma_min_critical)
        return (
            self.orientation_weight * (1.0 - ratio)
            + self.orientation_weight_near_singularity * ratio
        )

    def _joint_limit_avoidance(self, q: np.ndarray) -> np.ndarray:
        lower = self.model.lowerPositionLimit
        upper = self.model.upperPositionLimit
        valid = np.isfinite(lower) & np.isfinite(upper) & (upper > lower)
        correction = np.zeros_like(q)
        if not np.any(valid):
            return correction
        center = 0.5 * (lower + upper)
        span = np.maximum(upper - lower, 1e-6)
        correction[valid] = -self.joint_limit_avoidance_gain * (q[valid] - center[valid]) / span[valid]
        return correction

    def _clamp_q_in_place(self, q: np.ndarray) -> None:
        lower = self.model.lowerPositionLimit
        upper = self.model.upperPositionLimit
        valid = np.isfinite(lower) & np.isfinite(upper) & (upper > lower)
        q[valid] = np.minimum(np.maximum(q[valid], lower[valid]), upper[valid])

    def _failed_result(self, q: np.ndarray, reason: str) -> IKResult:
        return IKResult(
            success=False,
            q_motor=self.pin_to_motor(q),
            q_pin=np.array(q, dtype=np.float64),
            position_error=float("inf"),
            orientation_error=float("inf"),
            sigma_min=0.0,
            condition_number=float("inf"),
            damping=self.max_damping,
            orientation_weight=0.0,
            iterations=0,
            reason=reason,
        )

    def _diag_defaults(self) -> dict:
        return {
            "position_error": float("inf"),
            "orientation_error": float("inf"),
            "sigma_min": 0.0,
            "condition_number": float("inf"),
            "damping": self.max_damping,
            "orientation_weight": 0.0,
            "iterations": 0,
        }

    @staticmethod
    def _blend_rotation(reference_rot: np.ndarray, target_rot: np.ndarray, scale: float) -> np.ndarray:
        scale = min(max(float(scale), 0.0), 1.0)
        if scale <= 0.0:
            return np.array(reference_rot, dtype=np.float64)
        if scale >= 1.0:
            return np.array(target_rot, dtype=np.float64)
        delta = reference_rot.T @ target_rot
        return reference_rot @ pin.exp3(scale * pin.log3(delta))

    def _debug_model(self) -> None:
        msg = f"Loaded Pinocchio model from {self.urdf_path}; {self.debug_description()}"
        if self.logger is not None:
            self.logger.info(msg)

    def _warn(self, msg: str) -> None:
        if self.logger is not None:
            self.logger.warning(msg)
