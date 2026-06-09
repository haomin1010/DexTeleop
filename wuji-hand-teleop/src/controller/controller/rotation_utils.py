"""Rotation helpers with a SciPy-compatible subset fallback."""

from __future__ import annotations

import math

import numpy as np


def _skew(v: np.ndarray) -> np.ndarray:
    return np.array(
        [
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0],
        ],
        dtype=np.float64,
    )


def _rotvec_to_matrix(rotvec: np.ndarray) -> np.ndarray:
    theta = float(np.linalg.norm(rotvec))
    if theta < 1e-12:
        return np.eye(3, dtype=np.float64) + _skew(rotvec)
    axis = rotvec / theta
    k = _skew(axis)
    return np.eye(3, dtype=np.float64) + math.sin(theta) * k + (1.0 - math.cos(theta)) * (k @ k)


def _matrix_to_rotvec(matrix: np.ndarray) -> np.ndarray:
    matrix = np.asarray(matrix, dtype=np.float64)
    cos_theta = max(-1.0, min(1.0, (float(np.trace(matrix)) - 1.0) * 0.5))
    theta = math.acos(cos_theta)
    if theta < 1e-12:
        return np.zeros(3, dtype=np.float64)
    if abs(math.pi - theta) < 1e-5:
        axis = np.sqrt(np.maximum(np.diag(matrix) + 1.0, 0.0) * 0.5)
        axis[0] = math.copysign(axis[0], matrix[2, 1] - matrix[1, 2])
        axis[1] = math.copysign(axis[1], matrix[0, 2] - matrix[2, 0])
        axis[2] = math.copysign(axis[2], matrix[1, 0] - matrix[0, 1])
        norm = np.linalg.norm(axis)
        if norm > 1e-12:
            axis /= norm
        return axis * theta
    axis = np.array(
        [
            matrix[2, 1] - matrix[1, 2],
            matrix[0, 2] - matrix[2, 0],
            matrix[1, 0] - matrix[0, 1],
        ],
        dtype=np.float64,
    ) / (2.0 * math.sin(theta))
    return axis * theta


def _quat_to_matrix(quat: np.ndarray) -> np.ndarray:
    x, y, z, w = np.asarray(quat, dtype=np.float64)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _euler_zyx_to_matrix(angles: np.ndarray) -> np.ndarray:
    z, y, x = [float(v) for v in angles]
    cz, sz = math.cos(z), math.sin(z)
    cy, sy = math.cos(y), math.sin(y)
    cx, sx = math.cos(x), math.sin(x)
    rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
    ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    return rz @ ry @ rx


def _matrix_to_euler_zyx(matrix: np.ndarray) -> np.ndarray:
    m = np.asarray(matrix, dtype=np.float64)
    sy = -m[2, 0]
    y = math.asin(max(-1.0, min(1.0, sy)))
    cy = math.cos(y)
    if abs(cy) > 1e-8:
        z = math.atan2(m[1, 0], m[0, 0])
        x = math.atan2(m[2, 1], m[2, 2])
    else:
        z = math.atan2(-m[0, 1], m[1, 1])
        x = 0.0
    return np.array([z, y, x], dtype=np.float64)


def _matrix_to_euler_xyz(matrix: np.ndarray) -> np.ndarray:
    m = np.asarray(matrix, dtype=np.float64)
    y = math.asin(max(-1.0, min(1.0, m[0, 2])))
    cy = math.cos(y)
    if abs(cy) > 1e-8:
        x = math.atan2(-m[1, 2], m[2, 2])
        z = math.atan2(-m[0, 1], m[0, 0])
    else:
        x = math.atan2(m[2, 1], m[1, 1])
        z = 0.0
    return np.array([x, y, z], dtype=np.float64)


class FallbackRotation:
    def __init__(self, matrices: np.ndarray):
        matrices = np.asarray(matrices, dtype=np.float64)
        self._single = matrices.ndim == 2
        self._matrices = matrices.reshape((-1, 3, 3)) if self._single else matrices

    @classmethod
    def from_quat(cls, quat):
        return cls(_quat_to_matrix(np.asarray(quat, dtype=np.float64)))

    @classmethod
    def from_matrix(cls, matrix):
        return cls(np.asarray(matrix, dtype=np.float64))

    @classmethod
    def from_euler(cls, seq: str, angles, degrees: bool = False):
        if seq != "ZYX":
            raise ValueError("FallbackRotation only supports from_euler('ZYX', ...)")
        angles = np.asarray(angles, dtype=np.float64)
        if degrees:
            angles = np.radians(angles)
        return cls(_euler_zyx_to_matrix(angles))

    def as_matrix(self):
        return self._matrices[0].copy() if self._single else self._matrices.copy()

    def as_euler(self, seq: str, degrees: bool = False):
        values = []
        for matrix in self._matrices:
            if seq == "ZYX":
                euler = _matrix_to_euler_zyx(matrix)
            elif seq == "xyz":
                euler = _matrix_to_euler_xyz(matrix)
            else:
                raise ValueError(f"FallbackRotation does not support as_euler({seq!r})")
            values.append(euler)
        result = values[0] if self._single else np.stack(values, axis=0)
        return np.degrees(result) if degrees else result

    def as_rotvec(self):
        values = [_matrix_to_rotvec(matrix) for matrix in self._matrices]
        return values[0] if self._single else np.stack(values, axis=0)

    def magnitude(self):
        values = [float(np.linalg.norm(_matrix_to_rotvec(matrix))) for matrix in self._matrices]
        return values[0] if self._single else np.asarray(values, dtype=np.float64)

    def inv(self):
        matrices = np.swapaxes(self._matrices, -1, -2)
        return FallbackRotation(matrices[0] if self._single else matrices)

    def __mul__(self, other):
        matrix = self.as_matrix() @ other.as_matrix()
        return FallbackRotation(matrix)


class FallbackSlerp:
    def __init__(self, times, rotations: FallbackRotation):
        self._start = rotations.as_matrix()[0]
        self._end = rotations.as_matrix()[1]

    def __call__(self, times):
        matrices = []
        delta = self._start.T @ self._end
        rotvec = _matrix_to_rotvec(delta)
        for t in times:
            matrices.append(self._start @ _rotvec_to_matrix(rotvec * float(t)))
        return FallbackRotation(np.stack(matrices, axis=0))


try:
    from scipy.spatial.transform import Rotation, Slerp  # type: ignore
except Exception:
    Rotation = FallbackRotation
    Slerp = FallbackSlerp
