"""Small One-Euro filter used by the Tianji controller.

Kept local to the controller package so arm startup does not depend on the
optional pico_input ROS package being installed in the active overlay.
"""

from __future__ import annotations

import numpy as np


def _smoothing_factor(rate: float, cutoff: float) -> float:
    tau = 1.0 / (2.0 * np.pi * cutoff)
    return 1.0 / (1.0 + tau * rate)


class OneEuroFilter:
    """Adaptive low-pass filter for vector signals."""

    def __init__(
        self,
        rate: float,
        min_cutoff: float = 1.0,
        beta: float = 0.5,
        d_cutoff: float = 1.0,
    ) -> None:
        self.rate = float(rate)
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self._x_prev = None
        self._dx_prev = None

    def __call__(self, x: np.ndarray) -> np.ndarray:
        x = np.asarray(x, dtype=np.float64)
        if self._x_prev is None:
            self._x_prev = x.copy()
            self._dx_prev = np.zeros_like(x)
            return x.copy()

        dx = (x - self._x_prev) * self.rate
        a_d = _smoothing_factor(self.rate, self.d_cutoff)
        self._dx_prev = a_d * dx + (1.0 - a_d) * self._dx_prev

        speed = np.linalg.norm(self._dx_prev)
        cutoff = self.min_cutoff + self.beta * speed
        a = _smoothing_factor(self.rate, cutoff)
        self._x_prev = a * x + (1.0 - a) * self._x_prev

        return self._x_prev.copy()

    def reset(self) -> None:
        self._x_prev = None
        self._dx_prev = None
