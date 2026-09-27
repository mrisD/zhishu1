"""Small constant-velocity tracker for interception of a moving bottle."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TrackState:
    position: np.ndarray
    velocity: np.ndarray
    stamp: float
    observations: int


class AlphaBetaTracker:
    def __init__(self, alpha: float = 0.65, beta: float = 0.12, max_gap: float = 0.8):
        self.alpha = float(alpha)
        self.beta = float(beta)
        self.max_gap = float(max_gap)
        self._state = None

    @property
    def state(self):
        return self._state

    def reset(self):
        self._state = None

    def update(self, measurement, stamp: float) -> TrackState:
        measurement = np.asarray(measurement, dtype=np.float64).reshape(3)
        stamp = float(stamp)
        if self._state is None or stamp <= self._state.stamp or stamp - self._state.stamp > self.max_gap:
            self._state = TrackState(measurement.copy(), np.zeros(3), stamp, 1)
            return self._state

        dt = stamp - self._state.stamp
        predicted = self._state.position + self._state.velocity * dt
        residual = measurement - predicted
        position = predicted + self.alpha * residual
        velocity = self._state.velocity + (self.beta / dt) * residual
        self._state = TrackState(position, velocity, stamp, self._state.observations + 1)
        return self._state

    def predict(self, stamp: float) -> np.ndarray:
        if self._state is None:
            raise RuntimeError("tracker has no observations")
        dt = max(0.0, float(stamp) - self._state.stamp)
        return self._state.position + self._state.velocity * dt
