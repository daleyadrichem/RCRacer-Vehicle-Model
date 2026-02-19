"""
Deterministic throttle/brake engine model.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class EngineParams:
    """
    Engine configuration parameters.

    Parameters
    ----------
    max_drive_force : float
        Maximum traction force produced at full throttle [N].
    max_brake_force : float
        Maximum braking force [N].
    drivetrain_efficiency : float
        Efficiency factor (0–1).
    """

    max_drive_force: float
    max_brake_force: float
    drivetrain_efficiency: float = 1.0


class EngineModel:
    """
    Simple deterministic longitudinal force model.

    Force is computed as:
        throttle * max_drive_force * efficiency
    - brake * max_brake_force

    No speed dependency, torque curve,
    or traction limits are modeled.
    """

    def __init__(self, params: EngineParams) -> None:
        self._p: EngineParams = params

    def compute_force(
        self,
        throttle: float,
        brake: float,
    ) -> float:
        """
        Compute longitudinal force.

        Parameters
        ----------
        throttle : float
            Throttle command in [0, 1].
        brake : float
            Brake command in [0, 1].

        Returns
        -------
        float
            Longitudinal force [N].
        """
        throttle = float(np.clip(throttle, 0.0, 1.0))
        brake = float(np.clip(brake, 0.0, 1.0))

        drive = throttle * self._p.max_drive_force * self._p.drivetrain_efficiency
        braking = brake * self._p.max_brake_force

        return float(drive - braking)
