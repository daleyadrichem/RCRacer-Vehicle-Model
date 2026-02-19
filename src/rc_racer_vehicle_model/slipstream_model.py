"""
slipstream_model.py

Deterministic aerodynamic wake / slipstream model.

CORE LAYER
----------
No randomness.
No environment dependency.
"""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class SlipstreamParams:
    """
    Slipstream configuration.

    Parameters
    ----------
    wake_length : float
        Maximum wake length behind leading vehicle [m].
    wake_angle_deg : float
        Half-angle of wake cone [deg].
    drag_reduction_max : float
        Maximum drag reduction fraction (0–1).
    downforce_reduction_max : float
        Maximum downforce reduction fraction (0–1).
    decay_rate : float
        Exponential decay rate with distance.
    """

    wake_length: float
    wake_angle_deg: float
    drag_reduction_max: float
    downforce_reduction_max: float
    decay_rate: float


class SlipstreamModel:
    """
    Computes aerodynamic wake effects between two vehicles.
    """

    def __init__(self, params: SlipstreamParams) -> None:
        self._p = params
        self._wake_angle_rad = np.deg2rad(params.wake_angle_deg)

    def compute_effect(
        self,
        follower_position: np.ndarray,
        leader_position: np.ndarray,
        leader_heading: float,
    ) -> tuple[float, float]:
        """
        Compute drag and downforce multipliers.

        Parameters
        ----------
        follower_position : ndarray shape (2,)
            Global (x, y) position [m].
        leader_position : ndarray shape (2,)
            Global (x, y) position [m].
        leader_heading : float
            Heading angle [rad].

        Returns
        -------
        drag_multiplier : float
            Multiplicative factor in (0, 1] applied to drag.
            1.0 means no slipstream effect.
        downforce_multiplier : float
            Multiplicative factor in (0, 1] applied to
            downforce-sensitive tire forces.
        """

        rel = follower_position - leader_position

        # Transform into leader frame
        c = np.cos(-leader_heading)
        s = np.sin(-leader_heading)

        rel_x = c * rel[0] - s * rel[1]
        rel_y = s * rel[0] + c * rel[1]

        # Must be behind leader
        if rel_x >= 0.0:
            return 1.0, 1.0

        distance = abs(rel_x)

        if distance > self._p.wake_length:
            return 1.0, 1.0

        lateral_angle = abs(np.arctan2(rel_y, -rel_x))

        if lateral_angle > self._wake_angle_rad:
            return 1.0, 1.0

        # Exponential decay
        reduction = self._p.drag_reduction_max * np.exp(
            -self._p.decay_rate * distance
        )

        drag_multiplier = 1.0 - reduction

        downforce_multiplier = 1.0 - (
            self._p.downforce_reduction_max * np.exp(
                -self._p.decay_rate * distance
            )
        )

        return float(drag_multiplier), float(downforce_multiplier)
