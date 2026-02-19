"""
Aerodynamic model for drag.

Provides:
- Quadratic aerodynamic drag (signed, opposes velocity)
- Optional slipstream multiplier support
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class AeroParams:
    """
    Aerodynamic parameters.

    Parameters
    ----------
    rho : float
        Air density [kg/m^3].
    c_d_a : float
        Drag coefficient times frontal area (Cd * A) [m^2].
    """

    rho: float
    c_d_a: float


class AeroModel:
    """
    Quadratic aerodynamic model.

    Drag:
        F_drag = 0.5 * rho * CdA * v * |v| * drag_multiplier
    """

    def __init__(self, params: AeroParams) -> None:
        self._p = params

    def drag_force(
        self,
        velocity: float,
        drag_multiplier: float = 1.0,
    ) -> float:
        """
        Compute signed aerodynamic drag force.

        Parameters
        ----------
        velocity : float
            Longitudinal velocity [m/s].
        drag_multiplier : float
            Multiplier for slipstream effects.

        Returns
        -------
        float
            Signed drag force [N].

            Positive value means force acts in +x direction.
            Negative value means force acts in -x direction.

        Notes
        -----
        Uses v * |v| so drag always opposes motion.
        """

        v = float(velocity)

        return (
            0.5
            * self._p.rho
            * self._p.c_d_a
            * v
            * abs(v)
            * drag_multiplier
        )
