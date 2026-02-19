"""
linearization.py

Finite-difference linearization utility.

CORE LAYER
----------
Deterministic Jacobian computation.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from rc_racer_vehicle_model.vehicle_state import State

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from rc_racer_vehicle_model.vehicle_model import VehicleModel


FloatArray = NDArray[np.float64]


def linearize(
    model: "VehicleModel",
    state: State,
    action: FloatArray,
    dt: float,
    leader_state: State | None = None,
    eps: float = 1e-5,
) -> tuple[FloatArray, FloatArray]:
    """
    Compute linearized system matrices A, B via finite differences.

    Parameters
    ----------
    model : VehicleModel
    state : State
    action : ndarray
    dt : float
    leader_state : State | None
        Optional leader vehicle (for slipstream consistency).
    eps : float
        Finite difference step size.

    Returns
    -------
    A : ndarray (n, n)
    B : ndarray (n, m)
    """

    x0 = np.array(state.as_tuple(), dtype=np.float64)
    f0 = np.array(
        model.step(state, tuple(action), dt, leader_state).as_tuple()
    )

    n = x0.size
    m = action.size

    A = np.zeros((n, n))
    B = np.zeros((n, m))

    # State Jacobian
    for i in range(n):
        dx = np.zeros_like(x0)
        dx[i] = eps
        s_pert = State(*tuple(x0 + dx))

        f = np.array(
            model.step(s_pert, tuple(action), dt, leader_state).as_tuple()
        )

        A[:, i] = (f - f0) / eps

    # Input Jacobian
    for j in range(m):
        du = np.zeros_like(action)
        du[j] = eps

        f = np.array(
            model.step(state, tuple(action + du), dt, leader_state).as_tuple()
        )

        B[:, j] = (f - f0) / eps

    return A, B
