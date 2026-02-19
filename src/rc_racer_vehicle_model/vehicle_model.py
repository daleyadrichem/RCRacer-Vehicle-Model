"""
Dynamic bicycle vehicle model with:
- Steering actuator lag
- Engine/throttle model
- Aerodynamic drag + drafting
- Full dynamic bicycle equations
- Linearization API
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Tuple

import numpy as np

from rc_racer_vehicle_model.vehicle_state import State
from rc_racer_vehicle_model.engine_model import EngineModel
from rc_racer_vehicle_model.aero_model import AeroModel
from rc_racer_vehicle_model.slipstream_model import SlipstreamModel
from rc_racer_vehicle_model.linearization import linearize


@dataclass(frozen=True)
class DynamicVehicleParams:
    """
    Full dynamic bicycle parameters.

    Parameters
    ----------
    mass : float
        Vehicle mass [kg].
    wheelbase : float
        Total wheelbase [m].
    lf : float
        Distance from CG to front axle [m].
    lr : float
        Distance from CG to rear axle [m].
    iz : float
        Yaw moment of inertia [kg·m²].
    cf : float
        Front cornering stiffness [N/rad].
    cr : float
        Rear cornering stiffness [N/rad].
    steering_time_constant : float
        First-order steering actuator time constant [s].
    """

    mass: float
    wheelbase: float
    lf: float
    lr: float
    iz: float
    cf: float
    cr: float
    steering_time_constant: float


class VehicleModel:
    """
    Full dynamic bicycle vehicle model.
    """

    def __init__(
        self,
        dyn_params: DynamicVehicleParams,
        engine: EngineModel,
        aero: AeroModel,
        slipstream: SlipstreamModel,
    ) -> None:
        self._p = dyn_params
        self._engine = engine
        self._aero = aero
        self._slipstream = slipstream

        # Geometry consistency check
        if not np.isclose(self._p.lf + self._p.lr, self._p.wheelbase, rtol=0.0, atol=1e-6):
            raise ValueError(
                f"Inconsistent geometry: lf+lr={self._p.lf + self._p.lr} != wheelbase={self._p.wheelbase}"
            )

    def step(
        self,
        state: State,
        action: Tuple[float, float, float],
        dt: float,
        leader_state: State | None = None,
    ) -> State:
        """
        Advance dynamic bicycle model.

        Parameters
        ----------
        state : State
        action : Tuple[float, float, float]
            (throttle, brake, steering_command)
        dt : float
        leader_state : State | None
            Optional leader vehicle for slipstream effects.

        Returns
        -------
        State
        """

        throttle, brake, steering_command = action

        vx = float(state.vx)
        vy = float(state.vy)
        r = float(state.yaw_rate)
        delta = float(state.steering_angle)

        # ------------------------------------------------------------
        # Steering actuator lag (first-order)
        # ------------------------------------------------------------
        delta_dot = (steering_command - delta) / self._p.steering_time_constant
        delta = delta + delta_dot * dt

        # ------------------------------------------------------------
        # Slipstream modifiers
        # ------------------------------------------------------------
        drag_multiplier = 1.0
        downforce_multiplier = 1.0

        if self._slipstream is not None and leader_state is not None:
            follower_pos = np.array([state.x, state.y], dtype=np.float64)
            leader_pos = np.array([leader_state.x, leader_state.y], dtype=np.float64)

            drag_multiplier, downforce_multiplier = self._slipstream.compute_effect(
                follower_pos,
                leader_pos,
                leader_state.heading,
            )

        # ------------------------------------------------------------
        # Longitudinal force
        # ------------------------------------------------------------
        fx = float(self._engine.compute_force(throttle, brake))

        # Aero drag
        fx -= float(self._aero.drag_force(vx, drag_multiplier))

        # ------------------------------------------------------------
        # Slip angles (robust at low speed)
        # ------------------------------------------------------------
        vx_safe = float(np.sign(vx) * max(abs(vx), 0.5))  # prevent blow-up near zero speed

        alpha_f = float(np.arctan2(vy + self._p.lf * r, vx_safe) - delta)
        alpha_r = float(np.arctan2(vy - self._p.lr * r, vx_safe))

        # ------------------------------------------------------------
        # Linear tire forces
        # ------------------------------------------------------------
        fy_f = -self._p.cf * downforce_multiplier * alpha_f
        fy_r = -self._p.cr * downforce_multiplier * alpha_r

        # ------------------------------------------------------------
        # Equations of motion (body frame)
        # ------------------------------------------------------------
        vx_dot = (fx - fy_f * np.sin(delta)) / self._p.mass + vy * r
        vy_dot = (fy_f * np.cos(delta) + fy_r) / self._p.mass - vx * r
        r_dot = (self._p.lf * fy_f * np.cos(delta) - self._p.lr * fy_r) / self._p.iz

        vx = vx + vx_dot * dt
        vy = vy + vy_dot * dt
        r = r + r_dot * dt

        # ------------------------------------------------------------
        # Global position integration
        # ------------------------------------------------------------
        x = state.x + (vx * np.cos(state.heading) - vy * np.sin(state.heading)) * dt
        y = state.y + (vx * np.sin(state.heading) + vy * np.cos(state.heading)) * dt
        heading = state.heading + r * dt

        return State(
            x=float(x),
            y=float(y),
            heading=float(heading),
            vx=float(vx),
            vy=float(vy),
            yaw_rate=float(r),
            steering_angle=float(delta),
            progress_s=state.progress_s,
        )

    def linearize(
        self,
        state: State,
        action: Tuple[float, float, float],
        dt: float,
        leader_state: State | None = None,
    ):
        """
        Linearize system around state/action.
        """
        return linearize(
            self,
            state,
            np.array(action, dtype=np.float64),
            dt,
            leader_state,
        )
