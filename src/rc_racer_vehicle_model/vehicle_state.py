"""
Vehicle state definition for the racing simulation core.

This module defines:

- Immutable scalar State (single vehicle)
- Vectorized StateArray (batch vehicles)
- Serialization helpers for deterministic replay
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Iterable, Tuple

import numpy as np
from numpy.typing import NDArray


FloatArray = NDArray[np.float64]


# ============================================================
# Scalar State
# ============================================================


@dataclass(frozen=True)
class State:
    """
    Immutable full dynamic vehicle state.

    Parameters
    ----------
    x : float
        Global x-position [m].
    y : float
        Global y-position [m].
    heading : float
        Heading angle [rad].
    vx : float
        Longitudinal velocity vx [m/s].
    vy : float
        Lateral velocity in body frame [m/s].
    yaw_rate : float
        Yaw rate r [rad/s].
    steering_angle : float
        Steering angle [rad].
    progress_s : float
        Arc-length progress along track [m].
    """

    x: float
    y: float
    heading: float
    vx: float
    vy: float
    yaw_rate: float
    steering_angle: float
    progress_s: float

    def as_tuple(self) -> Tuple[float, ...]:
        """
        Convert state to tuple.

        Returns
        -------
        tuple of float
        """
        return (
            self.x,
            self.y,
            self.heading,
            self.vx,
            self.vy,
            self.yaw_rate,
            self.steering_angle,
            self.progress_s,
        )
    # --------------------------------------------------------

    def to_dict(self) -> Dict[str, float]:
        """
        Serialize state to dictionary.

        Returns
        -------
        dict
            JSON-safe representation.
        """
        return {
            "x": self.x,
            "y": self.y,
            "heading": self.heading,
            "vx": self.vx,
            "vy": self.vy,
            "yaw_rate": self.yaw_rate,
            "steering_angle": self.steering_angle,
            "progress_s": self.progress_s,
        }

    # --------------------------------------------------------

    @staticmethod
    def from_dict(data: Dict[str, Any]) -> State:
        """
        Deserialize state from dictionary.

        Parameters
        ----------
        data : dict

        Returns
        -------
        State
        """
        return State(
            x=float(data["x"]),
            y=float(data["y"]),
            heading=float(data["heading"]),
            vx=float(data["vx"]),
            vy=float(data["vy"]),
            yaw_rate=float(data["yaw_rate"]),
            steering_angle=float(data["steering_angle"]),
            progress_s=float(data["progress_s"]),
        )
    # --------------------------------------------------------

    def copy_with(
        self,
        *,
        x: float | None = None,
        y: float | None = None,
        heading: float | None = None,
        vx: float | None = None,
        vy: float | None = None,
        yaw_rate: float | None = None,
        steering_angle: float | None = None,
        progress_s: float | None = None,
    ) -> "State":
        """
        Return new State with selected fields replaced.
        """
        return State(
            x=self.x if x is None else x,
            y=self.y if y is None else y,
            heading=self.heading if heading is None else heading,
            vx=self.vx if vx is None else vx,
            vy=self.vy if vy is None else vy,
            yaw_rate=self.yaw_rate if yaw_rate is None else yaw_rate,
            steering_angle=self.steering_angle if steering_angle is None else steering_angle,
            progress_s=self.progress_s if progress_s is None else progress_s,
        )


# ============================================================
# Vectorized StateArray
# ============================================================


@dataclass(frozen=True)
class StateArray:
    """
    Vectorized vehicle state container.

    Designed for future vector_env.py for parallel stepping.

    All arrays must:
    - Have dtype float64
    - Have identical shape (N,)

    Parameters
    ----------
    x, y, heading, vx, vy, yaw_rate,
    steering_angle, progress_s : ndarray
    """

    x: FloatArray
    y: FloatArray
    heading: FloatArray
    vx: FloatArray
    vy: FloatArray
    yaw_rate: FloatArray
    steering_angle: FloatArray
    progress_s: FloatArray

    def __post_init__(self) -> None:
        shapes = {
            self.x.shape,
            self.y.shape,
            self.heading.shape,
            self.vx.shape,
            self.vy.shape,
            self.yaw_rate.shape,
            self.steering_angle.shape,
            self.progress_s.shape,
        }

        if len(shapes) != 1:
            raise ValueError("All arrays must have identical shape.")

        if len(self.x.shape) != 1:
            raise ValueError("StateArray fields must be 1D arrays of shape (N,).")

        for arr in (
            self.x,
            self.y,
            self.heading,
            self.vx,
            self.vy,
            self.yaw_rate,
            self.steering_angle,
            self.progress_s,
        ):
            if arr.dtype != np.float64:
                raise ValueError("All arrays must be float64.")

    # --------------------------------------------------------

    @property
    def batch_size(self) -> int:
        """
        Number of vehicles.

        Returns
        -------
        int
        """
        return self.x.shape[0]

    # --------------------------------------------------------

    def to_dict(self) -> Dict[str, list[float]]:
        """
        Serialize to JSON-safe dict.
        """
        return {
            "x": self.x.tolist(),
            "y": self.y.tolist(),
            "heading": self.heading.tolist(),
            "vx": self.vx.tolist(),
            "vy": self.vy.tolist(),
            "yaw_rate": self.yaw_rate.tolist(),
            "steering_angle": self.steering_angle.tolist(),
            "progress_s": self.progress_s.tolist(),
        }

    # --------------------------------------------------------

    @staticmethod
    def from_dict(data: Dict[str, Iterable[float]]) -> StateArray:
        """
        Deserialize from dictionary.
        """
        return StateArray(
            x=np.asarray(data["x"], dtype=np.float64),
            y=np.asarray(data["y"], dtype=np.float64),
            heading=np.asarray(data["heading"], dtype=np.float64),
            vx=np.asarray(data["vx"], dtype=np.float64),
            vy=np.asarray(data["vy"], dtype=np.float64),
            yaw_rate=np.asarray(data["yaw_rate"], dtype=np.float64),
            steering_angle=np.asarray(data["steering_angle"], dtype=np.float64),
            progress_s=np.asarray(data["progress_s"], dtype=np.float64),
        )