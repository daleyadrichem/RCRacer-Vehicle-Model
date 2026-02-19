"""
Factory system for generating deterministic
dynamic VehicleModel configurations.
"""

from __future__ import annotations

from typing import Callable

from rc_racer_vehicle_model.registry import Registry

# Legacy model
from rc_racer_vehicle_model.vehicle_model import (
    VehicleModel,
    DynamicVehicleParams,
)

from rc_racer_vehicle_model.engine_model import (
    EngineModel,
    EngineParams,
)

from rc_racer_vehicle_model.aero_model import (
    AeroModel,
    AeroParams,
)

from rc_racer_vehicle_model.slipstream_model import (
    SlipstreamModel,
    SlipstreamParams,
)

# ================================================================
# Registry
# ================================================================

_dynamic_registry: Registry[Callable[[], VehicleModel]] = Registry()


# ================================================================
# Dynamic Vehicle Presets
# ================================================================


def _dynamic_default() -> VehicleModel:
    """
    Balanced dynamic vehicle preset.

    Returns
    -------
    VehicleModel
        Fully configured dynamic bicycle model.
    """

    # --- Core dynamics ---
    dyn_params = DynamicVehicleParams(
        mass=1200.0,
        wheelbase=2.6,
        lf=1.3,
        lr=1.3,
        iz=1500.0,
        cf=80000.0,
        cr=80000.0,
        steering_time_constant=0.15,
    )

    # --- Engine ---
    engine = EngineModel(
        EngineParams(
            max_drive_force=9000.0,
            max_brake_force=12000.0,
            drivetrain_efficiency=0.95,
        )
    )

    # --- Aerodynamics ---
    aero = AeroModel(
        AeroParams(
            c_d_a=0.7,
            rho=1.225,
        )
    )

    slipstream = SlipstreamModel(
        SlipstreamParams(
            wake_length=20.0,
            wake_angle_deg=15.0,
            drag_reduction_max=0.4,
            downforce_reduction_max=0.25,
            decay_rate=0.15,
        )
    )


    return VehicleModel(
        dyn_params=dyn_params,
        engine=engine,
        aero=aero,
        slipstream=slipstream
    )


def _dynamic_gt3() -> VehicleModel:
    """
    High-downforce GT3-style car.
    """
    dyn_params = DynamicVehicleParams(
        mass=1350.0,
        wheelbase=2.8,
        lf=1.4,
        lr=1.4,
        iz=1800.0,
        cf=100000.0,
        cr=100000.0,
        steering_time_constant=0.12,
    )

    engine = EngineModel(
        EngineParams(
            max_drive_force=11000.0,
            max_brake_force=15000.0,
            drivetrain_efficiency=0.97,
        )
    )

    aero = AeroModel(
        AeroParams(
            c_d_a=0.9,
            rho=1.225,
        )
    )

    slipstream = SlipstreamModel(
        SlipstreamParams(
            wake_length=20.0,
            wake_angle_deg=15.0,
            drag_reduction_max=0.4,
            downforce_reduction_max=0.25,
            decay_rate=0.15,
        )
    )


    return VehicleModel(
        dyn_params=dyn_params,
        engine=engine,
        aero=aero,
        slipstream=slipstream
    )


def _dynamic_drift() -> VehicleModel:
    """
    Drift-oriented setup (rear unstable).
    """
    dyn_params = DynamicVehicleParams(
        mass=1100.0,
        wheelbase=2.5,
        lf=1.2,
        lr=1.3,
        iz=1300.0,
        cf=60000.0,
        cr=40000.0,  # weaker rear grip
        steering_time_constant=0.08,
    )

    engine = EngineModel(
        EngineParams(
            max_drive_force=13000.0,
            max_brake_force=10000.0,
            drivetrain_efficiency=0.9,
        )
    )

    aero = AeroModel(
        AeroParams(
            c_d_a=0.6,
            rho=1.225,
        )
    )

    slipstream = SlipstreamModel(
        SlipstreamParams(
            wake_length=20.0,
            wake_angle_deg=15.0,
            drag_reduction_max=0.4,
            downforce_reduction_max=0.25,
            decay_rate=0.15,
        )
    )


    return VehicleModel(
        dyn_params=dyn_params,
        engine=engine,
        aero=aero,
        slipstream=slipstream
    )


# ================================================================
# Register Presets
# ================================================================

_dynamic_registry.register("dynamic_default", _dynamic_default)
_dynamic_registry.register("dynamic_gt3", _dynamic_gt3)
_dynamic_registry.register("dynamic_drift", _dynamic_drift)


# ================================================================
# Public API
# ================================================================


class VehicleFactory:
    """
    Extended factory for dynamic vehicle models.
    """

    # ------------------------------------------------------------

    @staticmethod
    def create_dynamic(name: str) -> VehicleModel:
        """
        Create dynamic VehicleModel from preset.

        Parameters
        ----------
        name : str
            Preset name.

        Returns
        -------
        VehicleModel
        """
        return _dynamic_registry.create(name)

    # ------------------------------------------------------------

    @staticmethod
    def available_dynamic() -> list[str]:
        """
        Available dynamic presets.

        Returns
        -------
        list[str]
        """
        return _dynamic_registry.available
