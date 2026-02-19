from typing import Optional

from fastapi import FastAPI, HTTPException
from pydantic import BaseModel

from rc_racer_vehicle_model.vehicle_state import State
from rc_racer_vehicle_model.vehicle_factory import VehicleFactory
from rc_racer_vehicle_model.vehicle_model import VehicleModel

app = FastAPI(title="Racer Vehicle Model API")

# ============================================================
# In-Memory Simulation Context
# ============================================================

class SimulationContext:
    def __init__(self):
        self.vehicle: Optional[VehicleModel] = None
        self.state: Optional[State] = None

SIM = SimulationContext()


# ============================================================
# Request / Response Models
# ============================================================

class StartRequest(BaseModel):
    vehicle_name: str
    initial_state: State | None = None


class StepRequest(BaseModel):
    throttle: float
    brake: float
    steering: float
    dt: float


class StateResponse(BaseModel):
    x: float
    y: float
    heading: float
    vx: float
    vy: float
    yaw_rate: float
    steering_angle: float
    progress_s: float


# ============================================================
# Endpoints
# ============================================================

@app.get("/vehicle_list")
def vehicle_list():
    """
    List available dynamic vehicle presets.
    """
    return {
        "available_vehicles": VehicleFactory.available_dynamic()
    }


@app.post("/start")
def start_simulation(request: StartRequest):
    """
    Initialize simulation with selected vehicle preset.
    Optionally provide an initial state.
    """
    try:
        vehicle = VehicleFactory.create_dynamic(request.vehicle_name)
    except Exception:
        raise HTTPException(
            status_code=404,
            detail=f"Vehicle '{request.vehicle_name}' not found"
        )

    SIM.vehicle = vehicle

    # If user provided initial state → use it
    if request.initial_state is not None:
        SIM.state = State(
            x=request.initial_state.x,
            y=request.initial_state.y,
            heading=request.initial_state.heading,
            vx=request.initial_state.vx,
            vy=request.initial_state.vy,
            yaw_rate=request.initial_state.yaw_rate,
            steering_angle=request.initial_state.steering_angle,
            progress_s=request.initial_state.progress_s,
        )
    else:
        # Default zero state
        SIM.state = State(
            x=0.0,
            y=0.0,
            heading=0.0,
            vx=0.0,
            vy=0.0,
            yaw_rate=0.0,
            steering_angle=0.0,
            progress_s=0.0,
        )

    return {
        "status": "simulation_started",
        "vehicle": request.vehicle_name,
        "initial_state": SIM.state.as_tuple(),
    }



@app.post("/step", response_model=StateResponse)
def step_simulation(request: StepRequest):
    """
    Advance vehicle simulation one timestep.
    """
    if SIM.vehicle is None or SIM.state is None:
        raise HTTPException(
            status_code=400,
            detail="Simulation not started. Call /start first."
        )

    new_state = SIM.vehicle.step(
        SIM.state,
        action=(request.throttle, request.brake, request.steering),
        dt=request.dt,
        leader_state=None,
    )

    SIM.state = new_state

    return StateResponse(
        x=new_state.x,
        y=new_state.y,
        heading=new_state.heading,
        vx=new_state.vx,
        vy=new_state.vy,
        yaw_rate=new_state.yaw_rate,
        steering_angle=new_state.steering_angle,
        progress_s=new_state.progress_s,
    )

@app.get("/get_state", response_model=StateResponse)
def get_state():
    """
    Get current simulation state without advancing.
    """
    if SIM.vehicle is None or SIM.state is None:
        raise HTTPException(
            status_code=400,
            detail="Simulation not started. Call /start first."
        )

    state = SIM.state

    return StateResponse(
        x=state.x,
        y=state.y,
        heading=state.heading,
        vx=state.vx,
        vy=state.vy,
        yaw_rate=state.yaw_rate,
        steering_angle=state.steering_angle,
        progress_s=state.progress_s,
    )
