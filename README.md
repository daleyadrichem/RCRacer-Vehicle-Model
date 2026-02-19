# RC Racer Vehicle Model

Deterministic dynamic bicycle vehicle model designed for:

* Reinforcement learning
* Model-based control
* MPC
* System identification
* Racing simulation research

The model includes:

* Full dynamic bicycle equations
* Engine & brake force model
* Quadratic aerodynamic drag
* Slipstream / drafting model
* Steering actuator lag
* Finite-difference linearization
* FastAPI simulation server

---

# Project Structure

```
src/rc_racer_vehicle_model/
    vehicle_model.py
    vehicle_state.py
    engine_model.py
    aero_model.py
    slipstream_model.py
    linearization.py
    vehicle_factory.py
    vehicle_api.py

pyproject.toml
Dockerfile
README.md
```

---

# Dynamic Bicycle Model

The vehicle uses a **full dynamic bicycle model** in body frame coordinates.

State:

```
x, y, heading
vx, vy, yaw_rate
steering_angle
```

---

## Slip Angles

Front and rear slip angles:

$$
\alpha_f = \arctan\left(\frac{v_y + l_f r}{v_x}\right) - \delta
$$

$$
\alpha_r = \arctan\left(\frac{v_y - l_r r}{v_x}\right)
$$

---

## Linear Tire Model

$$
F_{yf} = -C_f \alpha_f
$$

$$
F_{yr} = -C_r \alpha_r
$$

---

## Longitudinal Force

The total longitudinal force is:

$$
F_x = F_{engine} - F_{drag}
$$

---

## Engine Model

Deterministic throttle/brake model:

$$
F_{engine} =
\text{throttle} \cdot F_{max} \cdot \eta
----------------------------------------

\text{brake} \cdot F_{brake}
$$

No torque curve or traction limits are modeled.

---

## Aerodynamic Drag

Quadratic drag:

$$
F_{drag} = \frac{1}{2} \rho C_d A ; v_x |v_x|
$$

Optionally scaled by slipstream multiplier.

---

## Dynamic Equations of Motion

Body-frame equations:

$$
\dot{v}*x = \frac{F_x - F*{yf}\sin\delta}{m} + v_y r
$$

$$
\dot{v}*y = \frac{F*{yf}\cos\delta + F_{yr}}{m} - v_x r
$$

$$
\dot{r} = \frac{l_f F_{yf}\cos\delta - l_r F_{yr}}{I_z}
$$

---

## Global Position Integration

$$
\dot{x} = v_x \cos(\psi) - v_y \sin(\psi)
$$

$$
\dot{y} = v_x \sin(\psi) + v_y \cos(\psi)
$$

$$
\dot{\psi} = r
$$

---

# Slipstream Model

The slipstream model reduces:

* Aerodynamic drag
* Downforce-dependent tire forces

If a follower is inside a wake cone behind a leader:

$$
drag_multiplier = 1 - D_{max} e^{-k d}
$$

$$
downforce_multiplier = 1 - L_{max} e^{-k d}
$$

Where:

* ( d ) = longitudinal distance
* ( k ) = decay rate

---

# Steering Actuator

First-order lag model:

$$
\dot{\delta} = \frac{\delta_{cmd} - \delta}{\tau}
$$

---

# Linearization

The system supports numerical linearization:

$$
x_{k+1} = A x_k + B u_k
$$

Jacobian matrices are computed using finite differences.

File:

```
linearization.py
```

---

# Vehicle Presets

Factory system provides presets:

* `dynamic_default`
* `dynamic_gt3`
* `dynamic_drift`

Example:

```python
from rc_racer_vehicle_model.vehicle_factory import VehicleFactory

vehicle = VehicleFactory.create_dynamic("dynamic_gt3")
```

---

# FastAPI Simulation Server

Start API:

```bash
uvicorn rc_racer_vehicle_model.vehicle_api:app --reload
```

Endpoints:

### List Vehicles

```
GET /vehicle_list
```

### Start Simulation

```
POST /start
{
    "vehicle_name": "dynamic_default"
}
```

### Step Simulation

```
POST /step
{
    "throttle": 1.0,
    "brake": 0.0,
    "steering": 0.1,
    "dt": 0.01
}
```

---

# Determinism

The entire model is:

* Fully deterministic
* No randomness
* No hidden state
* Fully reproducible

Suitable for:

* RL research
* MPC development
* Trajectory optimization
* System identification

---

# Docker

Build:

```bash
docker build -t rc-racer .
```

Run:

```bash
docker run -p 8000:8000 rc-racer
```

---

# Design Philosophy

This model intentionally avoids:

* Magic constants
* Hidden saturation
* Black-box physics
* Non-deterministic components

Everything is explicit and inspectable.

---

# Future Extensions

* Nonlinear tire model (Pacejka)
* Combined slip
* Load transfer
* Suspension model
* Track coordinate frame
* Multi-agent racing
* GPU vectorization

---

# License

MIT License

---