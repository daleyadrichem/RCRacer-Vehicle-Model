# 🏎️ RC Racer Vehicle Model

Deterministic dynamic bicycle vehicle model with:

* Throttle / brake engine model
* Aerodynamic drag
* Slipstream (drafting) interaction
* Steering actuator lag
* Full dynamic bicycle equations
* Finite-difference linearization API
* FastAPI simulation server

Project structure: 

---

# 📦 Project Structure

```
src/rc_racer_vehicle_model/
│
├── vehicle_model.py
├── vehicle_state.py
├── engine_model.py
├── aero_model.py
├── slipstream_model.py
├── linearization.py
├── vehicle_factory.py
├── vehicle_api.py
│
Dockerfile
pyproject.toml
README.md
```

Core modules:

* Engine model 
* Vehicle state definition 
* Slipstream model 
* Aerodynamic model 
* Dynamic bicycle model 
* REST API 
* Vehicle presets factory 
* Linearization utility 

---

# 🚀 Installation

```bash
pip install uv
uv sync
```

Or inside Docker:

```bash
docker build -t racer-model .
docker run -p 8000:8000 racer-model
```

---

# 🧠 Model Overview

The system implements a **dynamic bicycle model** with aerodynamic and drafting effects.

It is:

* Deterministic
* Continuous-time integrated via forward Euler
* Suitable for control, RL, MPC, and simulation

---

# 🏁 State Representation

Defined in 

The vehicle state is:

[
x = [x, y, \psi, v_x, v_y, r, \delta, s]
]

Where:

| Variable | Meaning                     |
| -------- | --------------------------- |
| x, y     | Global position [m]         |
| ψ        | Heading angle [rad]         |
| vx       | Longitudinal velocity [m/s] |
| vy       | Lateral velocity [m/s]      |
| r        | Yaw rate [rad/s]            |
| δ        | Steering angle [rad]        |
| s        | Track progress [m]          |

---

# 🔧 Engine Model

Defined in 

Simple deterministic longitudinal force:

[
F_x = T \cdot F_{drive,max} \cdot \eta - B \cdot F_{brake,max}
]

Where:

* T ∈ [0,1] = throttle
* B ∈ [0,1] = brake
* η = drivetrain efficiency

No torque curve, no traction limits.

---

# 🌬 Aerodynamic Model

Defined in 

Quadratic drag:

[
F_{drag} = \frac{1}{2} \rho C_d A ; v ; |v|
]

Properties:

* Signed force (always opposes motion)
* Slipstream multiplier supported

---

# 🌀 Slipstream Model

Defined in 

Models aerodynamic wake behind a leading vehicle.

Conditions:

* Follower must be behind leader
* Inside wake cone
* Within wake length

Drag multiplier:

[
m_{drag} = 1 - k_{drag} e^{-\lambda d}
]

Downforce multiplier:

[
m_{downforce} = 1 - k_{downforce} e^{-\lambda d}
]

Effect:

* Reduced drag (higher top speed)
* Reduced downforce (less grip)

---

# 🚗 Dynamic Bicycle Model

Defined in 

## Steering Actuator

First-order lag:

[
\dot{\delta} = \frac{\delta_{cmd} - \delta}{\tau}
]

---

## Slip Angles

[
\alpha_f = \arctan\left(\frac{v_y + l_f r}{v_x}\right) - \delta
]

[
\alpha_r = \arctan\left(\frac{v_y - l_r r}{v_x}\right)
]

---

## Tire Forces (Linear)

[
F_{yf} = -C_f \alpha_f
]

[
F_{yr} = -C_r \alpha_r
]

---

## Longitudinal Force

[
F_x = F_{engine} - F_{drag}
]

---

## Equations of Motion

Body frame:

[
\dot{v_x} = \frac{F_x - F_{yf}\sin\delta}{m} + v_y r
]

[
\dot{v_y} = \frac{F_{yf}\cos\delta + F_{yr}}{m} - v_x r
]

[
\dot{r} = \frac{l_f F_{yf}\cos\delta - l_r F_{yr}}{I_z}
]

---

## Global Integration

[
\dot{x} = v_x \cos\psi - v_y \sin\psi
]

[
\dot{y} = v_x \sin\psi + v_y \cos\psi
]

[
\dot{\psi} = r
]

Integrated with forward Euler:

[
x_{k+1} = x_k + \dot{x} dt
]

---

# 🧮 Linearization

Defined in 

Finite-difference Jacobians:

[
A = \frac{\partial f}{\partial x}
\quad
B = \frac{\partial f}{\partial u}
]

Returns:

```
A ∈ R^{8×8}
B ∈ R^{8×3}
```

Useful for:

* LQR
* MPC
* Control analysis

---

# 🏭 Vehicle Presets

Defined in 

Available presets:

* `dynamic_default`
* `dynamic_gt3`
* `dynamic_drift`

Example:

```python
from rc_racer_vehicle_model.vehicle_factory import VehicleFactory

vehicle = VehicleFactory.create_dynamic("dynamic_default")
```

---

# 🌐 REST API

Defined in 

Start server:

```bash
uv run uvicorn rc_racer_vehicle_model.vehicle_api:app --reload
```

### Endpoints

### GET `/vehicle_list`

Lists available presets.

### POST `/start`

Initialize simulation.

```json
{
  "vehicle_name": "dynamic_default"
}
```

### POST `/step`

```json
{
  "throttle": 0.8,
  "brake": 0.0,
  "steering": 0.1,
  "dt": 0.01
}
```

Returns updated vehicle state.

---

# 🎯 Use Cases

* Reinforcement Learning
* MPC / LQR control
* Multi-agent racing
* Slipstream strategy research
* Deterministic simulation for competitions

---

# ⚙️ Design Philosophy

* Deterministic core
* No hidden randomness
* Clean separation of:

  * Engine
  * Aero
  * Slipstream
  * Dynamics
* Linearization-ready
* API-ready

---

# 📌 Key Properties

| Feature         | Supported |
| --------------- | --------- |
| Dynamic bicycle | ✅         |
| Steering lag    | ✅         |
| Aero drag       | ✅         |
| Slipstream      | ✅         |
| Linearization   | ✅         |
| REST API        | ✅         |
| Deterministic   | ✅         |

---

# 🏁 Summary

This project provides a **research-grade deterministic racing vehicle model** suitable for:

* Control design
* AI training
* Competitive racing simulation
* Drafting strategy modeling

It balances physical realism with computational efficiency.

If you'd like, I can also generate:

* A math-only PDF documentation
* A diagram of the model structure
* An MPC example
* A comparison to kinematic bicycle models
* A multi-vehicle simulation example
