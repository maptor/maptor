# Multiphase Vehicle Launch

## Mathematical Formulation

### Problem Statement

Find the optimal thrust direction vectors $\mathbf{u}_i(t) = [u_{i,1}, u_{i,2}, u_{i,3}]^T$ for each phase that maximize the final payload mass:

$$J = -m_4(t_f)$$ (maximizing final payload mass by minimizing negative mass)

Subject to the atmospheric flight dynamics with Earth rotation:

$$\frac{d\mathbf{r}}{dt} = \mathbf{v}$$

$$\frac{d\mathbf{v}}{dt} = \mathbf{a}_{\text{thrust}} + \mathbf{a}_{\text{drag}} + \mathbf{a}_{\text{grav}}$$

$$\frac{dm}{dt} = -\dot{m}_{\text{prop}}$$

Where:
- $\mathbf{a}_{\text{thrust}} = \frac{T_{\text{total}}}{m} \mathbf{u}$
- $\mathbf{a}_{\text{drag}} = -\frac{\rho |\mathbf{v}_{\text{rel}}|}{2m} S C_D \mathbf{v}_{\text{rel}}$
- $\mathbf{a}_{\text{grav}} = -\frac{\mu}{|\mathbf{r}|^3} \mathbf{r}$
- $\mathbf{v}_{\text{rel}} = \mathbf{v} - \boldsymbol{\Omega} \times \mathbf{r}$

### Phase Configuration

**Phase 1** ($t \in [0, 75.2]$ s): 6 SRBs + First Stage
- Thrust: $T_1 = 6 \times 628500 + 1083100 = 4854100$ N
- Mass flow: $\dot{m}_1 = 6 \times 17010/75.2 + 95550/261 = 1719.7$ kg/s

**Phase 2** ($t \in [75.2, 150.4]$ s): 3 SRBs + First Stage
- Thrust: $T_2 = 3 \times 628500 + 1083100 = 2968600$ N
- Mass flow: $\dot{m}_2 = 3 \times 17010/75.2 + 95550/261 = 1043.7$ kg/s

**Phase 3** ($t \in [150.4, 261.0]$ s): First Stage Only
- Thrust: $T_3 = 1083100$ N
- Mass flow: $\dot{m}_3 = 95550/261 = 366.1$ kg/s

**Phase 4** ($t \in [261.0, 961.0]$ s): Second Stage Only
- Thrust: $T_4 = 110094$ N
- Mass flow: $\dot{m}_4 = 16820/700 = 24.0$ kg/s

### Terminal Orbital Constraints

The final state must satisfy the target orbital elements:
- Semi-major axis: $a = 24361140$ m
- Eccentricity: $e = 0.7308$
- Inclination: $i = 28.5°$
- Right ascension of ascending node: $\Omega = 269.8°$
- Argument of periapsis: $\omega = 130.5°$

### Boundary Conditions

**Initial Conditions**:
- Position: $\mathbf{r}(0) = [5505524, 0, 3050952]^T$ m
- Velocity: $\mathbf{v}(0) = [0, 403.2, 0]^T$ m/s (Earth rotation)
- Mass: $m(0) = 631464$ kg (total vehicle)

**Final Conditions**:
- Orbital elements as specified above
- Mass: $m(t_f) = \text{maximized}$

**Control Constraints**:
- $|\mathbf{u}_i| = 1$ for all phases (unit thrust vector)

### Physical Parameters

- Earth gravitational parameter: $\mu = 3.986012 \times 10^{14}$ m³/s²
- Earth radius: $R_E = 6378145$ m
- Earth rotation rate: $\Omega = 7.29211585 \times 10^{-5}$ rad/s
- Atmospheric density: $\rho = 1.225 e^{-h/7200}$ kg/m³
- Vehicle parameters: $S = 4\pi$ m², $C_D = 0.5$

### Vehicle Mass Breakdown

- **Solid Rocket Boosters**: 9 × 19290 kg total, 9 × 17010 kg propellant
- **First Stage**: 104380 kg total, 95550 kg propellant
- **Second Stage**: 19300 kg total, 16820 kg propellant
- **Payload**: 4164 kg

### State Variables

- $\mathbf{r}(t) = [x, y, z]^T$: Position vector (m)
- $\mathbf{v}(t) = [v_x, v_y, v_z]^T$: Velocity vector (m/s)
- $m(t)$: Vehicle mass (kg)

### Control Variables

- $\mathbf{u}_i(t) = [u_{i,1}, u_{i,2}, u_{i,3}]^T$: Unit thrust direction vector for phase $i$

### Notes

This problem demonstrates multiphase rocket trajectory optimization with realistic propulsion systems, atmospheric effects, Earth rotation, and orbital mechanics constraints. Each phase has different thrust and mass flow characteristics corresponding to stage separations.

## Running This Example

```bash
cd examples/multiphase_vehicle_launch
python multiphase_vehicle_launch.py
```
