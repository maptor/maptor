# Aero-Assisted Plane Change

## Mathematical Formulation

### Problem Statement

Find the optimal lift coefficient $C_L(t)$ and bank angle $\beta(t)$ that maximize the final velocity:

$$J = -v(t_f)$$ (maximizing velocity by minimizing negative velocity)

Subject to the atmospheric flight dynamics:

$$\frac{d\phi}{dt} = \frac{v \cos\gamma \sin\psi}{r}$$

$$\frac{dh}{dt} = v \sin\gamma$$

$$\frac{dv}{dt} = -a_1 \rho v^2 (1 + C_L^2) - \frac{\mu \sin\gamma}{r^2}$$

$$\frac{d\gamma}{dt} = a_0 \rho v (C_L \cos\beta + M \cos\gamma)$$

$$\frac{d\psi}{dt} = \frac{a_0 \rho v C_L \sin\beta}{\cos\gamma} - \frac{v \cos\gamma \cos\psi \tan\phi}{r}$$

Where:

$$r = R_e + h$$

$$\rho = \rho_0 e^{-(h-h_0)/h_r}$$

$$v_s = \sqrt{\frac{\mu}{R_e}}$$

$$\rho_s = \rho_0 e^{h_0/h_r}$$

$$M = \frac{1}{a_0 \rho r}\left(1 - \frac{\mu}{rv^2}\right)$$

$$\dot{q} = 17600 \sqrt{\frac{\rho}{\rho_s}} \left(\frac{v}{v_s}\right)^{3.15}$$

### Heat Rate Constraint

$$\dot{q}(t) \leq 800 \text{ BTU/ft}^2\text{-s}$$

### Terminal Constraint

$$\cos\phi(t_f) \cos\psi(t_f) = \cos(18°)$$

### Boundary Conditions

- **Initial conditions**: $\phi(0) = 0°$, $h(0) = 365000$ ft, $v(0) = 25745.704$ ft/s, $\gamma(0) = -0.55°$, $\psi(0) = 0°$
- **Final conditions**: $h(t_f) = 365000$ ft, others determined by optimization
- **Control bounds**: $0 \leq C_L \leq 2$, $0 \leq \beta \leq 180°$
- **State bounds**: $h \geq 0$, $20000 \leq v \leq 28000$ ft/s, $-89° \leq \phi, \gamma, \psi \leq 89°$

### Physical Parameters

- Earth radius: $R_e = 2.092643 \times 10^7$ ft
- Vehicle mass: $m = 3.315 \times 10^2$ slug
- Sea level density: $\rho_0 = 3.3195 \times 10^{-5}$ slug/ft³
- Reference altitude: $h_0 = 10^5$ ft
- Scale height: $h_r = 2.41388 \times 10^4$ ft
- Drag coefficient: $C_{D0} = 0.032$
- Lift-drag parameter: $k = 1.4$
- Reference area: $S = 1.2584 \times 10^2$ ft²
- Gravitational parameter: $\mu = 1.40895 \times 10^{16}$ ft³/s²
- Derived parameters: $a_0 = \frac{S}{2m}\sqrt{\frac{C_{D0}}{k}}$, $a_1 = \frac{C_{D0} S}{2m}$

### State Variables

- $\phi(t)$: Latitude (rad)
- $h(t)$: Altitude (ft)
- $v(t)$: Velocity magnitude (ft/s)
- $\gamma(t)$: Flight path angle (rad)
- $\psi(t)$: Azimuth angle (rad)

### Control Variables

- $C_L(t)$: Lift coefficient
- $\beta(t)$: Bank angle (rad)

### Notes

This problem involves using atmospheric forces to change orbital inclination while managing heat rate constraints and maximizing final velocity.

## Running This Example

```bash
cd examples/aero-assisted_plane_change
python aero-assisted_plane_change.py
```
