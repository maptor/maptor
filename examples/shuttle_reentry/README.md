# Shuttle Reentry

## Mathematical Formulation

### Problem Statement

Find the optimal angle of attack $\alpha(t)$ and bank angle $\beta(t)$ that maximize the crossrange:

$$J = -\theta(t_f)$$ (maximizing latitude by minimizing negative latitude)

Subject to the atmospheric entry dynamics:

$$\frac{dh}{dt} = v \sin\gamma$$

$$\frac{d\phi}{dt} = \frac{v \cos\gamma \sin\psi}{r \cos\theta}$$

$$\frac{d\theta}{dt} = \frac{v \cos\gamma \cos\psi}{r}$$

$$\frac{dv}{dt} = -\frac{D}{m} - \frac{\mu \sin\gamma}{r^2}$$

$$\frac{d\gamma}{dt} = \frac{L \cos\beta}{mv} + \cos\gamma\left(\frac{v}{r} - \frac{\mu}{vr^2}\right)$$

$$\frac{d\psi}{dt} = \frac{L \sin\beta}{mv \cos\gamma} + \frac{v \cos\gamma \sin\psi \sin\theta}{r \cos\theta}$$

Where the aerodynamic forces are:

$$L = \frac{1}{2} C_L \rho v^2 S$$

$$D = \frac{1}{2} C_D \rho v^2 S$$

$$C_L = A_0 + A_1 \alpha_{\deg}$$

$$C_D = B_0 + B_1 \alpha_{\deg} + B_2 \alpha_{\deg}^2$$

And the atmospheric model:

$$\rho = \rho_0 e^{-h/H_r}$$

$$\mu = \frac{\mu_E}{r^2}$$

### Boundary Conditions

- **Initial conditions**: $h(0) = 260$ km, $\phi(0) = 0°$, $\theta(0) = 0°$, $v(0) = 25.6$ km/s, $\gamma(0) = -1°$, $\psi(0) = 90°$
- **Final conditions**: $h(t_f) = 80$ km, $v(t_f) = 2.5$ km/s, $\gamma(t_f) = -5°$
- **Control bounds**: $-90° \leq \alpha \leq 90°$, $-90° \leq \beta \leq 1°$
- **State bounds**: $h \geq 0$, $-89° \leq \theta, \gamma, \psi \leq 89°$

### Physical Parameters

- Earth gravitational parameter: $\mu_E = 0.14076539 \times 10^{17}$ ft³/s²
- Earth radius: $R_E = 20902900$ ft
- Reference area: $S = 2690$ ft²
- Atmospheric density at sea level: $\rho_0 = 0.002378$ slug/ft³
- Density scale height: $H_r = 23800$ ft
- Vehicle mass: $m = 203000/32.174$ slug
- Aerodynamic coefficients: $A_0 = -0.20704$, $A_1 = 0.029244$, $B_0 = 0.07854$, $B_1 = -0.61592 \times 10^{-2}$, $B_2 = 0.621408 \times 10^{-3}$

### State Variables

- $h(t)$: Altitude (ft)
- $\phi(t)$: Longitude (rad)
- $\theta(t)$: Latitude (rad)
- $v(t)$: Velocity magnitude (ft/s)
- $\gamma(t)$: Flight path angle (rad)
- $\psi(t)$: Azimuth angle (rad)

### Control Variables

- $\alpha(t)$: Angle of attack (rad)
- $\beta(t)$: Bank angle (rad)

### Notes

This problem optimizes the Space Shuttle's atmospheric reentry trajectory to maximize crossrange capability while satisfying thermal and structural constraints.

## Running This Example

```bash
cd examples/shuttle_reentry
python shuttle_reentry.py
```
