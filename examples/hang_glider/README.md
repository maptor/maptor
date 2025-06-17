# Hang Glider Flight

## Mathematical Formulation

### Problem Statement

Find the optimal lift coefficient $C_L(t)$ that maximizes the final range:

$$J = -x(t_f)$$ (maximizing range by minimizing negative range)

Subject to the atmospheric flight dynamics:

$$\frac{dx}{dt} = v_x$$

$$\frac{dy}{dt} = v_y$$

$$\frac{dv_x}{dt} = -\frac{1}{m}(L \sin\eta + D \cos\eta)$$

$$\frac{dv_y}{dt} = \frac{1}{m}(L \cos\eta - D \sin\eta) - g$$

Where the aerodynamic forces are:

$$L = \frac{1}{2} C_L \rho S v_r^2$$

$$D = \frac{1}{2} C_D \rho S v_r^2$$

$$C_D = C_0 + K C_L^2$$

And the flight path parameters:

$$v_r = \sqrt{v_x^2 + V_y^2}$$

$$V_y = v_y - u_a(x)$$

$$\sin\eta = \frac{V_y}{v_r}, \quad \cos\eta = \frac{v_x}{v_r}$$

### Thermal Updraft Model

$$u_a(x) = U_m \left(1 - X\right) e^{-X}$$

where $X = (\frac{x}{R} - 2.5)^2$

### Boundary Conditions

- **Initial conditions**: $x(0) = 0$, $y(0) = 1000$, $v_x(0) = 13.2275675$, $v_y(0) = -1.28750052$
- **Final conditions**: $x(t_f) = \text{free}$, $y(t_f) = 900$, $v_x(t_f) = 13.2275675$, $v_y(t_f) = -1.28750052$
- **Control bounds**: $0 \leq C_L \leq 1.4$
- **State bounds**: $0 \leq x \leq 1500$ m, $0 \leq y \leq 1100$ m, $0 \leq v_x \leq 15$ m/s, $-4 \leq v_y \leq 4$ m/s

### Physical Parameters

- Mass: $m = 100$ kg
- Gravitational acceleration: $g = 9.80665$ m/s²
- Maximum updraft velocity: $U_m = 2.5$ m/s
- Thermal radius: $R = 100$ m
- Parasitic drag coefficient: $C_0 = 0.034$
- Induced drag factor: $K = 0.069662$
- Wing area: $S = 14$ m²
- Air density: $\rho = 1.13$ kg/m³

### State Variables

- $x(t)$: Horizontal position (m)
- $y(t)$: Altitude (m)
- $v_x(t)$: Horizontal velocity (m/s)
- $v_y(t)$: Vertical velocity (m/s)

### Control Variable

- $C_L(t)$: Lift coefficient

### Notes

This problem demonstrates atmospheric flight optimization where the hang glider must exploit thermal updrafts to maximize range while satisfying altitude and velocity constraints.

## Running This Example

```bash
cd examples/hang_glider
python hang_glider.py
```
