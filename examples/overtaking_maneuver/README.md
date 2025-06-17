# Robust Street Overtaking

## Mathematical Formulation

### Problem Statement

Find the optimal control inputs $a(t)$ and $\delta(t)$ that minimize the normalized time and control effort:

$$J = \frac{t_f}{t_{\text{scale}}} + 0.02 \int_0^{t_f} \left[\left(\frac{a}{a_{\text{scale}}}\right)^2 + \left(\frac{\delta}{\delta_{\text{scale}}}\right)^2\right] dt$$

Subject to the bicycle model dynamics:

$$\frac{dx}{dt} = u \cos\phi - v \sin\phi$$

$$\frac{dy}{dt} = u \sin\phi + v \cos\phi$$

$$\frac{d\phi}{dt} = \omega$$

$$\frac{du}{dt} = a + v \omega - \frac{F_{Y1} \sin\delta}{m}$$

$$\frac{dv}{dt} = -u \omega + \frac{F_{Y1} \cos\delta + F_{Y2}}{m}$$

$$\frac{d\omega}{dt} = \frac{l_f F_{Y1} \cos\delta - l_r F_{Y2}}{I_z}$$

Where the tire forces are:

$$\alpha_f = \frac{v + l_f \omega}{u_{\text{safe}}} - \delta$$

$$\alpha_r = \frac{v - l_r \omega}{u_{\text{safe}}}$$

$$F_{Y1} = -k_f \alpha_f$$

$$F_{Y2} = -k_r \alpha_r$$

$$u_{\text{safe}} = \max(u, u_{\min})$$

And dual collision avoidance constraints:

$$(x - x_{\text{obs1}}(t))^2 + (y - y_{\text{obs1}}(t))^2 \geq d_{\min}^2 - 1.0$$

$$(x - x_{\text{obs2}}(t))^2 + (y - y_{\text{obs2}}(t))^2 \geq d_{\min}^2 - 1.0$$

### Street Geometry

- **Street boundaries**: $x \in [-2.0, 20.0]$ m, $y \in [-10.0, 50.0]$ m
- **Lane width**: $w_{\text{lane}} = 3.6$ m
- **Street center**: $x_{\text{center}} = 9.0$ m
- **Right lane center**: $x_{\text{right}} = 10.8$ m
- **Left lane center**: $x_{\text{left}} = 7.2$ m

### Boundary Conditions

- **Initial conditions**: $x(0) = 10.8$, $y(0) = 0.0$, $\phi(0) = \frac{\pi}{2}$, $u(0) = 12.0$, $v(0) = 0$, $\omega(0) = 0$
- **Final conditions**: $x(t_f) = 10.8$, $y(t_f) = 50.0$, $\phi(t_f) = \frac{\pi}{2}$, others free
- **Control bounds**: $-6.0 \leq a \leq 6.0$ m/s², $-0.4 \leq \delta \leq 0.4$ rad
- **State bounds**: $0.5 \leq u \leq 30$ m/s, $-5 \leq v \leq 5$ m/s, $-2.5 \leq \omega \leq 2.5$ rad/s

### Physical Parameters

- **Vehicle mass**: $m = 1412$ kg
- **Yaw inertia**: $I_z = 1536.7$ kg⋅m²
- **Front axle distance**: $l_f = 1.06$ m
- **Rear axle distance**: $l_r = 1.85$ m
- **Front cornering stiffness**: $k_f = 128916$ N/rad
- **Rear cornering stiffness**: $k_r = 85944$ N/rad
- **Vehicle radius**: $r_v = 1.5$ m
- **Obstacle radius**: $r_o = 1.5$ m
- **Minimum separation**: $d_{\min} = r_v + r_o + 0.5 = 3.5$ m
- **Minimum velocity**: $u_{\min} = 0.5$ m/s

### Scaling Parameters

- **Time scale**: $t_{\text{scale}} = 5.0$ s
- **Acceleration scale**: $a_{\text{scale}} = 10.0$ m/s²
- **Steering scale**: $\delta_{\text{scale}} = 0.3$ rad

### Obstacle Trajectories

**Obstacle 1** (Right lane, moving forward):
- $(10.8, 15.0)$ at $t = 0$ s
- $(10.8, 40.0)$ at $t = 5$ s

**Obstacle 2** (Left lane, moving backward):
- $(7.2, 55.0)$ at $t = 0$ s
- $(7.2, 0.0)$ at $t = 15$ s

Linear interpolation with time clamping is used between waypoints.

### State Variables

- $x(t)$: Lateral position (m)
- $y(t)$: Longitudinal position (m)
- $\phi(t)$: Vehicle heading angle (rad)
- $u(t)$: Longitudinal velocity (m/s)
- $v(t)$: Lateral velocity (m/s)
- $\omega(t)$: Yaw rate (rad/s)

### Control Variables

- $a(t)$: Longitudinal acceleration (m/s²)
- $\delta(t)$: Front wheel steering angle (rad)

### Mesh Configuration

The problem uses an adaptive mesh with 11 polynomial segments:

- **Polynomial degrees**: $[4, 4, 4, 6, 7, 8, 6, 5, 4, 4, 4]$
- **Mesh nodes**: $[-1.0, -0.7, -0.4, -0.2, -0.1, 0.0, 0.2, 0.4, 0.6, 0.8, 0.9, 1.0]$

This provides higher resolution during critical overtaking phases.

### Notes

This problem demonstrates complex street overtaking with two moving obstacles in opposing directions. The vehicle must coordinate lane changes while avoiding both a forward-moving obstacle in the right lane and a backward-moving obstacle in the left lane. The formulation includes realistic vehicle dynamics, street constraints, and collision avoidance with safety margins.

## Running This Example

```bash
cd examples/overtaking_maneuver
python overtaking_maneuver.py
python overtaking_maneuver_animate.py
```
