# Dynamic Obstacle Avoidance

## Mathematical Formulation

### Problem Statement

Find the optimal control inputs $a(t)$ and $\delta(t)$ that minimize the transit time with control effort penalty:

$$J = t_f + 0.01 \int_0^{t_f} (a^2 + \delta^2) dt$$

Subject to the bicycle model dynamics:

$$\frac{dx}{dt} = u \cos\phi - v \sin\phi$$

$$\frac{dy}{dt} = u \sin\phi + v \cos\phi$$

$$\frac{d\phi}{dt} = \omega$$

$$\frac{du}{dt} = a + v \omega - \frac{F_{Y1} \sin\delta}{m}$$

$$\frac{dv}{dt} = -u \omega + \frac{F_{Y1} \cos\delta + F_{Y2}}{m}$$

$$\frac{d\omega}{dt} = \frac{l_f F_{Y1} \cos\delta - l_r F_{Y2}}{I_z}$$

Where the tire forces are:

$$\alpha_f = \frac{v + l_f \omega}{u} - \delta$$

$$\alpha_r = \frac{v - l_r \omega}{u}$$

$$F_{Y1} = -k_f \alpha_f$$

$$F_{Y2} = -k_r \alpha_r$$

And the collision avoidance constraint:

$$(x - x_{\text{obs}}(t))^2 + (y - y_{\text{obs}}(t))^2 \geq d_{\min}^2$$

### Boundary Conditions

- **Initial conditions**: $x(0) = 0$, $y(0) = 0$, $\phi(0) = \frac{\pi}{3}$, $u(0) = 5.0$, $v(0) = 0$, $\omega(0) = 0$
- **Final conditions**: $x(t_f) = 20$, $y(t_f) = 20$, others free
- **Control bounds**: $-8.0 \leq a \leq 8.0$ m/s², $-0.5 \leq \delta \leq 0.5$ rad
- **State bounds**: $0.05 \leq u \leq 20$ m/s, $-15 \leq v \leq 15$ m/s, $-3 \leq \omega \leq 3$ rad/s, $-40 \leq x,y \leq 40$ m

### Physical Parameters

- Vehicle mass: $m = 1412$ kg
- Yaw inertia: $I_z = 1536.7$ kg⋅m²
- Front axle distance: $l_f = 1.06$ m
- Rear axle distance: $l_r = 1.85$ m
- Front cornering stiffness: $k_f = 128916$ N/rad
- Rear cornering stiffness: $k_r = 85944$ N/rad
- Vehicle radius: $r_v = 1.5$ m
- Obstacle radius: $r_o = 2.5$ m
- Minimum separation: $d_{\min} = r_v + r_o = 4.0$ m
- Minimum velocity: $u_{\min} = 0.05$ m/s

### Obstacle Trajectory

The obstacle follows a predetermined path with waypoints:
- $(5.0, 5.0)$ at $t = 0$ s
- $(12.0, 12.0)$ at $t = 3$ s
- $(15.0, 15.0)$ at $t = 6$ s
- $(20.0, 20.0)$ at $t = 12$ s

Linear interpolation is used between waypoints with time clamping at boundaries.

### State Variables

- $x(t)$: Vehicle x-position (m)
- $y(t)$: Vehicle y-position (m)
- $\phi(t)$: Vehicle heading angle (rad)
- $u(t)$: Longitudinal velocity (m/s)
- $v(t)$: Lateral velocity (m/s)
- $\omega(t)$: Yaw rate (rad/s)

### Control Variables

- $a(t)$: Longitudinal acceleration (m/s²)
- $\delta(t)$: Front wheel steering angle (rad)

### Notes

This problem demonstrates collision avoidance with a moving obstacle using a full dynamic bicycle model.

## Running This Example

```bash
cd examples/dynamic_obstacle_avoidance
python dynamic_obstacle_avoidance.py
python dynamic_obstacle_avoidance_animate.py
```
