# Robot Arm Control

## Mathematical Formulation

### Problem Statement

Find the optimal control torques $u_1(t)$, $u_2(t)$, $u_3(t)$ that minimize the maneuver time:

$$J = t_f$$

Subject to the robot arm dynamic constraints:

$$\frac{dy_1}{dt} = y_2$$

$$\frac{dy_2}{dt} = \frac{u_1}{L}$$

$$\frac{dy_3}{dt} = y_4$$

$$\frac{dy_4}{dt} = \frac{u_2}{I_\theta}$$

$$\frac{dy_5}{dt} = y_6$$

$$\frac{dy_6}{dt} = \frac{u_3}{I_\phi}$$

Where the inertias are:

$$I_\phi = \frac{1}{3}\left[(L - y_1)^3 + y_1^3\right]$$

$$I_\theta = I_\phi \sin^2(y_5)$$

### Boundary Conditions

- **Initial conditions**: $y_1(0) = 4.5$, $y_2(0) = 0$, $y_3(0) = 0$, $y_4(0) = 0$, $y_5(0) = \frac{\pi}{4}$, $y_6(0) = 0$
- **Final conditions**: $y_1(t_f) = 4.5$, $y_2(t_f) = 0$, $y_3(t_f) = \frac{2\pi}{3}$, $y_4(t_f) = 0$, $y_5(t_f) = \frac{\pi}{4}$, $y_6(t_f) = 0$
- **Control bounds**: $-1 \leq u_i \leq 1$ for $i = 1,2,3$

### Physical Parameters

- Robot arm length: $L = 5$ m

### State Variables

- $y_1(t)$: Radial position coordinate (m)
- $y_2(t)$: Radial velocity (m/s)
- $y_3(t)$: Azimuthal angle (rad)
- $y_4(t)$: Azimuthal angular velocity (rad/s)
- $y_5(t)$: Elevation angle (rad)
- $y_6(t)$: Elevation angular velocity (rad/s)

### Control Variables

- $u_1(t)$: Radial control torque
- $u_2(t)$: Azimuthal control torque
- $u_3(t)$: Elevation control torque

### Notes

This problem involves controlling a robot arm with three degrees of freedom to perform a minimum-time maneuver with bounded control torques.

## Running This Example

```bash
cd examples/robot_arm
python robot_arm.py
```
