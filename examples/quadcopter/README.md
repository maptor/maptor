# Quadcopter Minimum Time Trajectory

## Mathematical Formulation

### Problem Statement

Find the optimal motor speeds $\omega_i(t)$ for $i = 1,2,3,4$ that minimize the flight time:

$$J = t_f + \int_0^{t_f} (\omega_1^2 + \omega_2^2 + \omega_3^2 + \omega_4^2) dt$$

Subject to the quadcopter dynamics with Newton-Euler equations:

$$\frac{d\mathbf{r}}{dt} = \mathbf{v}$$

$$\frac{d\mathbf{v}}{dt} = \frac{1}{m}(\mathbf{R}_b^g \mathbf{F}_T + \mathbf{F}_{\text{drag}}) + \mathbf{g}$$

$$\frac{d\boldsymbol{\Theta}}{dt} = \mathbf{W}(\boldsymbol{\Theta}) \boldsymbol{\omega}_b$$

$$\frac{d\boldsymbol{\omega}_b}{dt} = \mathbf{J}^{-1}(\boldsymbol{\tau} - \boldsymbol{\omega}_b \times \mathbf{J}\boldsymbol{\omega}_b - \mathbf{J}_r \boldsymbol{\omega}_b \times \mathbf{e}_z \Omega_{\text{prop}})$$

Where:
- $\mathbf{F}_T = [0, 0, -F_T]^T$ with $F_T = K_T(\omega_1^2 + \omega_2^2 + \omega_3^2 + \omega_4^2)$
- $\mathbf{F}_{\text{drag}} = -[K_{dx} \dot{X}, K_{dy} \dot{Y}, K_{dz} \dot{Z}]^T$
- $\boldsymbol{\tau} = [l_{\text{arm}} K_T(\omega_1^2 - \omega_3^2), l_{\text{arm}} K_T(\omega_2^2 - \omega_4^2), K_d(\omega_1^2 - \omega_2^2 + \omega_3^2 - \omega_4^2)]^T$
- $\Omega_{\text{prop}} = \omega_1 - \omega_2 + \omega_3 - \omega_4$

### Attitude Kinematics

$$\frac{d\phi}{dt} = p + \sin\phi \tan\theta \cdot q + \cos\phi \tan\theta \cdot r$$

$$\frac{d\theta}{dt} = \cos\phi \cdot q - \sin\phi \cdot r$$

$$\frac{d\psi}{dt} = \frac{\sin\phi \cdot q + \cos\phi \cdot r}{\cos\theta}$$

### Danger Zone Constraint

$$(X - X_{\text{danger}})^2 + (Y - Y_{\text{danger}})^2 \geq (R_{\text{danger}} + R_{\text{safety}})^2$$

### Boundary Conditions

- **Initial conditions**: $\mathbf{r}(0) = [1, 1, 1]^T$ m, $\mathbf{v}(0) = \mathbf{0}$, $\boldsymbol{\Theta}(0) = \mathbf{0}$, $\boldsymbol{\omega}_b(0) = \mathbf{0}$
- **Final conditions**: $\mathbf{r}(t_f) = [5, 5, 5]^T$ m, $\mathbf{v}(t_f) = \mathbf{0}$, $\boldsymbol{\Theta}(t_f) = \mathbf{0}$, $\boldsymbol{\omega}_b(t_f) = \mathbf{0}$
- **Control bounds**: $0 \leq \omega_i \leq 1500$ rad/s for $i = 1,2,3,4$
- **State bounds**: $|\phi|, |\theta| \leq 60°$, $Z \geq 0$

### Physical Parameters

- **Vehicle mass**: $m = 0.92$ kg (DJI Mavic 3)
- **Inertia matrix**: $\mathbf{J} = \text{diag}(0.0123, 0.0123, 0.0246)$ kg⋅m²
- **Rotor inertia**: $J_r = 3 \times 10^{-5}$ kg⋅m²
- **Arm length**: $l_{\text{arm}} = 0.19$ m
- **Thrust coefficient**: $K_T = 2.5 \times 10^{-6}$ N⋅s²/rad²
- **Drag coefficient**: $K_d = 6.3 \times 10^{-8}$ N⋅m⋅s²/rad²
- **Linear drag**: $K_{dx} = K_{dy} = 0.15$ N⋅s/m, $K_{dz} = 0.20$ N⋅s/m
- **Danger zone**: Center $(3, 3)$ m, radius $0.4$ m

### State Variables

- $X(t), Y(t), Z(t)$: Position coordinates (m)
- $\dot{X}(t), \dot{Y}(t), \dot{Z}(t)$: Velocity components (m/s)
- $\phi(t), \theta(t), \psi(t)$: Euler angles (roll, pitch, yaw) (rad)
- $p(t), q(t), r(t)$: Angular velocities (body frame) (rad/s)

### Control Variables

- $\omega_1(t), \omega_2(t), \omega_3(t), \omega_4(t)$: Motor angular speeds (rad/s)

### Notes

This problem demonstrates minimum-time quadcopter trajectory optimization with obstacle avoidance. Numerical scaling is applied for improved conditioning.

## Running This Example

```bash
cd examples/quadcopter
python quadcopter.py
python quadcopter_animate.py
```
