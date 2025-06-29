# Acrobot Swing-Up

## Mathematical Formulation

### Problem Statement

Find the optimal elbow torque $\tau(t)$ that swings up the acrobot from the hanging equilibrium to the inverted equilibrium while minimizing time and control effort:

$$J = t_f + 0.01 \int_0^{t_f} \tau^2 dt$$

Subject to the underactuated acrobot dynamics:

$$\frac{d\theta_1}{dt} = \dot{\theta}_1$$

$$\frac{d\theta_2}{dt} = \dot{\theta}_2$$

$$\frac{d\dot{\theta}_1}{dt} = \frac{-I_2 \left( g l_1 m_2 \sin(\theta_1) + g l_{c1} m_1 \sin(\theta_1) + g l_{c2} m_2 \sin(\theta_1 + \theta_2) - l_1 l_{c2} m_2 (2\dot{\theta}_1 + \dot{\theta}_2) \sin(\theta_2) \dot{\theta}_2 \right) + (I_2 + l_1 l_{c2} m_2 \cos(\theta_2)) \left( g l_{c2} m_2 \sin(\theta_1 + \theta_2) + l_1 l_{c2} m_2 \sin(\theta_2) \dot{\theta}_1^2 - \tau \right)}{I_1 I_2 + I_2 l_1^2 m_2 - l_1^2 l_{c2}^2 m_2^2 \cos^2(\theta_2)}$$

$$\frac{d\dot{\theta}_2}{dt} = \frac{(I_2 + l_1 l_{c2} m_2 \cos(\theta_2)) \left( g l_1 m_2 \sin(\theta_1) + g l_{c1} m_1 \sin(\theta_1) + g l_{c2} m_2 \sin(\theta_1 + \theta_2) - l_1 l_{c2} m_2 (2\dot{\theta}_1 + \dot{\theta}_2) \sin(\theta_2) \dot{\theta}_2 \right) - \left( g l_{c2} m_2 \sin(\theta_1 + \theta_2) + l_1 l_{c2} m_2 \sin(\theta_2) \dot{\theta}_1^2 - \tau \right) (I_1 + I_2 + l_1^2 m_2 + 2 l_1 l_{c2} m_2 \cos(\theta_2))}{I_1 I_2 + I_2 l_1^2 m_2 - l_1^2 l_{c2}^2 m_2^2 \cos^2(\theta_2)}$$

### Boundary Conditions

- **Initial conditions**: $\theta_1(0) = \frac{\pi}{6}$ rad, $\theta_2(0) = 0$ rad, $\dot{\theta}_1(0) = 0$ rad/s, $\dot{\theta}_2(0) = 0$ rad/s
- **Final conditions**: $\theta_1(t_f) = \pi$ rad, $\theta_2(t_f) = 0$ rad, $\dot{\theta}_1(t_f) = 0$ rad/s, $\dot{\theta}_2(t_f) = 0$ rad/s
- **Control bounds**: $-20.0 \leq \tau \leq 20.0$ N⋅m

### Physical Parameters

- **Upper arm mass**: $m_1 = 1.0$ kg
- **Forearm mass**: $m_2 = 1.0$ kg
- **Upper arm length**: $l_1 = 1.0$ m
- **Forearm length**: $l_2 = 1.0$ m
- **Upper arm COM distance**: $l_{c1} = 0.5$ m
- **Forearm COM distance**: $l_{c2} = 0.5$ m
- **Upper arm inertia about shoulder**: $I_1 = \frac{m_1 l_1^2}{3} = 0.333$ kg⋅m²
- **Forearm inertia about elbow**: $I_2 = \frac{m_2 l_2^2}{3} = 0.333$ kg⋅m²
- **Gravity**: $g = 9.81$ m/s²

### State Variables

- $\theta_1(t)$: Shoulder joint angle from downward vertical (rad)
- $\theta_2(t)$: Elbow joint angle relative to upper arm (rad)
- $\dot{\theta}_1(t)$: Shoulder joint angular velocity (rad/s)
- $\dot{\theta}_2(t)$: Elbow joint angular velocity (rad/s)

### Control Variable

- $\tau(t)$: Elbow joint torque (N⋅m)

### Notes

The acrobot is a classic underactuated robotics problem where only the elbow joint can be controlled while the shoulder joint is passive. This creates a challenging swing-up task requiring the system to use nonlinear coupling between the joints to achieve the inverted configuration. The problem demonstrates how underactuated systems can achieve full controllability through dynamic coupling, despite having fewer actuators than degrees of freedom.

## Dynamics Derivation

The acrobot dynamics were derived using Lagrangian mechanics with SymPy, including comprehensive term-by-term verification against established literature. The derivation systematically constructs the mass matrix, gravity vector, and Coriolis terms:

```{literalinclude} ../../../examples/acrobot/acrobot_dynamics.py
:language: python
:caption: examples/acrobot/acrobot_dynamics.py
:linenos:
```

This symbolic derivation produces the complex coupled dynamics equations used in the swing-up controller implementation, ensuring mathematical correctness through comparison with MIT's Underactuated Robotics course materials and providing complete transparency in the underlying mechanics.

## Running This Example

```bash
cd examples/acrobot
python acrobot.py
python acrobot_animate.py
```
