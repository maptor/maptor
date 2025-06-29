# Cartpole Swingup

## Mathematical Formulation

### Problem Statement

Find the optimal control force $F(t)$ that swings up an inverted pendulum (cartpole) from the downward stable position to the upward unstable position in minimum time:

$$J = t_f$$

Subject to the coupled cart-pole dynamics:

$$\frac{dx}{dt} = \dot{x}$$

$$\frac{d\theta}{dt} = \dot{\theta}$$

$$\frac{d\dot{x}}{dt} = \frac{4F + 3gm\sin(2\theta)/2 - 2lm\sin(\theta)\dot{\theta}^2}{4M + 3m\sin^2(\theta) + m}$$

$$\frac{d\dot{\theta}}{dt} = \frac{3[2g(M + m)\sin(\theta) + (2F - lm\sin(\theta)\dot{\theta}^2)\cos(\theta)]}{l(4M + 3m\sin^2(\theta) + m)}$$

### Boundary Conditions

- **Initial conditions**: $x(0) = 0$ m, $\theta(0) = \pi$ rad, $\dot{x}(0) = 0$ m/s, $\dot{\theta}(0) = 0$ rad/s
- **Final conditions**: $x(t_f) = 0$ m, $\theta(t_f) = 0$ rad, $\dot{x}(t_f) = 0$ m/s, $\dot{\theta}(t_f) = 0$ rad/s
- **Control bounds**: $-10.0 \leq F \leq 10.0$ N

### Physical Parameters

- **Cart mass**: $M = 1.0$ kg
- **Pole mass**: $m = 0.1$ kg
- **Pole length**: $l = 0.5$ m
- **Gravity**: $g = 9.81$ m/s²

### State Variables

- $x(t)$: Cart position (m)
- $\theta(t)$: Pole angle from upright vertical (rad)
- $\dot{x}(t)$: Cart velocity (m/s)
- $\dot{\theta}(t)$: Pole angular velocity (rad/s)

### Control Variable

- $F(t)$: Horizontal force applied to cart (N)

### Notes

This is a classic underactuated control problem where the cart-pole system must swing up from the natural downward equilibrium ($\theta = \pi$) to the unstable upward equilibrium ($\theta = 0$) using only horizontal forces on the cart. The coupling between cart motion and pole dynamics enables the swingup maneuver.

## Dynamics Derivation

The cart-pole dynamics were derived using Lagrangian mechanics with SymPy. The derivation handles the coupled cart-pole system with external force input:

```{literalinclude} ../../../examples/cartpole_swingup/cartpole_swingup_dynamics.py
:language: python
:caption: examples/cartpole_swingup/cartpole_swingup_dynamics.py
:linenos:
```

This symbolic derivation produces the nonlinear dynamics equations used in the swing-up controller implementation, ensuring mathematical correctness and providing educational insight into the underlying mechanics.

## Running This Example

```bash
cd examples/cartpole_swingup
python cartpole_swingup.py
python cartpole_swingup_animate.py
```
