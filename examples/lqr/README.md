# LQR Problem

## Mathematical Formulation

### Problem Statement

Find the optimal control $u(t)$ that minimizes the quadratic cost functional:

$$J = \int_0^1 \left( 0.625 x^2 + 0.5 x u + 0.5 u^2 \right) dt$$

Subject to the linear dynamic constraint:

$$\frac{dx}{dt} = 0.5 x + u$$

### Boundary Conditions

- **Initial condition**: $x(0) = 1.0$
- **Final condition**: $x(1) = \text{free}$
- **Control bounds**: $u \in \mathbb{R}$ (unconstrained)

### Physical Parameters

- Time horizon: $t_f = 1.0$ s
- System parameter: $a = 0.5$

### State Variables

- $x(t)$: System state

### Control Variable

- $u(t)$: Control input

## Running This Example

```bash
cd examples/lqr
python lqr.py
```
