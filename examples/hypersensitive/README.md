# Hypersensitive Problem

## Mathematical Formulation

### Problem Statement

Find the optimal control $u(t)$ that minimizes the quadratic cost functional:

$$J = \int_0^{40} \frac{1}{2}\left( x^2 + u^2 \right) dt$$

Subject to the nonlinear dynamic constraint:

$$\frac{dx}{dt} = -x^3 + u$$

### Boundary Conditions

- **Initial condition**: $x(0) = 1.5$
- **Final condition**: $x(40) = 1.0$
- **Control bounds**: $u \in \mathbb{R}$ (unconstrained)

### Physical Parameters

- Time horizon: $t_f = 40$ s

### State Variables

- $x(t)$: System state

### Control Variable

- $u(t)$: Control input

### Notes

This problem is known as "hypersensitive" because small changes in the control can lead to large changes in the state trajectory due to the nonlinear $-x^3$ term.

## Running This Example

```bash
cd examples/hypersensitive
python hypersensitive.py
```
