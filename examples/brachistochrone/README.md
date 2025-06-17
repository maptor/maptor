# Brachistochrone Problem

## Mathematical Formulation

### Problem Statement

Find the optimal control angle $\theta(t)$ that minimizes the transit time between two specified points under gravitational acceleration.

$$J = t_f$$

Subject to the dynamic constraints:

$$\frac{dx}{dt} = v \sin \theta$$

$$\frac{dy}{dt} = v \cos \theta$$

$$\frac{dv}{dt} = g \cos \theta$$

### Boundary Conditions

- **Initial conditions**: $x(0) = 0$, $y(0) = 0$, $v(0) = 0$
- **Final conditions**: $x(t_f) = 2.0$, $y(t_f) = 2.0$, $v(t_f)$ = free
- **Control bounds**: $0 \leq \theta \leq \frac{\pi}{2}$

### Physical Parameters

- Gravitational acceleration: $g = 9.81$ m/s²

### State Variables

- $x(t)$: Horizontal position (m)
- $y(t)$: Vertical position (m)
- $v(t)$: Speed magnitude (m/s)

### Control Variable

- $\theta(t)$: Path angle with respect to the vertical (rad)


## Running This Example

```bash
cd examples/brachistochrone
python brachistochrone.py
```
