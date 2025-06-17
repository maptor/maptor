# Race Car Problem

## Mathematical Formulation

### Problem Statement

Find the optimal throttle control $\theta(t)$ that minimizes the lap time:

$$J = t_f$$

Subject to the dynamic constraints:

$$\frac{d\text{pos}}{dt} = \text{speed}$$

$$\frac{d\text{speed}}{dt} = \theta - \text{speed}$$

And the speed limit constraint:

$$\text{speed}(t) \leq 1 - \frac{1}{2}\sin(2\pi \cdot \text{pos}(t))$$

### Boundary Conditions

- **Initial conditions**: $\text{pos}(0) = 0$, $\text{speed}(0) = 0$
- **Final conditions**: $\text{pos}(t_f) = 1.0$, $\text{speed}(t_f) = \text{free}$
- **Control bounds**: $0 \leq \theta \leq 1$

### Physical Parameters

- Track length: $L = 1.0$ (normalized)
- Maximum throttle: $\theta_{\max} = 1.0$

### State Variables

- $\text{pos}(t)$: Position along track (normalized, 0 to 1)
- $\text{speed}(t)$: Vehicle speed

### Control Variable

- $\theta(t)$: Throttle input (0 = no throttle, 1 = full throttle)

### Notes

The speed limit varies sinusoidally along the track, representing curves where the car must slow down and straights where higher speeds are allowed.

## Running This Example

```bash
cd examples/race_car
python race_car.py
```
