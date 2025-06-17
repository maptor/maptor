# Two-Phase Robot Tracking

## Mathematical Formulation

### Problem Statement

Find the optimal control forces $u_1(t)$ and $u_2(t)$ that minimize the tracking error across two phases:

$$J = \int_0^1 \left[ w_1(x_1 - x_{1,\text{ref}})^2 + w_2(x_2 - x_{2,\text{ref}})^2 + w_3(x_3 - x_{3,\text{ref}})^2 + w_4(x_4 - x_{4,\text{ref}})^2 \right] dt$$
$$+ \int_1^2 \left[ w_1(x_1 - x_{1,\text{ref}})^2 + w_2(x_2 - x_{2,\text{ref}})^2 + w_3(x_3 - x_{3,\text{ref}})^2 + w_4(x_4 - x_{4,\text{ref}})^2 \right] dt$$

Subject to the double integrator dynamics for both phases:

$$\frac{dx_1}{dt} = x_3$$

$$\frac{dx_2}{dt} = x_4$$

$$\frac{dx_3}{dt} = u_1$$

$$\frac{dx_4}{dt} = u_2$$

### Reference Trajectories

**Phase 1** ($t \in [0,1]$):
- $x_{1,\text{ref}} = \frac{t}{2}$, $x_{2,\text{ref}} = 0$, $x_{3,\text{ref}} = 0.5$, $x_{4,\text{ref}} = 0$

**Phase 2** ($t \in [1,2]$):
- $x_{1,\text{ref}} = 0.5$, $x_{2,\text{ref}} = \frac{t-1}{2}$, $x_{3,\text{ref}} = 0$, $x_{4,\text{ref}} = 0.5$

### Boundary Conditions

**Phase 1**:
- **Initial conditions**: $x_1(0) = 0$, $x_2(0) = 0$, $x_3(0) = 0.5$, $x_4(0) = 0$
- **Final conditions**: Continuous with Phase 2

**Phase 2**:
- **Initial conditions**: Continuous from Phase 1
- **Final conditions**: $x_1(2) = 0.5$, $x_2(2) = 0.5$, $x_3(2) = 0$, $x_4(2) = 0.5$
- **Control bounds**: $-10 \leq u_1, u_2 \leq 10$ for both phases
- **State bounds**: $-10 \leq x_i \leq 10$ for $i = 1,2,3,4$

### Physical Parameters

- Tracking weights: $w_1 = 100$, $w_2 = 100$, $w_3 = 500$, $w_4 = 500$
- Phase 1 duration: $t_1 = 1$ s
- Phase 2 duration: $t_2 = 1$ s

### State Variables

- $x_1(t)$: Position coordinate 1
- $x_2(t)$: Position coordinate 2
- $x_3(t)$: Velocity coordinate 1
- $x_4(t)$: Velocity coordinate 2

### Control Variables

- $u_1(t)$: Control force 1
- $u_2(t)$: Control force 2

### Notes

This problem demonstrates multiphase optimal control where a robot must track different reference trajectories in each phase with automatic continuity constraints between phases.

## Running This Example

```bash
cd examples/two_phase_robot
python two_phase_robot.py
```
