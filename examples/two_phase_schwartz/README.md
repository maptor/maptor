# Two-Phase Schwartz Problem

## Mathematical Formulation

### Problem Statement

Find the optimal control $u(t)$ that minimizes the final state penalty:

$$J = 5(x_0^2(t_f) + x_1^2(t_f))$$

Subject to the nonlinear dynamics for both phases:

$$\frac{dx_0}{dt} = x_1$$

$$\frac{dx_1}{dt} = u - 0.1(1 + 2x_0^2)x_1$$

### Phase-Specific Constraints

**Phase 1** ($t \in [0,1]$):
- **Elliptical exclusion constraint**: $1 - 9(x_0 - 1)^2 - \left(\frac{x_1 - 0.4}{0.3}\right)^2 \leq 0$
- **Control bounds**: $-1 \leq u \leq 1$

**Phase 2** ($t \in [1,2.9]$):
- **No path constraints**
- **Control bounds**: $-50 \leq u \leq 50$

### Boundary Conditions

**Phase 1**:
- **Initial conditions**: $x_0(0) = 1$, $x_1(0) = 1$
- **Final conditions**: Continuous with Phase 2

**Phase 2**:
- **Initial conditions**: Continuous from Phase 1
- **Final conditions**: $x_0(2.9) = \text{free}$, $x_1(2.9) = \text{free}$
- **State bounds**: $-20 \leq x_0 \leq 10$, $-10 \leq x_1 \leq 10$ (Phase 1), $-10 \leq x_1 \leq 10$ (Phase 2)

### Physical Parameters

- Phase 1 duration: $t_1 = 1$ s
- Phase 2 duration: $t_2 = 1.9$ s
- System damping parameter: $\alpha = 0.1$
- Final state penalty weight: $W = 5$

### State Variables

- $x_0(t)$: System state coordinate 1
- $x_1(t)$: System state coordinate 2

### Control Variable

- $u(t)$: Control input

### Notes

This problem features different feasible regions and control authority in each phase, demonstrating the flexibility of multiphase optimal control formulations. The elliptical constraint in Phase 1 creates a forbidden region that the trajectory must avoid.

## Running This Example

```bash
cd examples/two_phase_schwartz
python two_phase_schwartz.py
```
