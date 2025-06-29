# MAPTOR Mechanics Module

Converts SymPy Lagrangian mechanics to MAPTOR dynamics format.

## Purpose

The `lagrangian_to_maptor_dynamics()` function transforms symbolic equations from SymPy's mechanics module into CasADi expressions ready for MAPTOR optimal control problems.

## Basic Usage

```python
import sympy as sm
import sympy.physics.mechanics as me
from maptor.mechanics import lagrangian_to_maptor_dynamics

# Set up your SymPy mechanics problem
# (see SymPy mechanics tutorial for details)
L = me.Lagrangian(N, body1, body2)
LM = me.LagrangesMethod(L, [q1, q2], forcelist=loads)

# Define control forces/torques
control_forces = sm.Matrix([tau1, tau2])

# Convert to MAPTOR format
lagrangian_to_maptor_dynamics(LM, [q1, q2], control_forces)
```

Output provides copy-paste ready MAPTOR dynamics:
```python
# Generated output:
phase.dynamics({
    q1: q1_dot,
    q2: q2_dot,
    q1_dot: (complex_expression),
    q2_dot: (complex_expression),
})
```

## API

### `lagrangian_to_maptor_dynamics(lagranges_method, coordinates, control_forces, output_file=None)`

**Parameters:**
- `lagranges_method`: SymPy LagrangesMethod object
- `coordinates`: List of generalized coordinates (e.g., `[q1, q2]`)
- `control_forces`: SymPy Matrix of control forces, shape `(n_coords, 1)`
- `output_file`: Optional filename for backup (saved next to calling script)

**Returns:**
- `(casadi_equations, state_names)` for programmatic use

## SymPy Mechanics Setup

For SymPy mechanics setup (reference frames, bodies, forces), see:
**https://docs.sympy.org/latest/tutorials/physics/mechanics/index.html**

## Complete Examples

See MAPTOR examples directory:
- `examples/cartpole_swingup/`
- `examples/manipulator_2dof/`
- `examples/manipulator_3dof/`
