<p align="center">
  <img src="docs/source/_static/MAPTOR_banner.png" alt="MAPTOR" width="600">
</p>

# MAPTOR: Multiphase Adaptive Trajectory Optimizer

[![PyPI version](https://img.shields.io/pypi/v/maptor)](https://pypi.org/project/maptor/)
[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)
[![License: LGPL v3](https://img.shields.io/badge/License-LGPL_v3-blue.svg)](https://www.gnu.org/licenses/lgpl-3.0)

**Author:** [David Timothy](https://github.com/ChenDavidTimothy)

**Contact:** chendavidtimothy@gmail.com

A Python framework for **trajectory optimization** using optimal control. MAPTOR solves problems involving the motion of vehicles, robots, spacecraft, and other dynamic systems through space and time using the Legendre-Gauss-Radau pseudospectral method and phs-adaptive mesh refinement.

## What is Trajectory Optimization?

Trajectory optimization finds the best path for a dynamic system to follow, considering:
- **Physics**: System dynamics and constraints
- **Objectives**: Minimize time, fuel, energy, or tracking error
- **Constraints**: Safety limits, obstacle avoidance, boundary conditions

**Mathematical Foundation**: MAPTOR applies optimal control theory to transform continuous trajectory optimization problems into solvable nonlinear programming problems through spectral collocation methods.

## Core Methodology

MAPTOR implements the **Legendre-Gauss-Radau pseudospectral method** with:

- **Spectral accuracy**: Exponential convergence for smooth solutions
- **Adaptive mesh refinement**: Automatic error control through phs-adaptive mesh refinement method
- **Multiphase capability**: Complex missions with automatic phase linking
- **Symbolic computation**: Built on CasADi for exact differentiation and optimization

## Quick Start

```python
import maptor as mtor

# Minimum-time trajectory: reach target with bounded control
problem = mtor.Problem("Minimum Time to Target")
phase = problem.set_phase(1)

# Variables
t = phase.time(initial=0.0)  # Free final time
position = phase.state("position", initial=0.0, final=1.0)
velocity = phase.state("velocity", initial=0.0, final=0.0)
force = phase.control("force", boundary=(-2.0, 2.0))

# Dynamics and objective
phase.dynamics({position: velocity, velocity: force})
problem.minimize(t.final)

# Solve
phase.mesh([8], [-1.0, 1.0])
solution = mtor.solve_adaptive(problem)

if solution.status["success"]:
    print(f"Optimal time: {solution.status['objective']:.3f}")
    solution.plot()
```

## Problem Classes

**Beyond Spatial Trajectories**: MAPTOR also handles abstract optimal control problems where "trajectory" refers to the evolution of any system state over time (chemical processes, financial optimization, resource allocation).

## Installation

```bash
pip install maptor
```

**Requirements**: Python 3.10+, NumPy, SciPy, CasADi, Matplotlib

**Development Installation**:
```bash
git clone https://github.com/maptor/maptor.git
cd maptor
pip install -e .
```

## Documentation

| Resource | Description |
|----------|-------------|
| **[Installation Guide](https://maptor.github.io/maptor/installation.html)** | Setup and dependencies |
| **[Quick Start](https://maptor.github.io/maptor/quickstart.html)** | Basic workflow and first example |
| **[Problem Definition Tutorial](https://maptor.github.io/maptor/tutorials/problem_definition.html)** | Comprehensive problem construction guide |
| **[Solution Analysis Tutorial](https://maptor.github.io/maptor/tutorials/solution_access.html)** | Working with optimization results |
| **[Examples Gallery](https://maptor.github.io/maptor/examples/index.html)** | Complete problems with mathematical formulations |
| **[API Reference](https://maptor.github.io/maptor/api/index.html)** | Detailed function documentation |

## Example Trajectories

The examples gallery demonstrates trajectory optimization across multiple domains:

### Advanced Aerospace Applications
- **[Low-Thrust Orbit Transfer](https://maptor.github.io/maptor/examples/low_thrust_orbit_transfer.html)**: Orbital mechanics with J2/J3/J4 gravitational perturbations and modified equinoctial elements
- **[Multiphase Vehicle Launch](https://maptor.github.io/maptor/examples/multiphase_vehicle_launch.html)**: Realistic rocket trajectory with stage separations and orbital insertion
- **[Shuttle Reentry](https://maptor.github.io/maptor/examples/shuttle_reentry.html)**: Atmospheric entry with heat rate constraints and crossrange maximization
- **[Aero-Assisted Plane Change](https://maptor.github.io/maptor/examples/aero_assisted_plane_change.html)**: Atmospheric maneuvers for orbital inclination change

### Advanced Robotics and Control
- **[Quadcopter Flight](https://maptor.github.io/maptor/examples/quadcopter.html)**: Quadcopter dynamics with obstacle avoidance
- **[Dynamic Obstacle Avoidance](https://maptor.github.io/maptor/examples/dynamic_obstacle_avoidance.html)**: Dynamic bicycle model  moving obstacles
- **[Overtaking Maneuver](https://maptor.github.io/maptor/examples/overtaking_maneuver.html)**: Complex street scenario with dual moving obstacles
- **[Container Crane Control](https://maptor.github.io/maptor/examples/container_crane.html)**: Advanced control with swing suppression

### Atmospheric and Environmental Systems
- **[Hang Glider Flight](https://maptor.github.io/maptor/examples/hang_glider.html)**: Thermal soaring optimization with atmospheric updrafts

### Interdisciplinary Applications
- **[Two-Strain Tuberculosis Model](https://maptor.github.io/maptor/examples/two_strain_tuberculosis_model.html)**: Epidemiological optimal control with drug resistance dynamics

### Benchmark and Classical Problems
- **[Hypersensitive Problem](https://maptor.github.io/maptor/examples/hypersensitive.html)**: Challenging optimal control benchmark with sensitive dynamics
- **[Brachistochrone](https://maptor.github.io/maptor/examples/brachistochrone.html)**: Classical fastest descent problem

## Architecture

MAPTOR provides a layered architecture separating trajectory design from numerical implementation:

```
User API (Problem, solve_adaptive, solve_fixed_mesh)
         ↓
Trajectory Definition (States, controls, dynamics, constraints)
         ↓
Mathematical Framework (Radau pseudospectral method)
         ↓
Symbolic Computation (CasADi expressions and differentiation)
         ↓
Optimization (IPOPT nonlinear programming solver)
```

**Key Design Principles**:
- **Intuitive API**: Define trajectories naturally without numerical details
- **Automatic differentiation**: CasADi handles complex derivative computations
- **Adaptive precision**: Mesh refinement ensures solution accuracy
- **Multiphase support**: Complex missions with automatic phase transitions

## Contributing

We currently do not accept code submissions, but we welcome **issues and feedback reports** from the trajectory optimization and optimal control community. Please use [GitHub Issues](https://github.com/maptor/maptor/issues) to:

- Report bugs or unexpected behavior
- Request new features or enhancements
- Ask questions about usage or implementation
- Suggest improvements to documentation or examples
- Share feedback on your experience with MAPTOR

Your input helps improve MAPTOR for the entire community.

## License

MAPTOR is licensed under the [GNU Lesser General Public License v3.0](LICENSE). This allows use in both open source and proprietary applications while ensuring improvements to the core library remain open.

## Citation

If you use MAPTOR in academic research, please cite:

```bibtex
@software{maptor2025,
  title={MAPTOR: Multiphase Adaptive Trajectory Optimizer},
  author={Timothy, David},
  year={2025},
  url={https://github.com/maptor/maptor},
  version={0.2.0}
}
```

## References

MAPTOR builds upon established methods in computational optimal control:

**Optimal Control Theory and Methods**:
- Betts, J. T. (2020). *Practical Methods for Optimal Control Using Nonlinear Programming, Third Edition*. Society for Industrial and Applied Mathematics. https://doi.org/10.1137/1.9781611976199

**Pseudospectral Methods**:
- Agamawi, Y. M., & Rao, A. V. (2020). CGPOPS: A C++ Software for Solving Multiple-Phase Optimal Control Problems Using Adaptive Gaussian Quadrature Collocation and Sparse Nonlinear Programming. *ACM Transactions on Mathematical Software*, 46(3), Article 25. https://doi.org/10.1145/3390463

**Adaptive Mesh Refinement**:
- Haman III, G. V., & Rao, A. V. (2024). Adaptive Mesh Refinement and Error Estimation Method for Optimal Control Using Direct Collocation. *arXiv preprint arXiv:2410.07488*. https://arxiv.org/abs/2410.07488

**Symbolic Computation Framework**:
- Andersson, J. A. E., Gillis, J., Horn, G., Rawlings, J. B., & Diehl, M. (2019). CasADi -- A software framework for nonlinear optimization and optimal control. *Mathematical Programming Computation*, 11(1), 1-36. https://doi.org/10.1007/s12532-018-0139-4

## Support

- **Documentation**: [https://maptor.github.io/maptor](https://maptor.github.io/maptor)
- **Issues**: [GitHub Issues](https://github.com/maptor/maptor/issues)

## Acknowledgments

MAPTOR implements methods from the computational optimal control literature, particularly pseudospectral collocation techniques and adaptive mesh refinement strategies. The framework leverages CasADi for symbolic computation and automatic differentiation.

---

**Next Steps**: Begin with the [Quick Start Guide](https://maptor.github.io/maptor/quickstart.html) or explore the [Examples Gallery](https://maptor.github.io/maptor/examples/index.html) to see MAPTOR applied to trajectory optimization problems in your domain.
