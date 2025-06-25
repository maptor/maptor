"""
MAPTOR: Multiphase Adaptive Trajectory Optimizer

A Python framework for **trajectory optimization** using optimal control. MAPTOR solves problems involving the motion of vehicles, robots, spacecraft, and other dynamic systems through space and time using the Legendre-Gauss-Radau pseudospectral method and phs-adaptive mesh refinement.

Key Features:
    - Intuitive problem definition API
    - Adaptive mesh refinement for high-precision solutions
    - Multiphase trajectory support with automatic phase linking
    - Built-in plotting and solution analysis tools
    - Full type safety with comprehensive type hints

Quick Start:
    >>> import maptor as mtor
    >>> problem = mtor.Problem("Minimum Time Problem")
    >>> phase = problem.set_phase(1)
    >>> t = phase.time(initial=0.0)
    >>> x = phase.state("position", initial=0.0, final=1.0)
    >>> u = phase.control("force", boundary=(-1.0, 1.0))
    >>> phase.dynamics({x: u})
    >>> problem.minimize(t.final)
    >>> phase.mesh([8], [-1.0, 1.0])
    >>> solution = mtor.solve_adaptive(problem)
    >>> if solution.status["success"]:
    ...     solution.plot()

Documentation:
    https://maptor.github.io/maptor/

Repository:
    https://github.com/maptor/maptor

Logging:
    import logging
    logging.getLogger('maptor').setLevel(logging.INFO)  # Major operations
    logging.getLogger('maptor').setLevel(logging.DEBUG)  # Detailed debugging
"""

from __future__ import annotations

import logging

from maptor.exceptions import (
    ConfigurationError,
    DataIntegrityError,
    InterpolationError,
    MAPTORBaseError,
    SolutionExtractionError,
)
from maptor.problem import Problem
from maptor.solver import solve_adaptive, solve_fixed_mesh


__version__ = "0.2.0"
__author__ = "David Timothy"
__description__ = "Multiphase Adaptive Trajectory Optimizer"

__all__ = [
    "ConfigurationError",
    "DataIntegrityError",
    "InterpolationError",
    "MAPTORBaseError",
    "Problem",
    "SolutionExtractionError",
    "solve_adaptive",
    "solve_fixed_mesh",
]

logging.getLogger(__name__).addHandler(logging.NullHandler())


def _get_config() -> dict[str, str]:
    """Get MAPTOR configuration information."""
    return {
        "version": __version__,
        "author": __author__,
        "description": __description__,
        "repository": "https://github.com/maptor/maptor",
        "documentation": "https://maptor.github.io/maptor/",
        "license": "LGPL v3",
    }


def _show_config() -> None:
    """Print MAPTOR configuration (internal use)."""
    config = _get_config()
    print("MAPTOR Configuration:")
    for key, value in config.items():
        print(f"  {key}: {value}")
