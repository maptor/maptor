from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from typing import Any, Protocol, TypeAlias, TypedDict

import casadi as ca
import numpy as np
from numpy.typing import NDArray


# NUMERICAL SAFETY TYPES
FloatArray: TypeAlias = NDArray[np.float64]  # for numerical precision
NumericArrayLike: TypeAlias = (
    NDArray[np.floating[Any]]
    | NDArray[np.integer[Any]]
    | Sequence[float]
    | Sequence[int]
    | list[float]
    | list[int]
    | Sequence[Sequence[float]]
    | Sequence[Sequence[int]]
    | list[list[float]]
    | list[list[int]]
)

ConstraintInput: TypeAlias = float | int | tuple[float | int | None, float | int | None] | None
"""
Type alias for unified constraint specification.

Supported input types:
- float/int: Equality constraint (variable = value)
- tuple(lower, upper): Range constraint with None for unbounded sides
- None: No constraint specified
"""

PhaseID: TypeAlias = int
"""Phase identifier for multiphase problems."""


class ODESolverResult(Protocol):
    """Protocol for the result of ODE solvers like solve_ivp."""

    y: FloatArray
    t: FloatArray
    success: bool
    message: str


ODESolverCallable: TypeAlias = Callable[..., ODESolverResult]


class ProblemProtocol(Protocol):
    """Protocol defining the expected interface of a multiphase Problem object for solver."""

    # Essential multiphase properties
    _phases: dict[PhaseID, Any]
    _static_parameters: Any
    _multiphase_state: Any

    # Required attributes for solver interface
    solver_options: dict[str, object]

    # Essential solver methods
    def _get_phase_ids(self) -> list[PhaseID]:
        """Return ordered list of phase IDs"""
        ...

    def _get_phase_variable_counts(self, phase_id: PhaseID) -> tuple[int, int]:
        """Return (num_states, num_controls) for given phase"""
        ...

    def _get_total_variable_counts(self) -> tuple[int, int, int]:
        """Return (total_states, total_controls, num_static_params)"""
        ...

    def _get_phase_ordered_state_names(self, phase_id: PhaseID) -> list[str]:
        """Get state names for given phase in order"""
        ...

    def _get_phase_ordered_control_names(self, phase_id: PhaseID) -> list[str]:
        """Get control names for given phase in order"""
        ...

    def _get_phase_dynamics_function(self, phase_id: PhaseID) -> Callable[..., ca.MX]:
        """Get dynamics function for given phase (returns ca.MX directly)"""
        ...

    def _get_objective_function(self) -> Callable[..., ca.MX]:
        """Get multiphase objective function"""
        ...

    def _get_phase_integrand_function(self, phase_id: PhaseID) -> Callable[..., ca.MX] | None:
        """Get integrand function for given phase"""
        ...

    def _get_phase_path_constraints_function(
        self, phase_id: PhaseID
    ) -> Callable[..., list[Constraint]] | None:
        """Get path constraints function for given phase"""
        ...

    def _get_cross_phase_event_constraints_function(self) -> Callable[..., list[Constraint]] | None:
        """Get cross-phase event constraints function"""
        ...

    def validate_multiphase_configuration(self) -> None:
        """Validate the multiphase problem configuration"""
        ...

    def _get_phase_numerical_dynamics_function(self, phase_id: PhaseID) -> Any: ...


# UNIFIED CONSTRAINT SYSTEM
class Constraint:
    """Unified constraint class for optimal control problems."""

    def __init__(
        self,
        val: ca.MX | float,
        min_val: float | None = None,
        max_val: float | None = None,
        equals: float | None = None,
    ) -> None:
        self.val = val
        self.min_val = min_val
        self.max_val = max_val
        self.equals = equals

        # Constraint validation prevents conflicting specifications
        if equals is not None and (min_val is not None or max_val is not None):
            raise ValueError("Cannot specify equality constraint with bound constraints")
        if min_val is not None and max_val is not None and min_val > max_val:
            raise ValueError(f"min_val ({min_val}) must be <= max_val ({max_val})")

    def __repr__(self) -> str:
        if self.equals is not None:
            return f"Constraint(val == {self.equals})"

        bounds = []
        if self.min_val is not None:
            bounds.append(f"{self.min_val} <=")
        bounds.append("val")
        if self.max_val is not None:
            bounds.append(f"<= {self.max_val}")

        return f"Constraint({' '.join(bounds)})"


# ADAPTIVE ALGORITHM DATA
@dataclass
class AdaptiveAlgorithmData:
    """Data from adaptive mesh refinement algorithm with single source of truth."""

    target_tolerance: float
    total_iterations: int
    converged: bool
    phase_converged: dict[PhaseID, bool]
    final_phase_error_estimates: dict[PhaseID, list[float]]
    phase_gamma_factors: dict[PhaseID, FloatArray | None] = field(default_factory=dict)
    iteration_history: dict[int, IterationData] = field(default_factory=dict)


class OptimalControlSolution:
    """Solution to a multiphase optimal control problem.

    Contains optimized trajectories, objective value, solver diagnostics,
    and adaptive mesh refinement data for comprehensive solution analysis.
    """

    def __init__(self) -> None:
        self.success: bool = False
        self.message: str = "Solver not run yet."
        self.objective: float | None = None

        # Multiphase solution data
        self.phase_initial_times: dict[PhaseID, float] = {}
        self.phase_terminal_times: dict[PhaseID, float] = {}
        self.phase_time_states: dict[PhaseID, FloatArray] = {}
        self.phase_states: dict[PhaseID, list[FloatArray]] = {}
        self.phase_time_controls: dict[PhaseID, FloatArray] = {}
        self.phase_controls: dict[PhaseID, list[FloatArray]] = {}
        self.phase_integrals: dict[PhaseID, float | FloatArray] = {}
        self.static_parameters: FloatArray | None = None

        # Raw solver data for advanced analysis
        self.raw_solution: ca.OptiSol | None = None
        self.opti_object: ca.Opti | None = None

        # Mesh information per phase for solution interpretation
        self.phase_mesh_intervals: dict[PhaseID, list[int]] = {}
        self.phase_mesh_nodes: dict[PhaseID, FloatArray] = {}

        # Per-interval solution data per phase for adaptive refinement
        self.phase_solved_state_trajectories_per_interval: dict[PhaseID, list[FloatArray]] = {}
        self.phase_solved_control_trajectories_per_interval: dict[PhaseID, list[FloatArray]] = {}

        # Adaptive algorithm data for convergence analysis
        self.adaptive_data: AdaptiveAlgorithmData | None = None


@dataclass
class IterationData:
    """Per-iteration adaptive refinement metrics for research benchmarking.

    Captures exact algorithm state at each iteration for honest performance
    comparison with other pseudospectral methods
    """

    iteration: int
    phase_error_estimates: dict[PhaseID, list[float]]
    phase_collocation_points: dict[PhaseID, int]
    phase_mesh_intervals: dict[PhaseID, int]
    phase_polynomial_degrees: dict[PhaseID, list[int]]
    phase_mesh_nodes: dict[PhaseID, FloatArray]
    refinement_strategy: dict[PhaseID, dict[int, str]]
    total_collocation_points: int
    max_error_all_phases: float
    convergence_status: dict[PhaseID, bool]


class BenchmarkData(TypedDict):
    """Type definition for processed benchmark arrays."""

    mesh_iteration: list[int]
    estimated_error: list[float]
    collocation_points: list[int]
    mesh_intervals: list[int]
    polynomial_degrees: list[list[int]]
    refinement_strategy: list[dict[int, str]]
