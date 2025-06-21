import logging
from dataclasses import dataclass
from typing import Any

import numpy as np
from scipy.integrate import solve_ivp

from maptor.mtor_types import FloatArray, ODESolverCallable, OptimalControlSolution, PhaseID
from maptor.utils.constants import (
    DEFAULT_ERROR_SIM_POINTS,
    DEFAULT_ODE_ATOL_FACTOR,
    DEFAULT_ODE_MAX_STEP,
    DEFAULT_ODE_METHOD,
    DEFAULT_ODE_RTOL,
)


__all__ = [
    "AdaptiveParameters",
    "HRefineResult",
    "MultiphaseAdaptiveState",
    "PReduceResult",
    "PRefineResult",
    "_ensure_2d_array",
]

logger = logging.getLogger(__name__)


def _convert_casadi_to_numpy(casadi_value: Any) -> np.ndarray:
    if hasattr(casadi_value, "to_DM"):
        return np.array(casadi_value.to_DM(), dtype=np.float64)
    return np.array(casadi_value, dtype=np.float64)


def _reshape_1d_array(np_array: np.ndarray, expected_rows: int, expected_cols: int) -> np.ndarray:
    # CasADi returns inconsistent array shapes depending on problem size and solver backend

    array_length = len(np_array)
    expected_total = expected_rows * expected_cols

    if array_length == expected_total:
        return np_array.reshape(expected_rows, expected_cols)
    if array_length == expected_rows:
        return np_array.reshape(expected_rows, 1)
    return np_array.reshape(1, -1)


def _fix_array_orientation(np_array: np.ndarray, expected_rows: int) -> np.ndarray:
    if np_array.shape[0] != expected_rows and np_array.shape[1] == expected_rows:
        return np_array.T
    return np_array


def _create_configured_ode_solver(
    method: str,
    rtol: float,
    atol_factor: float,
    max_step: float | None,
) -> ODESolverCallable:
    def configured_solver(fun, t_span, y0, t_eval=None, **kwargs):
        kwargs["method"] = method
        kwargs["rtol"] = rtol
        kwargs["atol"] = rtol * atol_factor

        if max_step is not None:
            kwargs["max_step"] = max_step

        return solve_ivp(fun, t_span, y0, t_eval=t_eval, **kwargs)

    return configured_solver


@dataclass
class AdaptiveParameters:
    error_tolerance: float
    max_iterations: int
    min_polynomial_degree: int
    max_polynomial_degree: int
    ode_solver_tolerance: float = DEFAULT_ODE_RTOL
    num_error_sim_points: int = DEFAULT_ERROR_SIM_POINTS
    ode_method: str = DEFAULT_ODE_METHOD
    ode_max_step: float | None = DEFAULT_ODE_MAX_STEP
    ode_atol_factor: float = DEFAULT_ODE_ATOL_FACTOR
    ode_solver: ODESolverCallable | None = None

    def _get_ode_solver(self) -> ODESolverCallable:
        if self.ode_solver is not None:
            return self.ode_solver

        logger.debug("Using default ODE solver: scipy.integrate.solve_ivp with user configuration")

        return _create_configured_ode_solver(
            self.ode_method,
            self.ode_solver_tolerance,
            self.ode_atol_factor,
            self.ode_max_step,
        )


@dataclass
class MultiphaseAdaptiveState:
    """Tracks adaptive refinement state across all phases in unified NLP."""

    phase_polynomial_degrees: dict[PhaseID, list[int]]
    phase_mesh_points: dict[PhaseID, FloatArray]
    phase_converged: dict[PhaseID, bool]
    iteration: int = 0
    most_recent_unified_solution: OptimalControlSolution | None = None

    def _get_phase_ids(self) -> list[PhaseID]:
        return sorted(self.phase_polynomial_degrees.keys())

    def _configure_problem_meshes(self, problem: Any) -> None:
        for phase_id in self._get_phase_ids():
            if phase_id in problem._phases:
                phase_def = problem._phases[phase_id]
                phase_def.collocation_points_per_interval = self.phase_polynomial_degrees[phase_id]
                phase_def.global_normalized_mesh_nodes = self.phase_mesh_points[phase_id]


@dataclass
class PRefineResult:
    """Result of polynomial degree refinement."""

    actual_Nk_to_use: int
    was_p_successful: bool
    unconstrained_target_Nk: int


@dataclass
class HRefineResult:
    """Result of h-refinement."""

    collocation_nodes_for_new_subintervals: list[int]


@dataclass
class PReduceResult:
    """Result of p-reduction."""

    new_num_collocation_nodes: int


def _ensure_2d_array(casadi_value: Any, expected_rows: int, expected_cols: int) -> FloatArray:
    np_array = _convert_casadi_to_numpy(casadi_value)

    if expected_rows == 0:
        return np.empty((0, expected_cols), dtype=np.float64)

    if np_array.ndim == 1:
        np_array = _reshape_1d_array(np_array, expected_rows, expected_cols)

    np_array = _fix_array_orientation(np_array, expected_rows)

    return np_array
