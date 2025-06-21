import logging
from collections.abc import Callable
from typing import cast

import numpy as np

from maptor.mtor_types import (
    FloatArray,
    ODESolverCallable,
    OptimalControlSolution,
    PhaseID,
    ProblemProtocol,
)
from maptor.utils.constants import COORDINATE_PRECISION


__all__ = [
    "_calculate_gamma_normalizers_for_phase",
    "_calculate_relative_error_estimate",
    "_simulate_dynamics_for_phase_interval_error_estimation",
]

logger = logging.getLogger(__name__)


def _calculate_gamma_normalization_factors(max_state_values: FloatArray) -> FloatArray:
    gamma_denominator = 1.0 + max_state_values
    return (1.0 / np.maximum(gamma_denominator, np.float64(COORDINATE_PRECISION))).reshape(-1, 1)


def _find_maximum_state_values_across_phase_intervals(
    Y_solved_list: list[FloatArray],
) -> FloatArray:
    if not Y_solved_list:
        return np.array([], dtype=np.float64)

    valid_intervals = [Xk for Xk in Y_solved_list if Xk.size > 0]
    if not valid_intervals:
        return np.array([], dtype=np.float64)

    return cast(FloatArray, np.max(np.abs(np.concatenate(valid_intervals, axis=1)), axis=1))


def _calculate_trajectory_error_differences(
    sim_trajectory: FloatArray, nlp_trajectory: FloatArray, gamma_factors: FloatArray
) -> tuple[FloatArray, FloatArray]:
    abs_diff = np.abs(sim_trajectory - nlp_trajectory)
    scaled_errors = gamma_factors * abs_diff
    max_errors_per_state = (
        np.nanmax(scaled_errors, axis=1)
        if scaled_errors.size > 0
        else np.zeros(gamma_factors.shape[0], dtype=np.float64)
    )
    return abs_diff, max_errors_per_state


def _validate_error_inputs(
    max_fwd_errors_per_state: FloatArray, max_bwd_errors_per_state: FloatArray
) -> None:
    if max_fwd_errors_per_state.shape != max_bwd_errors_per_state.shape:
        raise ValueError("Input arrays must have the same shape.")


def _compute_combined_errors(
    max_fwd_errors_per_state: FloatArray, max_bwd_errors_per_state: FloatArray
) -> FloatArray:
    fwd_valid = ~np.isnan(max_fwd_errors_per_state)
    bwd_valid = ~np.isnan(max_bwd_errors_per_state)

    return np.where(
        fwd_valid & bwd_valid,
        np.maximum(max_fwd_errors_per_state, max_bwd_errors_per_state),
        np.where(
            fwd_valid,
            max_fwd_errors_per_state,
            np.where(bwd_valid, max_bwd_errors_per_state, np.nan),
        ),
    )


def _apply_error_bounds(max_error: float) -> float:
    if np.isnan(max_error):
        return np.inf
    return max(max_error, 1e-16) if 0.0 < max_error < 1e-16 else float(max_error)


def _calculate_combined_error_estimate(
    max_fwd_errors_per_state: FloatArray, max_bwd_errors_per_state: FloatArray
) -> float:
    _validate_error_inputs(max_fwd_errors_per_state, max_bwd_errors_per_state)

    combined_errors = _compute_combined_errors(max_fwd_errors_per_state, max_bwd_errors_per_state)
    max_error = float(np.nanmax(combined_errors)) if combined_errors.size > 0 else 0.0

    return _apply_error_bounds(max_error)


def _validate_simulation_preconditions(
    solution: OptimalControlSolution, phase_id: PhaseID, interval_idx: int
) -> tuple[bool, str]:
    if not solution.success or solution.raw_solution is None:
        return False, "Solution not successful or missing raw solution"

    required_attrs = ["phase_initial_times", "phase_terminal_times", "phase_mesh_nodes"]
    for attr in required_attrs:
        if phase_id not in getattr(solution, attr):
            return False, f"Missing {attr} for phase {phase_id}"

    global_mesh = solution.phase_mesh_nodes[phase_id]
    if interval_idx + 1 >= len(global_mesh):
        return False, f"Invalid interval index {interval_idx}"

    return True, ""


def _extract_time_parameters(
    solution: OptimalControlSolution, phase_id: PhaseID
) -> tuple[float, float, float]:
    t0 = solution.phase_initial_times[phase_id]
    tf = solution.phase_terminal_times[phase_id]
    alpha = (tf - t0) / 2.0
    alpha_0 = (tf + t0) / 2.0
    return alpha, alpha_0, t0


def _extract_interval_parameters(
    solution: OptimalControlSolution, phase_id: PhaseID, interval_idx: int
) -> tuple[float, float, float, float]:
    global_mesh = solution.phase_mesh_nodes[phase_id]
    tau_start, tau_end = global_mesh[interval_idx], global_mesh[interval_idx + 1]
    beta_k = (tau_end - tau_start) / 2.0

    if abs(beta_k) < COORDINATE_PRECISION:
        raise ValueError(f"Interval {interval_idx} has zero width")

    beta_k0 = (tau_end + tau_start) / 2.0
    return tau_start, tau_end, beta_k, beta_k0


def _create_numerical_dynamics_rhs(
    numerical_dynamics_function: Callable[
        [FloatArray, FloatArray, float, FloatArray | None], FloatArray
    ],
    control_evaluator: Callable,
    alpha: float,
    alpha_0: float,
    beta_k: float,
    beta_k0: float,
    overall_scaling: float,
    static_parameters: FloatArray | None,
) -> Callable[[float, FloatArray], FloatArray]:
    def dynamics_rhs(tau: float, state: FloatArray) -> FloatArray:
        control = (
            control_evaluator(tau).flatten()
            if control_evaluator(tau).ndim > 1
            else control_evaluator(tau)
        )
        global_tau = beta_k * tau + beta_k0
        physical_time = alpha * global_tau + alpha_0

        state_deriv_np = numerical_dynamics_function(
            state, control, physical_time, static_parameters
        )
        return overall_scaling * state_deriv_np

    return dynamics_rhs


def _extract_boundary_states(
    state_evaluator: Callable[[float | FloatArray], FloatArray],
) -> tuple[FloatArray, FloatArray]:
    initial_state = state_evaluator(-1.0)
    terminal_state = state_evaluator(1.0)

    initial_state = initial_state.flatten() if initial_state.ndim > 1 else initial_state
    terminal_state = terminal_state.flatten() if terminal_state.ndim > 1 else terminal_state

    return initial_state, terminal_state


def _run_simulation(
    ode_solver: ODESolverCallable,
    dynamics_rhs: Callable,
    t_span: tuple[float, float],
    y0: FloatArray,
    t_eval: FloatArray,
    num_states: int,
) -> tuple[bool, FloatArray]:
    try:
        sim_result = ode_solver(dynamics_rhs, t_span=t_span, y0=y0, t_eval=t_eval)
        if sim_result.success:
            return True, sim_result.y
        else:
            return False, np.full((num_states, len(t_eval)), np.nan, dtype=np.float64)
    except (RuntimeError, OverflowError, FloatingPointError):
        return False, np.full((num_states, len(t_eval)), np.nan, dtype=np.float64)


def _simulate_dynamics_for_phase_interval_error_estimation(
    phase_id: PhaseID,
    interval_idx: int,
    solution: OptimalControlSolution,
    problem: ProblemProtocol,
    state_evaluator: Callable[[float | FloatArray], FloatArray],
    control_evaluator: Callable[[float | FloatArray], FloatArray],
    ode_solver: ODESolverCallable,
    n_eval_points: int = 50,
    numerical_dynamics_function: Callable[
        [FloatArray, FloatArray, float, FloatArray | None], FloatArray
    ]
    | None = None,
) -> tuple[bool, FloatArray, FloatArray, FloatArray, FloatArray, FloatArray, FloatArray]:
    valid, _ = _validate_simulation_preconditions(solution, phase_id, interval_idx)
    if not valid:
        empty = np.array([], dtype=np.float64)
        return False, empty, empty, empty, empty, empty, empty

    num_states, _ = problem._get_phase_variable_counts(phase_id)

    if numerical_dynamics_function is None:
        numerical_dynamics_function = problem._get_phase_numerical_dynamics_function(phase_id)

    assert numerical_dynamics_function is not None, (
        f"Failed to get numerical dynamics function for phase {phase_id}"
    )
    alpha, alpha_0, _ = _extract_time_parameters(solution, phase_id)

    try:
        tau_start, tau_end, beta_k, beta_k0 = _extract_interval_parameters(
            solution, phase_id, interval_idx
        )
    except ValueError:
        empty = np.array([], dtype=np.float64)
        return False, empty, empty, empty, empty, empty, empty

    overall_scaling = alpha * beta_k

    static_parameters = None
    if hasattr(solution, "static_parameters") and solution.static_parameters is not None:
        static_parameters = np.asarray(solution.static_parameters, dtype=np.float64)

    dynamics_rhs = _create_numerical_dynamics_rhs(
        numerical_dynamics_function,
        control_evaluator,
        alpha,
        alpha_0,
        beta_k,
        beta_k0,
        overall_scaling,
        static_parameters,
    )

    initial_state, terminal_state = _extract_boundary_states(state_evaluator)

    fwd_tau_points = np.linspace(-1, 1, n_eval_points, dtype=np.float64)
    bwd_tau_points = np.linspace(1, -1, n_eval_points, dtype=np.float64)

    fwd_success, fwd_trajectory = _run_simulation(
        ode_solver, dynamics_rhs, (-1, 1), initial_state, fwd_tau_points, num_states
    )

    bwd_success, bwd_trajectory_raw = _run_simulation(
        ode_solver, dynamics_rhs, (1, -1), terminal_state, bwd_tau_points, num_states
    )

    bwd_trajectory = (
        np.fliplr(bwd_trajectory_raw)
        if bwd_success
        else np.full((num_states, len(bwd_tau_points)), np.nan, dtype=np.float64)
    )

    return (
        fwd_success and bwd_success,
        fwd_tau_points,
        fwd_trajectory,
        state_evaluator(fwd_tau_points),
        np.flip(bwd_tau_points),
        bwd_trajectory,
        state_evaluator(np.flip(bwd_tau_points)),
    )


def _calculate_relative_error_estimate(
    phase_id: PhaseID,
    interval_idx: int,
    success: bool,
    fwd_sim_traj: FloatArray,
    fwd_nlp_traj: FloatArray,
    bwd_sim_traj: FloatArray,
    bwd_nlp_traj: FloatArray,
    gamma_factors: FloatArray,
) -> float:
    if not success or fwd_sim_traj.size == 0 or bwd_sim_traj.size == 0:
        return np.inf

    if fwd_sim_traj.shape[0] == 0:
        return 0.0

    _, max_fwd_errors = _calculate_trajectory_error_differences(
        fwd_sim_traj, fwd_nlp_traj, gamma_factors
    )
    _, max_bwd_errors = _calculate_trajectory_error_differences(
        bwd_sim_traj, bwd_nlp_traj, gamma_factors
    )

    max_error = _calculate_combined_error_estimate(max_fwd_errors, max_bwd_errors)
    return np.inf if np.isnan(max_error) else max_error


def _calculate_gamma_normalizers_for_phase(
    solution: OptimalControlSolution, problem: ProblemProtocol, phase_id: PhaseID
) -> FloatArray | None:
    if not solution.success or solution.raw_solution is None:
        return None

    num_states, _ = problem._get_phase_variable_counts(phase_id)
    if num_states == 0:
        return np.array([], dtype=np.float64).reshape(0, 1)

    if phase_id not in solution.phase_solved_state_trajectories_per_interval:
        return None

    Y_solved_list = solution.phase_solved_state_trajectories_per_interval[phase_id]
    if not Y_solved_list:
        return None

    max_abs_values = _find_maximum_state_values_across_phase_intervals(Y_solved_list)
    return _calculate_gamma_normalization_factors(max_abs_values)
