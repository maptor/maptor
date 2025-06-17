import logging

import numpy as np

from maptor.adaptive.phs.numerical import (
    _map_global_normalized_tau_to_local_interval_tau,
)
from maptor.exceptions import ConfigurationError, DataIntegrityError, InterpolationError
from maptor.mtor_types import (
    FloatArray,
    OptimalControlSolution,
    PhaseID,
    ProblemProtocol,
)
from maptor.radau import (
    _compute_radau_collocation_components,
    _evaluate_lagrange_interpolation_at_points,
)
from maptor.utils.constants import COORDINATE_PRECISION


__all__ = ["_propagate_multiphase_solution_to_new_meshes"]

logger = logging.getLogger(__name__)


def _find_containing_interval_index(global_tau: float, mesh_points: FloatArray) -> int | None:
    if mesh_points.ndim != 1 or mesh_points.size < 2:
        return None

    tolerance = COORDINATE_PRECISION
    if global_tau < mesh_points[0] - tolerance or global_tau > mesh_points[-1] + tolerance:
        return (
            0
            if abs(global_tau - mesh_points[0]) < tolerance
            else (len(mesh_points) - 2 if abs(global_tau - mesh_points[-1]) < tolerance else None)
        )

    if abs(global_tau - mesh_points[-1]) < tolerance:
        return len(mesh_points) - 2

    return min(
        max(0, int(np.searchsorted(mesh_points, global_tau, side="right")) - 1),
        len(mesh_points) - 2,
    )


def _determine_interpolation_parameters(
    global_tau: float, prev_mesh_points: FloatArray
) -> tuple[int, float]:
    prev_interval_idx = _find_containing_interval_index(global_tau, prev_mesh_points)

    if prev_interval_idx is None:
        raise InterpolationError(
            f"Global tau {global_tau} outside mesh domain [{prev_mesh_points[0]}, {prev_mesh_points[-1]}]",
            "Mesh boundary mapping error",
        )

    prev_tau_start, prev_tau_end = (
        prev_mesh_points[prev_interval_idx],
        prev_mesh_points[prev_interval_idx + 1],
    )
    return prev_interval_idx, _map_global_normalized_tau_to_local_interval_tau(
        global_tau, prev_tau_start, prev_tau_end
    )


def _validate_interpolation_inputs(
    prev_trajectory_per_interval: list[FloatArray],
    prev_polynomial_degrees: list[int],
    target_polynomial_degrees: list[int],
    phase_id: PhaseID,
) -> None:
    if (
        len(prev_trajectory_per_interval) != len(prev_polynomial_degrees)
        or not prev_trajectory_per_interval
        or not target_polynomial_degrees
    ):
        raise InterpolationError(
            f"Phase {phase_id} trajectory/degree data inconsistency or empty",
            "Input data inconsistency",
        )


def _compute_target_interval_nodes(
    target_mesh_points: FloatArray, k: int, target_local_nodes: FloatArray
) -> FloatArray:
    target_tau_start, target_tau_end = target_mesh_points[k], target_mesh_points[k + 1]
    tau_range = target_tau_end - target_tau_start
    tau_offset = target_tau_start + target_tau_end
    return np.asarray((tau_range * target_local_nodes + tau_offset) / 2.0, dtype=np.float64)


def _get_target_basis_nodes(N_k_target: int, is_state_trajectory: bool) -> tuple[FloatArray, int]:
    target_basis = _compute_radau_collocation_components(N_k_target)

    if is_state_trajectory:
        target_local_nodes = target_basis.state_approximation_nodes
        num_target_nodes = N_k_target + 1
    else:
        target_local_nodes = target_basis.collocation_nodes
        num_target_nodes = N_k_target

    return target_local_nodes, num_target_nodes


def _interpolate_at_points(
    global_tau_points: FloatArray,
    prev_mesh_points: FloatArray,
    prev_trajectory_per_interval: list[FloatArray],
    prev_polynomial_degrees: list[int],
    num_variables: int,
    phase_id: PhaseID,
    is_state_trajectory: bool,
) -> FloatArray:
    num_points = len(global_tau_points)
    target_values = np.zeros((num_variables, num_points), dtype=np.float64)

    for j, global_tau in enumerate(global_tau_points):
        prev_interval_idx, prev_local_tau = _determine_interpolation_parameters(
            global_tau, prev_mesh_points
        )

        N_k_prev = prev_polynomial_degrees[prev_interval_idx]
        prev_basis = _compute_radau_collocation_components(N_k_prev)

        if is_state_trajectory:
            prev_nodes = prev_basis.state_approximation_nodes
            prev_weights = prev_basis.barycentric_weights_for_state_nodes
        else:
            prev_nodes = prev_basis.collocation_nodes
            prev_weights = prev_basis.barycentric_weights_for_collocation_nodes

        prev_values = prev_trajectory_per_interval[prev_interval_idx]

        interpolated_values = _evaluate_lagrange_interpolation_at_points(
            prev_nodes, prev_weights, prev_values, prev_local_tau
        )

        if interpolated_values.ndim > 1:
            interpolated_values = interpolated_values.flatten()

        if j == 0 and (
            np.any(np.isnan(interpolated_values)) or np.any(np.isinf(interpolated_values))
        ):
            raise DataIntegrityError(
                f"Numerical corruption in interpolation result for phase {phase_id}",
                "Interpolation result validation",
            )

        if len(interpolated_values) == num_variables:
            target_values[:, j] = interpolated_values
        elif num_variables != 0:
            raise InterpolationError(
                f"Phase {phase_id} dimension mismatch: interpolated {len(interpolated_values)} values, expected {num_variables}",
                "Interpolation output dimension error",
            )

    return target_values


def _process_single_target_interval(
    k: int,
    N_k_target: int,
    target_mesh_points: FloatArray,
    prev_mesh_points: FloatArray,
    prev_trajectory_per_interval: list[FloatArray],
    prev_polynomial_degrees: list[int],
    num_variables: int,
    phase_id: PhaseID,
    is_state_trajectory: bool,
) -> FloatArray:
    target_local_nodes, num_target_nodes = _get_target_basis_nodes(N_k_target, is_state_trajectory)
    global_tau_points = _compute_target_interval_nodes(target_mesh_points, k, target_local_nodes)

    return _interpolate_at_points(
        global_tau_points,
        prev_mesh_points,
        prev_trajectory_per_interval,
        prev_polynomial_degrees,
        num_variables,
        phase_id,
        is_state_trajectory,
    )


def _interpolate_phase_trajectory_to_new_mesh_streamlined(
    prev_trajectory_per_interval: list[FloatArray],
    prev_mesh_points: FloatArray,
    prev_polynomial_degrees: list[int],
    target_mesh_points: FloatArray,
    target_polynomial_degrees: list[int],
    num_variables: int,
    phase_id: PhaseID,
    is_state_trajectory: bool = True,
) -> list[FloatArray]:
    _validate_interpolation_inputs(
        prev_trajectory_per_interval, prev_polynomial_degrees, target_polynomial_degrees, phase_id
    )

    target_trajectories = []
    for k, N_k_target in enumerate(target_polynomial_degrees):
        target_traj_k = _process_single_target_interval(
            k,
            N_k_target,
            target_mesh_points,
            prev_mesh_points,
            prev_trajectory_per_interval,
            prev_polynomial_degrees,
            num_variables,
            phase_id,
            is_state_trajectory,
        )
        target_trajectories.append(target_traj_k)

    return target_trajectories


def _validate_propagation_preconditions(prev_solution: OptimalControlSolution) -> None:
    if not prev_solution.success:
        raise InterpolationError(
            "Cannot propagate from unsuccessful previous unified solution",
            "Invalid source solution for propagation",
        )


def _validate_target_configuration(
    phase_id: PhaseID,
    target_phase_polynomial_degrees: dict[PhaseID, list[int]],
    target_phase_mesh_points: dict[PhaseID, FloatArray],
) -> tuple[list[int], FloatArray]:
    if phase_id not in target_phase_polynomial_degrees or phase_id not in target_phase_mesh_points:
        raise ConfigurationError(
            f"Missing target mesh configuration for phase {phase_id}",
            "Target mesh configuration error",
        )

    target_degrees = target_phase_polynomial_degrees[phase_id]
    target_mesh = target_phase_mesh_points[phase_id]

    if len(target_degrees) != len(target_mesh) - 1:
        raise ConfigurationError(
            f"Phase {phase_id} target polynomial degrees count ({len(target_degrees)}) != target mesh intervals ({len(target_mesh) - 1})",
            "Target mesh configuration error",
        )

    return target_degrees, target_mesh


def _validate_previous_solution_data(
    prev_solution: OptimalControlSolution, phase_id: PhaseID
) -> tuple[list[FloatArray], list[FloatArray], FloatArray, list[int]]:
    required_keys = [
        "phase_solved_state_trajectories_per_interval",
        "phase_solved_control_trajectories_per_interval",
        "phase_mesh_nodes",
        "phase_mesh_intervals",
    ]
    if not all(phase_id in getattr(prev_solution, key) for key in required_keys):
        raise InterpolationError(
            f"Previous solution missing required data for phase {phase_id}",
            "Missing source data",
        )

    prev_states = prev_solution.phase_solved_state_trajectories_per_interval[phase_id]
    prev_controls = prev_solution.phase_solved_control_trajectories_per_interval[phase_id]
    prev_mesh = prev_solution.phase_mesh_nodes[phase_id]
    prev_degrees = prev_solution.phase_mesh_intervals[phase_id]

    if len(prev_states) != len(prev_degrees) or len(prev_controls) != len(prev_degrees):
        raise InterpolationError(
            f"Phase {phase_id} previous data inconsistency",
            "Previous solution data inconsistency",
        )

    return prev_states, prev_controls, prev_mesh, prev_degrees


def _interpolate_phase_data(
    prev_states: list[FloatArray],
    prev_controls: list[FloatArray],
    prev_mesh: FloatArray,
    prev_degrees: list[int],
    target_mesh: FloatArray,
    target_degrees: list[int],
    num_states: int,
    num_controls: int,
    phase_id: PhaseID,
) -> tuple[list[FloatArray], list[FloatArray]]:
    phase_states = _interpolate_phase_trajectory_to_new_mesh_streamlined(
        prev_states,
        prev_mesh,
        prev_degrees,
        target_mesh,
        target_degrees,
        num_states,
        phase_id,
        True,
    )

    phase_controls = _interpolate_phase_trajectory_to_new_mesh_streamlined(
        prev_controls,
        prev_mesh,
        prev_degrees,
        target_mesh,
        target_degrees,
        num_controls,
        phase_id,
        False,
    )

    return phase_states, phase_controls


def _apply_interpolated_guesses_to_phases(
    problem: ProblemProtocol,
    prev_solution: OptimalControlSolution,
    target_phase_polynomial_degrees: dict[PhaseID, list[int]],
    target_phase_mesh_points: dict[PhaseID, FloatArray],
) -> None:
    for phase_id in problem._get_phase_ids():
        target_degrees, target_mesh = _validate_target_configuration(
            phase_id, target_phase_polynomial_degrees, target_phase_mesh_points
        )

        prev_states, prev_controls, prev_mesh, prev_degrees = _validate_previous_solution_data(
            prev_solution, phase_id
        )

        num_states, num_controls = problem._get_phase_variable_counts(phase_id)

        phase_states, phase_controls = _interpolate_phase_data(
            prev_states,
            prev_controls,
            prev_mesh,
            prev_degrees,
            target_mesh,
            target_degrees,
            num_states,
            num_controls,
            phase_id,
        )

        # Apply directly to phase definition using existing validation
        phase_def = problem._phases[phase_id]
        from maptor.problem import initial_guess_problem

        initial_guess_problem._set_phase_initial_guess(
            phase_def,
            states=phase_states,
            controls=phase_controls,
            initial_time=prev_solution.phase_initial_times.get(phase_id),
            terminal_time=prev_solution.phase_terminal_times.get(phase_id),
            integrals=prev_solution.phase_integrals.get(phase_id),
        )


def _propagate_multiphase_solution_to_new_meshes(
    prev_solution: OptimalControlSolution,
    problem: ProblemProtocol,
    target_phase_polynomial_degrees: dict[PhaseID, list[int]],
    target_phase_mesh_points: dict[PhaseID, FloatArray],
) -> None:
    """Propagate previous solution to new mesh configurations using phase-level guesses.

    Directly applies interpolated guesses to phase definitions instead of returning
    a legacy MultiPhaseInitialGuess object.
    """
    _validate_propagation_preconditions(prev_solution)

    _apply_interpolated_guesses_to_phases(
        problem, prev_solution, target_phase_polynomial_degrees, target_phase_mesh_points
    )
