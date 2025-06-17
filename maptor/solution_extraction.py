import logging

import casadi as ca
import numpy as np

from .exceptions import DataIntegrityError, SolutionExtractionError
from .input_validation import _validate_array_numerical_integrity
from .mtor_types import FloatArray, OptimalControlSolution, PhaseID, ProblemProtocol
from .utils.coordinates import _tau_to_time


logger = logging.getLogger(__name__)


def _extract_multiphase_integral_values(
    casadi_solution_object: ca.OptiSol | None,
    opti_object: ca.Opti,
    phase_id: PhaseID,
    num_integrals: int,
) -> float | FloatArray | None:
    if (
        num_integrals == 0
        or not hasattr(opti_object, "multiphase_variables_reference")
        or opti_object.multiphase_variables_reference is None
        or casadi_solution_object is None
    ):
        return None

    try:
        variables = opti_object.multiphase_variables_reference
        if phase_id not in variables.phase_variables:
            return None

        phase_vars = variables.phase_variables[phase_id]
        if phase_vars.integral_variables is None:
            return None

        raw_value = casadi_solution_object.value(phase_vars.integral_variables)

        if isinstance(raw_value, ca.DM):
            np_array_value = np.asarray(raw_value.toarray())
            if num_integrals == 1:
                if np_array_value.size == 1:
                    result = float(np_array_value.item())
                    _validate_array_numerical_integrity(
                        np.array([result]),
                        f"Phase {phase_id} integral value",
                        "integral extraction",
                    )
                    return result
                else:
                    logger.warning(
                        f"Phase {phase_id}: For num_integrals=1, unexpected array shape {np_array_value.shape}"
                    )
                    if np_array_value.size > 0:
                        return float(np_array_value.flatten()[0])
                    else:
                        logger.warning(f"Phase {phase_id}: Empty integral value array")
                        return np.nan
            else:
                result_array = np_array_value.flatten().astype(np.float64)
                _validate_array_numerical_integrity(
                    result_array, f"Phase {phase_id} integral array", "integral extraction"
                )
                return result_array

        elif isinstance(raw_value, float | int):
            if num_integrals == 1:
                result = float(raw_value)
                _validate_array_numerical_integrity(
                    np.array([result]), f"Phase {phase_id} integral value", "integral extraction"
                )
                return result
            else:
                logger.warning(
                    f"Phase {phase_id}: Expected array for {num_integrals} integrals, got scalar {raw_value}"
                )
                return np.full(num_integrals, np.nan, dtype=np.float64)
        else:
            logger.warning(
                f"Phase {phase_id}: Unexpected CasADi value type: {type(raw_value)}, value: {raw_value}"
            )
            if num_integrals > 1:
                return np.full(num_integrals, np.nan, dtype=np.float64)
            elif num_integrals == 1:
                return np.nan
            return None

    except Exception as e:
        if isinstance(e, DataIntegrityError):
            raise
        logger.warning(f"Could not extract integral values for phase {phase_id}: {e}")
        if num_integrals > 1:
            return np.full(num_integrals, np.nan, dtype=np.float64)
        elif num_integrals == 1:
            return np.nan
        return None


def _phase_trajectory_extraction(
    phase_id: PhaseID,
    casadi_solution_object: ca.OptiSol,
    phase_vars,
    problem: ProblemProtocol,
    initial_time: float,
    terminal_time: float,
) -> tuple[
    dict[str, FloatArray], list[FloatArray], list[FloatArray], list[FloatArray], list[FloatArray]
]:
    phase_def = problem._phases[phase_id]
    num_states, num_controls = problem._get_phase_variable_counts(phase_id)
    num_mesh_intervals = len(phase_def.collocation_points_per_interval)
    global_mesh_nodes = phase_def.global_normalized_mesh_nodes

    state_trajectory_times: list[float] = []
    state_trajectory_values: list[list[float]] = [[] for _ in range(num_states)]
    control_trajectory_times: list[float] = []
    control_trajectory_values: list[list[float]] = [[] for _ in range(num_controls)]
    per_interval_states: list[FloatArray] = []
    per_interval_controls: list[FloatArray] = []

    for mesh_idx in range(num_mesh_intervals):
        collocation_points = phase_def.collocation_points_per_interval[mesh_idx]

        state_vars = phase_vars.state_matrices[mesh_idx]
        state_vals = casadi_solution_object.value(state_vars)

        if isinstance(state_vals, ca.DM | ca.MX):
            state_vals = np.array(state_vals.full())
        else:
            state_vals = np.array(state_vals)

        if num_states == 1 and state_vals.ndim == 1:
            state_vals = state_vals.reshape(1, -1)
        elif num_states > 1 and state_vals.ndim == 1:
            num_points = len(state_vals) // num_states
            if num_points * num_states == len(state_vals):
                state_vals = state_vals.reshape(num_states, num_points)

        _validate_array_numerical_integrity(
            state_vals, f"Phase {phase_id} state values for interval {mesh_idx}"
        )
        per_interval_states.append(state_vals)

        if num_controls > 0:
            control_vars = phase_vars.control_variables[mesh_idx]
            control_vals = casadi_solution_object.value(control_vars)

            if isinstance(control_vals, ca.DM | ca.MX):
                control_vals = np.array(control_vals.full())
            else:
                control_vals = np.array(control_vals)

            if num_controls == 1 and control_vals.ndim == 1:
                control_vals = control_vals.reshape(1, -1)
            elif num_controls > 1 and control_vals.ndim == 1:
                num_points = len(control_vals) // num_controls
                if num_points * num_controls == len(control_vals):
                    control_vals = control_vals.reshape(num_controls, num_points)

            _validate_array_numerical_integrity(
                control_vals, f"Phase {phase_id} control values for interval {mesh_idx}"
            )
            per_interval_controls.append(control_vals)

        from .radau import _compute_radau_collocation_components

        basis_components = _compute_radau_collocation_components(collocation_points)

        state_tau_nodes = basis_components.state_approximation_nodes
        control_tau_nodes = basis_components.collocation_nodes

        mesh_start = global_mesh_nodes[mesh_idx]
        mesh_end = global_mesh_nodes[mesh_idx + 1]

        state_physical_times_result = _tau_to_time(
            state_tau_nodes, mesh_start, mesh_end, initial_time, terminal_time
        )
        control_physical_times_result = _tau_to_time(
            control_tau_nodes, mesh_start, mesh_end, initial_time, terminal_time
        )

        if not isinstance(state_physical_times_result, np.ndarray):
            state_physical_times = np.asarray(state_physical_times_result, dtype=np.float64)
        else:
            state_physical_times = state_physical_times_result

        if not isinstance(control_physical_times_result, np.ndarray):
            control_physical_times = np.asarray(control_physical_times_result, dtype=np.float64)
        else:
            control_physical_times = control_physical_times_result

        for node_idx in range(len(state_physical_times)):
            physical_time = state_physical_times[node_idx]
            state_trajectory_times.append(physical_time)
            for var_idx in range(num_states):
                value = state_vals[var_idx, node_idx]
                _validate_array_numerical_integrity(
                    np.array([value]),
                    f"Phase {phase_id} state trajectory value at interval {mesh_idx}, node {node_idx}",
                )
                state_trajectory_values[var_idx].append(value)

        if num_controls > 0:
            for node_idx in range(len(control_physical_times)):
                physical_time = control_physical_times[node_idx]
                control_trajectory_times.append(physical_time)
                for var_idx in range(num_controls):
                    value = control_vals[var_idx, node_idx]
                    _validate_array_numerical_integrity(
                        np.array([value]),
                        f"Phase {phase_id} control trajectory value at interval {mesh_idx}, node {node_idx}",
                    )
                    control_trajectory_values[var_idx].append(value)

    trajectory_data: dict[str, FloatArray] = {
        "state_times": np.array(state_trajectory_times, dtype=np.float64),
        "control_times": np.array(control_trajectory_times, dtype=np.float64)
        if num_controls > 0
        else np.array([], dtype=np.float64),
    }

    state_values_arrays = [np.array(s_traj, dtype=np.float64) for s_traj in state_trajectory_values]
    control_values_arrays = (
        [np.array(c_traj, dtype=np.float64) for c_traj in control_trajectory_values]
        if num_controls > 0
        else []
    )

    return (
        trajectory_data,
        state_values_arrays,
        control_values_arrays,
        per_interval_states,
        per_interval_controls,
    )


def _extract_and_format_multiphase_solution(
    casadi_solution_object: ca.OptiSol | None,
    casadi_optimization_problem_object: ca.Opti,
    problem: ProblemProtocol,
) -> OptimalControlSolution:
    solution = OptimalControlSolution()
    solution.opti_object = casadi_optimization_problem_object

    if casadi_solution_object is None:
        solution.success = False
        solution.message = "Multiphase solver did not find a solution or was not run."
        return solution

    phase_ids = problem._get_phase_ids()
    total_states, total_controls, num_static_params = problem._get_total_variable_counts()

    if not hasattr(casadi_optimization_problem_object, "multiphase_variables_reference"):
        solution.success = False
        solution.message = "Missing multiphase variables reference in optimization object."
        return solution

    variables = casadi_optimization_problem_object.multiphase_variables_reference

    try:
        objective = float(
            casadi_solution_object.value(
                casadi_optimization_problem_object.multiphase_objective_expression_reference
            )
        )

        _validate_array_numerical_integrity(
            np.array([objective]), "multiphase objective value", "core solution extraction"
        )
        solution.objective = objective

    except Exception as e:
        if isinstance(e, DataIntegrityError):
            raise
        solution.success = False
        solution.message = f"Failed to extract core multiphase solution values: {e}"
        solution.raw_solution = casadi_solution_object
        raise SolutionExtractionError(
            f"failure in multiphase core value extraction: {e}",
            "Core solution processing error",
        ) from e

    if num_static_params > 0 and variables.static_parameters is not None:
        try:
            static_params_raw = casadi_solution_object.value(variables.static_parameters)
            if isinstance(static_params_raw, ca.DM):
                static_params_array = np.asarray(static_params_raw.toarray()).flatten()
            else:
                static_params_array = np.array(static_params_raw).flatten()

            _validate_array_numerical_integrity(
                static_params_array, "static parameters", "static parameter extraction"
            )
            solution.static_parameters = static_params_array.astype(np.float64)
        except Exception as e:
            if isinstance(e, DataIntegrityError):
                raise
            logger.warning(f"Failed to extract static parameters: {e}")
            solution.static_parameters = None

    for phase_id in phase_ids:
        if phase_id not in variables.phase_variables:
            continue

        phase_vars = variables.phase_variables[phase_id]
        phase_def = problem._phases[phase_id]
        num_states, num_controls = problem._get_phase_variable_counts(phase_id)
        num_integrals = phase_def.num_integrals

        try:
            initial_time = float(casadi_solution_object.value(phase_vars.initial_time))
            terminal_time = float(casadi_solution_object.value(phase_vars.terminal_time))

            _validate_array_numerical_integrity(
                np.array([initial_time, terminal_time]),
                f"Phase {phase_id} times",
                "phase time extraction",
            )

            solution.phase_initial_times[phase_id] = initial_time
            solution.phase_terminal_times[phase_id] = terminal_time

        except Exception as e:
            if isinstance(e, DataIntegrityError):
                raise
            logger.error(f"Failed to extract phase {phase_id} times: {e}")
            solution.phase_initial_times[phase_id] = float("nan")
            solution.phase_terminal_times[phase_id] = float("nan")
            continue

        integral_result = _extract_multiphase_integral_values(
            casadi_solution_object, casadi_optimization_problem_object, phase_id, num_integrals
        )
        if integral_result is not None:
            solution.phase_integrals[phase_id] = integral_result

        try:
            (
                trajectory_data,
                state_values_arrays,
                control_values_arrays,
                per_interval_states,
                per_interval_controls,
            ) = _phase_trajectory_extraction(
                phase_id,
                casadi_solution_object,
                phase_vars,
                problem,
                initial_time,
                terminal_time,
            )

            solution.phase_time_states[phase_id] = trajectory_data["state_times"]
            solution.phase_states[phase_id] = state_values_arrays

            solution.phase_time_controls[phase_id] = trajectory_data["control_times"]
            solution.phase_controls[phase_id] = control_values_arrays

            solution.phase_solved_state_trajectories_per_interval[phase_id] = per_interval_states
            if num_controls > 0:
                solution.phase_solved_control_trajectories_per_interval[phase_id] = (
                    per_interval_controls
                )

        except Exception as e:
            if isinstance(e, SolutionExtractionError | DataIntegrityError):
                raise
            raise SolutionExtractionError(
                f"Failed to extract phase {phase_id} trajectory data: {e}",
                "trajectory processing error",
            ) from e

        solution.phase_mesh_intervals[phase_id] = phase_def.collocation_points_per_interval.copy()
        solution.phase_mesh_nodes[phase_id] = phase_def.global_normalized_mesh_nodes.copy()

    solution.success = True
    solution.message = "Multiphase NLP solved successfully."
    solution.raw_solution = casadi_solution_object

    return solution
