from collections.abc import Callable

import casadi as ca

from maptor.mtor_types import Constraint, FloatArray, PhaseID, ProblemProtocol
from maptor.radau import RadauBasisComponents


def _apply_constraint(opti: ca.Opti, constraint: Constraint) -> None:
    if constraint.min_val is not None:
        opti.subject_to(constraint.val >= constraint.min_val)
    if constraint.max_val is not None:
        opti.subject_to(constraint.val <= constraint.max_val)
    if constraint.equals is not None:
        opti.subject_to(constraint.val == constraint.equals)


def _apply_phase_collocation_constraints(
    opti: ca.Opti,
    phase_id: PhaseID,
    mesh_interval_index: int,
    state_at_nodes: ca.MX,
    control_variables: ca.MX,
    basis_components: RadauBasisComponents,
    global_normalized_mesh_nodes: FloatArray,
    initial_time_variable: ca.MX,
    terminal_time_variable: ca.MX,
    dynamics_function: Callable[..., ca.MX],
    problem: ProblemProtocol | None = None,
    static_parameters_vec: ca.MX | None = None,
) -> None:
    from ..input_validation import _validate_dynamics_output
    from ..utils.coordinates import _tau_to_time

    num_colloc_nodes = len(basis_components.collocation_nodes)
    colloc_nodes_tau = basis_components.collocation_nodes.flatten()
    diff_matrix: ca.DM = ca.DM(basis_components.differentiation_matrix)

    state_derivative_at_colloc = ca.mtimes(state_at_nodes, diff_matrix.T)

    global_segment_length = (
        global_normalized_mesh_nodes[mesh_interval_index + 1]
        - global_normalized_mesh_nodes[mesh_interval_index]
    )
    tau_to_time_scaling = (
        (terminal_time_variable - initial_time_variable) * global_segment_length / 4.0
    )

    mesh_start = global_normalized_mesh_nodes[mesh_interval_index]
    mesh_end = global_normalized_mesh_nodes[mesh_interval_index + 1]

    for i_colloc in range(num_colloc_nodes):
        state_at_colloc = state_at_nodes[:, i_colloc]
        control_at_colloc = control_variables[:, i_colloc]

        local_colloc_tau_val = colloc_nodes_tau[i_colloc]
        physical_time_at_colloc = _tau_to_time(
            local_colloc_tau_val,
            mesh_start,
            mesh_end,
            initial_time_variable,
            terminal_time_variable,
        )

        state_derivative_rhs = dynamics_function(
            state_at_colloc, control_at_colloc, physical_time_at_colloc, static_parameters_vec
        )

        num_states = state_at_nodes.shape[0]
        state_derivative_rhs_vector = _validate_dynamics_output(state_derivative_rhs, num_states)

        opti.subject_to(
            state_derivative_at_colloc[:, i_colloc]
            == tau_to_time_scaling * state_derivative_rhs_vector
        )


def _apply_phase_path_constraints(
    opti: ca.Opti,
    phase_id: PhaseID,
    mesh_interval_index: int,
    state_at_nodes: ca.MX,
    control_variables: ca.MX,
    basis_components: RadauBasisComponents,
    global_normalized_mesh_nodes: FloatArray,
    initial_time_variable: ca.MX,
    terminal_time_variable: ca.MX,
    path_constraints_function: Callable[..., list[Constraint]],
    problem: ProblemProtocol | None = None,
    static_parameters_vec: ca.MX | None = None,
    static_parameter_symbols: list[ca.MX] | None = None,
) -> None:
    num_colloc_nodes = len(basis_components.collocation_nodes)
    colloc_nodes_tau = basis_components.collocation_nodes.flatten()
    global_segment_length = (
        global_normalized_mesh_nodes[mesh_interval_index + 1]
        - global_normalized_mesh_nodes[mesh_interval_index]
    )

    for i_colloc in range(num_colloc_nodes):
        state_at_colloc: ca.MX = state_at_nodes[:, i_colloc]
        control_at_colloc: ca.MX = control_variables[:, i_colloc]

        local_colloc_tau_val: float = colloc_nodes_tau[i_colloc]
        global_colloc_tau_val: ca.MX = (
            global_segment_length / 2 * local_colloc_tau_val
            + (
                global_normalized_mesh_nodes[mesh_interval_index + 1]
                + global_normalized_mesh_nodes[mesh_interval_index]
            )
            / 2
        )
        physical_time_at_colloc: ca.MX = (
            terminal_time_variable - initial_time_variable
        ) / 2 * global_colloc_tau_val + (terminal_time_variable + initial_time_variable) / 2

        path_constraints_result: list[Constraint] | Constraint = path_constraints_function(
            state_at_colloc,
            control_at_colloc,
            physical_time_at_colloc,
            static_parameters_vec,
            static_parameter_symbols,
            initial_time_variable,
            terminal_time_variable,
        )

        constraints_to_apply = (
            path_constraints_result
            if isinstance(path_constraints_result, list)
            else [path_constraints_result]
        )

        for constraint in constraints_to_apply:
            _apply_constraint(opti, constraint)


def _apply_multiphase_cross_phase_event_constraints(
    opti: ca.Opti,
    phase_endpoint_data: dict[PhaseID, dict[str, ca.MX]],
    static_parameters: ca.MX | None,
    problem: ProblemProtocol,
) -> None:
    cross_phase_constraints_function = problem._get_cross_phase_event_constraints_function()
    if cross_phase_constraints_function is None:
        return

    cross_phase_constraints_result: list[Constraint] | Constraint = (
        cross_phase_constraints_function(phase_endpoint_data, static_parameters)
    )

    constraints_to_apply = (
        cross_phase_constraints_result
        if isinstance(cross_phase_constraints_result, list)
        else [cross_phase_constraints_result]
    )

    for constraint in constraints_to_apply:
        _apply_constraint(opti, constraint)
