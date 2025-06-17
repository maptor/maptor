from collections.abc import Callable

import casadi as ca

from ..mtor_types import (
    FloatArray,
    PhaseID,
)
from ..radau import RadauBasisComponents
from ..utils.coordinates import _tau_to_time


def _setup_phase_integrals(
    opti: ca.Opti,
    phase_id: PhaseID,
    mesh_interval_index: int,
    state_at_nodes: ca.MX,
    control_variables: ca.MX,
    basis_components: RadauBasisComponents,
    global_normalized_mesh_nodes: FloatArray,
    initial_time_variable: ca.MX,
    terminal_time_variable: ca.MX,
    integral_integrand_function: Callable[..., ca.MX],
    num_integrals: int,
    accumulated_integral_expressions: list[ca.MX],
    static_parameters_vec: ca.MX | None = None,
) -> None:
    num_colloc_nodes = len(basis_components.collocation_nodes)
    colloc_nodes_tau = basis_components.collocation_nodes.flatten()
    quad_weights = basis_components.quadrature_weights.flatten()

    # Single scaling factor calculation
    global_segment_length = (
        global_normalized_mesh_nodes[mesh_interval_index + 1]
        - global_normalized_mesh_nodes[mesh_interval_index]
    )
    tau_to_time_scaling = (
        (terminal_time_variable - initial_time_variable) * global_segment_length / 4.0
    )

    mesh_start = global_normalized_mesh_nodes[mesh_interval_index]
    mesh_end = global_normalized_mesh_nodes[mesh_interval_index + 1]

    for integral_index in range(num_integrals):
        quad_sum: ca.MX = ca.MX(0)

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

            # Calculate integrand and add to quadrature sum
            weight = quad_weights[i_colloc]
            integrand_value = integral_integrand_function(
                state_at_colloc,
                control_at_colloc,
                physical_time_at_colloc,
                integral_index,
                static_parameters_vec,
            )
            quad_sum += weight * integrand_value

        accumulated_integral_expressions[integral_index] += tau_to_time_scaling * quad_sum


def _apply_phase_integral_constraints(
    opti: ca.Opti,
    integral_variables: ca.MX,
    accumulated_integral_expressions: list[ca.MX],
    num_integrals: int,
    phase_id: PhaseID,
) -> None:
    if num_integrals == 1:
        opti.subject_to(integral_variables == accumulated_integral_expressions[0])
    else:
        for i in range(num_integrals):
            opti.subject_to(integral_variables[i] == accumulated_integral_expressions[i])
