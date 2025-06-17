from collections.abc import Callable
from dataclasses import dataclass

import casadi as ca

from ..mtor_types import PhaseID, ProblemProtocol
from ..utils.constants import LARGE_VALUE, TIME_PRECISION
from .types_solver import (
    _MultiPhaseVariable,
    _PhaseIntervalBundle,
    _PhaseVariable,
)


@dataclass
class _BoundConstraint:
    # Unified bound constraint representation

    lower: float
    upper: float
    is_fixed: bool

    @classmethod
    def from_bounds(cls, bounds: tuple[float, float]) -> "_BoundConstraint":
        """Create constraint from bounds tuple."""
        lower, upper = bounds
        return cls(lower=lower, upper=upper, is_fixed=(lower == upper))


@dataclass
class _VariableCreationContext:
    """context for variable creation operations."""

    opti: ca.Opti
    problem: ProblemProtocol
    phase_id: PhaseID
    num_states: int
    num_controls: int
    num_mesh_intervals: int
    num_integrals: int


@dataclass
class _VariableCreator:
    # Unified variable creation strategy.

    creator: Callable[[_VariableCreationContext], ca.MX | list[ca.MX] | None]
    constraint_applier: Callable[[ca.Opti, list[ca.MX], _VariableCreationContext], None] | None = (
        None
    )


def _apply_bound_constraints(opti: ca.Opti, variable: ca.MX, constraint: _BoundConstraint) -> None:
    if constraint.is_fixed:
        opti.subject_to(variable == constraint.lower)
    else:
        if constraint.lower > -LARGE_VALUE:
            opti.subject_to(variable >= constraint.lower)
        if constraint.upper < LARGE_VALUE:
            opti.subject_to(variable <= constraint.upper)


def _apply_time_constraints(
    opti: ca.Opti, time_variables: list[ca.MX], context: _VariableCreationContext
) -> None:
    initial_time_var, terminal_time_var = time_variables
    phase_def = context.problem._phases[context.phase_id]

    # Apply bound constraints
    t0_constraint = _BoundConstraint.from_bounds(phase_def.t0_bounds)
    tf_constraint = _BoundConstraint.from_bounds(phase_def.tf_bounds)

    _apply_bound_constraints(opti, initial_time_var, t0_constraint)
    _apply_bound_constraints(opti, terminal_time_var, tf_constraint)

    # Minimum interval prevents singular coordinate transformations
    opti.subject_to(terminal_time_var > initial_time_var + TIME_PRECISION)


def _create_time_variables(context: _VariableCreationContext) -> list[ca.MX]:
    initial_time_variable = context.opti.variable()
    terminal_time_variable = context.opti.variable()
    return [initial_time_variable, terminal_time_variable]


def _create_state_variables(context: _VariableCreationContext) -> list[ca.MX]:
    state_variables = []
    for _ in range(context.num_mesh_intervals + 1):
        state_var = context.opti.variable(context.num_states)
        state_variables.append(state_var)
    return state_variables


def _create_control_variables(context: _VariableCreationContext) -> list[ca.MX]:
    phase_def = context.problem._phases[context.phase_id]
    control_variables = []

    for k in range(context.num_mesh_intervals):
        num_colloc_points = phase_def.collocation_points_per_interval[k]
        control_var = context.opti.variable(context.num_controls, num_colloc_points)
        control_variables.append(control_var)

    return control_variables


def _create_integral_variables(context: _VariableCreationContext) -> ca.MX | None:
    if context.num_integrals > 0:
        return (
            context.opti.variable(context.num_integrals)
            if context.num_integrals > 1
            else context.opti.variable()
        )
    return None


def _create_static_parameter_variables(context: _VariableCreationContext) -> ca.MX:
    _, _, num_static_params = context.problem._get_total_variable_counts()
    return (
        context.opti.variable(num_static_params)
        if num_static_params > 1
        else context.opti.variable()
    )


def _create_variable_creators() -> dict[str, _VariableCreator]:
    return {
        "time": _VariableCreator(
            creator=_create_time_variables,
            constraint_applier=_apply_time_constraints,
        ),
        "states": _VariableCreator(
            creator=_create_state_variables,
            constraint_applier=None,
        ),
        "controls": _VariableCreator(
            creator=_create_control_variables,
            constraint_applier=None,
        ),
        "integrals": _VariableCreator(
            creator=_create_integral_variables,
            constraint_applier=None,
        ),
    }


def _create_phase_variables_unified(
    context: _VariableCreationContext,
) -> dict[str, ca.MX | list[ca.MX] | None]:
    creators = _create_variable_creators()
    variables: dict[str, ca.MX | list[ca.MX] | None] = {}

    for var_type, creator_obj in creators.items():
        # Create variables
        created_vars = creator_obj.creator(context)
        variables[var_type] = created_vars

        # Apply constraints if needed - only for time variables with list type
        if creator_obj.constraint_applier is not None and isinstance(created_vars, list):
            creator_obj.constraint_applier(context.opti, created_vars, context)

    return variables


def _create_variable_context(
    opti: ca.Opti, problem: ProblemProtocol, phase_id: PhaseID
) -> _VariableCreationContext:
    num_states, num_controls = problem._get_phase_variable_counts(phase_id)
    phase_def = problem._phases[phase_id]
    num_mesh_intervals = len(phase_def.collocation_points_per_interval)
    num_integrals = phase_def.num_integrals

    return _VariableCreationContext(
        opti=opti,
        problem=problem,
        phase_id=phase_id,
        num_states=num_states,
        num_controls=num_controls,
        num_mesh_intervals=num_mesh_intervals,
        num_integrals=num_integrals,
    )


def _setup_phase_optimization_variables(
    opti: ca.Opti,
    problem: ProblemProtocol,
    phase_id: PhaseID,
) -> _PhaseVariable:
    context = _create_variable_context(opti, problem, phase_id)
    variables = _create_phase_variables_unified(context)

    # Extract variables with proper type validation
    time_vars = variables["time"]
    if not isinstance(time_vars, list) or len(time_vars) != 2:
        raise ValueError(f"Expected time variables to be list of 2 MX, got {type(time_vars)}")
    initial_time, terminal_time = time_vars

    state_vars = variables["states"]
    if not isinstance(state_vars, list):
        raise ValueError(f"Expected state variables to be list of MX, got {type(state_vars)}")
    state_at_mesh_nodes = state_vars

    control_vars = variables["controls"]
    if not isinstance(control_vars, list):
        raise ValueError(f"Expected control variables to be list of MX, got {type(control_vars)}")
    control_variables = control_vars

    integral_vars = variables["integrals"]
    # integral_vars can be ca.MX or None
    integral_variables = integral_vars if isinstance(integral_vars, ca.MX) else None

    return _PhaseVariable(
        phase_id=phase_id,
        initial_time=initial_time,
        terminal_time=terminal_time,
        state_at_mesh_nodes=state_at_mesh_nodes,
        control_variables=control_variables,
        integral_variables=integral_variables,
    )


def _setup_multiphase_optimization_variables(
    opti: ca.Opti,
    problem: ProblemProtocol,
) -> _MultiPhaseVariable:
    multiphase_vars = _MultiPhaseVariable()

    # Process all phases with flattened loop
    for phase_id in problem._get_phase_ids():
        phase_vars = _setup_phase_optimization_variables(opti, problem, phase_id)
        multiphase_vars.phase_variables[phase_id] = phase_vars

    # Create static parameters if needed
    _, _, num_static_params = problem._get_total_variable_counts()
    if num_static_params > 0:
        static_context = _VariableCreationContext(
            opti=opti,
            problem=problem,
            phase_id=0,  # phase_id not used for static params
            num_states=0,
            num_controls=0,
            num_mesh_intervals=0,
            num_integrals=0,
        )
        multiphase_vars.static_parameters = _create_static_parameter_variables(static_context)

    return multiphase_vars


@dataclass
class _InteriorNodeContext:
    opti: ca.Opti
    phase_id: PhaseID
    num_states: int
    num_colloc_nodes: int
    state_at_global_mesh_nodes: list[ca.MX]
    mesh_interval_index: int


def _create_interior_variables(context: _InteriorNodeContext) -> ca.MX:
    num_interior_nodes = context.num_colloc_nodes - 1
    return context.opti.variable(context.num_states, num_interior_nodes)


def _populate_state_columns_with_interior(
    state_columns: list[ca.MX], interior_nodes_var: ca.MX
) -> None:
    num_interior_nodes = interior_nodes_var.shape[1]
    for i in range(num_interior_nodes):
        state_columns[i + 1] = interior_nodes_var[:, i]


def _setup_interior_nodes(context: _InteriorNodeContext) -> tuple[list[ca.MX], ca.MX | None]:
    # Early return for simple case
    if context.num_colloc_nodes <= 1:
        return [], None

    num_interior_nodes = context.num_colloc_nodes - 1
    if num_interior_nodes <= 0:
        return [], None

    # Create state columns structure
    state_columns = [ca.MX(context.num_states, 1) for _ in range(context.num_colloc_nodes + 1)]

    # Set boundary columns
    state_columns[0] = context.state_at_global_mesh_nodes[context.mesh_interval_index]
    state_columns[context.num_colloc_nodes] = context.state_at_global_mesh_nodes[
        context.mesh_interval_index + 1
    ]

    # Create and populate interior variables
    interior_nodes_var = _create_interior_variables(context)
    _populate_state_columns_with_interior(state_columns, interior_nodes_var)

    return state_columns, interior_nodes_var


def setup_phase_interval_state_variables(
    opti: ca.Opti,
    phase_id: PhaseID,
    mesh_interval_index: int,
    num_states: int,
    num_colloc_nodes: int,
    state_at_global_mesh_nodes: list[ca.MX],
) -> _PhaseIntervalBundle:
    context = _InteriorNodeContext(
        opti=opti,
        phase_id=phase_id,
        num_states=num_states,
        num_colloc_nodes=num_colloc_nodes,
        state_at_global_mesh_nodes=state_at_global_mesh_nodes,
        mesh_interval_index=mesh_interval_index,
    )

    state_columns, interior_nodes_var = _setup_interior_nodes(context)

    # Convert to matrix format
    if state_columns:
        state_matrix = ca.horzcat(*state_columns)
        state_matrix = ca.MX(state_matrix)
    else:
        # Handle simple case without interior nodes
        state_matrix = ca.horzcat(
            state_at_global_mesh_nodes[mesh_interval_index],
            state_at_global_mesh_nodes[mesh_interval_index + 1],
        )
        state_matrix = ca.MX(state_matrix)

    return state_matrix, interior_nodes_var
