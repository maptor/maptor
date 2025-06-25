from __future__ import annotations

from collections.abc import Callable

import casadi as ca

from ..mtor_types import Constraint, PhaseID
from .state import (
    MultiPhaseVariableState,
    PhaseDefinition,
    _EndpointConstraint,
    _FixedConstraint,
    _RangeBoundaryConstraint,
)


def _add_path_constraint(phase_def: PhaseDefinition, constraint_expr: ca.MX | float | int) -> None:
    if isinstance(constraint_expr, ca.MX):
        phase_def.path_constraints.append(constraint_expr)
    else:
        phase_def.path_constraints.append(ca.MX(constraint_expr))


def _add_event_constraint(
    multiphase_state: MultiPhaseVariableState, constraint_expr: ca.MX | float | int
) -> None:
    if isinstance(constraint_expr, ca.MX):
        multiphase_state.cross_phase_constraints.append(constraint_expr)
    else:
        multiphase_state.cross_phase_constraints.append(ca.MX(constraint_expr))


def _symbolic_constraint_to_constraint(expr: ca.MX) -> Constraint:
    try:
        OP_EQ = getattr(ca, "OP_EQ", None)
        OP_LE = getattr(ca, "OP_LE", None)
        OP_GE = getattr(ca, "OP_GE", None)

        if (
            isinstance(expr, ca.MX)
            and hasattr(expr, "is_op")
            and OP_EQ is not None
            and isinstance(OP_EQ, int)
        ):
            if expr.is_op(OP_EQ):
                lhs = expr.dep(0)
                rhs = expr.dep(1)
                return Constraint(val=lhs - rhs, equals=0.0)

            elif OP_LE is not None and isinstance(OP_LE, int) and expr.is_op(OP_LE):
                lhs = expr.dep(0)
                rhs = expr.dep(1)
                return Constraint(val=lhs - rhs, max_val=0.0)

            elif OP_GE is not None and isinstance(OP_GE, int) and expr.is_op(OP_GE):
                lhs = expr.dep(0)
                rhs = expr.dep(1)
                return Constraint(val=lhs - rhs, min_val=0.0)

    except (AttributeError, TypeError, NotImplementedError):
        pass

    return Constraint(val=expr, equals=0.0)


def _boundary_constraint_to_constraints(
    boundary_constraint: _EndpointConstraint, variable_expression: ca.MX
) -> list[Constraint]:
    constraints: list[Constraint] = []

    if boundary_constraint.equals is not None:
        constraints.append(Constraint(val=variable_expression, equals=boundary_constraint.equals))
    else:
        if boundary_constraint.lower is not None:
            constraints.append(
                Constraint(val=variable_expression, min_val=boundary_constraint.lower)
            )
        if boundary_constraint.upper is not None:
            constraints.append(
                Constraint(val=variable_expression, max_val=boundary_constraint.upper)
            )

    return constraints


def _map_symbol_to_vector_element(
    symbol: ca.MX, vector: ca.MX, index: int, vector_length: int
) -> ca.MX:
    return vector if vector_length == 1 else vector[index]


def _process_state_boundary_constraints(
    state_boundary_constraints: list[_RangeBoundaryConstraint | None],  # Updated type
    states_vec: ca.MX,
    result: list[Constraint],
) -> None:
    for i, boundary_constraint in enumerate(state_boundary_constraints):
        if boundary_constraint is None or not boundary_constraint.has_constraint():
            continue

        variable_expr = _map_symbol_to_vector_element(
            states_vec, states_vec, i, len(state_boundary_constraints)
        )
        result.extend(_range_boundary_constraint_to_constraints(boundary_constraint, variable_expr))


def _process_control_boundary_constraints(
    control_boundary_constraints: list[_RangeBoundaryConstraint | None],
    controls_vec: ca.MX,
    result: list[Constraint],
) -> None:
    for i, boundary_constraint in enumerate(control_boundary_constraints):
        if boundary_constraint is None or not boundary_constraint.has_constraint():
            continue

        variable_expr = _map_symbol_to_vector_element(
            controls_vec, controls_vec, i, len(control_boundary_constraints)
        )
        result.extend(_range_boundary_constraint_to_constraints(boundary_constraint, variable_expr))


def _range_boundary_constraint_to_constraints(
    boundary_constraint: _RangeBoundaryConstraint, variable_expression: ca.MX
) -> list[Constraint]:
    """Convert range boundary constraint to optimization constraints."""
    constraints: list[Constraint] = []

    if boundary_constraint.lower is not None:
        constraints.append(Constraint(val=variable_expression, min_val=boundary_constraint.lower))
    if boundary_constraint.upper is not None:
        constraints.append(Constraint(val=variable_expression, max_val=boundary_constraint.upper))

    return constraints


def _fixed_constraint_to_constraints(
    fixed_constraint: _FixedConstraint, variable_expression: ca.MX
) -> list[Constraint]:
    """Convert fixed constraint to optimization constraints."""
    if fixed_constraint.equals is not None:
        return [Constraint(val=variable_expression, equals=fixed_constraint.equals)]
    return []


def _process_static_parameter_constraints(
    multiphase_state: MultiPhaseVariableState,
    static_parameters_vec: ca.MX | None,
    result: list[Constraint],
) -> None:
    if static_parameters_vec is None:
        return

    static_params = multiphase_state.static_parameters
    num_params = len(static_params.parameter_info)

    for i, param_info in enumerate(static_params.parameter_info):
        param_expr = _map_symbol_to_vector_element(
            static_parameters_vec, static_parameters_vec, i, num_params
        )

        # Handle range boundary constraints
        if (
            param_info.boundary_constraint is not None
            and param_info.boundary_constraint.has_constraint()
        ):
            result.extend(
                _range_boundary_constraint_to_constraints(
                    param_info.boundary_constraint, param_expr
                )
            )

        # Handle fixed constraints (non-symbolic ones)
        if (
            param_info.fixed_constraint is not None
            and param_info.fixed_constraint.has_constraint()
            and not param_info.fixed_constraint.is_symbolic()
        ):
            result.extend(_fixed_constraint_to_constraints(param_info.fixed_constraint, param_expr))


def _process_symbolic_path_constraints(
    path_constraints: list[ca.MX],
    subs_map: dict[ca.MX, ca.MX],
    result: list[Constraint],
) -> None:
    for expr in path_constraints:
        substituted_expr = ca.substitute([expr], list(subs_map.keys()), list(subs_map.values()))[0]
        result.append(_symbolic_constraint_to_constraint(substituted_expr))


def _map_phase_symbols_to_substitution(
    phase_def: PhaseDefinition,
    states_vec: ca.MX,
    controls_vec: ca.MX,
    time: ca.MX,
    initial_time_variable: ca.MX | None,
    terminal_time_variable: ca.MX | None,
) -> dict[ca.MX, ca.MX]:
    subs_map = {}

    state_syms = phase_def._get_ordered_state_symbols()
    for i, state_sym in enumerate(state_syms):
        subs_map[state_sym] = _map_symbol_to_vector_element(
            state_sym, states_vec, i, len(state_syms)
        )

    control_syms = phase_def._get_ordered_control_symbols()
    for i, control_sym in enumerate(control_syms):
        subs_map[control_sym] = _map_symbol_to_vector_element(
            control_sym, controls_vec, i, len(control_syms)
        )

    if phase_def.sym_time is not None:
        subs_map[phase_def.sym_time] = time

    if phase_def.sym_time_initial is not None and initial_time_variable is not None:
        subs_map[phase_def.sym_time_initial] = initial_time_variable

    if phase_def.sym_time_final is not None and terminal_time_variable is not None:
        subs_map[phase_def.sym_time_final] = terminal_time_variable

    return subs_map


def _map_static_parameters_to_substitution(
    static_parameters_vec: ca.MX | None,
    static_parameter_symbols: list[ca.MX] | None,
    subs_map: dict[ca.MX, ca.MX],
) -> None:
    if static_parameters_vec is not None and static_parameter_symbols is not None:
        num_params = len(static_parameter_symbols)
        for i, param_sym in enumerate(static_parameter_symbols):
            subs_map[param_sym] = _map_symbol_to_vector_element(
                param_sym, static_parameters_vec, i, num_params
            )


def _build_substitution_map(
    phase_def: PhaseDefinition,
    states_vec: ca.MX,
    controls_vec: ca.MX,
    time: ca.MX,
    static_parameters_vec: ca.MX | None,
    static_parameter_symbols: list[ca.MX] | None,
    initial_time_variable: ca.MX | None,
    terminal_time_variable: ca.MX | None,
) -> dict[ca.MX, ca.MX]:
    subs_map = _map_phase_symbols_to_substitution(
        phase_def, states_vec, controls_vec, time, initial_time_variable, terminal_time_variable
    )

    _map_static_parameters_to_substitution(
        static_parameters_vec, static_parameter_symbols, subs_map
    )

    return subs_map


def _check_has_path_constraints(phase_def: PhaseDefinition) -> bool:
    has_path_constraints = bool(phase_def.path_constraints)

    state_boundary_constraints = [info.boundary_constraint for info in phase_def.state_info]
    control_boundary_constraints = [info.boundary_constraint for info in phase_def.control_info]

    has_state_boundary = any(
        constraint is not None and constraint.has_constraint()
        for constraint in state_boundary_constraints
    )
    has_control_boundary = any(
        constraint is not None and constraint.has_constraint()
        for constraint in control_boundary_constraints
    )

    return has_path_constraints or has_state_boundary or has_control_boundary


def _create_path_constraints(
    phase_def: PhaseDefinition,
) -> Callable[..., list[Constraint]]:
    state_boundary_constraints = [info.boundary_constraint for info in phase_def.state_info]
    control_boundary_constraints = [info.boundary_constraint for info in phase_def.control_info]

    def _path_constraints(
        states_vec: ca.MX,
        controls_vec: ca.MX,
        time: ca.MX,
        static_parameters_vec: ca.MX | None = None,
        static_parameter_symbols: list[ca.MX] | None = None,
        initial_time_variable: ca.MX | None = None,
        terminal_time_variable: ca.MX | None = None,
    ) -> list[Constraint]:
        result: list[Constraint] = []

        subs_map = _build_substitution_map(
            phase_def,
            states_vec,
            controls_vec,
            time,
            static_parameters_vec,
            static_parameter_symbols,
            initial_time_variable,
            terminal_time_variable,
        )

        _process_symbolic_path_constraints(phase_def.path_constraints, subs_map, result)
        _process_state_boundary_constraints(state_boundary_constraints, states_vec, result)
        _process_control_boundary_constraints(control_boundary_constraints, controls_vec, result)

        return result

    return _path_constraints


def _get_phase_path_constraints_function(
    phase_def: PhaseDefinition,
) -> Callable[..., list[Constraint]] | None:
    if not _check_has_path_constraints(phase_def):
        return None

    return _create_path_constraints(phase_def)


def _map_phase_time_symbols(
    phase_def: PhaseDefinition, endpoint_data: dict[str, ca.MX], subs_map: dict[ca.MX, ca.MX]
) -> None:
    if phase_def.sym_time_initial is not None:
        subs_map[phase_def.sym_time_initial] = endpoint_data["t0"]
    if phase_def.sym_time_final is not None:
        subs_map[phase_def.sym_time_final] = endpoint_data["tf"]
    if phase_def.sym_time is not None:
        subs_map[phase_def.sym_time] = endpoint_data["tf"]


def _map_phase_state_symbols(
    phase_def: PhaseDefinition, endpoint_data: dict[str, ca.MX], subs_map: dict[ca.MX, ca.MX]
) -> None:
    """Map phase state symbols with guaranteed non-None symbols."""
    state_initial_syms = phase_def._get_ordered_state_initial_symbols()  # Now returns list[ca.MX]
    state_final_syms = phase_def._get_ordered_state_final_symbols()  # Now returns list[ca.MX]
    state_syms = phase_def._get_ordered_state_symbols()

    x0_vec = endpoint_data["x0"]
    xf_vec = endpoint_data["xf"]
    num_states = len(state_syms)

    for i, (sym_initial, sym_final, sym_current) in enumerate(
        zip(state_initial_syms, state_final_syms, state_syms, strict=True)
    ):
        subs_map[sym_initial] = _map_symbol_to_vector_element(sym_initial, x0_vec, i, num_states)
        subs_map[sym_final] = _map_symbol_to_vector_element(sym_final, xf_vec, i, num_states)
        subs_map[sym_current] = _map_symbol_to_vector_element(sym_current, xf_vec, i, num_states)


def _map_phase_integral_symbols(
    phase_def: PhaseDefinition, endpoint_data: dict[str, ca.MX], subs_map: dict[ca.MX, ca.MX]
) -> None:
    if "q" in endpoint_data and endpoint_data["q"] is not None:
        for i, integral_sym in enumerate(phase_def.integral_symbols):
            if i < endpoint_data["q"].shape[0]:
                subs_map[integral_sym] = endpoint_data["q"][i]


def _map_cross_phase_static_parameters(
    multiphase_state: MultiPhaseVariableState,
    static_parameters_vec: ca.MX | None,
    subs_map: dict[ca.MX, ca.MX],
) -> None:
    if static_parameters_vec is not None:
        static_param_syms = multiphase_state.static_parameters.get_ordered_parameter_symbols()
        num_params = len(static_param_syms)
        for i, param_sym in enumerate(static_param_syms):
            subs_map[param_sym] = _map_symbol_to_vector_element(
                param_sym, static_parameters_vec, i, num_params
            )


def _process_cross_phase_substitution_map(
    multiphase_state: MultiPhaseVariableState,
    phase_endpoint_vectors: dict[PhaseID, dict[str, ca.MX]],
    static_parameters_vec: ca.MX | None,
) -> dict[ca.MX, ca.MX]:
    """Process cross-phase substitution mapping with proper type annotation."""
    subs_map: dict[ca.MX, ca.MX] = {}

    for phase_id, phase_def in multiphase_state.phases.items():
        if phase_id not in phase_endpoint_vectors:
            continue

        endpoint_data = phase_endpoint_vectors[phase_id]

        _map_phase_time_symbols(phase_def, endpoint_data, subs_map)
        _map_phase_state_symbols(phase_def, endpoint_data, subs_map)
        _map_phase_integral_symbols(phase_def, endpoint_data, subs_map)

    _map_cross_phase_static_parameters(multiphase_state, static_parameters_vec, subs_map)

    return subs_map


def _process_cross_phase_symbolic_constraints(
    multiphase_state: MultiPhaseVariableState,
    subs_map: dict[ca.MX, ca.MX],
    result: list[Constraint],
) -> None:
    for expr in multiphase_state.cross_phase_constraints:
        substituted_expr = ca.substitute([expr], list(subs_map.keys()), list(subs_map.values()))[0]
        constraint = _symbolic_constraint_to_constraint(substituted_expr)
        result.append(constraint)


def _process_phase_endpoint_boundary_constraints(
    multiphase_state: MultiPhaseVariableState,
    phase_endpoint_vectors: dict[PhaseID, dict[str, ca.MX]],
    constraint_type: str,
    result: list[Constraint],
) -> None:
    for phase_id, phase_def in multiphase_state.phases.items():
        if phase_id not in phase_endpoint_vectors:
            continue

        endpoint_data = phase_endpoint_vectors[phase_id]

        if constraint_type == "initial":
            boundary_constraints = [info.initial_constraint for info in phase_def.state_info]
            endpoint_vec = endpoint_data["x0"]
        else:
            boundary_constraints = [info.final_constraint for info in phase_def.state_info]
            endpoint_vec = endpoint_data["xf"]

        num_states = len(phase_def.state_info)
        for i, boundary_constraint in enumerate(boundary_constraints):
            if (
                boundary_constraint is None
                or not boundary_constraint.has_constraint()
                or boundary_constraint.is_symbolic()
            ):
                continue

            constraint_expr = _map_symbol_to_vector_element(
                endpoint_vec, endpoint_vec, i, num_states
            )
            result.extend(_boundary_constraint_to_constraints(boundary_constraint, constraint_expr))


def _process_phase_initial_boundary_constraints(
    multiphase_state: MultiPhaseVariableState,
    phase_endpoint_vectors: dict[PhaseID, dict[str, ca.MX]],
    result: list[Constraint],
) -> None:
    _process_phase_endpoint_boundary_constraints(
        multiphase_state, phase_endpoint_vectors, "initial", result
    )


def _process_phase_final_boundary_constraints(
    multiphase_state: MultiPhaseVariableState,
    phase_endpoint_vectors: dict[PhaseID, dict[str, ca.MX]],
    result: list[Constraint],
) -> None:
    _process_phase_endpoint_boundary_constraints(
        multiphase_state, phase_endpoint_vectors, "final", result
    )


def _check_has_cross_phase_constraints(multiphase_state: MultiPhaseVariableState) -> bool:
    has_cross_phase_constraints = bool(multiphase_state.cross_phase_constraints)

    has_phase_event_constraints = False
    for phase_def in multiphase_state.phases.values():
        state_initial_constraints = [info.initial_constraint for info in phase_def.state_info]
        state_final_constraints = [info.final_constraint for info in phase_def.state_info]

        if any(
            constraint is not None and constraint.has_constraint() and not constraint.is_symbolic()
            for constraint in (state_initial_constraints + state_final_constraints)
        ):
            has_phase_event_constraints = True
            break

    has_static_param_constraints = False
    for param_info in multiphase_state.static_parameters.parameter_info:
        # Check range boundary constraints (never symbolic)
        if (
            param_info.boundary_constraint is not None
            and param_info.boundary_constraint.has_constraint()
        ):
            has_static_param_constraints = True
            break

        # Check fixed constraints (can be symbolic or non-symbolic)
        if (
            param_info.fixed_constraint is not None
            and param_info.fixed_constraint.has_constraint()
            and not param_info.fixed_constraint.is_symbolic()
        ):
            has_static_param_constraints = True
            break

    return (
        has_cross_phase_constraints or has_phase_event_constraints or has_static_param_constraints
    )


def _create_cross_phase_event_constraints(
    multiphase_state: MultiPhaseVariableState,
) -> Callable[..., list[Constraint]]:
    def _cross_phase_event_constraints(
        phase_endpoint_vectors: dict[PhaseID, dict[str, ca.MX]], static_parameters_vec: ca.MX | None
    ) -> list[Constraint]:
        result: list[Constraint] = []

        subs_map = _process_cross_phase_substitution_map(
            multiphase_state, phase_endpoint_vectors, static_parameters_vec
        )

        _process_cross_phase_symbolic_constraints(multiphase_state, subs_map, result)
        _process_phase_initial_boundary_constraints(
            multiphase_state, phase_endpoint_vectors, result
        )
        _process_phase_final_boundary_constraints(multiphase_state, phase_endpoint_vectors, result)
        _process_static_parameter_constraints(multiphase_state, static_parameters_vec, result)

        return result

    return _cross_phase_event_constraints


def _get_cross_phase_event_constraints_function(
    multiphase_state: MultiPhaseVariableState,
) -> Callable[..., list[Constraint]] | None:
    if not _check_has_cross_phase_constraints(multiphase_state):
        return None

    return _create_cross_phase_event_constraints(multiphase_state)
