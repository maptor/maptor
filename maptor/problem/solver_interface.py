from __future__ import annotations

from collections.abc import Callable
from typing import cast

import casadi as ca
import numpy as np

from ..mtor_types import FloatArray, PhaseID
from .casadi_build import (
    _build_static_parameter_substitution_map,
    _build_unified_casadi_function_inputs,
    _build_unified_multiphase_symbol_inputs,
    _build_unified_symbol_substitution_map,
)
from .state import MultiPhaseVariableState, PhaseDefinition


def _get_static_param_count(static_parameter_symbols: list[ca.MX] | None) -> int:
    return len(static_parameter_symbols) if static_parameter_symbols else 0


def _prepare_static_params_input(
    static_parameters_vec: ca.MX | None, num_static_params: int
) -> ca.MX:
    if static_parameters_vec is None or num_static_params == 0:
        return ca.MX(max(1, num_static_params), 1)
    return static_parameters_vec


def _extract_dynamics_output(result, phase_id: PhaseID) -> ca.MX:
    dynamics_output = result[0] if isinstance(result, list | tuple) else result
    if dynamics_output is None:
        raise ValueError(f"Phase {phase_id} dynamics function returned None")
    return cast(ca.MX, dynamics_output)


def _build_dynamics_casadi_function(
    phase_def: PhaseDefinition, static_parameter_symbols: list[ca.MX] | None
) -> ca.Function:
    states_vec, controls_vec, time, static_params_vec, function_inputs = (
        _build_unified_casadi_function_inputs(phase_def, static_parameter_symbols)
    )

    subs_map = _build_static_parameter_substitution_map(static_parameter_symbols, static_params_vec)

    state_syms = phase_def._get_ordered_state_symbols()
    dynamics_expr = []
    for state_sym in state_syms:
        if state_sym in phase_def.dynamics_expressions:
            expr = phase_def.dynamics_expressions[state_sym]
            casadi_expr = ca.MX(expr) if not isinstance(expr, ca.MX) else expr

            if subs_map:
                casadi_expr = ca.substitute(
                    [casadi_expr], list(subs_map.keys()), list(subs_map.values())
                )[0]

            dynamics_expr.append(casadi_expr)
        else:
            dynamics_expr.append(ca.MX(0))

    dynamics_vec = ca.vertcat(*dynamics_expr) if dynamics_expr else ca.MX()
    return ca.Function(f"dynamics_p{phase_def.phase_id}", function_inputs, [dynamics_vec])


def _create_dynamics(
    dynamics_func: ca.Function, phase_def: PhaseDefinition, num_static_params: int
) -> Callable[..., ca.MX]:
    def _dynamics(
        states_vec: ca.MX,
        controls_vec: ca.MX,
        time: ca.MX,
        static_parameters_vec: ca.MX | None = None,
    ) -> ca.MX:
        static_params_input = _prepare_static_params_input(static_parameters_vec, num_static_params)
        result = dynamics_func(states_vec, controls_vec, time, static_params_input)
        return _extract_dynamics_output(result, phase_def.phase_id)

    return _dynamics


def _create_numerical_dynamics(
    dynamics_func: ca.Function, num_static_params: int
) -> Callable[[FloatArray, FloatArray, float, FloatArray | None], FloatArray]:
    def _numerical_dynamics(
        states: FloatArray,
        controls: FloatArray,
        time: float,
        static_parameters: FloatArray | None = None,
    ) -> FloatArray:
        if static_parameters is None:
            static_params_array = np.zeros(max(1, num_static_params), dtype=np.float64)
        else:
            static_params_array = np.asarray(static_parameters, dtype=np.float64).flatten()

        states_array = np.asarray(states, dtype=np.float64)
        controls_array = np.asarray(controls, dtype=np.float64)
        time_scalar = float(time)

        result = dynamics_func(states_array, controls_array, time_scalar, static_params_array)

        if hasattr(result, "full"):
            return np.asarray(result.full(), dtype=np.float64).flatten()  # type: ignore[attr-defined]
        else:
            return np.asarray(result, dtype=np.float64).flatten()

    return _numerical_dynamics


def _build_unified_phase_dynamics_functions(
    phase_def: PhaseDefinition, static_parameter_symbols: list[ca.MX] | None = None
) -> tuple[
    Callable[..., ca.MX], Callable[[FloatArray, FloatArray, float, FloatArray | None], FloatArray]
]:
    dynamics_func = _build_dynamics_casadi_function(phase_def, static_parameter_symbols)
    num_static_params = _get_static_param_count(static_parameter_symbols)

    symbolic_dynamics = _create_dynamics(dynamics_func, phase_def, num_static_params)
    numerical_dynamics = _create_numerical_dynamics(dynamics_func, num_static_params)

    return symbolic_dynamics, numerical_dynamics


def _build_integrand_casadi_functions(
    phase_def: PhaseDefinition, static_parameter_symbols: list[ca.MX] | None
) -> list[ca.Function]:
    states_vec, controls_vec, time, static_params_vec, function_inputs = (
        _build_unified_casadi_function_inputs(phase_def, static_parameter_symbols)
    )

    subs_map = _build_static_parameter_substitution_map(static_parameter_symbols, static_params_vec)

    integrand_funcs = []
    for i, expr in enumerate(phase_def.integral_expressions):
        processed_expr = expr
        if subs_map:
            processed_expr = ca.substitute([expr], list(subs_map.keys()), list(subs_map.values()))[
                0
            ]

        integrand_funcs.append(
            ca.Function(f"integrand_{i}_p{phase_def.phase_id}", function_inputs, [processed_expr])
        )

    return integrand_funcs


def _create_integrand(
    integrand_funcs: list[ca.Function], num_static_params: int
) -> Callable[..., ca.MX]:
    def _integrand(
        states_vec: ca.MX,
        controls_vec: ca.MX,
        time: ca.MX,
        integral_idx: int,
        static_parameters_vec: ca.MX | None = None,
    ) -> ca.MX:
        if integral_idx >= len(integrand_funcs):
            return ca.MX(0.0)

        static_params_input = _prepare_static_params_input(static_parameters_vec, num_static_params)

        result = integrand_funcs[integral_idx](states_vec, controls_vec, time, static_params_input)
        integrand_output = result[0] if isinstance(result, list | tuple) else result
        return cast(ca.MX, integrand_output)

    return _integrand


def _build_phase_integrand_function(
    phase_def: PhaseDefinition, static_parameter_symbols: list[ca.MX] | None = None
) -> Callable[..., ca.MX] | None:
    if not phase_def.integral_expressions:
        return None

    integrand_funcs = _build_integrand_casadi_functions(phase_def, static_parameter_symbols)
    num_static_params = _get_static_param_count(static_parameter_symbols)
    return _create_integrand(integrand_funcs, num_static_params)


def _build_objective_casadi_function(multiphase_state: MultiPhaseVariableState) -> ca.Function:
    phase_inputs, s_vec = _build_unified_multiphase_symbol_inputs(multiphase_state)

    phase_symbols_map = _build_unified_symbol_substitution_map(
        multiphase_state, phase_inputs, s_vec
    )

    objective_expr = multiphase_state.objective_expression
    if phase_symbols_map:
        objective_expr = ca.substitute(
            [objective_expr], list(phase_symbols_map.keys()), list(phase_symbols_map.values())
        )[0]

    return ca.Function("multiphase_objective", phase_inputs, [objective_expr])


def _prepare_phase_endpoint_inputs(
    multiphase_state: MultiPhaseVariableState, phase_endpoint_data: dict[PhaseID, dict[str, ca.MX]]
) -> list[ca.MX]:
    inputs = []

    for phase_id in sorted(multiphase_state.phases.keys()):
        if phase_id in phase_endpoint_data:
            data = phase_endpoint_data[phase_id]
            phase_def = multiphase_state.phases[phase_id]

            q_val = (
                data["q"] if data["q"] is not None else ca.DM(max(1, phase_def.num_integrals), 1)
            )

            inputs.extend([data["t0"], data["tf"], data["x0"], data["xf"], q_val])
        else:
            phase_def = multiphase_state.phases[phase_id]
            num_states = len(phase_def.state_info)
            inputs.extend(
                [
                    ca.DM(1, 1),
                    ca.DM(1, 1),
                    ca.DM(num_states, 1),
                    ca.DM(num_states, 1),
                    ca.DM(max(1, phase_def.num_integrals), 1),
                ]
            )

    return inputs


def _create_unified_multiphase_objective(
    obj_func: ca.Function, multiphase_state: MultiPhaseVariableState
) -> Callable[..., ca.MX]:
    def _unified_multiphase_objective(
        phase_endpoint_data: dict[PhaseID, dict[str, ca.MX]], static_parameters_vec: ca.MX | None
    ) -> ca.MX:
        inputs = _prepare_phase_endpoint_inputs(multiphase_state, phase_endpoint_data)

        if static_parameters_vec is not None:
            inputs.append(static_parameters_vec)
        else:
            num_params = multiphase_state.static_parameters.get_parameter_count()
            inputs.append(ca.DM.zeros(max(1, num_params), 1))  # type: ignore[arg-type]

        result = obj_func(*inputs)
        obj_output = result[0] if isinstance(result, list | tuple) else result
        return cast(ca.MX, obj_output)

    return _unified_multiphase_objective


def _build_multiphase_objective_function(
    multiphase_state: MultiPhaseVariableState,
) -> Callable[..., ca.MX]:
    if multiphase_state.objective_expression is None:
        raise ValueError("Multiphase objective expression not defined")

    obj_func = _build_objective_casadi_function(multiphase_state)
    return _create_unified_multiphase_objective(obj_func, multiphase_state)
