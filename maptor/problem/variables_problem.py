from __future__ import annotations

from typing import Any, cast

import casadi as ca

from ..exceptions import ConfigurationError
from ..input_validation import _validate_constraint_input_format, _validate_string_not_empty
from .state import (
    BoundaryInput,
    ConstraintInput,
    FixedInput,
    MultiPhaseVariableState,
    PhaseDefinition,
    StaticParameterState,
    _EndpointConstraint,
    _FixedConstraint,
    _RangeBoundaryConstraint,
)


def _extract_casadi_symbol(expr: Any) -> ca.MX:
    if hasattr(expr, "_symbolic_var"):
        return expr._symbolic_var
    return expr


def _create_phase_symbol(base_name: str, phase_id: int, suffix: str = "") -> ca.MX:
    name = f"{base_name}_p{phase_id}{suffix}" if suffix else f"{base_name}_p{phase_id}"
    return ca.MX.sym(name, 1)  # type: ignore[arg-type]


def _convert_expression_to_casadi(
    expr: Any, expression_type: str, allow_callable_error: bool = True
) -> ca.MX:
    try:
        if isinstance(expr, ca.MX):
            return expr
        elif hasattr(expr, "_symbolic_var"):
            return _extract_casadi_symbol(expr)
        else:
            return ca.MX(expr)
    except Exception as e:
        if callable(expr) and allow_callable_error:
            raise ConfigurationError(
                f"{expression_type} appears to be a function {expr}. Did you forget to call it?"
            ) from e
        else:
            raise ConfigurationError(
                f"Cannot convert {expression_type} of type {type(expr)} to CasADi MX: {expr}"
            ) from e


def _validate_dynamics_key_exists(
    state_sym: ca.MX | StateVariableImpl, ordered_state_symbols: list[ca.MX], phase_id: int
) -> None:
    underlying_sym = _extract_casadi_symbol(state_sym)

    for sym in ordered_state_symbols:
        if underlying_sym is sym:
            return

    raise ConfigurationError(f"Dynamics provided for undefined state variable in phase {phase_id}")


def _convert_dynamics_dict_to_casadi(
    dynamics_dict: dict[ca.MX | Any, ca.MX | float | int | Any], phase_id: int
) -> dict[ca.MX, ca.MX]:
    converted_dict = {}

    for key, value in dynamics_dict.items():
        storage_key = _extract_casadi_symbol(key)
        storage_value = _convert_expression_to_casadi(value, "Dynamics expression")
        converted_dict[storage_key] = storage_value

    return converted_dict


class _SymbolicVariableBase:
    def __init__(self, symbolic_var: ca.MX) -> None:
        self._symbolic_var = symbolic_var

    def __call__(self, other: Any = None) -> ca.MX:
        if other is None:
            return self._symbolic_var
        raise NotImplementedError("Variable indexing not yet implemented")

    def __casadi_MX__(self) -> ca.MX:  # noqa: N802
        return self._symbolic_var

    def __array_function__(self, func, args, kwargs):
        converted_args = [self._symbolic_var if arg is self else arg for arg in args]
        return func(*converted_args, **kwargs)

    def __getattr__(self, name: str) -> Any:
        if name.startswith("_"):
            raise AttributeError(f"'{type(self).__name__}' object has no attribute '{name}'")
        return getattr(self._symbolic_var, name)

    def __hash__(self) -> int:
        return hash(self._symbolic_var)

    def __eq__(self, other: Any) -> bool:
        if hasattr(other, "_symbolic_var"):
            return self._symbolic_var is other._symbolic_var
        return self._symbolic_var is other

    def __add__(self, other: Any) -> ca.MX:
        return self._symbolic_var + other

    def __radd__(self, other: Any) -> ca.MX:
        return other + self._symbolic_var

    def __sub__(self, other: Any) -> ca.MX:
        return self._symbolic_var - other

    def __rsub__(self, other: Any) -> ca.MX:
        return other - self._symbolic_var

    def __mul__(self, other: Any) -> ca.MX:
        return self._symbolic_var * other

    def __rmul__(self, other: Any) -> ca.MX:
        return other * self._symbolic_var

    def __truediv__(self, other: Any) -> ca.MX:
        return self._symbolic_var / other

    def __rtruediv__(self, other: Any) -> ca.MX:
        return other / self._symbolic_var

    def __pow__(self, other: Any) -> ca.MX:
        return self._symbolic_var**other

    def __neg__(self) -> ca.MX:
        return cast(ca.MX, -self._symbolic_var)

    def __lt__(self, other: Any) -> ca.MX:
        return self._symbolic_var < other

    def __le__(self, other: Any) -> ca.MX:
        return self._symbolic_var <= other

    def __gt__(self, other: Any) -> ca.MX:
        return self._symbolic_var > other

    def __ge__(self, other: Any) -> ca.MX:
        return self._symbolic_var >= other

    def __ne__(self, other: Any) -> ca.MX:
        return self._symbolic_var != other


class TimeVariableImpl(_SymbolicVariableBase):
    def __init__(self, sym_var: ca.MX, sym_initial: ca.MX, sym_final: ca.MX) -> None:
        super().__init__(sym_var)
        self._sym_initial = sym_initial
        self._sym_final = sym_final

    @property
    def initial(self) -> ca.MX:
        return self._sym_initial

    @property
    def final(self) -> ca.MX:
        return self._sym_final


class StateVariableImpl(_SymbolicVariableBase):
    def __init__(self, sym_var: ca.MX, sym_initial: ca.MX, sym_final: ca.MX) -> None:
        super().__init__(sym_var)
        self._sym_initial = sym_initial
        self._sym_final = sym_final

    @property
    def initial(self) -> ca.MX:
        return self._sym_initial

    @property
    def final(self) -> ca.MX:
        return self._sym_final


def _create_time_symbols(phase_id: int) -> tuple[ca.MX, ca.MX, ca.MX]:
    sym_time = _create_phase_symbol("t", phase_id)
    sym_t0 = _create_phase_symbol("t0", phase_id)
    sym_tf = _create_phase_symbol("tf", phase_id)
    return sym_time, sym_t0, sym_tf


def _create_state_symbols(name: str, phase_id: int) -> tuple[ca.MX, ca.MX, ca.MX]:
    sym_var = _create_phase_symbol(name, phase_id)
    sym_initial = _create_phase_symbol(f"{name}_initial", phase_id)
    sym_final = _create_phase_symbol(f"{name}_final", phase_id)
    return sym_var, sym_initial, sym_final


def create_phase_time_variable(
    phase_def: PhaseDefinition, initial: ConstraintInput = 0.0, final: ConstraintInput = None
) -> TimeVariableImpl:
    _validate_constraint_input_format(initial, f"phase {phase_def.phase_id} initial time")
    _validate_constraint_input_format(final, f"phase {phase_def.phase_id} final time")

    sym_time, sym_t0, sym_tf = _create_time_symbols(phase_def.phase_id)

    if initial is None:
        initial = 0.0

    t0_constraint = _EndpointConstraint(initial)
    tf_constraint = _EndpointConstraint(final)

    phase_def.t0_constraint = t0_constraint
    phase_def.tf_constraint = tf_constraint
    phase_def.sym_time = sym_time
    phase_def.sym_time_initial = sym_t0
    phase_def.sym_time_final = sym_tf

    return TimeVariableImpl(sym_time, sym_t0, sym_tf)


def _create_phase_state_variable(
    phase_def: PhaseDefinition,
    name: str,
    initial: ConstraintInput = None,
    final: ConstraintInput = None,
    boundary: BoundaryInput = None,
) -> StateVariableImpl:
    _validate_string_not_empty(name, f"phase {phase_def.phase_id} state name")
    _validate_constraint_input_format(initial, f"phase {phase_def.phase_id} state '{name}' initial")
    _validate_constraint_input_format(final, f"phase {phase_def.phase_id} state '{name}' final")
    _validate_constraint_input_format(
        boundary, f"phase {phase_def.phase_id} state '{name}' boundary"
    )

    sym_var, sym_initial, sym_final = _create_state_symbols(name, phase_def.phase_id)

    initial_constraint = _EndpointConstraint(initial) if initial is not None else None
    final_constraint = _EndpointConstraint(final) if final is not None else None
    boundary_constraint = _RangeBoundaryConstraint(boundary) if boundary is not None else None

    phase_def.add_state(
        name=name,
        symbol=sym_var,
        initial_symbol=sym_initial,
        final_symbol=sym_final,
        initial_constraint=initial_constraint,
        final_constraint=final_constraint,
        boundary_constraint=boundary_constraint,
    )

    return StateVariableImpl(sym_var, sym_initial, sym_final)


def create_phase_control_variable(
    phase_def: PhaseDefinition,
    name: str,
    boundary: BoundaryInput = None,  # Ranges only
) -> ca.MX:
    _validate_string_not_empty(name, f"phase {phase_def.phase_id} control name")
    _validate_constraint_input_format(
        boundary, f"phase {phase_def.phase_id} control '{name}' boundary"
    )

    sym_var = _create_phase_symbol(name, phase_def.phase_id)
    boundary_constraint = _RangeBoundaryConstraint(boundary) if boundary is not None else None

    phase_def.add_control(name=name, symbol=sym_var, boundary_constraint=boundary_constraint)

    return sym_var


def _create_static_parameter(
    static_params: StaticParameterState,
    name: str,
    boundary: BoundaryInput = None,  # Ranges only
    fixed: FixedInput = None,  # Equality/symbolic only
) -> ca.MX:
    _validate_string_not_empty(name, f"parameter '{name}' name")
    _validate_constraint_input_format(boundary, f"parameter '{name}' boundary")
    _validate_constraint_input_format(fixed, f"parameter '{name}' fixed")
    if boundary is not None and fixed is not None:
        raise ConfigurationError(
            f"Parameter '{name}' cannot have both boundary and fixed constraints"
        )

    sym_var = ca.MX.sym(f"param_{name}", 1)  # type: ignore[arg-type]

    boundary_constraint = _RangeBoundaryConstraint(boundary) if boundary is not None else None
    fixed_constraint = _FixedConstraint(fixed) if fixed is not None else None

    static_params.add_parameter(
        name=name,
        symbol=sym_var,
        boundary_constraint=boundary_constraint,
        fixed_constraint=fixed_constraint,
    )

    return sym_var


def _set_phase_dynamics(
    phase_def: PhaseDefinition,
    dynamics_dict: dict[ca.MX | StateVariableImpl, ca.MX | float | int | StateVariableImpl],
) -> None:
    ordered_state_symbols = phase_def._get_ordered_state_symbols()

    for state_sym in dynamics_dict.keys():
        _validate_dynamics_key_exists(state_sym, ordered_state_symbols, phase_def.phase_id)

    converted_dict = _convert_dynamics_dict_to_casadi(dynamics_dict, phase_def.phase_id)
    phase_def.dynamics_expressions = converted_dict

    phase_def._dynamics_function = None
    phase_def._numerical_dynamics_function = None
    phase_def._functions_built = False


def _create_integral_symbol(phase_def: PhaseDefinition) -> ca.MX:
    integral_name = f"integral_{len(phase_def.integral_expressions)}_p{phase_def.phase_id}"
    return ca.MX.sym(integral_name, 1)  # type: ignore[arg-type]


def _set_phase_integral(phase_def: PhaseDefinition, integrand_expr: ca.MX | float | int) -> ca.MX:
    integral_sym = _create_integral_symbol(phase_def)
    pure_expr = _convert_expression_to_casadi(integrand_expr, "Integrand")

    phase_def.integral_expressions.append(pure_expr)
    phase_def.integral_symbols.append(integral_sym)
    phase_def.num_integrals = len(phase_def.integral_expressions)

    phase_def._integrand_function = None
    phase_def._functions_built = False

    return integral_sym


def _set_multiphase_objective(
    multiphase_state: MultiPhaseVariableState, objective_expr: ca.MX | float | int
) -> None:
    pure_expr = _convert_expression_to_casadi(objective_expr, "Objective")
    multiphase_state.objective_expression = pure_expr

    multiphase_state._objective_function = None
    multiphase_state._functions_built = False
