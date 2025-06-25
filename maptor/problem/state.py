from __future__ import annotations

import threading
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import TypeAlias, cast

import casadi as ca

from ..exceptions import ConfigurationError, DataIntegrityError
from ..input_validation import _validate_constraint_input_format, _validate_string_not_empty
from ..mtor_types import FloatArray, PhaseID
from ..utils.constants import LARGE_VALUE


ConstraintInput: TypeAlias = (
    float | int | tuple[float | int | None, float | int | None] | None | ca.MX
)

BoundaryInput: TypeAlias = tuple[float | int | None, float | int | None] | None
FixedInput: TypeAlias = float | int | ca.MX | None


def _register_variable_name(
    name: str, name_to_index: dict[str, int], names_list: list[str], error_context: str
) -> int:
    if name in name_to_index:
        raise DataIntegrityError(
            f"{error_context} '{name}' already exists", "Variable naming conflict"
        )

    index = len(names_list)
    name_to_index[name] = index
    names_list.append(name)
    return index


def _rollback_variable_registration(
    name: str, name_to_index: dict[str, int], names_list: list[str]
) -> None:
    name_to_index.pop(name, None)
    names_list.pop()


def _process_numeric_constraint_input(constraint_input: float | int) -> tuple[float, float, float]:
    value = float(constraint_input)
    return value, value, value


def _process_tuple_constraint_input(
    constraint_input: tuple[float | int | None, float | int | None],
) -> tuple[float | None, float | None]:
    lower_val, upper_val = constraint_input
    lower = None if lower_val is None else float(lower_val)
    upper = None if upper_val is None else float(upper_val)
    return lower, upper


def _create_variable_info_with_rollback(
    name: str,
    name_to_index: dict[str, int],
    names_list: list[str],
    error_context: str,
    var_info_factory: Callable[[], _VariableInfo],
) -> None:
    _register_variable_name(name, name_to_index, names_list, error_context)

    try:
        var_info = var_info_factory()
        if hasattr(var_info, "_target_list") and var_info._target_list is not None:
            var_info._target_list.append(var_info)
    except Exception as e:
        _rollback_variable_registration(name, name_to_index, names_list)
        raise DataIntegrityError(
            f"Failed to create variable info for '{name}': {e}",
            "Variable creation error",
        ) from e


def _collect_symbolic_constraint(
    name: str,
    constraint_type: str,
    constraint: _EndpointConstraint | None,
    symbolic_constraints: list[tuple[str, str, ca.MX]],
) -> None:
    if constraint is not None and constraint.is_symbolic():
        symbolic_constraints.append(
            (name, constraint_type, cast(ca.MX, constraint.symbolic_expression))
        )


class _EndpointConstraint:
    def __init__(self, constraint_input: ConstraintInput = None) -> None:
        _validate_constraint_input_format(constraint_input, "boundary constraint")

        self.equals: float | None = None
        self.lower: float | None = None
        self.upper: float | None = None
        self.symbolic_expression: ca.MX | None = None

        self._process_constraint_input(constraint_input)

    def _process_constraint_input(self, constraint_input: ConstraintInput) -> None:
        if constraint_input is None:
            return
        elif isinstance(constraint_input, ca.MX):
            self.symbolic_expression = constraint_input
        elif isinstance(constraint_input, int | float):
            self.equals, self.lower, self.upper = _process_numeric_constraint_input(
                constraint_input
            )
        elif isinstance(constraint_input, tuple):
            self.lower, self.upper = _process_tuple_constraint_input(constraint_input)

    def has_constraint(self) -> bool:
        return (
            self.equals is not None
            or self.lower is not None
            or self.upper is not None
            or self.symbolic_expression is not None
        )

    def is_symbolic(self) -> bool:
        return self.symbolic_expression is not None

    def __repr__(self) -> str:
        if self.symbolic_expression is not None:
            return f"_EndpointConstraint(symbolic={self.symbolic_expression})"
        elif self.equals is not None:
            return f"_EndpointConstraint(equals={self.equals})"
        elif self.lower is not None and self.upper is not None:
            return f"_EndpointConstraint(lower={self.lower}, upper={self.upper})"
        elif self.lower is not None:
            return f"_EndpointConstraint(lower={self.lower})"
        elif self.upper is not None:
            return f"_EndpointConstraint(upper={self.upper})"
        else:
            return "_EndpointConstraint(no constraint)"


# maptor/problem/state.py - Add new classes, keep original _EndpointConstraint for initial/final


class _RangeBoundaryConstraint:
    def __init__(self, boundary_input: BoundaryInput = None) -> None:
        self.lower: float | None = None
        self.upper: float | None = None

        if boundary_input is not None:
            if not isinstance(boundary_input, tuple):
                raise ConfigurationError(
                    "boundary= argument only accepts range tuples like (lower, upper)"
                )
            self._process_boundary_input(boundary_input)

    def _process_boundary_input(self, boundary_input: tuple) -> None:
        lower, upper = _process_tuple_constraint_input(boundary_input)

        # Add range validation here
        if lower is not None and upper is not None and lower > upper:
            raise ConfigurationError(
                f"Invalid range: lower bound ({lower}) > upper bound ({upper})"
            )

        self.lower, self.upper = lower, upper

    def has_constraint(self) -> bool:
        return self.lower is not None or self.upper is not None

    def is_symbolic(self) -> bool:
        return False

    def __repr__(self) -> str:
        if self.lower is not None and self.upper is not None:
            return f"_RangeBoundaryConstraint(lower={self.lower}, upper={self.upper})"
        elif self.lower is not None:
            return f"_RangeBoundaryConstraint(lower={self.lower})"
        elif self.upper is not None:
            return f"_RangeBoundaryConstraint(upper={self.upper})"
        else:
            return "_RangeBoundaryConstraint(no constraint)"


class _FixedConstraint:
    # Handles fixed parameter values - equality and symbolic expressions only.

    def __init__(self, fixed_input: FixedInput = None) -> None:
        self.equals: float | None = None
        self.symbolic_expression: ca.MX | None = None

        if fixed_input is not None:
            self._process_fixed_input(fixed_input)

    def _process_fixed_input(self, fixed_input: FixedInput) -> None:
        if isinstance(fixed_input, ca.MX):
            self.symbolic_expression = fixed_input
        elif isinstance(fixed_input, int | float):
            self.equals = float(fixed_input)
        else:
            raise ConfigurationError(
                f"fixed= argument only accepts numeric values or symbolic expressions, got {type(fixed_input)}"
            )

    def has_constraint(self) -> bool:
        return self.equals is not None or self.symbolic_expression is not None

    def is_symbolic(self) -> bool:
        return self.symbolic_expression is not None

    def __repr__(self) -> str:
        if self.symbolic_expression is not None:
            return f"_FixedConstraint(symbolic={self.symbolic_expression})"
        elif self.equals is not None:
            return f"_FixedConstraint(equals={self.equals})"
        else:
            return "_FixedConstraint(no constraint)"


@dataclass
class _VariableInfo:
    symbol: ca.MX
    initial_symbol: ca.MX | None = None
    final_symbol: ca.MX | None = None
    initial_constraint: _EndpointConstraint | None = None
    final_constraint: _EndpointConstraint | None = None
    boundary_constraint: _RangeBoundaryConstraint | None = None
    fixed_constraint: _FixedConstraint | None = None
    _target_list: list[_VariableInfo] | None = None


@dataclass
class PhaseDefinition:
    phase_id: PhaseID

    state_info: list[_VariableInfo] = field(default_factory=list)
    control_info: list[_VariableInfo] = field(default_factory=list)
    state_name_to_index: dict[str, int] = field(default_factory=dict)
    control_name_to_index: dict[str, int] = field(default_factory=dict)
    state_names: list[str] = field(default_factory=list)
    control_names: list[str] = field(default_factory=list)

    sym_time: ca.MX | None = None
    sym_time_initial: ca.MX | None = None
    sym_time_final: ca.MX | None = None

    dynamics_expressions: dict[ca.MX, ca.MX] = field(default_factory=dict)
    path_constraints: list[ca.MX] = field(default_factory=list)

    integral_expressions: list[ca.MX] = field(default_factory=list)
    integral_symbols: list[ca.MX] = field(default_factory=list)
    num_integrals: int = 0

    t0_constraint: _EndpointConstraint = field(default_factory=lambda: _EndpointConstraint(0.0))
    tf_constraint: _EndpointConstraint = field(default_factory=lambda: _EndpointConstraint())

    collocation_points_per_interval: list[int] = field(default_factory=list)
    global_normalized_mesh_nodes: FloatArray | None = None
    mesh_configured: bool = False

    # Phase-level initial guess storage
    guess_states: list[FloatArray] | None = None
    guess_controls: list[FloatArray] | None = None
    guess_initial_time: float | None = None
    guess_terminal_time: float | None = None
    guess_integrals: float | FloatArray | None = None

    _ordering_lock: threading.Lock = field(default_factory=threading.Lock)

    symbolic_boundary_constraints: list[tuple[str, str, ca.MX]] = field(default_factory=list)

    _dynamics_function: Callable[..., ca.MX] | None = None
    _integrand_function: Callable[..., ca.MX] | None = None
    _path_constraints_function: Callable[..., list] | None = None
    _numerical_dynamics_function: (
        Callable[[FloatArray, FloatArray, float, FloatArray | None], FloatArray] | None
    ) = None
    _functions_built: bool = False

    def _create_state_variable_info(
        self,
        symbol: ca.MX,
        initial_symbol: ca.MX | None,
        final_symbol: ca.MX | None,
        initial_constraint: _EndpointConstraint | None,
        final_constraint: _EndpointConstraint | None,
        boundary_constraint: _RangeBoundaryConstraint | None,
    ) -> _VariableInfo:
        var_info = _VariableInfo(
            symbol=symbol,
            initial_symbol=initial_symbol,
            final_symbol=final_symbol,
            initial_constraint=initial_constraint,
            final_constraint=final_constraint,
            boundary_constraint=boundary_constraint,
        )
        var_info._target_list = self.state_info
        return var_info

    def _create_control_variable_info(
        self, symbol: ca.MX, boundary_constraint: _RangeBoundaryConstraint | None
    ) -> _VariableInfo:
        var_info = _VariableInfo(
            symbol=symbol,
            boundary_constraint=boundary_constraint,
        )
        var_info._target_list = self.control_info
        return var_info

    def _collect_state_symbolic_constraints(
        self,
        name: str,
        initial_constraint: _EndpointConstraint | None,
        final_constraint: _EndpointConstraint | None,
    ) -> None:
        _collect_symbolic_constraint(
            name, "initial", initial_constraint, self.symbolic_boundary_constraints
        )
        _collect_symbolic_constraint(
            name, "final", final_constraint, self.symbolic_boundary_constraints
        )

    def add_state(
        self,
        name: str,
        symbol: ca.MX,
        initial_symbol: ca.MX | None = None,
        final_symbol: ca.MX | None = None,
        initial_constraint: _EndpointConstraint | None = None,
        final_constraint: _EndpointConstraint | None = None,
        boundary_constraint: _RangeBoundaryConstraint | None = None,
    ) -> None:
        _validate_string_not_empty(name, "State variable name")

        with self._ordering_lock:

            def create_state_info():
                var_info = self._create_state_variable_info(
                    symbol,
                    initial_symbol,
                    final_symbol,
                    initial_constraint,
                    final_constraint,
                    boundary_constraint,
                )
                self._collect_state_symbolic_constraints(name, initial_constraint, final_constraint)
                return var_info

            _create_variable_info_with_rollback(
                name,
                self.state_name_to_index,
                self.state_names,
                f"State in phase {self.phase_id}",
                create_state_info,
            )

    def add_control(
        self, name: str, symbol: ca.MX, boundary_constraint: _RangeBoundaryConstraint | None = None
    ) -> None:
        _validate_string_not_empty(name, "Control variable name")

        with self._ordering_lock:

            def create_control_info():
                return self._create_control_variable_info(symbol, boundary_constraint)

            _create_variable_info_with_rollback(
                name,
                self.control_name_to_index,
                self.control_names,
                f"Control in phase {self.phase_id}",
                create_control_info,
            )

    def get_variable_counts(self) -> tuple[int, int]:
        return len(self.state_info), len(self.control_info)

    def _get_ordered_state_symbols(self) -> list[ca.MX]:
        return [info.symbol for info in self.state_info]

    def _get_ordered_control_symbols(self) -> list[ca.MX]:
        return [info.symbol for info in self.control_info]

    def _get_ordered_state_initial_symbols(self) -> list[ca.MX]:
        symbols = []
        for info in self.state_info:
            if info.initial_symbol is None:
                raise DataIntegrityError(
                    f"State variable in phase {self.phase_id} has None initial_symbol",
                    "State symbol integrity violation",
                )
            symbols.append(info.initial_symbol)
        return symbols

    def _get_ordered_state_final_symbols(self) -> list[ca.MX]:
        symbols = []
        for info in self.state_info:
            if info.final_symbol is None:
                raise DataIntegrityError(
                    f"State variable in phase {self.phase_id} has None final_symbol",
                    "State symbol integrity violation",
                )
            symbols.append(info.final_symbol)
        return symbols

    def _get_time_bounds(
        self, constraint: _EndpointConstraint, constraint_type: str
    ) -> tuple[float, float]:
        if constraint.is_symbolic():
            return (-LARGE_VALUE, LARGE_VALUE)
        if constraint.equals is not None:
            return (constraint.equals, constraint.equals)
        lower = constraint.lower if constraint.lower is not None else -LARGE_VALUE
        upper = constraint.upper if constraint.upper is not None else LARGE_VALUE
        return (lower, upper)

    @property
    def t0_bounds(self) -> tuple[float, float]:
        return self._get_time_bounds(self.t0_constraint, "initial")

    @property
    def tf_bounds(self) -> tuple[float, float]:
        return self._get_time_bounds(self.tf_constraint, "final")


@dataclass
class StaticParameterState:
    parameter_info: list[_VariableInfo] = field(default_factory=list)
    parameter_name_to_index: dict[str, int] = field(default_factory=dict)
    parameter_names: list[str] = field(default_factory=list)
    symbolic_boundary_constraints: list[tuple[str, str, ca.MX]] = field(default_factory=list)
    _ordering_lock: threading.Lock = field(default_factory=threading.Lock)

    def _create_parameter_variable_info(
        self,
        symbol: ca.MX,
        boundary_constraint: _RangeBoundaryConstraint | None,
        fixed_constraint: _FixedConstraint | None,
    ) -> _VariableInfo:
        var_info = _VariableInfo(
            symbol=symbol,
            boundary_constraint=boundary_constraint,
            fixed_constraint=fixed_constraint,
        )
        var_info._target_list = self.parameter_info
        return var_info

    def _collect_parameter_symbolic_constraints(
        self,
        name: str,
        fixed_constraint: _FixedConstraint | None,
    ) -> None:
        if fixed_constraint is not None and fixed_constraint.is_symbolic():
            self.symbolic_boundary_constraints.append(
                (name, "fixed", cast(ca.MX, fixed_constraint.symbolic_expression))
            )

    def add_parameter(
        self,
        name: str,
        symbol: ca.MX,
        boundary_constraint: _RangeBoundaryConstraint | None = None,
        fixed_constraint: _FixedConstraint | None = None,
    ) -> None:
        _validate_string_not_empty(name, "Parameter name")

        with self._ordering_lock:

            def create_parameter_info():
                var_info = self._create_parameter_variable_info(
                    symbol, boundary_constraint, fixed_constraint
                )
                self._collect_parameter_symbolic_constraints(name, fixed_constraint)
                return var_info

            _create_variable_info_with_rollback(
                name,
                self.parameter_name_to_index,
                self.parameter_names,
                "Parameter",
                create_parameter_info,
            )

    def get_parameter_count(self) -> int:
        return len(self.parameter_info)

    def get_ordered_parameter_symbols(self) -> list[ca.MX]:
        return [info.symbol for info in self.parameter_info]


@dataclass
@dataclass
class MultiPhaseVariableState:
    phases: dict[PhaseID, PhaseDefinition] = field(default_factory=dict)
    static_parameters: StaticParameterState = field(default_factory=StaticParameterState)
    cross_phase_constraints: list[ca.MX] = field(default_factory=list)
    objective_expression: ca.MX | None = None
    guess_static_parameters: dict[str, float] | None = None
    _objective_function: Callable[..., ca.MX] | None = None
    _cross_phase_constraints_function: Callable[..., list] | None = None
    _functions_built: bool = False

    def set_phase(self, phase_id: PhaseID) -> PhaseDefinition:
        if phase_id in self.phases:
            raise DataIntegrityError(
                f"Phase {phase_id} already exists", "Phase definition conflict"
            )

        phase_def = PhaseDefinition(phase_id=phase_id)
        self.phases[phase_id] = phase_def
        return phase_def

    def _get_phase_ids(self) -> list[PhaseID]:
        return sorted(self.phases.keys())

    def _get_total_variable_counts(self) -> tuple[int, int, int]:
        total_states = sum(len(phase.state_info) for phase in self.phases.values())
        total_controls = sum(len(phase.control_info) for phase in self.phases.values())
        num_static_params = self.static_parameters.get_parameter_count()
        return total_states, total_controls, num_static_params
