import logging
import math
from collections.abc import Sequence
from typing import Any, TypeVar

import casadi as ca
import numpy as np

from .exceptions import ConfigurationError, DataIntegrityError
from .mtor_types import FloatArray, NumericArrayLike, PhaseID, ProblemProtocol
from .utils.constants import COORDINATE_PRECISION, NUMERICAL_ZERO


logger = logging.getLogger(__name__)
T = TypeVar("T")


# ==========================
# CORE VALIDATION PRIMITIVES
# ==========================


def _validate_positive_integer(value: Any, name: str, min_value: int = 1) -> None:
    if not isinstance(value, int):
        raise ConfigurationError(f"{name} must be integer, got {type(value)}")
    if value < min_value:
        raise ConfigurationError(f"{name} must be >= {min_value}, got {value}")


def _validate_positive_number(value: Any, name: str) -> None:
    if not isinstance(value, int | float):
        raise ConfigurationError(f"{name} must be numeric, got {type(value)}")
    if value <= 0:
        raise ConfigurationError(f"{name} must be positive, got {value}")
    if math.isnan(value) or math.isinf(value):
        raise ConfigurationError(f"{name} cannot be NaN or infinite, got {value}")


def _validate_string_not_empty(value: Any, name: str) -> None:
    if not isinstance(value, str):
        raise ConfigurationError(f"{name} must be string, got {type(value)}")
    if not value.strip():
        raise ConfigurationError(f"{name} cannot be empty")


def _validate_array_numerical_integrity(
    array: FloatArray, name: str, context: str = "validation"
) -> None:
    if np.any(np.isnan(array)) or np.any(np.isinf(array)):
        raise DataIntegrityError(
            f"{name} contains NaN or Inf values", f"Numerical corruption in {context}"
        )


# ======================
# CONSTRAINT VALIDATION
# ======================


def _validate_constraint_input_format(constraint_input: Any, context: str) -> None:
    if constraint_input is None or isinstance(constraint_input, ca.MX):
        return

    if isinstance(constraint_input, int | float):
        if math.isnan(constraint_input) or math.isinf(constraint_input):
            raise ConfigurationError(
                f"Constraint cannot be NaN/infinite: {constraint_input}", context
            )
    elif isinstance(constraint_input, tuple):
        if len(constraint_input) != 2:
            raise ConfigurationError(
                f"Constraint tuple must have 2 elements, got {len(constraint_input)}", context
            )

        lower, upper = constraint_input
        for i, val in enumerate([lower, upper]):
            if val is not None:
                if not isinstance(val, int | float):
                    raise ConfigurationError(
                        f"Constraint bound {i} must be numeric/None, got {type(val)}", context
                    )
                if math.isnan(val) or math.isinf(val):
                    raise ConfigurationError(
                        f"Constraint bound {i} cannot be NaN/infinite: {val}", context
                    )

        if lower is not None and upper is not None and lower > upper:
            raise ConfigurationError(f"Lower bound ({lower}) > upper bound ({upper})", context)
    else:
        raise ConfigurationError(f"Invalid constraint type: {type(constraint_input)}", context)


# ================
# MESH VALIDATION
# ================


def _validate_mesh_configuration(
    polynomial_degrees: list[int], mesh_points: FloatArray, num_intervals: int
) -> None:
    if not isinstance(polynomial_degrees, list) or not polynomial_degrees:
        raise ConfigurationError("Polynomial degrees must be non-empty list")

    for i, degree in enumerate(polynomial_degrees):
        _validate_positive_integer(degree, f"polynomial degree for interval {i}")

    _validate_positive_integer(num_intervals, "number of mesh intervals")

    if len(polynomial_degrees) != num_intervals:
        raise ConfigurationError(
            f"Polynomial degrees count ({len(polynomial_degrees)}) != intervals ({num_intervals})"
        )

    if len(mesh_points) != num_intervals + 1:
        raise ConfigurationError(
            f"Mesh points count ({len(mesh_points)}) != intervals+1 ({num_intervals + 1})"
        )

    if not np.isclose(mesh_points[0], -1.0, atol=NUMERICAL_ZERO):
        raise ConfigurationError(f"First mesh point must be -1.0, got {mesh_points[0]}")
    if not np.isclose(mesh_points[-1], 1.0, atol=NUMERICAL_ZERO):
        raise ConfigurationError(f"Last mesh point must be 1.0, got {mesh_points[-1]}")

    mesh_diffs = np.diff(mesh_points)
    if not np.all(mesh_diffs > COORDINATE_PRECISION):
        raise ConfigurationError(
            f"Mesh points must be strictly increasing with min spacing {COORDINATE_PRECISION}"
        )


# ===================
# PROBLEM VALIDATION
# ===================


def _validate_problem_dimensions(
    num_states: int, num_controls: int, num_static_params: int, context: str = "problem"
) -> None:
    for count, name in [
        (num_states, "states"),
        (num_controls, "controls"),
        (num_static_params, "static parameters"),
    ]:
        if not isinstance(count, int) or count < 0:
            raise ConfigurationError(
                f"Number of {name} must be non-negative integer, got {count}", context
            )


def _find_state_name_by_symbol(phase_def, target_symbol):
    state_symbols = phase_def._get_ordered_state_symbols()
    for i, state_sym in enumerate(state_symbols):
        if state_sym is target_symbol:
            return phase_def.state_names[i]
    return f"unknown_state_{target_symbol.name()}"


def _validate_complete_dynamics(phase_def, phase_id):
    state_symbols = phase_def._get_ordered_state_symbols()

    for state_sym in state_symbols:
        if state_sym not in phase_def.dynamics_expressions:
            state_name = _find_state_name_by_symbol(phase_def, state_sym)
            raise ConfigurationError(
                f"Phase {phase_id} state '{state_name}' missing dynamics equation. "
                f"Either provide dynamics or use a parameter instead."
            )


def _validate_phase_configuration(problem: ProblemProtocol, phase_id: PhaseID) -> None:
    if phase_id not in problem._phases:
        raise ConfigurationError(f"Phase {phase_id} not found in problem")

    phase_def = problem._phases[phase_id]

    if not phase_def.mesh_configured:
        raise ConfigurationError(f"Phase {phase_id} mesh must be configured - call phase.mesh()")

    _validate_mesh_configuration(
        phase_def.collocation_points_per_interval,
        phase_def.global_normalized_mesh_nodes,
        len(phase_def.collocation_points_per_interval),
    )

    if not phase_def.dynamics_expressions:
        raise ConfigurationError(
            f"Phase {phase_id} dynamics must be defined - call phase.dynamics()"
        )

    num_states, num_controls = phase_def.get_variable_counts()
    if num_states == 0:
        raise ConfigurationError(f"Phase {phase_id} must have at least one state variable")


def _validate_multiphase_problem_ready_for_solving(problem: ProblemProtocol) -> None:
    if not hasattr(problem, "_phases") or not problem._phases:
        raise ConfigurationError("Problem must have at least one phase - use problem.phase()")

    total_states, total_controls, num_static_params = problem._get_total_variable_counts()
    _validate_problem_dimensions(
        total_states, total_controls, num_static_params, "multiphase problem"
    )

    for phase_id in problem._get_phase_ids():
        _validate_phase_configuration(problem, phase_id)

    if (
        not hasattr(problem, "_multiphase_state")
        or problem._multiphase_state.objective_expression is None
    ):
        raise ConfigurationError("Problem must have objective - call problem.minimize()")

    problem.validate_multiphase_configuration()


# =========================
# INITIAL GUESS VALIDATION
# =========================


def _validate_integral_values(
    integrals: float | NumericArrayLike | None, num_integrals: int
) -> None:
    if integrals is None:
        return

    if num_integrals == 1:
        if not isinstance(integrals, int | float):
            raise DataIntegrityError(f"Single integral must be scalar, got {type(integrals)}")
        _validate_array_numerical_integrity(np.array([integrals]), "integral value")
    elif num_integrals > 1:
        if isinstance(integrals, int | float):
            raise DataIntegrityError(f"Multiple integrals ({num_integrals}) need array, got scalar")

        integrals_array = np.array(integrals, dtype=np.float64)
        if integrals_array.size != num_integrals:
            raise DataIntegrityError(
                f"Integral count mismatch: expected {num_integrals}, got {integrals_array.size}"
            )
        _validate_array_numerical_integrity(integrals_array, "integral values")


# ============================
# ADAPTIVE SOLVER VALIDATION
# ============================


def _validate_adaptive_solver_parameters(
    error_tolerance: float,
    max_iterations: int,
    min_polynomial_degree: int,
    max_polynomial_degree: int,
) -> None:
    _validate_positive_number(error_tolerance, "error tolerance")
    _validate_positive_integer(max_iterations, "max iterations")
    _validate_positive_integer(min_polynomial_degree, "min polynomial degree")
    _validate_positive_integer(max_polynomial_degree, "max polynomial degree")

    if max_polynomial_degree < min_polynomial_degree:
        raise ConfigurationError(
            f"Max degree ({max_polynomial_degree}) < min degree ({min_polynomial_degree})"
        )


# ===========================
# DYNAMICS OUTPUT VALIDATION
# ===========================


def _validate_dynamics_output(output: Any, num_states: int) -> ca.MX:
    if output is None:
        raise DataIntegrityError("Dynamics function returned None", "Dynamics evaluation error")

    if isinstance(output, ca.MX):
        if output.shape[0] == num_states and output.shape[1] == 1:
            return output
        elif output.shape[0] == 1 and output.shape[1] == num_states:
            return output.T
        elif output.shape[0] == num_states:
            return output
        else:
            raise DataIntegrityError(
                f"Dynamics ca.MX shape mismatch: got {output.shape}, expected ({num_states}, 1)"
            )

    if isinstance(output, ca.DM):
        result = ca.MX(output)
        if result.shape[0] == 1 and num_states > 1:
            result = result.T
        return result

    if isinstance(output, Sequence):
        return _validate_dynamics_output(list(output), num_states)

    raise DataIntegrityError(
        f"Unsupported dynamics output type: {type(output)}", "Dynamics type error"
    )


# ============================================================================
# UTILITY FUNCTIONS FOR CASADI INTEGRATION
# ============================================================================


def _set_integral_guess_values(
    opti: ca.Opti, integral_vars: ca.MX, guess: float | FloatArray | None, num_integrals: int
) -> None:
    if guess is None:
        return

    if num_integrals == 1:
        if isinstance(guess, int | float):
            opti.set_initial(integral_vars, float(guess))
        else:
            raise ConfigurationError(f"Single integral needs scalar guess, got {type(guess)}")
    elif num_integrals > 1:
        guess_array = np.array(guess, dtype=np.float64)
        opti.set_initial(integral_vars, guess_array.flatten())
