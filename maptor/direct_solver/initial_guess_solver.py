import logging
from dataclasses import dataclass
from typing import Any

import casadi as ca
import numpy as np

from ..input_validation import _set_integral_guess_values
from ..mtor_types import FloatArray, PhaseID, ProblemProtocol
from .types_solver import _MultiPhaseVariable, _PhaseVariable


logger = logging.getLogger(__name__)


@dataclass
class _PhaseGuessContext:
    opti: ca.Opti
    phase_vars: _PhaseVariable
    phase_id: PhaseID
    num_states: int
    num_controls: int
    num_mesh_intervals: int
    num_integrals: int
    initial_guess: Any


def _apply_state_guesses(context: _PhaseGuessContext, phase_states: list[FloatArray]) -> None:
    # Apply global mesh node states
    for k in range(context.num_mesh_intervals):
        state_guess_k = phase_states[k]
        num_provided_states = state_guess_k.shape[0]

        # Set initial node guess (only once)
        if k == 0:
            initial_var = context.phase_vars.state_at_mesh_nodes[0]
            if num_provided_states == context.num_states:
                context.opti.set_initial(initial_var, state_guess_k[:, 0])
            else:
                logger.info(
                    f"Phase {context.phase_id} initial state guess has {num_provided_states} states, "
                    f"expected {context.num_states}. Using CasADi defaults for missing states."
                )

        # Set terminal node guess
        terminal_var = context.phase_vars.state_at_mesh_nodes[k + 1]
        if num_provided_states == context.num_states:
            context.opti.set_initial(terminal_var, state_guess_k[:, -1])
        else:
            logger.info(
                f"Phase {context.phase_id} terminal state guess has {num_provided_states} states, "
                f"expected {context.num_states}. Using CasADi defaults for missing states."
            )

    # Apply interior state node guesses
    for k in range(context.num_mesh_intervals):
        interior_var = context.phase_vars.interior_variables[k]
        if interior_var is not None:
            state_guess_k = phase_states[k]
            num_provided_states = state_guess_k.shape[0]

            if num_provided_states == context.num_states:
                num_interior_nodes = interior_var.shape[1]
                interior_guess = state_guess_k[:, 1 : 1 + num_interior_nodes]
                context.opti.set_initial(interior_var, interior_guess)
            else:
                logger.info(
                    f"Phase {context.phase_id} interior state guess has {num_provided_states} states, "
                    f"expected {context.num_states}. Using CasADi defaults for missing states."
                )


def _apply_control_guesses(context: _PhaseGuessContext, phase_controls: list[FloatArray]) -> None:
    for k in range(context.num_mesh_intervals):
        control_guess_k = phase_controls[k]
        num_provided_controls = control_guess_k.shape[0]

        if num_provided_controls == context.num_controls:
            context.opti.set_initial(context.phase_vars.control_variables[k], control_guess_k)
        else:
            logger.info(
                f"Phase {context.phase_id} control guess has {num_provided_controls} controls, "
                f"expected {context.num_controls}. Using CasADi defaults for missing controls."
            )


def _apply_integral_guesses(
    context: _PhaseGuessContext, phase_integrals: float | FloatArray
) -> None:
    if context.num_integrals > 0 and context.phase_vars.integral_variables is not None:
        _set_integral_guess_values(
            context.opti,
            context.phase_vars.integral_variables,
            phase_integrals,
            context.num_integrals,
        )


def _apply_phase_guesses_from_phase_definition(
    opti: ca.Opti,
    phase_vars: _PhaseVariable,
    phase_def: Any,  # PhaseDefinition
    problem: ProblemProtocol,
    phase_id: PhaseID,
) -> None:
    num_states, num_controls = problem._get_phase_variable_counts(phase_id)
    num_mesh_intervals = len(phase_def.collocation_points_per_interval)
    num_integrals = phase_def.num_integrals

    # Apply time guesses using existing logic
    if phase_def.guess_initial_time is not None:
        opti.set_initial(phase_vars.initial_time, phase_def.guess_initial_time)

    if phase_def.guess_terminal_time is not None:
        opti.set_initial(phase_vars.terminal_time, phase_def.guess_terminal_time)

    # Apply state guesses using existing logic
    if phase_def.guess_states is not None:
        context = _PhaseGuessContext(
            opti=opti,
            phase_vars=phase_vars,
            phase_id=phase_id,
            num_states=num_states,
            num_controls=num_controls,
            num_mesh_intervals=num_mesh_intervals,
            num_integrals=num_integrals,
            initial_guess=None,
        )
        _apply_state_guesses(context, phase_def.guess_states)

    # Apply control guesses using existing logic
    if phase_def.guess_controls is not None:
        context = _PhaseGuessContext(
            opti=opti,
            phase_vars=phase_vars,
            phase_id=phase_id,
            num_states=num_states,
            num_controls=num_controls,
            num_mesh_intervals=num_mesh_intervals,
            num_integrals=num_integrals,
            initial_guess=None,
        )
        _apply_control_guesses(context, phase_def.guess_controls)

    # Apply integral guesses using existing logic
    if phase_def.guess_integrals is not None:
        context = _PhaseGuessContext(
            opti=opti,
            phase_vars=phase_vars,
            phase_id=phase_id,
            num_states=num_states,
            num_controls=num_controls,
            num_mesh_intervals=num_mesh_intervals,
            num_integrals=num_integrals,
            initial_guess=None,
        )
        _apply_integral_guesses(context, phase_def.guess_integrals)


def _apply_multiphase_initial_guess(
    opti: ca.Opti,
    variables: _MultiPhaseVariable,
    problem: ProblemProtocol,
) -> None:
    # Apply phase-level guesses
    for phase_id in problem._get_phase_ids():
        if phase_id in variables.phase_variables:
            phase_vars = variables.phase_variables[phase_id]
            phase_def = problem._phases[phase_id]
            _apply_phase_guesses_from_phase_definition(
                opti, phase_vars, phase_def, problem, phase_id
            )

    # Apply static parameter guesses
    if (
        problem._multiphase_state.guess_static_parameters is not None
        and variables.static_parameters is not None
    ):
        param_names = problem._static_parameters.parameter_names
        param_guesses_dict = problem._multiphase_state.guess_static_parameters

        ordered_guesses = []
        for name in param_names:
            if name in param_guesses_dict:
                ordered_guesses.append(param_guesses_dict[name])
            else:
                ordered_guesses.append(0.0)

        opti.set_initial(variables.static_parameters, np.array(ordered_guesses, dtype=np.float64))
