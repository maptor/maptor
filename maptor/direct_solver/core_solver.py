import logging
from collections.abc import Callable
from dataclasses import dataclass

import casadi as ca

from ..exceptions import DataIntegrityError, SolutionExtractionError
from ..mtor_types import (
    Constraint,
    FloatArray,
    OptimalControlSolution,
    PhaseID,
    ProblemProtocol,
)
from ..problem.state import PhaseDefinition
from ..radau import RadauBasisComponents, _compute_radau_collocation_components
from ..solution_extraction import _extract_and_format_multiphase_solution
from .constraints_solver import (
    _apply_multiphase_cross_phase_event_constraints,
    _apply_phase_collocation_constraints,
    _apply_phase_path_constraints,
)
from .initial_guess_solver import _apply_multiphase_initial_guess
from .integrals_solver import _apply_phase_integral_constraints, _setup_phase_integrals
from .types_solver import (
    _MultiPhaseVariable,
    _PhaseVariable,
)
from .variables_solver import (
    _setup_multiphase_optimization_variables,
    setup_phase_interval_state_variables,
)


logger = logging.getLogger(__name__)


@dataclass
class _PhaseFunctions:
    dynamics_function: Callable[..., ca.MX]
    path_constraints_function: Callable[..., list[Constraint]] | None
    integral_integrand_function: Callable[..., ca.MX] | None


@dataclass
class _MeshIntervalContext:
    phase_id: PhaseID
    interval_index: int
    num_states: int
    num_colloc_nodes: int
    global_mesh_nodes: FloatArray
    basis_components: RadauBasisComponents

    initial_time_var: ca.MX
    terminal_time_var: ca.MX

    static_parameters_vec: ca.MX | None
    static_parameter_symbols: list[ca.MX] | None

    num_integrals: int
    accumulated_integral_expressions: list[ca.MX]


@dataclass
class _SolverConfiguration:
    opti: ca.Opti
    variables: _MultiPhaseVariable
    phase_endpoint_data: dict[PhaseID, dict[str, ca.MX]]
    problem: ProblemProtocol


def _extract_phase_endpoint_data(
    variables: _MultiPhaseVariable, problem: ProblemProtocol
) -> dict[PhaseID, dict[str, ca.MX]]:
    phase_endpoint_data = {}

    for phase_id, phase_vars in variables.phase_variables.items():
        num_mesh_intervals = len(problem._phases[phase_id].collocation_points_per_interval)

        initial_state = phase_vars.state_at_mesh_nodes[0]
        terminal_state = phase_vars.state_at_mesh_nodes[num_mesh_intervals]

        phase_endpoint_data[phase_id] = {
            "t0": phase_vars.initial_time,
            "tf": phase_vars.terminal_time,
            "x0": initial_state,
            "xf": terminal_state,
            "q": phase_vars.integral_variables,
        }

    return phase_endpoint_data


def _setup_mesh_interval_variables(
    config: _SolverConfiguration,
    phase_vars: _PhaseVariable,
    context: _MeshIntervalContext,
) -> tuple[ca.MX, ca.MX | None]:
    state_at_nodes, interior_nodes_var = setup_phase_interval_state_variables(
        config.opti,
        context.phase_id,
        context.interval_index,
        context.num_states,
        context.num_colloc_nodes,
        phase_vars.state_at_mesh_nodes,
    )

    phase_vars.state_matrices.append(state_at_nodes)
    phase_vars.interior_variables.append(interior_nodes_var)

    return state_at_nodes, interior_nodes_var


def _apply_mesh_interval_constraints(
    config: _SolverConfiguration,
    context: _MeshIntervalContext,
    state_at_nodes: ca.MX,
    control_variables: ca.MX,
    functions: _PhaseFunctions,
) -> None:
    _apply_phase_collocation_constraints(
        config.opti,
        context.phase_id,
        context.interval_index,
        state_at_nodes,
        control_variables,
        context.basis_components,
        context.global_mesh_nodes,
        context.initial_time_var,
        context.terminal_time_var,
        functions.dynamics_function,
        config.problem,
        context.static_parameters_vec,
    )

    if functions.path_constraints_function is not None:
        _apply_phase_path_constraints(
            config.opti,
            context.phase_id,
            context.interval_index,
            state_at_nodes,
            control_variables,
            context.basis_components,
            context.global_mesh_nodes,
            context.initial_time_var,
            context.terminal_time_var,
            functions.path_constraints_function,
            config.problem,
            context.static_parameters_vec,
            context.static_parameter_symbols,
        )


def _setup_mesh_interval_integrals(
    config: _SolverConfiguration,
    context: _MeshIntervalContext,
    state_at_nodes: ca.MX,
    control_variables: ca.MX,
    functions: _PhaseFunctions,
) -> None:
    if context.num_integrals == 0 or functions.integral_integrand_function is None:
        return

    _setup_phase_integrals(
        config.opti,
        context.phase_id,
        context.interval_index,
        state_at_nodes,
        control_variables,
        context.basis_components,
        context.global_mesh_nodes,
        context.initial_time_var,
        context.terminal_time_var,
        functions.integral_integrand_function,
        context.num_integrals,
        context.accumulated_integral_expressions,
        context.static_parameters_vec,
    )


def _process_single_mesh_interval(
    config: _SolverConfiguration,
    phase_vars: _PhaseVariable,
    context: _MeshIntervalContext,
    functions: _PhaseFunctions,
) -> None:
    try:
        state_at_nodes, _ = _setup_mesh_interval_variables(config, phase_vars, context)

        control_variables = phase_vars.control_variables[context.interval_index]

        _apply_mesh_interval_constraints(
            config, context, state_at_nodes, control_variables, functions
        )

        _setup_mesh_interval_integrals(
            config, context, state_at_nodes, control_variables, functions
        )

    except Exception as e:
        if isinstance(e, DataIntegrityError):
            raise
        raise DataIntegrityError(
            f"Failed to process phase {context.phase_id} interval {context.interval_index}: {e}",
            "MAPTOR phase interval setup error",
        ) from e


def _create_mesh_interval_context(
    phase_id: PhaseID,
    interval_index: int,
    phase_def: PhaseDefinition,
    config: _SolverConfiguration,
    accumulated_integral_expressions: list[ca.MX],
) -> _MeshIntervalContext:
    num_states, _ = config.problem._get_phase_variable_counts(phase_id)
    num_colloc_nodes = phase_def.collocation_points_per_interval[interval_index]

    static_parameter_symbols = None
    if config.variables.static_parameters is not None:
        static_parameter_symbols = config.problem._static_parameters.get_ordered_parameter_symbols()

    endpoint_data = config.phase_endpoint_data[phase_id]

    if phase_def.global_normalized_mesh_nodes is None:
        raise DataIntegrityError(
            f"Phase {phase_id} mesh not configured", "Mesh configuration error"
        )

    return _MeshIntervalContext(
        phase_id=phase_id,
        interval_index=interval_index,
        num_states=num_states,
        num_colloc_nodes=num_colloc_nodes,
        global_mesh_nodes=phase_def.global_normalized_mesh_nodes,
        basis_components=_compute_radau_collocation_components(num_colloc_nodes),
        initial_time_var=endpoint_data["t0"],
        terminal_time_var=endpoint_data["tf"],
        static_parameters_vec=config.variables.static_parameters,
        static_parameter_symbols=static_parameter_symbols,
        num_integrals=phase_def.num_integrals,
        accumulated_integral_expressions=accumulated_integral_expressions,
    )


def _extract_phase_functions(config: _SolverConfiguration, phase_id: PhaseID) -> _PhaseFunctions:
    return _PhaseFunctions(
        dynamics_function=config.problem._get_phase_dynamics_function(phase_id),
        path_constraints_function=config.problem._get_phase_path_constraints_function(phase_id),
        integral_integrand_function=config.problem._get_phase_integrand_function(phase_id),
    )


def _process_phase_mesh_intervals(
    config: _SolverConfiguration,
    phase_id: PhaseID,
    phase_vars: _PhaseVariable,
) -> None:
    phase_def = config.problem._phases[phase_id]
    num_mesh_intervals = len(phase_def.collocation_points_per_interval)

    accumulated_integral_expressions = (
        [ca.MX(0) for _ in range(phase_def.num_integrals)] if phase_def.num_integrals > 0 else []
    )

    functions = _extract_phase_functions(config, phase_id)

    for interval_index in range(num_mesh_intervals):
        context = _create_mesh_interval_context(
            phase_id, interval_index, phase_def, config, accumulated_integral_expressions
        )
        _process_single_mesh_interval(config, phase_vars, context, functions)

    if phase_def.num_integrals > 0 and phase_vars.integral_variables is not None:
        _apply_phase_integral_constraints(
            config.opti,
            phase_vars.integral_variables,
            accumulated_integral_expressions,
            phase_def.num_integrals,
            phase_id,
        )


def _process_all_phases(config: _SolverConfiguration) -> None:
    for phase_id in config.problem._get_phase_ids():
        if phase_id not in config.variables.phase_variables:
            continue

        phase_vars = config.variables.phase_variables[phase_id]
        _process_phase_mesh_intervals(config, phase_id, phase_vars)


def _setup_objective_and_constraints(config: _SolverConfiguration) -> None:
    objective_function = config.problem._get_objective_function()

    try:
        objective_value = objective_function(
            config.phase_endpoint_data,
            config.variables.static_parameters,
        )
        config.opti.minimize(objective_value)

    except Exception as e:
        raise DataIntegrityError(
            f"Failed to set up multiphase objective function: {e}",
            "Multiphase objective function evaluation error",
        ) from e

    _apply_multiphase_cross_phase_event_constraints(
        config.opti,
        config.phase_endpoint_data,
        config.variables.static_parameters,
        config.problem,
    )


def _configure_solver(config: _SolverConfiguration) -> None:
    solver_options_to_use = config.problem.solver_options or {}

    try:
        config.opti.solver("ipopt", solver_options_to_use)
    except Exception as e:
        raise DataIntegrityError(
            f"Failed to configure solver: {e}", "Invalid solver options"
        ) from e

    config.opti.multiphase_variables_reference = config.variables

    objective_function = config.problem._get_objective_function()
    objective_expression = objective_function(
        config.phase_endpoint_data, config.variables.static_parameters
    )
    config.opti.multiphase_objective_expression_reference = objective_expression


def _create_solver_configuration(problem: ProblemProtocol) -> _SolverConfiguration:
    try:
        opti = ca.Opti()
        variables = _setup_multiphase_optimization_variables(opti, problem)
        phase_endpoint_data = _extract_phase_endpoint_data(variables, problem)

        return _SolverConfiguration(
            opti=opti,
            variables=variables,
            phase_endpoint_data=phase_endpoint_data,
            problem=problem,
        )

    except Exception as e:
        if isinstance(e, DataIntegrityError):
            raise
        raise DataIntegrityError(
            f"Failed to create solver configuration: {e}",
            "MAPTOR solver setup error",
        ) from e


def _execute_solve(config: _SolverConfiguration) -> OptimalControlSolution:
    try:
        solver_solution = config.opti.solve()
        logger.debug("Multiphase NLP solver completed successfully")

        try:
            solution_obj = _extract_and_format_multiphase_solution(
                solver_solution, config.opti, config.problem
            )
            logger.debug("Multiphase solution extraction completed")
            return solution_obj

        except Exception as e:
            logger.error("Multiphase solution extraction failed: %s", str(e))
            raise SolutionExtractionError(
                f"Failed to extract multiphase solution: {e}",
                "MAPTOR multiphase solution processing error",
            ) from e

    except RuntimeError as e:
        logger.warning("Multiphase NLP solver failed: %s", str(e))
        return _handle_solver_failure(config, e)


def _handle_solver_failure(
    config: _SolverConfiguration, error: RuntimeError
) -> OptimalControlSolution:
    try:
        solution_obj = _extract_and_format_multiphase_solution(None, config.opti, config.problem)
    except Exception as extract_error:
        logger.error(
            "Multiphase solution extraction failed after solver failure: %s",
            str(extract_error),
        )
        raise SolutionExtractionError(
            f"Failed to extract multiphase solution after solver failure: {extract_error}",
            "multiphase solution extraction error",
        ) from extract_error

    solution_obj.success = False
    solution_obj.message = f"Multiphase solver runtime error: {error}"

    _extract_debug_values(config, solution_obj)
    return solution_obj


def _extract_debug_values(
    config: _SolverConfiguration, solution_obj: OptimalControlSolution
) -> None:
    try:
        if hasattr(config.opti, "debug") and config.opti.debug is not None:
            for phase_id, phase_vars in config.variables.phase_variables.items():
                try:
                    solution_obj.phase_initial_times[phase_id] = float(
                        config.opti.debug.value(phase_vars.initial_time)
                    )
                    solution_obj.phase_terminal_times[phase_id] = float(
                        config.opti.debug.value(phase_vars.terminal_time)
                    )
                except Exception as e:
                    logger.debug(f"Could not extract debug values for phase {phase_id}: {e}")
            logger.debug("Retrieved debug values from failed multiphase solve")
    except Exception as e:
        logger.debug(f"Could not extract debug values from failed multiphase solve: {e}")


def _solve_multiphase_radau_collocation(problem: ProblemProtocol) -> OptimalControlSolution:
    logger.debug("Starting multiphase Radau collocation solver")

    phase_ids = problem._get_phase_ids()
    total_states, total_controls, num_static_params = problem._get_total_variable_counts()

    logger.debug(
        "Multiphase problem structure: phases=%d, total_states=%d, total_controls=%d, static_params=%d",
        len(phase_ids),
        total_states,
        total_controls,
        num_static_params,
    )

    config = _create_solver_configuration(problem)

    logger.debug("Processing %d phases", len(phase_ids))
    _process_all_phases(config)

    logger.debug("Setting up multiphase objective and cross-phase constraints")
    _setup_objective_and_constraints(config)

    logger.debug("Applying multiphase initial guess")
    _apply_multiphase_initial_guess(config.opti, config.variables, problem)

    logger.debug("Configuring NLP solver")
    _configure_solver(config)

    logger.debug("Executing multiphase NLP solve")
    return _execute_solve(config)
