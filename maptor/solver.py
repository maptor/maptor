import logging
from typing import cast

from maptor.direct_solver import _solve_multiphase_radau_collocation
from maptor.input_validation import (
    _validate_adaptive_solver_parameters,
    _validate_multiphase_problem_ready_for_solving,
)
from maptor.mtor_types import (
    ODESolverCallable,
    OptimalControlSolution,
    ProblemProtocol,
)
from maptor.problem import Problem
from maptor.solution import Solution
from maptor.utils.constants import (
    DEFAULT_ADAPTIVE_ERROR_TOLERANCE,
    DEFAULT_ADAPTIVE_MAX_ITERATIONS,
    DEFAULT_ERROR_SIM_POINTS,
    DEFAULT_MAX_POLYNOMIAL_DEGREE,
    DEFAULT_MIN_POLYNOMIAL_DEGREE,
    DEFAULT_ODE_ATOL_FACTOR,
    DEFAULT_ODE_MAX_STEP,
    DEFAULT_ODE_METHOD,
    DEFAULT_ODE_RTOL,
)


logger = logging.getLogger(__name__)


def solve_fixed_mesh(
    problem: Problem,
    nlp_options: dict[str, object] | None = None,
    show_summary: bool = True,
) -> Solution:
    """
    Solve optimal control problem using fixed pseudospectral meshes.

    Solves the problem using the exact mesh configuration specified in each
    phase without adaptive refinement. Provides direct control over mesh
    discretization for computational efficiency or specific accuracy requirements.

    Args:
        problem: Configured Problem instance with mesh, dynamics, and objective
        nlp_options: IPOPT solver options with full customization:

            - Any IPOPT option as "ipopt.option_name": value

        show_summary: Whether to display solution summary (default: True)

    Returns:
        Solution: Optimization results with trajectories and solver diagnostics

    Examples:
        Basic usage:

        >>> solution = mtor.solve_fixed_mesh(problem)

        Custom solver options:

        >>> solution = mtor.solve_fixed_mesh(
        ...     problem,
        ...     nlp_options={
        ...         "ipopt.print_level": 5,       # Verbose output
        ...         "ipopt.max_iter": 1000,       # Iteration limit
        ...         "ipopt.tol": 1e-6             # Tolerance
        ...     }
        ... )

        High-accuracy solver settings:

        >>> solution = mtor.solve_fixed_mesh(
        ...     problem,
        ...     nlp_options={
        ...         "ipopt.tol": 1e-10,
        ...         "ipopt.constr_viol_tol": 1e-9,
        ...         "ipopt.hessian_approximation": "exact"
        ...     }
        ... )

        Linear solver options:

        >>> solution = mtor.solve_fixed_mesh(
        ...     problem,
        ...     nlp_options={
        ...         "ipopt.linear_solver": "mumps",
        ...         "ipopt.mumps_mem_percent": 50000
        ...     }
        ... )

        Silent solving:

        >>> solution = mtor.solve_fixed_mesh(
        ...     problem,
        ...     nlp_options={"ipopt.print_level": 0},
        ...     show_summary=False
        ... )
    """
    logger.info("Starting multiphase fixed-mesh solve: problem='%s'", problem.name)

    if logger.isEnabledFor(logging.DEBUG):
        phase_ids = problem._get_phase_ids()
        total_states, total_controls, num_static_params = problem._get_total_variable_counts()
        logger.debug(
            "Problem dimensions: phases=%d, total_states=%d, total_controls=%d, static_params=%d",
            len(phase_ids),
            total_states,
            total_controls,
            num_static_params,
        )

    _validate_multiphase_problem_ready_for_solving(cast(ProblemProtocol, problem))

    problem.solver_options = nlp_options or {}
    logger.debug("NLP solver options: %s", problem.solver_options)

    protocol_problem = cast(ProblemProtocol, problem)
    solution_data: OptimalControlSolution = _solve_multiphase_radau_collocation(protocol_problem)

    if solution_data.success:
        logger.info(
            "Fixed-mesh solve completed successfully: objective=%.6e",
            solution_data.objective or 0.0,
        )
    else:
        logger.warning("Fixed-mesh solve failed: %s", solution_data.message)

    return Solution(solution_data, protocol_problem, auto_summary=show_summary)


def solve_adaptive(
    problem: Problem,
    error_tolerance: float = DEFAULT_ADAPTIVE_ERROR_TOLERANCE,
    max_iterations: int = DEFAULT_ADAPTIVE_MAX_ITERATIONS,
    min_polynomial_degree: int = DEFAULT_MIN_POLYNOMIAL_DEGREE,
    max_polynomial_degree: int = DEFAULT_MAX_POLYNOMIAL_DEGREE,
    ode_solver_tolerance: float = DEFAULT_ODE_RTOL,
    ode_method: str = DEFAULT_ODE_METHOD,
    ode_max_step: float | None = DEFAULT_ODE_MAX_STEP,
    ode_atol_factor: float = DEFAULT_ODE_ATOL_FACTOR,
    num_error_sim_points: int = DEFAULT_ERROR_SIM_POINTS,
    ode_solver: ODESolverCallable | None = None,
    nlp_options: dict[str, object] | None = None,
    show_summary: bool = True,
) -> Solution:
    """
    Solve optimal control problem using adaptive mesh refinement.

    Automatically refines mesh until target error tolerance is achieved across
    all phases. Provides high-accuracy solutions with computational efficiency
    through adaptive polynomial degree and interval refinement.

    Args:
        problem: Configured Problem instance with initial mesh configurations

        error_tolerance: Target relative error tolerance with ranges:
            - 1e-3: Coarse accuracy, fast solving
            - 1e-6: Standard accuracy (default)
            - 1e-9: High accuracy, slower convergence

        max_iterations: Maximum refinement iterations:
            - 5-10: Standard problems
            - 15-25: Complex problems
            - 30+: Very challenging problems (default: 30)

        min_polynomial_degree: Minimum polynomial degree per interval:
            - 3: Fast, lower accuracy (default)
            - 4-5: Balanced accuracy/speed
            - 6+: High accuracy start

        max_polynomial_degree: Maximum polynomial degree per interval:
            - 8-10: Standard limit (default: 10)
            - 12-15: High accuracy problems
            - 20+: Very smooth solutions only

        ode_solver_tolerance: ODE integration tolerance for error estimation:
            - 1e-8: Standard (default)
            - 1e-10: High accuracy error estimation
            - 1e-5: Faster, less accurate

        ode_method: ODE integration method options:
            - "RK45": Runge-Kutta 4(5) (default)
            - "RK23": Runge-Kutta 2(3)
            - "DOP853": Dormand-Prince 8(5,3)
            - "LSODA": Automatic stiff/non-stiff
            - "Radau": Implicit Runge-Kutta

        ode_max_step: Maximum ODE step size:
            - None: Automatic (default)
            - float: Fixed maximum step

        ode_atol_factor: Absolute tolerance factor (atol = rtol * factor):
            - 1e-8 (default)

        num_error_sim_points: Points for error simulation:
            - 30-50: Standard (default: 50)
            - 100+: High-resolution error estimation

        ode_solver: Custom ODE solver function:
            - None: Use scipy.solve_ivp (default)
            - Callable: Custom solver implementation

        nlp_options: IPOPT options for each NLP solve (same as solve_fixed_mesh)

        show_summary: Display solution summary (default: True)

    Returns:
        Solution: High-accuracy adaptive solution with refinement diagnostics

    Examples:
        Basic adaptive solving:

        >>> solution = mtor.solve_adaptive(problem)

        High-accuracy solving:

        >>> solution = mtor.solve_adaptive(
        ...     problem,
        ...     error_tolerance=1e-8,
        ...     max_iterations=20,
        ...     max_polynomial_degree=15
        ... )

        Fast approximate solving:

        >>> solution = mtor.solve_adaptive(
        ...     problem,
        ...     error_tolerance=1e-3,
        ...     max_iterations=5,
        ...     min_polynomial_degree=3,
        ...     max_polynomial_degree=6
        ... )

        Custom ODE solver settings:

        >>> solution = mtor.solve_adaptive(
        ...     problem,
        ...     ode_method="LSODA",
        ...     ode_solver_tolerance=1e-9,
        ...     num_error_sim_points=100
        ... )

        Complex problem settings:

        >>> solution = mtor.solve_adaptive(
        ...     problem,
        ...     error_tolerance=1e-6,
        ...     max_iterations=30,
        ...     min_polynomial_degree=4,
        ...     max_polynomial_degree=12,
        ...     nlp_options={
        ...         "ipopt.max_iter": 3000,
        ...         "ipopt.tol": 1e-8,
        ...         "ipopt.linear_solver": "mumps"
        ...     }
        ... )

        Override initial guess:

        >>> custom_guess = MultiPhaseInitialGuess(...)
        >>> solution = mtor.solve_adaptive(
        ...     problem,
        ...     initial_guess=custom_guess
        ... )

        Silent adaptive solving:

        >>> solution = mtor.solve_adaptive(
        ...     problem,
        ...     nlp_options={"ipopt.print_level": 0},
        ...     show_summary=False
        ... )

        Polynomial degree ranges:

        >>> # Conservative polynomial progression
        >>> solution = mtor.solve_adaptive(
        ...     problem,
        ...     min_polynomial_degree=3,
        ...     max_polynomial_degree=8
        ... )
        >>>
        >>> # Aggressive high-accuracy
        >>> solution = mtor.solve_adaptive(
        ...     problem,
        ...     min_polynomial_degree=6,
        ...     max_polynomial_degree=20
        ... )

        Error tolerance ranges:

        >>> # Quick verification
        >>> solution = mtor.solve_adaptive(problem, error_tolerance=1e-3)
        >>> # Production accuracy
        >>> solution = mtor.solve_adaptive(problem, error_tolerance=1e-6)
        >>> # Research accuracy
        >>> solution = mtor.solve_adaptive(problem, error_tolerance=1e-9)
    """
    logger.info(
        "Starting multiphase adaptive solve: problem='%s', tolerance=%.1e, max_iter=%d",
        problem.name,
        error_tolerance,
        max_iterations,
    )

    logger.debug(
        "Adaptive parameters: poly_degree=[%d,%d], ode_tol=%.1e, sim_points=%d",
        min_polynomial_degree,
        max_polynomial_degree,
        ode_solver_tolerance,
        num_error_sim_points,
    )

    _validate_adaptive_solver_parameters(
        error_tolerance, max_iterations, min_polynomial_degree, max_polynomial_degree
    )
    _validate_multiphase_problem_ready_for_solving(cast(ProblemProtocol, problem))

    problem.solver_options = nlp_options or {}
    protocol_problem = cast(ProblemProtocol, problem)

    if logger.isEnabledFor(logging.DEBUG):
        for phase_id in problem._get_phase_ids():
            phase_def = problem._phases[phase_id]
            if phase_def.mesh_configured:
                logger.debug(
                    "Phase %d initial mesh: degrees=%s, points=%d",
                    phase_id,
                    phase_def.collocation_points_per_interval,
                    len(phase_def.global_normalized_mesh_nodes)
                    if phase_def.global_normalized_mesh_nodes is not None
                    else 0,
                )

    from maptor.adaptive.phs.algorithm import solve_multiphase_phs_adaptive_internal

    solution_data: OptimalControlSolution = solve_multiphase_phs_adaptive_internal(
        problem=protocol_problem,
        error_tolerance=error_tolerance,
        max_iterations=max_iterations,
        min_polynomial_degree=min_polynomial_degree,
        max_polynomial_degree=max_polynomial_degree,
        ode_solver_tolerance=ode_solver_tolerance,
        ode_method=ode_method,
        ode_max_step=ode_max_step,
        ode_atol_factor=ode_atol_factor,
        ode_solver=ode_solver,
        num_error_sim_points=num_error_sim_points,
    )

    if solution_data.success:
        total_intervals = sum(
            len(intervals) for intervals in solution_data.phase_mesh_intervals.values()
        )
        logger.info(
            "Multiphase adaptive solve converged: objective=%.6e, total_intervals=%d",
            solution_data.objective or 0.0,
            total_intervals,
        )
    else:
        logger.warning("Multiphase adaptive solve failed: %s", solution_data.message)

    return Solution(solution_data, protocol_problem, auto_summary=show_summary)
