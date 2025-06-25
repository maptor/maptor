import logging
from collections.abc import Sequence
from typing import Any

import casadi as ca

from maptor.exceptions import ConfigurationError

from ..input_validation import (
    _validate_complete_dynamics,
    _validate_constraint_input_format,
    _validate_string_not_empty,
)
from ..mtor_types import NumericArrayLike, PhaseID
from . import constraints_problem, initial_guess_problem, mesh, solver_interface, variables_problem
from .constraints_problem import (
    _get_cross_phase_event_constraints_function,
    _get_phase_path_constraints_function,
)
from .state import BoundaryInput, ConstraintInput, FixedInput, MultiPhaseVariableState
from .variables_problem import StateVariableImpl, TimeVariableImpl


logger = logging.getLogger(__name__)


def _validate_phase_exists(phases: dict[PhaseID, Any], phase_id: PhaseID) -> None:
    if phase_id not in phases:
        raise ConfigurationError(f"Phase {phase_id} does not exist")


def _validate_constraint_inputs(name: str, boundary: ConstraintInput, context: str) -> None:
    _validate_string_not_empty(name, f"{context} name")
    _validate_constraint_input_format(boundary, f"{context} '{name}' boundary")


def _log_constraint_addition(
    constraint_count: int, phase_id: PhaseID, constraint_type: str
) -> None:
    logger.debug(
        "Added %d %s constraint(s) to phase %d", constraint_count, constraint_type, phase_id
    )


def _validate_constraint_expressions_not_empty(
    constraint_expressions: tuple, phase_id: PhaseID, constraint_type: str
) -> None:
    if not constraint_expressions:
        raise ConfigurationError(
            f"Phase {phase_id} {constraint_type}_constraints() requires at least one constraint expression"
        )


def _process_symbolic_time_constraints(
    phase_def: Any, phase_id: PhaseID, cross_phase_constraints: list[ca.MX]
) -> None:
    if phase_def.t0_constraint.is_symbolic():
        if (
            phase_def.sym_time_initial is None
            or phase_def.t0_constraint.symbolic_expression is None
        ):
            raise ConfigurationError(
                f"Phase {phase_id} has an undefined symbolic time initial expression."
            )
        constraint_expr = phase_def.sym_time_initial - phase_def.t0_constraint.symbolic_expression
        cross_phase_constraints.append(constraint_expr)
        logger.debug(f"Added automatic time initial constraint for phase {phase_id}")

    if phase_def.tf_constraint.is_symbolic():
        if phase_def.sym_time_final is None or phase_def.tf_constraint.symbolic_expression is None:
            raise ConfigurationError(
                f"Phase {phase_id} has an undefined symbolic time final expression."
            )
        constraint_expr = phase_def.sym_time_final - phase_def.tf_constraint.symbolic_expression
        cross_phase_constraints.append(constraint_expr)
        logger.debug(f"Added automatic time final constraint for phase {phase_id}")


def _process_single_symbolic_boundary_constraint(
    var_name: str,
    constraint_type: str,
    symbolic_expr: ca.MX,
    phase_def: Any,
    phase_id: PhaseID,
    cross_phase_constraints: list[ca.MX],
) -> None:
    state_index = phase_def.state_name_to_index[var_name]

    if constraint_type == "initial":
        state_initial_sym = phase_def._get_ordered_state_initial_symbols()[state_index]
        constraint_expr = state_initial_sym - symbolic_expr
        cross_phase_constraints.append(constraint_expr)
        logger.debug(f"Added automatic initial constraint for phase {phase_id} state '{var_name}'")

    elif constraint_type == "final":
        state_final_sym = phase_def._get_ordered_state_final_symbols()[state_index]
        constraint_expr = state_final_sym - symbolic_expr
        cross_phase_constraints.append(constraint_expr)
        logger.debug(f"Added automatic final constraint for phase {phase_id} state '{var_name}'")

    elif constraint_type == "boundary":
        state_initial_sym = phase_def._get_ordered_state_initial_symbols()[state_index]
        state_final_sym = phase_def._get_ordered_state_final_symbols()[state_index]

        initial_constraint = state_initial_sym - symbolic_expr
        final_constraint = state_final_sym - symbolic_expr

        cross_phase_constraints.extend([initial_constraint, final_constraint])
        logger.debug(
            f"Added automatic boundary constraints for phase {phase_id} state '{var_name}'"
        )


def _process_symbolic_state_constraints(
    phase_def: Any, phase_id: PhaseID, cross_phase_constraints: list[ca.MX]
) -> None:
    for var_name, constraint_type, symbolic_expr in phase_def.symbolic_boundary_constraints:
        _process_single_symbolic_boundary_constraint(
            var_name, constraint_type, symbolic_expr, phase_def, phase_id, cross_phase_constraints
        )


def _process_symbolic_parameter_constraints(
    multiphase_state: MultiPhaseVariableState, cross_phase_constraints: list[ca.MX]
) -> None:
    static_params = multiphase_state.static_parameters
    for param_name, _, symbolic_expr in static_params.symbolic_boundary_constraints:
        param_index = static_params.parameter_name_to_index[param_name]
        param_symbols = static_params.get_ordered_parameter_symbols()
        param_symbol = param_symbols[param_index]

        constraint_expr = param_symbol - symbolic_expr
        cross_phase_constraints.append(constraint_expr)
        logger.debug(f"Added automatic parameter constraint for '{param_name}'")


def _validate_phase_requirements(phases: dict[PhaseID, Any]) -> None:
    if not phases:
        raise ConfigurationError("Problem must have at least one phase defined")

    for phase_id, phase_def in phases.items():
        if not phase_def.dynamics_expressions:
            raise ConfigurationError(f"Phase {phase_id} must have dynamics defined")
        _validate_complete_dynamics(phase_def, phase_id)
        if not phase_def.mesh_configured:
            raise ConfigurationError(
                f"Phase {phase_id} must have mesh configured before validation. "
                f"Call phase.mesh(polynomial_degrees, mesh_points) first."
            )


def _process_symbolic_constraints_for_all_phases(
    multiphase_state: MultiPhaseVariableState, cross_phase_constraints: list[ca.MX]
) -> None:
    logger.debug("Processing symbolic boundary constraints for automatic cross-phase linking")

    for phase_id, phase_def in multiphase_state.phases.items():
        _process_symbolic_time_constraints(phase_def, phase_id, cross_phase_constraints)
        _process_symbolic_state_constraints(phase_def, phase_id, cross_phase_constraints)

    _process_symbolic_parameter_constraints(multiphase_state, cross_phase_constraints)


def _build_all_phase_functions(multiphase_state: MultiPhaseVariableState) -> None:
    static_parameter_symbols = multiphase_state.static_parameters.get_ordered_parameter_symbols()

    logger.debug("Building functions for all phases")
    for phase_id, phase_def in multiphase_state.phases.items():
        if phase_def._functions_built:
            continue

        logger.debug(f"Building functions for phase {phase_id}")

        phase_def._dynamics_function, phase_def._numerical_dynamics_function = (
            solver_interface._build_unified_phase_dynamics_functions(
                phase_def, static_parameter_symbols
            )
        )

        phase_def._integrand_function = solver_interface._build_phase_integrand_function(
            phase_def, static_parameter_symbols
        )

        phase_def._path_constraints_function = _get_phase_path_constraints_function(phase_def)

        phase_def._functions_built = True

    if not multiphase_state._functions_built:
        logger.debug("Building multiphase objective function")
        multiphase_state._objective_function = (
            solver_interface._build_multiphase_objective_function(multiphase_state)
        )

        multiphase_state._cross_phase_constraints_function = (
            _get_cross_phase_event_constraints_function(multiphase_state)
        )

        multiphase_state._functions_built = True


class Phase:
    """
    Single phase definition for multiphase optimal control problems.

    A Phase represents one segment of a multiphase trajectory with its own time
    domain, state variables, control inputs, dynamics, and constraints. Phases
    can be linked together through symbolic constraints to create complex
    multiphase missions.

    The Phase class provides a fluent interface for defining:

    - Time variables with boundary conditions
    - State variables with initial, final, and path constraints
    - Control variables with bounds
    - System dynamics as differential equations
    - Integral cost terms and constraints
    - Path constraints applied throughout the phase
    - Event constraints at phase boundaries
    - Mesh discretization for numerical solution

    Note:
        Phase objects are created through Problem.set_phase() and should not
        be instantiated directly.

    Examples:
        Basic single-phase problem setup:

        >>> problem = mtor.Problem("Rocket Ascent")
        >>> phase = problem.set_phase(1)
        >>>
        >>> # Define time and variables
        >>> t = phase.time(initial=0, final=10)
        >>> h = phase.state("altitude", initial=0, final=1000)
        >>> v = phase.state("velocity", initial=0)
        >>> T = phase.control("thrust", boundary=(0, 2000))
        >>>
        >>> # Set dynamics
        >>> phase.dynamics({h: v, v: T/1000 - 9.81})
        >>>
        >>> # Add constraints and mesh
        >>> phase.path_constraints(h >= 0, T <= 1500)
        >>> phase.mesh([5, 5], [-1, 0, 1])

        Multiphase trajectory with automatic linking:

        >>> # Phase 1: Boost
        >>> p1 = problem.set_phase(1)
        >>> t1 = p1.time(initial=0, final=120)
        >>> h1 = p1.state("altitude", initial=0)
        >>> v1 = p1.state("velocity", initial=0)
        >>> # ... dynamics and constraints
        >>>
        >>> # Phase 2: Coast (automatically linked)
        >>> p2 = problem.set_phase(2)
        >>> t2 = p2.time(initial=t1.final, final=300)  # Continuous time
        >>> h2 = p2.state("altitude", initial=h1.final)  # Continuous altitude
        >>> v2 = p2.state("velocity", initial=v1.final)  # Continuous velocity
        >>> # ... dynamics for coast phase
    """

    def __init__(self, problem: "Problem", phase_id: PhaseID) -> None:
        self.problem = problem
        self.phase_id = phase_id
        self._phase_def = self.problem._multiphase_state.set_phase(self.phase_id)

    def time(
        self, initial: ConstraintInput = 0.0, final: ConstraintInput = None
    ) -> TimeVariableImpl:
        """
        Define the time variable for this phase with boundary conditions.

        Creates the time coordinate for this phase with comprehensive constraint
        specification. Supports fixed times, bounded ranges, and symbolic expressions
        for multiphase trajectory continuity.

        Args:
            initial: Initial time constraint with full constraint syntax:

                - float: Fixed initial time (e.g., 0.0)
                - (lower, upper): Bounded initial time range (e.g., (0, 10))
                - (None, upper): Upper bounded only (e.g., (None, 5))
                - (lower, None): Lower bounded only (e.g., (2, None))
                - ca.MX: Symbolic expression for phase linking
                - None: Unconstrained initial time (optimization variable)

            final: Final time constraint with full constraint syntax:

                - float: Fixed final time (e.g., 100.0)
                - (lower, upper): Bounded final time range (e.g., (90, 110))
                - (None, upper): Upper bounded only (e.g., (None, 200))
                - (lower, None): Lower bounded only (e.g., (50, None))
                - ca.MX: Symbolic expression for phase linking
                - None: Free final time (optimization variable)

        Returns:
            TimeVariableImpl: Time variable with .initial and .final properties

        Examples:
            Fixed values:

            >>> t = phase.time(initial=0.0, final=10.0)

            Ranges:

            >>> t = phase.time(initial=(0, 5), final=(8, 12))

            Single-sided bounds:

            >>> t = phase.time(initial=(None, 5), final=(10, None))

            Free variables:

            >>> t = phase.time()  # Both free

            Symbolic linking:

            >>> t2 = phase2.time(initial=t1.final)
        """
        return variables_problem.create_phase_time_variable(self._phase_def, initial, final)

    def state(
        self,
        name: str,
        initial: ConstraintInput = None,
        final: ConstraintInput = None,
        boundary: BoundaryInput = None,
    ) -> StateVariableImpl:
        """
        Define a state variable with comprehensive constraint specification.

        Creates a state variable with exhaustive constraint capabilities including
        boundary conditions, path bounds, and symbolic multiphase linking. All
        constraint types support the full constraint syntax.

        Args:
            name: Unique state variable name within this phase
            initial: Initial state constraint with full syntax:

                - float: Fixed initial value (e.g., 0.0)
                - (lower, upper): Bounded initial range (e.g., (0, 10))
                - (None, upper): Upper bounded only (e.g., (None, 100))
                - (lower, None): Lower bounded only (e.g., (50, None))
                - ca.MX: Symbolic expression for multiphase continuity
                - None: Unconstrained initial state

            final: Final state constraint with full syntax:

                - float: Fixed final value (e.g., 1000.0)
                - (lower, upper): Bounded final range (e.g., (990, 1010))
                - (None, upper): Upper bounded only (e.g., (None, 1200))
                - (lower, None): Lower bounded only (e.g., (800, None))
                - ca.MX: Symbolic expression for multiphase continuity
                - None: Unconstrained final state

            boundary: Path constraint applied throughout trajectory:

                - (lower, upper): State bounds throughout (e.g., (0, 1000))
                - (None, upper): Upper path bound only (e.g., (None, 500))
                - (lower, None): Lower path bound only (e.g., (0, None))
                - None: No path bounds

            Returns:
                StateVariableImpl: State variable with .initial and .final properties

            Examples:
                Fixed values:

                >>> altitude = phase.state("altitude", initial=0.0, final=1000.0)

                Ranges:

                >>> position = phase.state("position", initial=(0, 5), final=(95, 105))

                Single-sided bounds:

                >>> velocity = phase.state("velocity", initial=0, final=(None, 200))

                Path bounds:

                >>> mass = phase.state("mass", boundary=(100, 1000))

                All constraint types:

                >>> state = phase.state("x", initial=(0, 10), final=(90, 110), boundary=(0, None))

                Symbolic linking:

                >>> h2 = phase2.state("altitude", initial=h1.final)

                Unconstrained:

                >>> free_state = phase.state("free_variable")
        """
        return variables_problem._create_phase_state_variable(
            self._phase_def, name, initial, final, boundary
        )

    def control(self, name: str, boundary: BoundaryInput = None) -> ca.MX:
        """
        Define a control variable with comprehensive bound specification.

        Creates a control input with exhaustive constraint capabilities. Control
        variables represent actuator commands that can be optimized subject to
        physical or design limitations.

        Args:
            name: Unique control variable name within this phase
            boundary: Control bounds with constraint syntax:

                - (lower, upper): Symmetric/asymmetric bounds (e.g., (-50, 100))
                - (None, upper): Upper bound only (e.g., (None, 1000))
                - (lower, None): Lower bound only (e.g., (0, None))
                - None: Unconstrained control

        Returns:
            ca.MX: Control variable for use in dynamics and cost functions

        Examples:
            Bounded control:

            >>> thrust = phase.control("thrust", boundary=(0, 2000))

            Single-sided bounds:

            >>> power = phase.control("power", boundary=(0, None))
            >>> brake = phase.control("brake", boundary=(None, 100))

            Unconstrained:

            >>> free_control = phase.control("force")
        """
        return variables_problem.create_phase_control_variable(self._phase_def, name, boundary)

    def dynamics(
        self,
        dynamics_dict: dict[ca.MX | StateVariableImpl, ca.MX | float | int | StateVariableImpl],
    ) -> None:
        """
        Define differential equations with comprehensive expression support.

        Specifies system ODEs describing state evolution. Dynamics expressions
        can involve states, controls, time, parameters, and arbitrary mathematical
        relationships. Must provide dynamics for all states in the phase.

        Args:
            dynamics_dict: Maps state variables to time derivatives (dx/dt).
                Keys: State variables from state() method
                Values: Expressions for derivatives using states, controls, parameters

        Examples:
            Basic dynamics:

            >>> pos = phase.state("position")
            >>> vel = phase.state("velocity")
            >>> thrust = phase.control("thrust")
            >>> phase.dynamics({pos: vel, vel: thrust - 0.1*vel})

            With mass:

            >>> mass = phase.state("mass", initial=1000)
            >>> phase.dynamics({
            ...     pos: vel,
            ...     vel: thrust/mass - 9.81,
            ...     mass: -thrust * 0.001
            ... })

            With parameters:

            >>> drag_coeff = problem.parameter("drag")
            >>> phase.dynamics({pos: vel, vel: thrust - drag_coeff*vel**2})

            Multiple states/controls:

            >>> phase.dynamics({
            ...     x: vx,
            ...     y: vy,
            ...     vx: fx,
            ...     vy: fy - 9.81
            ... })
        """
        self._phase_def._functions_built = False
        variables_problem._set_phase_dynamics(self._phase_def, dynamics_dict)
        logger.info(
            "Dynamics defined for phase %d with %d state variables",
            self.phase_id,
            len(dynamics_dict),
        )

    def add_integral(self, integrand_expr: ca.MX | float | int) -> ca.MX:
        """
        Add integral term with comprehensive integrand expression support.

        Creates integral variables for cost functions, constraint integrals, and
        accumulated quantities. Supports arbitrary expressions involving states,
        controls, time, and parameters.

        Args:
            integrand_expr: Expression to integrate over phase duration.
                Can involve any combination of states, controls, time, parameters.

        Returns:
            ca.MX: Integral variable for use in objectives and constraints

        Examples:
            Quadratic cost:

            >>> cost = phase.add_integral(x**2 + u**2)
            >>> problem.minimize(cost)

            Resource consumption:

            >>> fuel = phase.add_integral(thrust * 0.001)

            Distance traveled:

            >>> distance = phase.add_integral(velocity)

            Multiple integrals:

            >>> energy = phase.add_integral(thrust**2)
            >>> fuel = phase.add_integral(thrust * rate)
            >>> problem.minimize(energy + 10*fuel)
        """
        self._phase_def._functions_built = False
        return variables_problem._set_phase_integral(self._phase_def, integrand_expr)

    def path_constraints(self, *constraint_expressions: ca.MX | float | int) -> None:
        r"""
        Add path constraints enforced continuously throughout the trajectory.

        Path constraints are enforced at all collocation points, ensuring conditions
        hold throughout the phase. Use for safety limits, physical bounds, and
        trajectory shaping that cannot be expressed through state boundary parameters.

        Args:
            \*constraint_expressions: Constraint expressions enforced continuously

        Examples:
            State bounds:

            >>> phase.path_constraints(altitude >= 0, velocity <= 250)

            Control bounds:

            >>> phase.path_constraints(thrust >= 0, thrust <= 2000)

            Complex expressions:

            >>> phase.path_constraints((x-50)**2 + (y-50)**2 >= 100)

            Multiple constraints:

            >>> phase.path_constraints(
            ...     altitude >= 0,
            ...     velocity <= 200,
            ...     acceleration <= 20
            ... )
        """
        _validate_constraint_expressions_not_empty(constraint_expressions, self.phase_id, "path")

        self._phase_def._functions_built = False
        for expr in constraint_expressions:
            constraints_problem._add_path_constraint(self._phase_def, expr)

        _log_constraint_addition(len(constraint_expressions), self.phase_id, "path")

    def event_constraints(self, *constraint_expressions: ca.MX | float | int) -> None:
        r"""
        Add event constraints for expressions not representable as state/control bounds.

        **Primary Purpose:** Constraints on integral terms, static parameters,
        cross-phase expressions, and complex mathematical relationships that cannot
        be expressed through state initial/final/boundary or control boundary parameters.

        **When to use event_constraints vs state parameters:**
        - Use state parameters for simple state boundaries: `state("x", final=(90, 110))`
        - Use event_constraints for integrals, parameters, and complex expressions

        Args:
            \*constraint_expressions: Constraint expressions involving:

                - Integral terms from `phase.add_integral()`
                - Static parameters from `problem.parameter()`
                - Cross-phase linking expressions
                - Complex mathematical relationships
                - Multi-variable constraint expressions

        Examples:
            Integral constraints:

            >>> fuel_used = phase.add_integral(thrust * 0.001)
            >>> phase.event_constraints(fuel_used <= 100)

            Parameter constraints:

            >>> mass = problem.parameter("mass", boundary=(100, 1000))
            >>> phase.event_constraints(mass >= 200)

            Cross-phase linking:

            >>> phase2.event_constraints(h2.initial == h1.final)

            Complex expressions:

            >>> phase.event_constraints(x.final**2 + y.final**2 >= 100)
        """
        _validate_constraint_expressions_not_empty(constraint_expressions, self.phase_id, "event")

        for expr in constraint_expressions:
            constraints_problem._add_event_constraint(self.problem._multiphase_state, expr)

        _log_constraint_addition(len(constraint_expressions), self.phase_id, "event")

    def mesh(self, polynomial_degrees: list[int], mesh_points: NumericArrayLike) -> None:
        """
        Configure pseudospectral mesh with comprehensive discretization control.

        Defines mesh discretization for Radau pseudospectral method with precise
        control over polynomial degrees and interval distribution. Critical for
        balancing solution accuracy with computational efficiency.

        Args:
            polynomial_degrees: Polynomial degree for each mesh interval.
                Higher degrees: better accuracy, more computational cost.
                Typical range: 3-12, up to 15 for very smooth solutions.
            mesh_points: Normalized mesh points in [-1, 1] defining interval
                boundaries. Length must equal len(polynomial_degrees) + 1.
                Points automatically scaled to actual phase time domain.

        Examples:
            Uniform mesh:

            >>> phase.mesh([4, 4, 4], [-1, -1/3, 1/3, 1])

            Non-uniform intervals:

            >>> phase.mesh([6, 4], [-1, -0.5, 1])

            High accuracy:

            >>> phase.mesh([10, 10], [-1, 0, 1])

            Single interval:

            >>> phase.mesh([5], [-1, 1])
        """

        logger.info(
            "Setting mesh for phase %d: %d intervals", self.phase_id, len(polynomial_degrees)
        )
        mesh._configure_phase_mesh(self._phase_def, polynomial_degrees, mesh_points)

    def guess(
        self,
        states: Sequence[NumericArrayLike] | None = None,
        controls: Sequence[NumericArrayLike] | None = None,
        initial_time: float | None = None,
        terminal_time: float | None = None,
        integrals: float | NumericArrayLike | None = None,
    ) -> None:
        """
        Provide initial guess for this phase's optimization variables.

        Supplies starting values for NLP solver with phase-specific variable coverage.
        Arrays must match this phase's mesh configuration exactly.

        Args:
            states: State trajectory guess per mesh interval.
                Structure: [interval_arrays]
                Each interval_array: shape (num_states, num_collocation_points)

            controls: Control trajectory guess per mesh interval.
                Structure: [interval_arrays]
                Each interval_array: shape (num_controls, num_mesh_points)

            initial_time: Initial time guess for this phase.

            terminal_time: Final time guess for this phase.

            integrals: Integral value guess for this phase.
                - float: Single integral
                - array: Multiple integrals

        Examples:
            Basic single phase:

            >>> states_guess = [np.array([[0, 1, 2, 3, 4], [0, 0, 1, 2, 3]])]
            >>> controls_guess = [np.array([[1, 1, 1, 1]])]
            >>> phase.guess(states=states_guess, controls=controls_guess, terminal_time=10.0)

            With integrals:

            >>> phase.guess(states=states_guess, integrals=50.0)
        """
        components = []
        if states is not None:
            components.append("states")
        if controls is not None:
            components.append("controls")
        if initial_time is not None:
            components.append("initial_time")
        if terminal_time is not None:
            components.append("terminal_time")
        if integrals is not None:
            components.append("integrals")

        logger.info("Setting initial guess for phase %d: %s", self.phase_id, ", ".join(components))

        # Reuse existing validation logic
        initial_guess_problem._set_phase_initial_guess(
            self._phase_def,
            states=states,
            controls=controls,
            initial_time=initial_time,
            terminal_time=terminal_time,
            integrals=integrals,
        )


class Problem:
    """
    Multiphase optimal control problem definition and configuration interface.

    The Problem class is the main entry point for defining optimal control problems
    in MAPTOR. It supports both single-phase and multiphase trajectory optimization
    with automatic phase linking, static parameter optimization, and comprehensive
    constraint specification.

    Key capabilities:

    - **Multiphase trajectory definition** with automatic continuity
    - **Static parameter optimization** (design variables)
    - **Flexible objective functions** (minimize/maximize any expression)
    - **Initial guess specification** for improved convergence
    - **Solver configuration** and validation
    - **Cross-phase constraints** and event handling

    The Problem follows a builder pattern where you incrementally define phases,
    variables, dynamics, constraints, and objectives before solving.

    Examples:
        Single-phase minimum time problem:

        >>> import maptor as mtor
        >>>
        >>> problem = mtor.Problem("Minimum Time")
        >>> phase = problem.set_phase(1)
        >>>
        >>> # Variables and dynamics
        >>> t = phase.time(initial=0.0)
        >>> x = phase.state("position", initial=0, final=1)
        >>> v = phase.state("velocity", initial=0)
        >>> u = phase.control("force", boundary=(-1, 1))
        >>>
        >>> phase.dynamics({x: v, v: u})
        >>> problem.minimize(t.final)
        >>>
        >>> # Solve
        >>> phase.mesh([5, 5], [-1, 0, 1])
        >>> solution = mtor.solve_fixed_mesh(problem)

        Multiphase rocket trajectory:

        >>> problem = mtor.Problem("Rocket Launch")
        >>>
        >>> # Boost phase
        >>> boost = problem.set_phase(1)
        >>> t1 = boost.time(initial=0, final=120)
        >>> h1 = boost.state("altitude", initial=0)
        >>> v1 = boost.state("velocity", initial=0)
        >>> m1 = boost.state("mass", initial=1000)
        >>> T1 = boost.control("thrust", boundary=(0, 2000))
        >>>
        >>> boost.dynamics({
        ...     h1: v1,
        ...     v1: T1/m1 - 9.81,
        ...     m1: -T1 * 0.001
        ... })
        >>>
        >>> # Coast phase with automatic continuity
        >>> coast = problem.set_phase(2)
        >>> t2 = coast.time(initial=t1.final, final=300)
        >>> h2 = coast.state("altitude", initial=h1.final)
        >>> v2 = coast.state("velocity", initial=v1.final)
        >>> m2 = coast.state("mass", initial=m1.final)
        >>>
        >>> coast.dynamics({h2: v2, v2: -9.81, m2: 0})
        >>>
        >>> # Objective and solve
        >>> problem.minimize(-h2.final)  # Maximize final altitude
        >>> # ... mesh configuration and solve

        Problem with static parameters:

        >>> problem = mtor.Problem("Design Optimization")
        >>>
        >>> # Design parameters to optimize
        >>> engine_mass = problem.parameter("engine_mass", boundary=(50, 200))
        >>> thrust_level = problem.parameter("max_thrust", boundary=(1000, 5000))
        >>>
        >>> # Use parameters in dynamics
        >>> total_mass = vehicle_mass + engine_mass
        >>> phase.dynamics({v: thrust_level/total_mass - 9.81})
        >>>
        >>> # Multi-objective: maximize performance, minimize mass
        >>> performance = altitude.final
        >>> mass_penalty = engine_mass * 10
        >>> problem.minimize(mass_penalty - performance)
    """

    def __init__(self, name: str = "Multiphase Problem") -> None:
        """
        Initialize a new optimal control problem.

        Args:
            name: Descriptive name for the problem (used in logging and output)

        Examples:
            >>> problem = mtor.Problem("Spacecraft Trajectory")
            >>> problem = mtor.Problem("Robot Path Planning")
            >>> problem = mtor.Problem()  # Uses default name
        """
        _validate_string_not_empty(name, "Problem name")
        self.name = name
        logger.debug("Created multiphase problem: '%s'", name)

        self._multiphase_state = MultiPhaseVariableState()
        self.solver_options: dict[str, Any] = {}

    def set_phase(self, phase_id: PhaseID) -> Phase:
        """
        Create and configure a new phase in the multiphase problem.

        Each phase represents a distinct segment of the trajectory with its own
        time domain, dynamics, and constraints. Phases can be linked through
        symbolic boundary constraints for trajectory continuity.

        Args:
            phase_id: Unique integer identifier for this phase. Phases are solved
                in order of their IDs, so use sequential numbering (1, 2, 3...).

        Returns:
            Phase: Phase object for defining variables, dynamics, and constraints

        Raises:
            ConfigurationError: If phase_id already exists

        Examples:
            Single phase problem:

            >>> problem = mtor.Problem("Single Phase")
            >>> phase = problem.set_phase(1)
            >>> # Define phase variables, dynamics, constraints...

            Sequential multiphase problem:

            >>> problem = mtor.Problem("Three Phase Mission")
            >>>
            >>> # Launch phase
            >>> launch = problem.set_phase(1)
            >>> # ... configure launch phase
            >>>
            >>> # Coast phase
            >>> coast = problem.set_phase(2)
            >>> # ... configure coast phase
            >>>
            >>> # Landing phase
            >>> landing = problem.set_phase(3)
            >>> # ... configure landing phase

            Phase naming convention:

            >>> # Use descriptive variable names for clarity
            >>> ascent_phase = problem.set_phase(1)
            >>> orbit_phase = problem.set_phase(2)
            >>> descent_phase = problem.set_phase(3)
        """
        if phase_id in self._multiphase_state.phases:
            raise ConfigurationError(f"Phase {phase_id} already exists")

        logger.debug("Adding phase %d to problem '%s'", phase_id, self.name)
        return Phase(self, phase_id)

    def parameter(
        self, name: str, boundary: BoundaryInput = None, fixed: FixedInput = None
    ) -> ca.MX:
        """
        Define a static parameter for design optimization with exhaustive constraint syntax.

        Creates optimization variables that remain constant throughout the mission
        but can be varied to optimize performance. Supports all constraint types
        for comprehensive design space specification.

        Args:
            name: Unique parameter name
            boundary: Parameter optimization bounds:

                - (lower, upper): Bounded parameter range (e.g., (100, 500))
                - (None, upper): Upper bounded only (e.g., (None, 1000))
                - (lower, None): Lower bounded only (e.g., (0, None))
                - None: Unconstrained parameter

            fixed: Fixed parameter value:

                - float: Fixed constant (e.g., 9.81)
                - ca.MX: Symbolic relationship (e.g., mass1 * 2.0)
                - None: Not fixed (use boundary for optimization)

        Returns:
            ca.MX: Parameter variable for use across all phases

        Raises:
            ConfigurationError: If both boundary and fixed are specified

        Examples:
            Bounded parameter:

            >>> mass = problem.parameter("mass", boundary=(100, 500))

            Single-sided bounds:

            >>> area = problem.parameter("area", boundary=(10, None))
            >>> drag = problem.parameter("drag", boundary=(None, 0.5))

            Fixed parameter:

            >>> gravity = problem.parameter("gravity", fixed=9.81)

            Fixed relationship:

            >>> mass2 = problem.parameter("mass2", fixed=mass1 * 2.0)

            Unconstrained:

            >>> free_param = problem.parameter("design_var")
        """
        _validate_constraint_inputs(name, boundary, "Parameter")

        param_var = variables_problem._create_static_parameter(
            self._multiphase_state.static_parameters, name, boundary, fixed
        )
        logger.debug("Static parameter created: name='%s'", name)
        return param_var

    def parameter_guess(self, **parameter_guesses: float) -> None:
        r"""Set initial guesses for static parameters.

        Args:
            \*\*parameter_guesses: Parameter names mapped to guess values

        Examples:
            >>> mass = problem.parameter("mass", boundary=(100, 500))
            >>> drag = problem.parameter("drag", boundary=(0, 0.1))
            >>> problem.parameter_guess(mass=300.0, drag=0.05)
        """
        if not parameter_guesses:
            return

        for name in parameter_guesses:
            if name not in self._multiphase_state.static_parameters.parameter_name_to_index:
                raise ConfigurationError(
                    f"Parameter '{name}' not defined. Call problem.parameter('{name}', ...) first.",
                    "Parameter guess configuration error",
                )

        self._multiphase_state.guess_static_parameters = parameter_guesses
        logger.debug("Static parameter guesses set: %s", list(parameter_guesses.keys()))

    def minimize(self, objective_expr: ca.MX | float | int) -> None:
        """
        Set objective function with comprehensive expression support.

        Defines scalar optimization objective supporting final states, integral
        terms, static parameters, and complex multi-phase expressions. Can combine
        multiple objective components with arbitrary mathematical relationships.

        Args:
            objective_expr: Scalar expression to minimize. Supports:

                - Final state values (state.final)
                - Integral terms (from add_integral())
                - Static parameters (from parameter())
                - Time variables (time.final)
                - Complex mathematical combinations

        Examples:
            Minimum time:

            >>> problem.minimize(t.final)

            Final state:

            >>> problem.minimize(-altitude.final)  # Maximize altitude

            Integral cost:

            >>> energy = phase.add_integral(thrust**2)
            >>> problem.minimize(energy)

            Multi-objective:

            >>> fuel_cost = phase.add_integral(thrust * price)
            >>> time_cost = t.final * rate
            >>> problem.minimize(fuel_cost + time_cost)

            Parameter optimization:

            >>> mass = problem.parameter("mass")
            >>> problem.minimize(mass - performance)
        """
        self._multiphase_state._functions_built = False
        variables_problem._set_multiphase_objective(self._multiphase_state, objective_expr)
        logger.info("Multiphase objective function defined")

    @property
    def _phases(self) -> dict[PhaseID, Any]:
        return self._multiphase_state.phases

    @property
    def _static_parameters(self) -> Any:
        return self._multiphase_state.static_parameters

    def _get_phase_ids(self) -> list[PhaseID]:
        return self._multiphase_state._get_phase_ids()

    def _get_phase_variable_counts(self, phase_id: PhaseID) -> tuple[int, int]:
        _validate_phase_exists(self._multiphase_state.phases, phase_id)
        return self._multiphase_state.phases[phase_id].get_variable_counts()

    def _get_total_variable_counts(self) -> tuple[int, int, int]:
        return self._multiphase_state._get_total_variable_counts()

    def _get_phase_ordered_state_names(self, phase_id: PhaseID) -> list[str]:
        _validate_phase_exists(self._multiphase_state.phases, phase_id)
        return self._multiphase_state.phases[phase_id].state_names.copy()

    def _get_phase_ordered_control_names(self, phase_id: PhaseID) -> list[str]:
        _validate_phase_exists(self._multiphase_state.phases, phase_id)
        return self._multiphase_state.phases[phase_id].control_names.copy()

    def _get_phase_dynamics_function(self, phase_id: PhaseID) -> Any:
        _validate_phase_exists(self._multiphase_state.phases, phase_id)
        phase_def = self._multiphase_state.phases[phase_id]

        if phase_def._dynamics_function is None:
            raise ConfigurationError(
                f"Phase {phase_id} dynamics function not built - call validate_multiphase_configuration() first"
            )

        return phase_def._dynamics_function

    def _get_phase_numerical_dynamics_function(self, phase_id: PhaseID) -> Any:
        _validate_phase_exists(self._multiphase_state.phases, phase_id)
        phase_def = self._multiphase_state.phases[phase_id]

        if phase_def._numerical_dynamics_function is None:
            raise ConfigurationError(
                f"Phase {phase_id} numerical dynamics function not built - call validate_multiphase_configuration() first"
            )

        return phase_def._numerical_dynamics_function

    def _get_objective_function(self) -> Any:
        if self._multiphase_state._objective_function is None:
            raise ConfigurationError(
                "Multiphase objective function not built - call validate_multiphase_configuration() first"
            )

        return self._multiphase_state._objective_function

    def _get_phase_integrand_function(self, phase_id: PhaseID) -> Any:
        _validate_phase_exists(self._multiphase_state.phases, phase_id)
        phase_def = self._multiphase_state.phases[phase_id]
        return phase_def._integrand_function

    def _get_phase_path_constraints_function(self, phase_id: PhaseID) -> Any:
        _validate_phase_exists(self._multiphase_state.phases, phase_id)
        phase_def = self._multiphase_state.phases[phase_id]
        return phase_def._path_constraints_function

    def _get_cross_phase_event_constraints_function(self) -> Any:
        return self._multiphase_state._cross_phase_constraints_function

    def validate_multiphase_configuration(self) -> None:
        _process_symbolic_constraints_for_all_phases(
            self._multiphase_state, self._multiphase_state.cross_phase_constraints
        )

        _validate_phase_requirements(self._multiphase_state.phases)

        if self._multiphase_state.objective_expression is None:
            raise ConfigurationError("Problem must have objective function defined")

        _build_all_phase_functions(self._multiphase_state)

        logger.debug(
            "Multiphase configuration validated: %d phases, %d cross-phase constraints",
            len(self._multiphase_state.phases),
            len(self._multiphase_state.cross_phase_constraints),
        )
