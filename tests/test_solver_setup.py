import casadi as ca
import numpy as np
import pytest

import maptor as mtor
from maptor.direct_solver.initial_guess_solver import _apply_multiphase_initial_guess
from maptor.direct_solver.variables_solver import (
    _setup_multiphase_optimization_variables,
    setup_phase_interval_state_variables,
)
from maptor.exceptions import ConfigurationError
from maptor.input_validation import _validate_multiphase_problem_ready_for_solving


class TestVariableSetupIntegration:
    def test_single_phase_variable_creation(self):
        problem = mtor.Problem("Single Phase Setup")
        phase = problem.set_phase(1)

        t = phase.time(initial=0, final=1)
        x = phase.state("position", initial=0, final=1)
        v = phase.state("velocity", initial=0)
        u = phase.control("thrust", boundary=(0, 100))

        phase.dynamics({x: v, v: u})
        problem.minimize(t.final)

        # Configure mesh
        phase.mesh([3, 4], [-1, 0, 1])

        # Create solver and test variable setup
        opti = ca.Opti()
        variables = _setup_multiphase_optimization_variables(opti, problem)  # type: ignore[arg-type]

        # Verify phase variables structure
        assert 1 in variables.phase_variables
        phase_vars = variables.phase_variables[1]

        # Check time variables
        assert phase_vars.initial_time is not None
        assert phase_vars.terminal_time is not None
        assert isinstance(phase_vars.initial_time, ca.MX)
        assert isinstance(phase_vars.terminal_time, ca.MX)

        expected_nodes = 2 + 1  # num_intervals + 1
        assert len(phase_vars.state_at_mesh_nodes) == expected_nodes

        # Check control variables (one per interval)
        assert len(phase_vars.control_variables) == 2  # num_intervals

        # Check dimensions match problem definition
        num_states, num_controls = problem._get_phase_variable_counts(1)
        assert num_states == 2  # position, velocity
        assert num_controls == 1  # thrust

    def test_multiphase_variable_organization(self):
        problem = mtor.Problem("Multiphase Setup")

        # Phase 1: 2 states, 1 control
        p1 = problem.set_phase(1)
        _t1 = p1.time(initial=0, final=1)
        x1 = p1.state("x", initial=0)
        v1 = p1.state("v", initial=0)
        u1 = p1.control("u")
        p1.dynamics({x1: v1, v1: u1})
        p1.mesh([3], [-1, 1])

        # Phase 2: 3 states, 2 controls
        p2 = problem.set_phase(2)
        t2 = p2.time(initial=1, final=2)
        x2 = p2.state("x", initial=x1.final)
        y2 = p2.state("y", initial=0)
        v2 = p2.state("v", initial=v1.final)
        u2 = p2.control("u")
        w2 = p2.control("w")
        p2.dynamics({x2: v2, y2: w2, v2: u2})
        p2.mesh([4, 4], [-1, 0, 1])

        problem.minimize(t2.final)

        opti = ca.Opti()
        variables = _setup_multiphase_optimization_variables(opti, problem)  # type: ignore[arg-type]

        # Verify both phases are present
        assert 1 in variables.phase_variables
        assert 2 in variables.phase_variables

        # Check phase 1 structure
        p1_vars = variables.phase_variables[1]
        assert len(p1_vars.state_at_mesh_nodes) == 2  # 1 interval + 1
        assert len(p1_vars.control_variables) == 1  # 1 interval

        # Check phase 2 structure
        p2_vars = variables.phase_variables[2]
        assert len(p2_vars.state_at_mesh_nodes) == 3  # 2 intervals + 1
        assert len(p2_vars.control_variables) == 2  # 2 intervals

    def test_static_parameter_integration(self):
        problem = mtor.Problem("Parameter Integration")

        # Create static parameters
        mass = problem.parameter("mass", boundary=(100, 1000))
        thrust_max = problem.parameter("thrust_max", boundary=(500, 2000))

        phase = problem.set_phase(1)
        t = phase.time(initial=0, final=1)
        v = phase.state("velocity", initial=0)
        u = phase.control("throttle", boundary=(0, 1))

        # Use parameters in dynamics
        phase.dynamics({v: u * thrust_max / mass})
        problem.minimize(t.final)

        phase.mesh([3], [-1, 1])

        opti = ca.Opti()
        variables = _setup_multiphase_optimization_variables(opti, problem)  # type: ignore[arg-type]

        # Verify static parameters were created
        assert variables.static_parameters is not None
        assert isinstance(variables.static_parameters, ca.MX)

        # Check parameter count matches
        _, _, num_static_params = problem._get_total_variable_counts()
        assert num_static_params == 2

    def test_integral_variable_creation(self):
        problem = mtor.Problem("Integral Variables")
        phase = problem.set_phase(1)

        _t = phase.time(initial=0, final=1)
        x = phase.state("x", initial=0)
        u = phase.control("u")

        phase.dynamics({x: u})

        # Add multiple integrals
        cost1 = phase.add_integral(x**2)
        cost2 = phase.add_integral(u**2)
        cost3 = phase.add_integral(1.0)  # Time integral

        problem.minimize(cost1 + 0.1 * cost2 + 0.01 * cost3)

        phase.mesh([3], [-1, 1])

        opti = ca.Opti()
        variables = _setup_multiphase_optimization_variables(opti, problem)  # type: ignore[arg-type]

        phase_vars = variables.phase_variables[1]

        # Verify integral variables were created
        assert phase_vars.integral_variables is not None
        assert isinstance(phase_vars.integral_variables, ca.MX)

        # Check integral count
        assert phase._phase_def.num_integrals == 3


class TestMeshIntervalHandling:
    def test_uniform_mesh_interval_setup(self):
        problem = mtor.Problem("Uniform Mesh")
        phase = problem.set_phase(1)

        x = phase.state("x", initial=0, final=1)
        v = phase.state("v", initial=0)
        u = phase.control("u")

        phase.dynamics({x: v, v: u})
        problem.minimize(x.final)

        # Uniform mesh: 3 intervals, each with 4 collocation points
        phase.mesh([4, 4, 4], [-1, -0.3, 0.3, 1])

        opti = ca.Opti()
        variables = _setup_multiphase_optimization_variables(opti, problem)  # type: ignore[arg-type]

        phase_vars = variables.phase_variables[1]

        # Test individual interval setup
        for k in range(3):  # 3 intervals
            state_matrix, interior_nodes = setup_phase_interval_state_variables(
                opti, 1, k, 2, 4, phase_vars.state_at_mesh_nodes
            )

            # Each interval should have proper state matrix structure
            assert state_matrix is not None
            assert isinstance(state_matrix, ca.MX)

            # Interior nodes for degree 4 should exist (degree > 1)
            assert interior_nodes is not None

    def test_non_uniform_mesh_interval_setup(self):
        problem = mtor.Problem("Non-uniform Mesh")
        phase = problem.set_phase(1)

        x = phase.state("x", initial=0)
        u = phase.control("u")

        phase.dynamics({x: u})
        problem.minimize(x.final)

        # Non-uniform mesh: different degrees per interval
        phase.mesh([3, 6, 4], [-1, -0.5, 0.5, 1])

        opti = ca.Opti()
        variables = _setup_multiphase_optimization_variables(opti, problem)  # type: ignore[arg-type]

        phase_vars = variables.phase_variables[1]

        # Control variables should match mesh structure
        assert len(phase_vars.control_variables) == 3  # 3 intervals

        # Each control variable should have correct dimensions
        expected_colloc_points = [3, 6, 4]
        for k, expected_points in enumerate(expected_colloc_points):
            control_var = phase_vars.control_variables[k]
            assert control_var.shape[1] == expected_points  # num_controls x num_colloc_points

    def test_minimal_mesh_setup(self):
        problem = mtor.Problem("Minimal Mesh")
        phase = problem.set_phase(1)

        x = phase.state("x", initial=0, final=1)
        u = phase.control("u")

        phase.dynamics({x: u})
        problem.minimize(x.final)

        # Minimal mesh: single interval
        phase.mesh([3], [-1, 1])

        opti = ca.Opti()
        variables = _setup_multiphase_optimization_variables(opti, problem)  # type: ignore[arg-type]

        phase_vars = variables.phase_variables[1]

        # Single interval setup
        assert len(phase_vars.state_at_mesh_nodes) == 2  # 1 interval + 1
        assert len(phase_vars.control_variables) == 1  # 1 interval

    def test_high_order_mesh_setup(self):
        problem = mtor.Problem("High Order Mesh")
        phase = problem.set_phase(1)

        x = phase.state("x", initial=0)
        v = phase.state("v", initial=0)
        u = phase.control("u")

        phase.dynamics({x: v, v: u})
        problem.minimize(x.final)

        # High-order mesh
        phase.mesh([10, 12], [-1, 0, 1])

        opti = ca.Opti()
        variables = _setup_multiphase_optimization_variables(opti, problem)  # type: ignore[arg-type]

        phase_vars = variables.phase_variables[1]

        # High-order intervals should create appropriate structures
        assert len(phase_vars.control_variables) == 2  # 2 intervals

        # Check control variable dimensions
        assert phase_vars.control_variables[0].shape[1] == 10  # First interval: 10 points
        assert phase_vars.control_variables[1].shape[1] == 12  # Second interval: 12 points


class TestInitialGuessApplication:
    def test_complete_initial_guess_application(self):
        problem = mtor.Problem("Complete Guess")
        phase = problem.set_phase(1)

        _t = phase.time(initial=0, final=1)
        x = phase.state("position", initial=0, final=1)
        v = phase.state("velocity", initial=0)
        u = phase.control("thrust")

        phase.dynamics({x: v, v: u})

        energy = phase.add_integral(u**2)
        problem.minimize(energy)

        phase.mesh([3, 4], [-1, 0, 1])

        states_guess = [
            np.array(
                [
                    [0.0, 0.25, 0.5, 0.75],
                    [0.0, 0.5, 1.0, 1.0],
                ]
            ),
            np.array(
                [
                    [0.75, 0.8, 0.9, 0.95, 1.0],
                    [1.0, 0.8, 0.6, 0.3, 0.0],
                ]
            ),
        ]

        controls_guess = [
            np.array([[1.0, 1.0, 1.0]]),
            np.array([[-1.0, -1.0, -1.0, -1.0]]),
        ]

        phase.guess(
            states=states_guess,
            controls=controls_guess,
            initial_time=0.0,
            terminal_time=2.0,
            integrals=1.5,
        )

        opti = ca.Opti()
        variables = _setup_multiphase_optimization_variables(opti, problem)  # type: ignore[arg-type]

        # Manually populate interior_variables to match mesh structure (2 intervals)
        variables.phase_variables[1].interior_variables = [None, None]

        _apply_multiphase_initial_guess(opti, variables, problem)  # type: ignore[arg-type]

    def test_partial_initial_guess_application(self):
        problem = mtor.Problem("Partial Guess")
        phase = problem.set_phase(1)

        t = phase.time(initial=0)  # Free final time
        x = phase.state("x", initial=0)
        u = phase.control("u")

        phase.dynamics({x: u})
        problem.minimize(t.final)

        phase.mesh([3], [-1, 1])

        # Only provide terminal time guess (partial)
        phase.guess(terminal_time=5.0)

        opti = ca.Opti()
        variables = _setup_multiphase_optimization_variables(opti, problem)  # type: ignore[arg-type]

        # Should handle partial guess without errors
        _apply_multiphase_initial_guess(opti, variables, problem)  # type: ignore[arg-type]


class TestValidationIntegration:
    def test_problem_validation_before_setup(self):
        problem = mtor.Problem("Validation Test")
        phase = problem.set_phase(1)

        x = phase.state("x", initial=0)
        _u = phase.control("u")

        # Missing dynamics - should be caught by validation
        problem.minimize(x.final)
        phase.mesh([3], [-1, 1])

        with pytest.raises(
            ConfigurationError, match="Phase 1 dynamics must be defined - call phase.dynamics()"
        ):
            _validate_multiphase_problem_ready_for_solving(problem)  # type: ignore[arg-type]

    def test_mesh_configuration_validation(self):
        problem = mtor.Problem("Mesh Validation")
        phase = problem.set_phase(1)

        x = phase.state("x", initial=0)
        u = phase.control("u")

        phase.dynamics({x: u})
        problem.minimize(x.final)

        # Missing mesh configuration
        with pytest.raises(ConfigurationError, match="mesh must be configured"):
            _validate_multiphase_problem_ready_for_solving(problem)  # type: ignore[arg-type]

    def test_objective_validation(self):
        problem = mtor.Problem("Objective Validation")
        phase = problem.set_phase(1)

        x = phase.state("x", initial=0)
        u = phase.control("u")

        phase.dynamics({x: u})
        phase.mesh([3], [-1, 1])

        # Missing objective
        with pytest.raises(ConfigurationError, match="must have objective"):
            _validate_multiphase_problem_ready_for_solving(problem)  # type: ignore[arg-type]


class TestBoundaryConstraintApplication:
    def test_time_bound_constraints(self):
        problem = mtor.Problem("Time Bounds")
        phase = problem.set_phase(1)

        # Bounded initial and final times
        t = phase.time(initial=(0, 1), final=(5, 10))
        x = phase.state("x", initial=0)
        u = phase.control("u")

        phase.dynamics({x: u})
        problem.minimize(t.final)

        phase.mesh([3], [-1, 1])

        opti = ca.Opti()
        variables = _setup_multiphase_optimization_variables(opti, problem)  # type: ignore[arg-type]

        # Time bounds should be applied as constraints
        # Verify by checking that opti has constraints (hard to verify specific constraints)
        assert variables.phase_variables[1].initial_time is not None
        assert variables.phase_variables[1].terminal_time is not None

    def test_state_bound_constraints(self):
        problem = mtor.Problem("State Bounds")
        phase = problem.set_phase(1)

        t = phase.time(initial=0, final=1)

        # States with various boundary constraints
        pos = phase.state("position", initial=0, final=1, boundary=(0, 10))
        vel = phase.state("velocity", initial=0, boundary=(-5, 5))

        u = phase.control("thrust", boundary=(-2, 2))

        phase.dynamics({pos: vel, vel: u})
        problem.minimize(t.final)

        phase.mesh([3], [-1, 1])

        opti = ca.Opti()
        variables = _setup_multiphase_optimization_variables(opti, problem)  # type: ignore[arg-type]

        # Boundary constraints should be set up for path constraint processing
        phase_vars = variables.phase_variables[1]
        assert len(phase_vars.state_at_mesh_nodes) == 2  # 1 interval + 1
        assert len(phase_vars.control_variables) == 1  # 1 interval

    def test_control_bound_constraints(self):
        problem = mtor.Problem("Control Bounds")
        phase = problem.set_phase(1)

        x = phase.state("x", initial=0)

        # NEW API: Only range constraints for controls
        thrust = phase.control("thrust", boundary=(0, 100))
        steering = phase.control("steering", boundary=(-30, 30))
        free_ctrl = phase.control("free")

        # OLD API: Fixed control via boundary=scalar - NOW SHOULD FAIL
        with pytest.raises(
            ConfigurationError, match="boundary= argument only accepts range tuples"
        ):
            phase.control("fixed", boundary=5.0)  # type: ignore[arg-type]

        phase.dynamics({x: thrust + steering + free_ctrl})
        problem.minimize(x.final)

        phase.mesh([3], [-1, 1])

        opti = ca.Opti()
        variables = _setup_multiphase_optimization_variables(opti, problem)  # type: ignore[arg-type]

        # Control variables should be created properly
        phase_vars = variables.phase_variables[1]
        control_var = phase_vars.control_variables[0]

        # Should have 3 controls and 3 collocation points
        assert control_var.shape[0] == 3  # num_controls
        assert control_var.shape[1] == 3  # num_collocation_points
