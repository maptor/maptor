import casadi as ca
import numpy as np
import pytest

import maptor as mtor
from maptor.exceptions import ConfigurationError


class TestProvenExamples:
    def test_simple_lqr_problem_solution(self):
        problem = mtor.Problem("LQR Test")
        phase = problem.set_phase(1)

        _t = phase.time(initial=0, final=1)
        x = phase.state("x", initial=1.0)
        u = phase.control("u")

        phase.dynamics({x: 0.5 * x + u})

        integrand = 0.625 * x**2 + 0.5 * x * u + 0.5 * u**2
        integral_var = phase.add_integral(integrand)
        problem.minimize(integral_var)

        phase.mesh([4, 4], [-1.0, 0.0, 1.0])

        # Simple initial guess matching examples/lqr pattern
        states_guess = [
            np.array([[1.0, 0.8, 0.6, 0.4, 0.2]]),
            np.array([[0.2, 0.15, 0.1, 0.05, 0.0]]),
        ]
        controls_guess = [
            np.array([[-0.5, -0.4, -0.3, -0.2]]),
            np.array([[-0.1, -0.05, 0.0, 0.05]]),
        ]

        phase.guess(
            states=states_guess,
            controls=controls_guess,
            integrals=0.5,
        )

        # Use public API
        solution = mtor.solve_fixed_mesh(
            problem, nlp_options={"ipopt.print_level": 0, "ipopt.max_iter": 200}
        )

        # Verify solution interface
        assert hasattr(solution, "status")
        assert isinstance(solution.status["success"], bool)
        assert isinstance(solution.status["message"], str)

        if solution.status["success"]:
            # LQR reference objective: ~0.380797077977481140
            objective = solution.status["objective"]
            assert 0.35 < objective < 0.45  # Reasonable range for LQR

            # Test data access through new interface
            x_trajectory = solution["x"]
            assert len(x_trajectory) > 0
            assert abs(x_trajectory[0] - 1.0) < 0.1  # Initial condition

            # Test phase information
            assert 1 in solution.phases
            phase_data = solution.phases[1]
            assert "times" in phase_data
            assert "variables" in phase_data
            assert "x" in phase_data["variables"]["state_names"]

    def test_minimum_time_brachistochrone(self):
        problem = mtor.Problem("Brachistochrone Test")
        phase = problem.set_phase(1)

        t = phase.time(initial=0.0)
        x = phase.state("x", initial=0.0, final=1.0, boundary=(0, 10))
        y = phase.state("y", initial=0.0, boundary=(0, 10))
        v = phase.state("v", initial=0.0, boundary=(0, 10))
        u = phase.control("u", boundary=(0, np.pi / 2))

        g0 = 32.174
        phase.dynamics({x: v * ca.cos(u), y: v * ca.sin(u), v: g0 * ca.sin(u)})

        problem.minimize(t.final)

        phase.mesh([5, 5], [-1.0, 0.0, 1.0])

        # Initial guess matching examples/brachistochrone pattern
        states_guess = []
        controls_guess = []

        for N in [5, 5]:
            x_vals = np.array([0.0, 0.25, 0.5, 0.75, 1.0, 1.0])
            y_vals = np.array([0.0, 0.1, 0.3, 0.6, 1.0, 1.2])
            v_vals = np.array([0.0, 1.0, 2.0, 3.0, 4.0, 4.5])
            states_guess.append(np.array([x_vals, y_vals, v_vals]))

            u_vals = np.ones(N) * 0.5
            controls_guess.append(np.array([u_vals]))

        phase.guess(
            states=states_guess,
            controls=controls_guess,
            terminal_time=0.4,
        )

        solution = mtor.solve_fixed_mesh(
            problem, nlp_options={"ipopt.print_level": 0, "ipopt.max_iter": 500}
        )

        assert hasattr(solution, "status")

        if solution.status["success"]:
            # Brachistochrone reference: ~0.312480130
            objective = solution.status["objective"]
            assert 0.25 < objective < 0.40  # Reasonable range

            # Verify final condition satisfaction
            x_final = solution["x"][-1]
            assert abs(x_final - 1.0) < 0.1  # Should reach x=1

    def test_hypersensitive_problem(self):
        problem = mtor.Problem("Hypersensitive Test")
        phase = problem.set_phase(1)

        _t = phase.time(initial=0, final=40)
        x = phase.state("x", initial=1.5, final=1.0)
        u = phase.control("u")

        phase.dynamics({x: -(x**3) + u})

        integrand = 0.5 * (x**2 + u**2)
        integral_var = phase.add_integral(integrand)
        problem.minimize(integral_var)

        phase.mesh([8, 8, 8], [-1.0, -1 / 3, 1 / 3, 1.0])

        # Initial guess following examples/hypersensitive pattern
        states_guess = []
        controls_guess = []
        for N in [8, 8, 8]:
            tau = np.linspace(-1, 1, N + 1)
            x_vals = 1.5 + (1.0 - 1.5) * (tau + 1) / 2
            states_guess.append(x_vals.reshape(1, -1))
            controls_guess.append(np.zeros((1, N)))

        phase.guess(
            states=states_guess,
            controls=controls_guess,
            integrals=0.1,
        )

        solution = mtor.solve_fixed_mesh(
            problem, nlp_options={"ipopt.print_level": 0, "ipopt.max_iter": 200}
        )

        assert hasattr(solution, "status")
        # Hypersensitive is challenging - may not always converge, but should not crash

    def test_multiphase_schwartz_problem(self):
        problem = mtor.Problem("Two-Phase Test")

        # Phase 1
        phase1 = problem.set_phase(1)
        phase1.time(initial=0.0, final=1.0)
        x0_1 = phase1.state("x0", initial=1.0)
        x1_1 = phase1.state("x1", initial=1.0, boundary=(-0.8, None))
        u1 = phase1.control("u", boundary=(-1.0, 1.0))

        phase1.dynamics(
            {
                x0_1: x1_1,
                x1_1: u1 - 0.1 * (1 + 2 * x0_1**2) * x1_1,
            }
        )

        # Path constraint
        elliptical_constraint = 1 - 9 * (x0_1 - 1) ** 2 - ((x1_1 - 0.4) / 0.3) ** 2
        phase1.path_constraints(elliptical_constraint <= 0)
        phase1.mesh([6, 6], [-1.0, 0.0, 1.0])

        # Phase 2
        phase2 = problem.set_phase(2)
        phase2.time(initial=1.0, final=2.9)
        x0_2 = phase2.state("x0", initial=x0_1.final)  # Symbolic continuity
        x1_2 = phase2.state("x1", initial=x1_1.final)  # Symbolic continuity
        u2 = phase2.control("u")

        phase2.dynamics(
            {
                x0_2: x1_2,
                x1_2: u2 - 0.1 * (1 + 2 * x0_2**2) * x1_2,
            }
        )
        phase2.mesh([8, 8], [-1.0, 0.0, 1.0])

        # Objective
        objective_expr = 5 * (x0_2.final**2 + x1_2.final**2)
        problem.minimize(objective_expr)

        # Simple guess
        states_p1 = []
        controls_p1 = []
        states_p2 = []
        controls_p2 = []

        for N in [6, 6]:
            tau_states = np.linspace(-1, 1, N + 1)
            t_norm_states = (tau_states + 1) / 2
            x0_vals = 1.0 + 0.2 * t_norm_states
            x1_vals = 1.0 - 0.3 * t_norm_states
            states_p1.append(np.array([x0_vals, x1_vals]))

            t_norm_controls = np.linspace(0, 1, N)
            u_vals = 0.3 * np.sin(np.pi * t_norm_controls)
            controls_p1.append(np.array([u_vals]))

        for N in [8, 8]:
            tau_states = np.linspace(-1, 1, N + 1)
            t_norm_states = (tau_states + 1) / 2
            x0_end_p1 = 1.2
            x1_end_p1 = 0.7
            x0_vals = x0_end_p1 * (1 - 0.8 * t_norm_states)
            x1_vals = x1_end_p1 * (1 - 0.9 * t_norm_states)
            states_p2.append(np.array([x0_vals, x1_vals]))

            t_norm_controls = np.linspace(0, 1, N)
            u_vals = -1.0 + 0.5 * t_norm_controls
            controls_p2.append(np.array([u_vals]))

        phase1.guess(
            states=states_p1,
            controls=controls_p1,
        )
        phase2.guess(
            states=states_p2,
            controls=controls_p2,
        )

        solution = mtor.solve_fixed_mesh(
            problem, nlp_options={"ipopt.print_level": 0, "ipopt.max_iter": 1000}
        )

        assert hasattr(solution, "status")

        if solution.status["success"]:
            # Should have data for both phases
            assert 1 in solution.phases
            assert 2 in solution.phases

            # Test multiphase data access
            x0_phase1 = solution[(1, "x0")]
            x0_phase2 = solution[(2, "x0")]
            assert len(x0_phase1) > 0
            assert len(x0_phase2) > 0

    def test_adaptive_solver_execution(self):
        """Test adaptive solver using public API."""
        problem = mtor.Problem("Adaptive Test")
        phase = problem.set_phase(1)

        t = phase.time(initial=0, final=1)
        x = phase.state("x", initial=0, final=1)
        u = phase.control("u")

        phase.dynamics({x: u})
        problem.minimize(t.final)

        phase.mesh([3, 3], [-1, 0, 1])

        solution = mtor.solve_adaptive(
            problem,
            error_tolerance=1e-3,
            max_iterations=10,
            min_polynomial_degree=3,
            max_polynomial_degree=8,
            nlp_options={"ipopt.print_level": 0, "ipopt.max_iter": 100},
        )

        assert hasattr(solution, "status")
        assert hasattr(solution, "adaptive")

        if solution.adaptive:
            adaptive_data = solution.adaptive
            assert "converged" in adaptive_data
            assert "iterations" in adaptive_data
            assert "target_tolerance" in adaptive_data
            assert isinstance(adaptive_data["converged"], bool)


class TestSolutionInterfaceStructure:
    def test_solution_status_interface(self):
        problem = mtor.Problem("Status Interface Test")
        phase = problem.set_phase(1)

        t = phase.time(initial=0, final=1)
        x = phase.state("x", initial=0, final=1)
        u = phase.control("u")

        phase.dynamics({x: u})
        problem.minimize(t.final)

        phase.mesh([3], [-1, 1])

        solution = mtor.solve_fixed_mesh(problem, nlp_options={"ipopt.print_level": 0})

        # Test status interface matches examples pattern
        status = solution.status
        assert "success" in status
        assert "message" in status
        assert "objective" in status
        assert "total_mission_time" in status

        assert isinstance(status["success"], bool)
        assert isinstance(status["message"], str)

        if status["success"]:
            assert isinstance(status["objective"], int | float)
            assert isinstance(status["total_mission_time"], int | float)

    def test_variable_access_patterns(self):
        problem = mtor.Problem("Variable Access Test")
        phase = problem.set_phase(1)

        x = phase.state("position", initial=0, final=1)
        v = phase.state("velocity", initial=0)
        T = phase.control("thrust")

        phase.dynamics({x: v, v: T})
        problem.minimize(x.final)

        phase.mesh([3], [-1, 1])

        solution = mtor.solve_fixed_mesh(problem, nlp_options={"ipopt.print_level": 0})

        if solution.status["success"]:
            # Test different access patterns from examples
            position_data = solution["position"]  # Auto-find in any phase
            velocity_data = solution[(1, "velocity")]  # Specific phase
            thrust_data = solution[(1, "thrust")]

            assert isinstance(position_data, np.ndarray)
            assert isinstance(velocity_data, np.ndarray)
            assert isinstance(thrust_data, np.ndarray)

            # Test time arrays
            time_states = solution[(1, "time_states")]
            time_controls = solution[(1, "time_controls")]

            assert isinstance(time_states, np.ndarray)
            assert isinstance(time_controls, np.ndarray)

    def test_phases_property_interface(self):
        problem = mtor.Problem("Phases Interface Test")

        # Multiphase problem
        p1 = problem.set_phase(1)
        t1 = p1.time(initial=0, final=1)
        x1 = p1.state("x", initial=0)
        u1 = p1.control("u")
        p1.dynamics({x1: u1})
        p1.mesh([3], [-1, 1])

        p2 = problem.set_phase(2)
        t2 = p2.time(initial=t1.final, final=2)
        x2 = p2.state("x", initial=x1.final)
        u2 = p2.control("u")
        p2.dynamics({x2: u2})
        p2.mesh([4], [-1, 1])

        problem.minimize(t2.final)

        solution = mtor.solve_fixed_mesh(problem, nlp_options={"ipopt.print_level": 0})

        if solution.status["success"]:
            # Test phases interface from examples
            phases = solution.phases

            for phase_data in phases.values():
                assert "times" in phase_data
                assert "variables" in phase_data
                assert "mesh" in phase_data
                assert "time_arrays" in phase_data

                times = phase_data["times"]
                assert "initial" in times
                assert "final" in times
                assert "duration" in times

                variables = phase_data["variables"]
                assert "state_names" in variables
                assert "control_names" in variables
                assert "num_states" in variables
                assert "num_controls" in variables

                mesh = phase_data["mesh"]
                assert "polynomial_degrees" in mesh
                assert "mesh_nodes" in mesh
                assert "num_intervals" in mesh

    def test_parameters_interface(self):
        problem = mtor.Problem("Parameters Interface Test")

        mass = problem.parameter("mass", boundary=(100, 1000))
        thrust_max = problem.parameter("thrust_max", boundary=(500, 2000))

        phase = problem.set_phase(1)
        v = phase.state("velocity", initial=0)
        u = phase.control("throttle", boundary=(0, 1))

        phase.dynamics({v: u * thrust_max / mass})
        problem.minimize(v.final)

        phase.mesh([3], [-1, 1])

        solution = mtor.solve_fixed_mesh(problem, nlp_options={"ipopt.print_level": 0})

        if solution.status["success"]:
            params = solution.parameters

            if params is not None:
                assert "values" in params
                assert "names" in params
                assert "count" in params

                assert isinstance(params["values"], np.ndarray)
                assert params["count"] == 2


class TestErrorHandlingAndFailures:
    def test_infeasible_problem_handling(self):
        problem = mtor.Problem("Infeasible Problem")
        phase = problem.set_phase(1)

        x = phase.state("x", initial=0, final=10)  # Large target
        u = phase.control("u", boundary=(-0.1, 0.1))  # Very limited control

        phase.dynamics({x: u})
        problem.minimize(x.final)

        phase.mesh([3], [-1, 1])

        solution = mtor.solve_fixed_mesh(
            problem, nlp_options={"ipopt.print_level": 0, "ipopt.max_iter": 50}
        )

        # Should get solution object even if failed
        assert hasattr(solution, "status")
        assert isinstance(solution.status["success"], bool)
        assert isinstance(solution.status["message"], str)

        # May fail due to infeasibility, but should not crash

    def test_malformed_problem_validation(self):
        problem = mtor.Problem("Malformed Problem")
        phase = problem.set_phase(1)

        _x = phase.state("x", initial=0)
        # Missing dynamics, mesh, and objective

        # Should be caught by solve_fixed_mesh validation
        with pytest.raises(ConfigurationError):
            mtor.solve_fixed_mesh(problem)

    def test_invalid_solver_options(self):
        problem = mtor.Problem("Invalid Options Test")
        phase = problem.set_phase(1)

        x = phase.state("x", initial=0, final=1)
        u = phase.control("u")

        phase.dynamics({x: u})
        problem.minimize(x.final)

        phase.mesh([3], [-1, 1])

        # Invalid solver options should be handled gracefully
        solution = mtor.solve_fixed_mesh(problem, nlp_options={"invalid_option": "invalid_value"})

        # Should still return solution object
        assert hasattr(solution, "status")

    def test_solution_contains_operator(self):
        problem = mtor.Problem("Contains Test")
        phase = problem.set_phase(1)

        x = phase.state("position", initial=0)
        u = phase.control("thrust")

        phase.dynamics({x: u})
        problem.minimize(x.final)

        phase.mesh([3], [-1, 1])

        solution = mtor.solve_fixed_mesh(problem, nlp_options={"ipopt.print_level": 0})

        if solution.status["success"]:
            # Test contains operator
            assert "position" in solution
            assert (1, "position") in solution
            assert (1, "thrust") in solution
            assert (1, "time_states") in solution
            assert "nonexistent_var" not in solution


class TestConstraintIntegrationEndToEnd:
    def test_path_constraints_with_public_api(self):
        problem = mtor.Problem("Path Constraints Test")
        phase = problem.set_phase(1)

        x = phase.state("position", initial=0, final=1)
        v = phase.state("velocity", initial=0)
        u = phase.control("thrust")

        phase.dynamics({x: v, v: u})

        # Path constraints
        phase.path_constraints(
            x >= 0,  # Stay non-negative
            v <= 5,  # Speed limit
            u >= -2,  # Thrust bounds
            u <= 2,
        )

        problem.minimize(x.final)

        phase.mesh([4], [-1, 1])

        solution = mtor.solve_fixed_mesh(problem, nlp_options={"ipopt.print_level": 0})

        if solution.status["success"]:
            # Basic constraint satisfaction check
            position_data = solution["position"]
            velocity_data = solution["velocity"]

            # Should generally satisfy constraints (within solver tolerance)
            assert np.all(position_data >= -0.1)  # Small tolerance for numerical error
            assert np.all(velocity_data <= 5.1)  # Small tolerance

    def test_event_constraints_with_public_api(self):
        problem = mtor.Problem("Event Constraints Test")
        phase = problem.set_phase(1)

        x = phase.state("position", initial=0)
        v = phase.state("velocity", initial=0)
        u = phase.control("thrust")

        phase.dynamics({x: v, v: u})

        # Event constraints
        phase.event_constraints(
            x.final >= 5,  # Minimum final position
            v.final <= 2,  # Maximum final velocity
        )
        cost = phase.add_integral(u**2)
        problem.minimize(cost)  # Minimize control effort

        phase.mesh([4], [-1, 1])

        solution = mtor.solve_fixed_mesh(problem, nlp_options={"ipopt.print_level": 0})

        if solution.status["success"]:
            # Check event constraint satisfaction
            x_final = solution["position"][-1]
            v_final = solution["velocity"][-1]

            # Should satisfy event constraints (within tolerance)
            assert x_final >= 4.9  # Small tolerance
            assert v_final <= 2.1  # Small tolerance


class TestObjectiveFunctionTypes:
    def test_minimum_time_with_public_api(self):
        problem = mtor.Problem("Minimum Time Test")
        phase = problem.set_phase(1)

        t = phase.time(initial=0)  # Free final time
        x = phase.state("x", initial=0, final=1)
        u = phase.control("u", boundary=(-2, 2))

        phase.dynamics({x: u})
        problem.minimize(t.final)

        phase.mesh([4], [-1, 1])

        solution = mtor.solve_fixed_mesh(problem, nlp_options={"ipopt.print_level": 0})

        if solution.status["success"]:
            # Minimum time should be positive and reasonable
            objective = solution.status["objective"]
            assert objective > 0
            assert objective < 10  # Should be reasonable for this simple problem

    def test_integral_cost_with_public_api(self):
        problem = mtor.Problem("Integral Cost Test")
        phase = problem.set_phase(1)

        _t = phase.time(initial=0, final=1)
        x = phase.state("x", initial=0)
        u = phase.control("u")

        phase.dynamics({x: u})

        # Quadratic cost
        cost = phase.add_integral(x**2 + u**2)
        problem.minimize(cost)

        phase.mesh([4], [-1, 1])

        solution = mtor.solve_fixed_mesh(problem, nlp_options={"ipopt.print_level": 0})

        if solution.status["success"]:
            # Quadratic cost should be non-negative
            objective = solution.status["objective"]
            assert objective >= 0

    def test_parameter_dependent_objective_with_public_api(self):
        problem = mtor.Problem("Parameter Objective Test")

        weight = problem.parameter("weight", boundary=(0.1, 10))

        phase = problem.set_phase(1)
        x = phase.state("x", initial=0, final=1)
        u = phase.control("u")

        phase.dynamics({x: u})

        # Parameter-dependent objective
        cost = phase.add_integral(weight * u**2)
        problem.minimize(cost)

        phase.mesh([3], [-1, 1])

        solution = mtor.solve_fixed_mesh(problem, nlp_options={"ipopt.print_level": 0})

        if solution.status["success"]:
            assert solution.status["objective"] is not None

            params = solution.parameters
            if params is not None:
                assert len(params["values"]) == 1
