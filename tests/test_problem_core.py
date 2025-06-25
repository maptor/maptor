import casadi as ca
import numpy as np
import pytest

import maptor as mtor
from maptor.exceptions import ConfigurationError, DataIntegrityError


class TestProblemConstruction:
    def test_single_phase_variable_creation(self):
        problem = mtor.Problem("Test Single Phase")
        phase = problem.set_phase(1)

        # Time variable creation
        t = phase.time(initial=0.0, final=1.0)
        assert hasattr(t, "initial")
        assert hasattr(t, "final")
        assert isinstance(t._symbolic_var, ca.MX)

        # State variable creation with all constraint types
        x = phase.state("position", initial=0.0, final=1.0)
        _v = phase.state("velocity", initial=0.0, boundary=(-10, 10))
        _free_state = phase.state("free", boundary=None)

        assert hasattr(x, "initial")
        assert hasattr(x, "final")
        assert isinstance(x._symbolic_var, ca.MX)

        # Control variable creation
        u = phase.control("thrust", boundary=(0, 100))
        _free_control = phase.control("free_control")

        assert isinstance(u, ca.MX)

        # Verify internal storage integrity
        assert len(phase._phase_def.state_info) == 3
        assert len(phase._phase_def.control_info) == 2
        assert "position" in phase._phase_def.state_names
        assert "thrust" in phase._phase_def.control_names

    def test_dynamics_assignment_validation(self):
        problem = mtor.Problem("Dynamics Test")
        phase = problem.set_phase(1)

        x = phase.state("x", initial=0)
        v = phase.state("v", initial=0)
        u = phase.control("u")

        # Valid dynamics assignment
        phase.dynamics({x: v, v: u - 0.1 * v})

        # Verify dynamics storage
        assert len(phase._phase_def.dynamics_expressions) == 2
        assert x._symbolic_var in phase._phase_def.dynamics_expressions
        assert v._symbolic_var in phase._phase_def.dynamics_expressions

        # Test dynamics with mathematical expressions
        phase2 = problem.set_phase(2)
        r = phase2.state("radius", initial=1000)
        theta = phase2.state("angle", initial=0)
        vr = phase2.state("vr", initial=0)
        vt = phase2.state("vt", initial=100)

        # Complex nonlinear dynamics (satellite-like)
        mu = 3.986e14
        phase2.dynamics({r: vr, theta: vt / r, vr: vt**2 / r - mu / r**2, vt: -vr * vt / r})

        assert len(phase2._phase_def.dynamics_expressions) == 4

    def test_objective_setting_formats(self):
        problem = mtor.Problem("Objective Test")
        phase = problem.set_phase(1)

        t = phase.time(initial=0)
        x = phase.state("x", initial=1)
        u = phase.control("u")

        phase.dynamics({x: u})

        # Minimum time objective
        problem.minimize(t.final)
        assert problem._multiphase_state.objective_expression is not None

        # Reset for integral objective test
        problem._multiphase_state.objective_expression = None

        # Integral objective
        cost_integrand = x**2 + 0.5 * u**2
        cost = phase.add_integral(cost_integrand)
        problem.minimize(cost)

        # Verify integral creation
        assert len(phase._phase_def.integral_expressions) == 1
        assert phase._phase_def.num_integrals == 1

    def test_multiphase_symbolic_linking(self):
        problem = mtor.Problem("Multiphase Test")

        # Phase 1 setup
        p1 = problem.set_phase(1)
        t1 = p1.time(initial=0, final=1)
        x1 = p1.state("x", initial=0)
        v1 = p1.state("v", initial=0)
        u1 = p1.control("u")
        p1.dynamics({x1: v1, v1: u1})

        # Phase 2 with symbolic continuity
        p2 = problem.set_phase(2)
        t2 = p2.time(initial=t1.final, final=2)  # Time continuity
        x2 = p2.state("x", initial=x1.final)  # State continuity
        v2 = p2.state("v", initial=v1.final)  # State continuity
        u2 = p2.control("u")
        p2.dynamics({x2: v2, v2: u2})

        p1.mesh([3], [-1, 1])
        p2.mesh([3], [-1, 1])

        problem.minimize(t2.final)

        # Force symbolic constraint processing
        problem.validate_multiphase_configuration()

        # Verify cross-phase constraints were created
        # The symbolic linking should create automatic constraints
        assert len(problem._multiphase_state.cross_phase_constraints) > 0

    def test_parameter_optimization(self):
        problem = mtor.Problem("Parameter Test")

        # NEW API: Use boundary= for ranges, fixed= for constants
        mass = problem.parameter("mass", boundary=(100, 1000))
        thrust_max = problem.parameter("thrust_max", boundary=(500, 2000))
        gravity = problem.parameter("gravity", fixed=9.81)  # Use fixed= not boundary=
        _free_param = problem.parameter("free_param")

        phase = problem.set_phase(1)
        t = phase.time(initial=0, final=1)
        v = phase.state("velocity", initial=0)
        u = phase.control("throttle", boundary=(0, 1))

        # Use parameters in dynamics
        phase.dynamics({v: u * thrust_max / mass - gravity})
        problem.minimize(t.final)

        # Verify parameter storage
        assert len(problem._static_parameters.parameter_info) == 4
        assert "mass" in problem._static_parameters.parameter_names

        # Verify constraint types
        mass_info = problem._static_parameters.parameter_info[0]
        gravity_info = problem._static_parameters.parameter_info[2]

        assert mass_info.boundary_constraint is not None
        assert mass_info.boundary_constraint.lower == 100
        assert mass_info.boundary_constraint.upper == 1000

        assert gravity_info.fixed_constraint is not None
        assert gravity_info.fixed_constraint.equals == 9.81


class TestProblemFailureModes:
    def test_duplicate_phase_creation_fails(self):
        problem = mtor.Problem("Duplicate Test")
        problem.set_phase(1)

        with pytest.raises(ConfigurationError, match="Phase 1 already exists"):
            problem.set_phase(1)

    def test_duplicate_variable_names_fail(self):
        problem = mtor.Problem("Duplicate Variables")
        phase = problem.set_phase(1)

        phase.state("x", initial=0)

        # Should fail on duplicate state name
        with pytest.raises(DataIntegrityError, match="already exists"):
            phase.state("x", initial=1)

        phase.control("u")

        # Should fail on duplicate control name
        with pytest.raises(DataIntegrityError, match="already exists"):
            phase.control("u")

    def test_dynamics_for_undefined_state_fails(self):
        problem = mtor.Problem("Invalid Dynamics")
        phase = problem.set_phase(1)

        x = phase.state("x", initial=0)
        u = phase.control("u")

        # Create a fake state symbol not registered with phase
        fake_state = ca.MX.sym("fake", 1)  # type: ignore[arg-type]

        with pytest.raises(ConfigurationError, match="undefined state variable"):
            phase.dynamics({fake_state: u, x: 0})

    def test_incomplete_dynamics_detected(self):
        problem = mtor.Problem("Incomplete Dynamics")
        phase = problem.set_phase(1)

        x = phase.state("x", initial=0)
        v = phase.state("v", initial=0)
        _u = phase.control("u")

        # Only provide dynamics for one state
        phase.dynamics({x: v})  # Missing dynamics for v

        phase.mesh([3], [-1, 1])
        problem.minimize(x.final)

        # Should be detected during validation
        with pytest.raises(ConfigurationError, match="missing dynamics equation"):
            problem.validate_multiphase_configuration()

    def test_invalid_constraint_formats_fail(self):
        problem = mtor.Problem("Invalid Constraints")
        phase = problem.set_phase(1)

        # Invalid tuple length - should fail
        with pytest.raises(ConfigurationError):
            phase.state("x", initial=(1, 2, 3))  # type: ignore[arg-type]

        # Invalid bound ordering - should fail with specific error message
        with pytest.raises(ConfigurationError, match="Lower bound \\(10\\) > upper bound \\(5\\)"):
            phase.state("y", boundary=(10, 5))

        # Invalid type for boundary= - should fail
        with pytest.raises(ConfigurationError, match="Invalid constraint type: <class 'str'>"):
            phase.control("u", boundary="invalid")  # type: ignore[arg-type]

    def test_empty_string_names_fail(self):
        problem = mtor.Problem("Empty Names")
        phase = problem.set_phase(1)

        with pytest.raises(ConfigurationError, match="cannot be empty"):
            phase.state("", initial=0)

        with pytest.raises(ConfigurationError, match="cannot be empty"):
            phase.control("   ")  # Whitespace only

        with pytest.raises(ConfigurationError, match="cannot be empty"):
            problem.parameter(" \t ")  # Mixed whitespace

    def test_parameter_constraint_mutual_exclusion(self):
        """Test that parameters reject both boundary= and fixed=."""
        problem = mtor.Problem("Parameter Constraints")

        with pytest.raises(ConfigurationError, match="cannot have both boundary and fixed"):
            problem.parameter("param", boundary=(1, 10), fixed=5.0)


class TestOrderIndependence:
    """Test that setup order doesn't affect correctness."""

    def test_mesh_before_guess_order(self):
        problem = mtor.Problem("Mesh First")
        phase = problem.set_phase(1)

        t = phase.time(initial=0, final=1)
        x = phase.state("x", initial=1, final=0)
        u = phase.control("u")

        phase.dynamics({x: u})
        problem.minimize(t.final)

        # Set mesh first
        phase.mesh([3, 3], [-1, 0, 1])

        # Then set guess
        states_guess = [np.array([[1.0, 0.5, 0.25, 0.0]]), np.array([[0.0, -0.25, -0.5, -1.0]])]
        controls_guess = [np.array([[-1, -1, -1]]), np.array([[-1, -1, -1]])]

        phase.guess(states=states_guess, controls=controls_guess)

        # Should not raise any errors
        problem.validate_multiphase_configuration()

    def test_guess_before_mesh_order(self):
        problem = mtor.Problem("Guess First")
        phase = problem.set_phase(1)

        x = phase.state("x", initial=0, final=1)
        u = phase.control("u")

        phase.dynamics({x: u})
        problem.minimize(x.final)

        # Set guess first (before mesh configuration)
        states_guess = [np.array([[0.0, 0.5, 1.0]])]
        controls_guess = [np.array([[1, 1]])]

        phase.guess(states=states_guess, controls=controls_guess)

        # Then set mesh
        phase.mesh([2], [-1, 1])

        # Should work without issues
        problem.validate_multiphase_configuration()

    def test_objective_timing_independence(self):
        # Objective early
        problem1 = mtor.Problem("Early Objective")
        phase1 = problem1.set_phase(1)
        t1 = phase1.time(initial=0, final=1)

        problem1.minimize(t1.final)  # Set objective early

        x1 = phase1.state("x", initial=0)
        u1 = phase1.control("u")
        phase1.dynamics({x1: u1})

        phase1.mesh([3], [-1, 1])

        # Objective late
        problem2 = mtor.Problem("Late Objective")
        phase2 = problem2.set_phase(1)
        t2 = phase2.time(initial=0, final=1)
        x2 = phase2.state("x", initial=0)
        u2 = phase2.control("u")
        phase2.dynamics({x2: u2})

        phase2.mesh([3], [-1, 1])

        problem2.minimize(t2.final)  # Set objective late

        # Both should validate successfully
        problem1.validate_multiphase_configuration()
        problem2.validate_multiphase_configuration()
