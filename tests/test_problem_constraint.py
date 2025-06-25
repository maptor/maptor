import casadi as ca
import pytest

import maptor as mtor
from maptor.exceptions import ConfigurationError
from maptor.mtor_types import Constraint


class TestPathConstraints:
    def test_inequality_path_constraints(self):
        problem = mtor.Problem("Path Constraints Test")
        phase = problem.set_phase(1)

        x = phase.state("position", initial=0, final=1)
        v = phase.state("velocity", initial=0)
        u = phase.control("thrust", boundary=(0, 100))

        phase.dynamics({x: v, v: u - 0.1 * v})

        # Add various inequality constraints
        phase.path_constraints(
            x >= 0,  # Lower bound
            v <= 50,  # Upper bound
            x <= 10,  # Another upper bound
            v >= -5,  # Another lower bound
        )

        # Verify constraints were stored
        assert len(phase._phase_def.path_constraints) == 4

    def test_nonlinear_path_constraints(self):
        problem = mtor.Problem("Nonlinear Constraints")
        phase = problem.set_phase(1)

        x = phase.state("x_pos", initial=0)
        y = phase.state("y_pos", initial=0)
        u1 = phase.control("u_x")
        u2 = phase.control("u_y")

        phase.dynamics({x: u1, y: u2})

        # Circular obstacle avoidance constraint
        obstacle_center_x, obstacle_center_y = 5, 5
        obstacle_radius = 2
        obstacle_constraint = (x - obstacle_center_x) ** 2 + (
            y - obstacle_center_y
        ) ** 2 >= obstacle_radius**2

        # Dynamic pressure constraint (atmospheric flight example)
        velocity_magnitude = ca.sqrt(u1**2 + u2**2)
        altitude = y  # Simplified
        rho = 1.225 * ca.exp(-altitude / 8400)  # Atmospheric density
        dynamic_pressure = 0.5 * rho * velocity_magnitude**2

        phase.path_constraints(
            obstacle_constraint,
            dynamic_pressure <= 50000,
            x >= 0,  # Stay in positive x
            y >= 0,  # Stay above ground
        )

        assert len(phase._phase_def.path_constraints) == 4

    def test_path_constraint_validation_failure(self):
        problem = mtor.Problem("Invalid Path Constraints")
        phase = problem.set_phase(1)

        x = phase.state("x", initial=0)
        phase.dynamics({x: 0})

        # Empty constraint list should fail
        with pytest.raises(ConfigurationError, match="requires at least one constraint"):
            phase.path_constraints()

    def test_mixed_constraint_types(self):
        problem = mtor.Problem("Mixed Constraints")
        phase = problem.set_phase(1)

        x = phase.state("x", initial=0)
        y = phase.state("y", initial=1)
        z = phase.state("z", initial=0)
        u = phase.control("u")

        phase.dynamics({x: u, y: 0, z: x})

        # Mix of constraint types
        phase.path_constraints(
            x >= 0,  # Inequality
            y**2 + z**2 <= 4,  # Nonlinear inequality
            ca.sin(x) + ca.cos(y) >= -1,  # Transcendental inequality
        )

        assert len(phase._phase_def.path_constraints) == 3


class TestEventConstraints:
    def test_boundary_value_constraints(self):
        problem = mtor.Problem("Boundary Constraints")
        phase = problem.set_phase(1)

        x = phase.state("position", initial=0)
        v = phase.state("velocity", initial=0)
        u = phase.control("thrust")

        phase.dynamics({x: v, v: u})

        # Event constraints on final states
        phase.event_constraints(
            x.final >= 100,  # Minimum final position
            v.final <= 5,  # Maximum final velocity
            ca.fabs(v.final) <= 1,  # Final velocity near zero
        )

        # Verify cross-phase constraints were added
        assert len(problem._multiphase_state.cross_phase_constraints) == 3

    def test_integral_constraints(self):
        problem = mtor.Problem("Integral Constraints")
        phase = problem.set_phase(1)

        v = phase.state("velocity", initial=0)
        m = phase.state("mass", initial=1000)
        T = phase.control("thrust", boundary=(0, 2000))

        phase.dynamics({v: T / m, m: -T * 0.001})  # Fuel consumption

        # Fuel consumption constraint
        fuel_used = phase.add_integral(T * 0.001)

        phase.event_constraints(
            fuel_used <= 100,  # Maximum fuel consumption
            v.final >= 200,  # Minimum final velocity
        )

        assert len(problem._multiphase_state.cross_phase_constraints) >= 2

    def test_multiphase_continuity_constraints(self):
        problem = mtor.Problem("Multiphase Continuity")

        # Phase 1
        p1 = problem.set_phase(1)
        _t1 = p1.time(initial=0, final=1)
        x1 = p1.state("x", initial=0)
        v1 = p1.state("v", initial=0)
        u1 = p1.control("u")
        p1.dynamics({x1: v1, v1: u1})

        # Phase 2
        p2 = problem.set_phase(2)
        t2 = p2.time(initial=1, final=2)
        x2 = p2.state("x", initial=1)  # Not automatically linked
        v2 = p2.state("v", initial=0)  # Not automatically linked
        u2 = p2.control("u")
        p2.dynamics({x2: v2, v2: u2})

        # Manual continuity constraints
        p2.event_constraints(
            x2.initial == x1.final,  # Position continuity
            v2.initial == v1.final,  # Velocity continuity
        )

        problem.minimize(t2.final)

        # Should have continuity constraints
        assert len(problem._multiphase_state.cross_phase_constraints) == 2

    def test_event_constraint_validation_failure(self):
        problem = mtor.Problem("Invalid Event Constraints")
        phase = problem.set_phase(1)

        x = phase.state("x", initial=0)
        phase.dynamics({x: 0})

        # Empty constraint list should fail
        with pytest.raises(ConfigurationError, match="requires at least one constraint"):
            phase.event_constraints()


class TestBoundaryConstraints:
    def test_state_boundary_constraints(self):
        """Test various state boundary constraint formats."""
        problem = mtor.Problem("State Boundaries")
        phase = problem.set_phase(1)

        # Different boundary constraint types
        fixed_state = phase.state("fixed", initial=0, final=1)
        bounded_state = phase.state("bounded", boundary=(-5, 5))
        lower_bounded = phase.state("lower", boundary=(0, None))
        upper_bounded = phase.state("upper", boundary=(None, 10))
        free_state = phase.state("free")

        u = phase.control("u")

        # Simple dynamics for all states
        phase.dynamics(
            {fixed_state: 0, bounded_state: u, lower_bounded: u, upper_bounded: u, free_state: u}
        )

        # Verify boundary constraints were stored correctly
        state_info = phase._phase_def.state_info
        assert state_info[0].initial_constraint.equals == 0  # fixed_state initial
        assert state_info[0].final_constraint.equals == 1  # fixed_state final
        assert state_info[1].boundary_constraint.lower == -5  # bounded_state
        assert state_info[1].boundary_constraint.upper == 5

    def test_control_boundary_constraints(self):
        problem = mtor.Problem("Control Boundaries")
        phase = problem.set_phase(1)

        x = phase.state("x", initial=0)

        # NEW API: Only range tuples allowed for boundary=
        bounded_control = phase.control("bounded", boundary=(0, 100))
        lower_only = phase.control("lower", boundary=(10, None))
        upper_only = phase.control("upper", boundary=(None, 90))
        free_control = phase.control("free")

        # OLD API: boundary=50 for equality - NOW SHOULD FAIL
        with pytest.raises(
            ConfigurationError, match="boundary= argument only accepts range tuples"
        ):
            phase.control("fixed", boundary=50)  # type: ignore[arg-type]

        phase.dynamics({x: bounded_control + lower_only + upper_only + free_control})

        # Verify valid controls were created
        control_info = phase._phase_def.control_info
        assert control_info[0].boundary_constraint.lower == 0
        assert control_info[0].boundary_constraint.upper == 100

    def test_time_boundary_constraints(self):
        problem = mtor.Problem("Time Boundaries")
        phase = problem.set_phase(1)

        # Various time constraint formats
        _t = phase.time(initial=0, final=(5, 10))  # Bounded final time

        x = phase.state("x", initial=0)
        phase.dynamics({x: 1})

        # Verify time constraints
        assert phase._phase_def.t0_constraint.equals == 0
        assert phase._phase_def.tf_constraint.lower == 5
        assert phase._phase_def.tf_constraint.upper == 10


class TestConstraintFailureModes:
    def test_nan_inf_constraint_values_fail(self):
        """Verify NaN/Inf constraint values are rejected."""
        problem = mtor.Problem("NaN Constraints")
        phase = problem.set_phase(1)

        # NaN/Inf should be caught by input validation before reaching _RangeBoundaryConstraint
        with pytest.raises(ConfigurationError, match="NaN|infinite|Invalid"):
            phase.state("x", initial=float("nan"))

        with pytest.raises(ConfigurationError, match="NaN|infinite|Invalid"):
            phase.state("y", final=float("inf"))

        with pytest.raises(ConfigurationError, match="NaN|infinite|Invalid"):
            phase.control("u", boundary=(float("nan"), 10))

    def test_invalid_tuple_constraints_fail(self):
        problem = mtor.Problem("Invalid Tuples")
        phase = problem.set_phase(1)

        # Wrong tuple length - should fail in _process_tuple_constraint_input
        with pytest.raises(
            ConfigurationError, match="Constraint tuple must have 2 elements, got 3"
        ):
            phase.state("x", boundary=(1, 2, 3))  # type: ignore[arg-type]

        # Invalid bound ordering - should fail in _RangeBoundaryConstraint
        with pytest.raises(ConfigurationError):
            phase.state("y", boundary=(10, 5))

    def test_conflicting_constraints_fail(self):
        # This would test the Constraint class validation
        with pytest.raises(
            ValueError, match="Cannot specify equality constraint with bound constraints"
        ):
            Constraint(val=ca.MX.sym("x", 1), equals=5, min_val=0, max_val=10)  # type: ignore[arg-type]

        with pytest.raises(ValueError, match=r"min_val \(10\) must be <= max_val \(5\)"):
            Constraint(val=ca.MX.sym("x", 1), min_val=10, max_val=5)  # type: ignore[arg-type]

    def test_boundary_scalar_values_now_fail(self):
        """Test that old API scalar boundary values now fail."""
        problem = mtor.Problem("Boundary Scalar Rejection")
        phase = problem.set_phase(1)

        # Controls: boundary=scalar should fail
        with pytest.raises(ConfigurationError):
            phase.control("ctrl", boundary=50.0)  # type: ignore[arg-type]

        # States: boundary=scalar should fail
        with pytest.raises(ConfigurationError):
            phase.state("state", boundary=100.0)  # type: ignore[arg-type]

        # Parameters: boundary=scalar should fail
        with pytest.raises(ConfigurationError):
            problem.parameter("param", boundary=9.81)  # type: ignore[arg-type]


class TestSymbolicConstraints:
    def test_automatic_time_continuity(self):
        problem = mtor.Problem("Auto Time Continuity")

        # Phase 1
        p1 = problem.set_phase(1)
        t1 = p1.time(initial=0, final=1)
        x1 = p1.state("x", initial=0)
        p1.dynamics({x1: 1})

        # Phase 2 with symbolic time continuity
        p2 = problem.set_phase(2)
        t2 = p2.time(initial=t1.final, final=2)  # Symbolic linking
        x2 = p2.state("x", initial=0)
        p2.dynamics({x2: 1})

        p1.mesh([3], [-1, 1])
        p2.mesh([3], [-1, 1])

        problem.minimize(t2.final)

        # Force constraint processing
        problem.validate_multiphase_configuration()

        # Should have automatic time continuity constraint
        assert len(problem._multiphase_state.cross_phase_constraints) > 0

    def test_automatic_state_continuity(self):
        problem = mtor.Problem("Auto State Continuity")

        # Phase 1
        p1 = problem.set_phase(1)
        p1.time(initial=0, final=1)
        x1 = p1.state("position", initial=0)
        v1 = p1.state("velocity", initial=10)
        p1.dynamics({x1: v1, v1: 0})

        # Phase 2 with symbolic state continuity
        p2 = problem.set_phase(2)
        p2.time(initial=1, final=2)
        x2 = p2.state("position", initial=x1.final)  # Symbolic linking
        v2 = p2.state("velocity", initial=v1.final)  # Symbolic linking
        p2.dynamics({x2: v2, v2: -1})

        p1.mesh([3], [-1, 1])
        p2.mesh([3], [-1, 1])

        problem.minimize(v2.final)

        # Force constraint processing
        problem.validate_multiphase_configuration()

        # Should have automatic state continuity constraints
        assert len(problem._multiphase_state.cross_phase_constraints) >= 2

    def test_mixed_symbolic_and_explicit_constraints(self):
        problem = mtor.Problem("Mixed Constraints")

        # Phase 1
        p1 = problem.set_phase(1)
        t1 = p1.time(initial=0, final=1)
        h1 = p1.state("altitude", initial=0)
        v1 = p1.state("velocity", initial=0)
        m1 = p1.state("mass", initial=1000)
        T1 = p1.control("thrust")
        p1.dynamics({h1: v1, v1: T1 / m1, m1: -T1 * 0.001})

        # Phase 2 with mixed continuity
        p2 = problem.set_phase(2)
        _t2 = p2.time(initial=t1.final, final=2)  # Automatic time continuity
        h2 = p2.state("altitude", initial=h1.final)  # Automatic altitude continuity
        v2 = p2.state("velocity", initial=0)  # Discontinuous velocity (stage separation)
        m2 = p2.state("mass", initial=500)  # Discontinuous mass (stage separation)
        T2 = p2.control("thrust")
        p2.dynamics({h2: v2, v2: T2 / m2, m2: -T2 * 0.001})

        # Manual constraint for velocity jump
        p2.event_constraints(
            v2.initial >= v1.final - 50,  # Velocity loss bounded
            m2.initial <= m1.final,  # Mass must decrease
        )

        p1.mesh([3], [-1, 1])
        p2.mesh([3], [-1, 1])

        problem.minimize(-h2.final)

        problem.validate_multiphase_configuration()

        # Should have both automatic and manual constraints
        assert len(problem._multiphase_state.cross_phase_constraints) >= 4


class TestConstraintExpressionFormats:
    def test_trigonometric_constraints(self):
        problem = mtor.Problem("Trigonometric Constraints")
        phase = problem.set_phase(1)

        angle = phase.state("angle", initial=0)
        omega = phase.state("angular_velocity", initial=1)
        u = phase.control("torque")

        phase.dynamics({angle: omega, omega: u})

        # Trigonometric path constraints
        phase.path_constraints(
            ca.sin(angle) >= -0.9,  # Avoid large negative angles
            ca.cos(angle) <= 1.1,  # Trigonometric bound
            ca.tan(angle) <= 2,  # Tangent limit
            angle >= -ca.pi,  # Angular bounds
            angle <= ca.pi,
        )

        assert len(phase._phase_def.path_constraints) == 5

    def test_polynomial_constraints(self):
        problem = mtor.Problem("Polynomial Constraints")
        phase = problem.set_phase(1)

        x = phase.state("x", initial=1)
        y = phase.state("y", initial=0)
        u = phase.control("u")

        phase.dynamics({x: y, y: u})

        # Polynomial constraints of various orders
        phase.path_constraints(
            x**2 + y**2 <= 25,  # Circular constraint (quadratic)
            x**3 - 2 * x**2 + x >= 0,  # Cubic constraint
            x**4 + y**4 <= 100,  # Quartic constraint
        )

        assert len(phase._phase_def.path_constraints) == 3

    def test_exponential_logarithmic_constraints(self):
        problem = mtor.Problem("Exponential Constraints")
        phase = problem.set_phase(1)

        x = phase.state("x", initial=1, boundary=(0.1, 10))  # Positive for log
        v = phase.state("v", initial=0)
        u = phase.control("u")

        phase.dynamics({x: v, v: u})

        # Exponential and logarithmic constraints
        phase.path_constraints(
            ca.exp(x) <= 100,  # Exponential upper bound
            ca.log(x) >= -2,  # Logarithmic lower bound (x >= e^-2)
            ca.sqrt(x) <= 3,  # Square root constraint
        )

        assert len(phase._phase_def.path_constraints) == 3
