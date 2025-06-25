import pytest

import maptor as mtor
from maptor.exceptions import ConfigurationError


class TestStateBoundaryChanges:
    def test_state_boundary_accepts_ranges(self):
        problem = mtor.Problem("State Range Test")
        phase = problem.set_phase(1)

        # All valid range formats
        h1 = phase.state("altitude", boundary=(0, 10000))  # Both bounds
        h2 = phase.state("velocity", boundary=(None, 250))  # Upper only
        h3 = phase.state("mass", boundary=(100, None))  # Lower only
        h4 = phase.state("position", boundary=None)  # No bounds

        # Should not raise any exceptions
        assert h1 is not None
        assert h2 is not None
        assert h3 is not None
        assert h4 is not None

    def test_state_boundary_rejects_equality(self):
        problem = mtor.Problem("State Equality Reject Test")
        phase = problem.set_phase(1)

        with pytest.raises(ConfigurationError):
            phase.state("altitude", boundary=1000.0)  # type: ignore[arg-type] # Should fail - no equality

    def test_state_boundary_rejects_symbolic(self):
        problem = mtor.Problem("State Symbolic Reject Test")
        phase = problem.set_phase(1)

        h1 = phase.state("alt1", initial=0)

        with pytest.raises(ConfigurationError):
            phase.state("alt2", boundary=h1.final)  # type: ignore[arg-type] # Should fail - no symbolic

    def test_state_initial_final_still_support_symbolic(self):
        problem = mtor.Problem("State Symbolic Linking Test")
        phase1 = problem.set_phase(1)
        phase2 = problem.set_phase(2)

        # Phase 1
        h1 = phase1.state("altitude", initial=0, final=None)

        # Phase 2 - symbolic linking should still work
        h2 = phase2.state("altitude", initial=h1.final, final=15000)

        assert h2 is not None

    def test_state_initial_final_numeric_constraints(self):
        problem = mtor.Problem("State Numeric Test")
        phase = problem.set_phase(1)

        # All valid numeric constraint formats
        h1 = phase.state("alt1", initial=0.0, final=1000.0)  # Fixed values
        h2 = phase.state("alt2", initial=(0, 10), final=(990, 1010))  # Ranges
        h3 = phase.state("alt3", initial=(None, 5), final=(800, None))  # Single-sided
        h4 = phase.state("alt4", initial=None, final=None)  # Free

        assert all(s is not None for s in [h1, h2, h3, h4])


class TestControlBoundaryChanges:
    def test_control_boundary_accepts_ranges(self):
        problem = mtor.Problem("Control Range Test")
        phase = problem.set_phase(1)

        # All valid range formats
        u1 = phase.control("thrust", boundary=(0, 2000))  # Both bounds
        u2 = phase.control("brake", boundary=(None, 100))  # Upper only
        u3 = phase.control("power", boundary=(0, None))  # Lower only
        u4 = phase.control("free", boundary=None)  # No bounds

        assert all(u is not None for u in [u1, u2, u3, u4])

    def test_control_boundary_rejects_equality(self):
        problem = mtor.Problem("Control Equality Reject Test")
        phase = problem.set_phase(1)

        with pytest.raises(ConfigurationError):
            phase.control("thrust", boundary=500.0)  # type: ignore[arg-type] # Should fail - no equality

    def test_control_boundary_rejects_symbolic(self):
        problem = mtor.Problem("Control Symbolic Reject Test")
        phase = problem.set_phase(1)

        param = problem.parameter("max_thrust", fixed=2000.0)

        with pytest.raises(ConfigurationError):
            phase.control("thrust", boundary=param)  # type: ignore[arg-type]  # Should fail - no symbolic


class TestParameterBoundaryAndFixed:
    def test_parameter_boundary_accepts_ranges(self):
        problem = mtor.Problem("Parameter Range Test")

        # All valid range formats for optimization
        p1 = problem.parameter("mass", boundary=(100, 500))  # Both bounds
        p2 = problem.parameter("drag", boundary=(None, 0.1))  # Upper only
        p3 = problem.parameter("area", boundary=(10, None))  # Lower only
        p4 = problem.parameter("free", boundary=None)  # No bounds

        assert all(p is not None for p in [p1, p2, p3, p4])

    def test_parameter_fixed_accepts_constants(self):
        problem = mtor.Problem("Parameter Fixed Constants Test")

        # Fixed numeric values
        p1 = problem.parameter("gravity", fixed=9.81)
        p2 = problem.parameter("pi", fixed=3.14159)
        p3 = problem.parameter("zero", fixed=0.0)

        assert all(p is not None for p in [p1, p2, p3])

    def test_parameter_fixed_accepts_symbolic(self):
        problem = mtor.Problem("Parameter Fixed Symbolic Test")

        # Symbolic relationships
        mass1 = problem.parameter("mass1", boundary=(100, 500))
        mass2 = problem.parameter("mass2", fixed=mass1 * 2.0)  # Relationship
        mass3 = problem.parameter("mass3", fixed=mass1 + mass2)  # Complex relationship

        assert all(p is not None for p in [mass1, mass2, mass3])

    def test_parameter_boundary_rejects_equality(self):
        problem = mtor.Problem("Parameter Boundary Equality Reject Test")

        with pytest.raises(ConfigurationError):
            problem.parameter("mass", boundary=1000.0)  # type: ignore[arg-type] # Should fail - use fixed=

    def test_parameter_rejects_both_boundary_and_fixed(self):
        problem = mtor.Problem("Parameter Both Constraints Test")

        with pytest.raises(ConfigurationError, match="cannot have both boundary and fixed"):
            problem.parameter("mass", boundary=(100, 500), fixed=300.0)

    def test_parameter_allows_neither_constraint(self):
        problem = mtor.Problem("Parameter No Constraints Test")

        # Should work - unconstrained parameter
        param = problem.parameter("free_param")
        assert param is not None


class TestConstraintValidation:
    def test_invalid_boundary_tuple_formats(self):
        problem = mtor.Problem("Invalid Boundary Test")
        phase = problem.set_phase(1)

        # Invalid tuple formats should fail
        with pytest.raises(ConfigurationError):
            phase.state("bad1", boundary=(1, 2, 3))  # type: ignore[arg-type]  # Too many elements

        with pytest.raises(ConfigurationError):
            phase.state("bad2", boundary=(5,))  # type: ignore[arg-type] # Too few elements

        with pytest.raises(ConfigurationError):
            phase.state("bad3", boundary="invalid")  # type: ignore[arg-type] # Wrong type

    def test_invalid_fixed_formats(self):
        problem = mtor.Problem("Invalid Fixed Test")

        # Invalid fixed formats should fail
        with pytest.raises(ConfigurationError):
            problem.parameter("bad1", fixed=(100, 500))  # type: ignore[arg-type]  # Should be boundary=

        with pytest.raises(ConfigurationError):
            problem.parameter("bad2", fixed="invalid")  # type: ignore[arg-type]  # Wrong type

    def test_boundary_range_validation(self):
        problem = mtor.Problem("Range Validation Test")
        phase = problem.set_phase(1)

        # Lower > upper should fail in validation
        with pytest.raises(ConfigurationError):
            phase.state("bad_range", boundary=(100, 50))  # Invalid range


class TestCompleteWorkingExample:
    def test_complete_problem_definition(self):
        problem = mtor.Problem("Complete API Test")
        phase = problem.set_phase(1)

        # Time
        t = phase.time(initial=0.0, final=10.0)

        # States with clean boundary syntax
        h = phase.state("altitude", initial=0, final=1000, boundary=(0, None))
        v = phase.state("velocity", initial=0, boundary=(None, 200))
        m = phase.state("mass", initial=1000, boundary=(100, 1000))

        # Controls with clean boundary syntax
        thrust = phase.control("thrust", boundary=(0, 2000))

        # Parameters with clean separation
        drag_coeff = problem.parameter("drag", boundary=(0.01, 0.1))  # Optimize
        gravity = problem.parameter("gravity", fixed=9.81)  # Fixed
        fuel_ratio = problem.parameter("fuel_ratio", fixed=drag_coeff * 10.0)  # Relationship

        # Dynamics
        phase.dynamics({h: v, v: thrust / m - gravity, m: -thrust * 0.001})

        # Objective
        problem.minimize(-h.final)

        # Mesh
        phase.mesh([5], [-1, 1])

        # Should validate without errors
        problem.validate_multiphase_configuration()

        # Verify all variables were created correctly
        assert t is not None
        assert h is not None
        assert v is not None
        assert m is not None
        assert thrust is not None
        assert drag_coeff is not None
        assert gravity is not None
        assert fuel_ratio is not None

    def test_multiphase_symbolic_linking_still_works(self):
        problem = mtor.Problem("Multiphase Linking Test")

        # Phase 1
        phase1 = problem.set_phase(1)
        t1 = phase1.time(initial=0, final=120)
        h1 = phase1.state("altitude", initial=0, boundary=(0, None))
        v1 = phase1.state("velocity", initial=0)
        u1 = phase1.control("thrust", boundary=(0, 2000))

        phase1.dynamics({h1: v1, v1: u1 - 9.81})

        # Phase 2 - symbolic continuity should still work
        phase2 = problem.set_phase(2)
        _t2 = phase2.time(initial=t1.final, final=300)  # Time continuity
        h2 = phase2.state("altitude", initial=h1.final)  # State continuity
        v2 = phase2.state("velocity", initial=v1.final)  # State continuity
        u2 = phase2.control("thrust", boundary=(0, 1000))

        phase2.dynamics({h2: v2, v2: u2 - 9.81})

        # Objective
        problem.minimize(-h2.final)

        # Mesh
        phase1.mesh([3], [-1, 1])
        phase2.mesh([3], [-1, 1])

        # Should validate - symbolic linking preserved
        problem.validate_multiphase_configuration()


class TestBackwardCompatibilityBreaks:
    def test_old_state_boundary_equality_fails(self):
        problem = mtor.Problem("Old State API Test")
        phase = problem.set_phase(1)

        with pytest.raises(ConfigurationError):
            # This used to work but should now fail
            phase.state("altitude", boundary=1000.0)  # type: ignore[arg-type]

    def test_old_control_boundary_equality_fails(self):
        problem = mtor.Problem("Old Control API Test")
        phase = problem.set_phase(1)

        with pytest.raises(ConfigurationError):
            # This used to work but should now fail
            phase.control("thrust", boundary=500.0)  # type: ignore[arg-type]

    def test_old_parameter_boundary_equality_fails(self):
        problem = mtor.Problem("Old Parameter API Test")

        with pytest.raises(ConfigurationError):
            # This used to work but should now fail
            problem.parameter("mass", boundary=1000.0)  # type: ignore[arg-type]


if __name__ == "__main__":
    # Run with: python -m pytest test_boundary_api_changes.py -v
    pytest.main([__file__, "-v"])
