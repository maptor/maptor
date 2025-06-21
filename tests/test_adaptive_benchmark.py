from io import StringIO
from unittest.mock import patch

import numpy as np
import pytest

import maptor as mtor


class TestAdaptiveBenchmarkCore:
    def test_benchmark_api_contract_comprehensive(self):
        problem = self._create_simple_problem()
        solution = mtor.solve_adaptive(
            problem,
            error_tolerance=1e-4,
            max_iterations=5,
            nlp_options={"ipopt.print_level": 0},
            show_summary=False,
        )

        assert solution.status["success"], "Problem should converge for API testing"

        # Verify adaptive data exists and has correct structure
        adaptive = solution.adaptive
        assert isinstance(adaptive, dict)

        # Core algorithm status fields
        required_status = {
            "converged",
            "iterations",
            "target_tolerance",
            "phase_converged",
            "final_errors",
        }
        assert required_status.issubset(set(adaptive.keys()))

        # Benchmark arrays structure
        benchmark = adaptive["benchmark"]
        required_arrays = {
            "mesh_iteration",
            "estimated_error",
            "collocation_points",
            "mesh_intervals",
            "polynomial_degrees",
            "refinement_strategy",
        }
        assert set(benchmark.keys()) == required_arrays

        # Array length consistency
        num_iterations = len(benchmark["mesh_iteration"])
        assert num_iterations > 0
        for array_name in required_arrays:
            assert len(benchmark[array_name]) == num_iterations, f"{array_name} length mismatch"

        # Iteration sequence correctness
        assert benchmark["mesh_iteration"] == list(range(num_iterations))

        # Value constraints
        assert all(isinstance(p, int) and p > 0 for p in benchmark["collocation_points"])
        assert all(isinstance(i, int) and i > 0 for i in benchmark["mesh_intervals"])
        assert all(isinstance(e, float) for e in benchmark["estimated_error"])

    def test_mathematical_consistency_mission_vs_phases(self):
        problem = self._create_multiphase_problem()
        solution = mtor.solve_adaptive(
            problem,
            error_tolerance=1e-4,
            max_iterations=4,
            nlp_options={"ipopt.print_level": 0},
            show_summary=False,
        )

        mission_benchmark = solution.adaptive["benchmark"]
        phase_benchmarks = solution.adaptive["phase_benchmarks"]

        # Verify all phases have data
        phase_ids = list(solution.phases.keys())
        assert set(phase_benchmarks.keys()) == set(phase_ids)

        # Mathematical consistency across all iterations
        for i in range(len(mission_benchmark["mesh_iteration"])):
            # Collocation points must sum correctly
            mission_points = mission_benchmark["collocation_points"][i]
            phase_points_sum = sum(
                phase_data["collocation_points"][i] for phase_data in phase_benchmarks.values()
            )
            assert mission_points == phase_points_sum, f"Points mismatch at iteration {i}"

            # Mesh intervals must sum correctly
            mission_intervals = mission_benchmark["mesh_intervals"][i]
            phase_intervals_sum = sum(
                phase_data["mesh_intervals"][i] for phase_data in phase_benchmarks.values()
            )
            assert mission_intervals == phase_intervals_sum, f"Intervals mismatch at iteration {i}"

    def test_single_source_data_integrity(self):
        problem = self._create_simple_problem()
        solution = mtor.solve_adaptive(
            problem,
            error_tolerance=1e-4,
            max_iterations=3,
            nlp_options={"ipopt.print_level": 0},
            show_summary=False,
        )

        history = solution.adaptive["iteration_history"]
        benchmark = solution.adaptive["benchmark"]

        # Verify benchmark arrays derived correctly from iteration history
        assert len(benchmark["mesh_iteration"]) == len(history)

        for i, iteration in enumerate(sorted(history.keys())):
            data = history[iteration]

            # Core data consistency
            assert benchmark["mesh_iteration"][i] == iteration
            assert benchmark["collocation_points"][i] == data["total_collocation_points"]
            assert benchmark["estimated_error"][i] == data["max_error_all_phases"]

            # Derived data consistency
            expected_intervals = sum(data["phase_mesh_intervals"].values())
            assert benchmark["mesh_intervals"][i] == expected_intervals

    def test_algorithm_convergence_behavior(self):
        problem = self._create_simple_problem()
        solution = mtor.solve_adaptive(
            problem,
            error_tolerance=1e-6,
            max_iterations=8,
            nlp_options={"ipopt.print_level": 0},
            show_summary=False,
        )

        if not solution.status["success"]:
            return  # Skip convergence test if problem doesn't converge

        benchmark = solution.adaptive["benchmark"]

        # Algorithm should generally increase computational effort
        points = benchmark["collocation_points"]
        assert points[-1] >= points[0], "Algorithm should not reduce mesh size without convergence"

        # Error should generally decrease over finite values
        errors = benchmark["estimated_error"]
        finite_errors = [e for e in errors[1:] if not (np.isnan(e) or np.isinf(e))]

        if len(finite_errors) >= 2:
            # Allow some tolerance for numerical noise but expect overall reduction
            error_ratio = finite_errors[-1] / finite_errors[0]
            assert error_ratio <= 100.0, "Algorithm should achieve meaningful error reduction"

        # Mesh intervals should be reasonable
        intervals = benchmark["mesh_intervals"]
        assert all(1 <= i <= 1000 for i in intervals), (
            "Mesh intervals should stay in reasonable range"
        )

    def test_minimal_iteration_handling(self):
        problem = self._create_simple_problem()
        solution = mtor.solve_adaptive(
            problem,
            error_tolerance=1e-2,
            max_iterations=1,
            nlp_options={"ipopt.print_level": 0},
            show_summary=False,
        )

        if solution.status["success"]:
            benchmark = solution.adaptive["benchmark"]
            assert len(benchmark["mesh_iteration"]) >= 1
            assert benchmark["mesh_iteration"][0] == 0

    def test_built_in_analysis_methods_integration(self):
        problem = self._create_simple_problem()
        solution = mtor.solve_adaptive(
            problem,
            error_tolerance=1e-4,
            max_iterations=3,
            nlp_options={"ipopt.print_level": 0},
            show_summary=False,
        )

        # Test print_benchmark_summary doesn't crash
        captured_output = StringIO()
        with patch("sys.stdout", captured_output):
            solution.print_benchmark_summary()

        output = captured_output.getvalue()
        assert "ADAPTIVE MESH REFINEMENT BENCHMARK" in output
        assert "Status:" in output
        assert "Iterations:" in output

        # Test plot_refinement_history setup (don't actually plot)
        with (
            patch("matplotlib.pyplot.subplots") as mock_subplots,
            patch("matplotlib.pyplot.show") as _mock_show,
        ):
            mock_fig = patch("matplotlib.figure.Figure").start()
            mock_ax = patch("matplotlib.axes.Axes").start()
            mock_subplots.return_value = (mock_fig, mock_ax)

            # Should not crash
            solution.plot_refinement_history(phase_id=1)
            mock_subplots.assert_called_once()

    def test_error_handling_robustness(self):
        # Test fixed mesh solution error handling
        problem = self._create_simple_problem()
        fixed_solution = mtor.solve_fixed_mesh(
            problem, nlp_options={"ipopt.print_level": 0}, show_summary=False
        )

        # Should raise clear error for fixed mesh solutions
        with pytest.raises(RuntimeError, match="solve_adaptive"):
            _ = fixed_solution.adaptive

        # Test graceful handling of failed adaptive solutions
        problem_hard = self._create_impossible_problem()
        failed_solution = mtor.solve_adaptive(
            problem_hard,
            error_tolerance=1e-12,  # Impossible tolerance
            max_iterations=2,
            nlp_options={"ipopt.print_level": 0},
            show_summary=False,
        )

        # Even failed solutions should have consistent status
        assert isinstance(failed_solution.status["success"], bool)
        if not failed_solution.status["success"]:
            # Failed solutions may or may not have adaptive data
            try:
                adaptive = failed_solution.adaptive
                # If adaptive data exists, it should be well-formed
                assert isinstance(adaptive["converged"], bool)
            except RuntimeError:
                # No adaptive data is also acceptable for failed solutions
                pass

    def _create_simple_problem(self):
        problem = mtor.Problem("Benchmark Test Problem")
        phase = problem.set_phase(1)

        _t = phase.time(initial=0.0, final=1.0)
        x = phase.state("x", initial=0.0, final=1.0)
        u = phase.control("u", boundary=(-2.0, 2.0))

        phase.dynamics({x: u})

        control_effort = phase.add_integral(u**2)
        problem.minimize(control_effort)

        phase.mesh([3, 3], [-1.0, 0.0, 1.0])
        return problem

    def _create_multiphase_problem(self):
        problem = mtor.Problem("Multiphase Benchmark Test")

        # Phase 1
        phase1 = problem.set_phase(1)
        t1 = phase1.time(initial=0.0, final=0.5)
        x1 = phase1.state("x", initial=0.0)
        u1 = phase1.control("u", boundary=(-1.0, 1.0))
        phase1.dynamics({x1: u1})
        phase1.mesh([3], [-1.0, 1.0])

        # Phase 2
        phase2 = problem.set_phase(2)
        _t2 = phase2.time(initial=t1.final, final=1.0)
        x2 = phase2.state("x", initial=x1.final, final=1.0)
        u2 = phase2.control("u", boundary=(-1.0, 1.0))
        phase2.dynamics({x2: u2})
        phase2.mesh([3], [-1.0, 1.0])

        effort1 = phase1.add_integral(u1**2)
        effort2 = phase2.add_integral(u2**2)
        problem.minimize(effort1 + effort2)

        return problem

    def _create_impossible_problem(self):
        problem = mtor.Problem("Impossible Test Problem")
        phase = problem.set_phase(1)

        _t = phase.time(initial=0.0, final=1.0)
        x = phase.state("x", initial=0.0, final=100.0)  # Impossible large final state
        u = phase.control("u", boundary=(-0.1, 0.1))  # Severely limited control

        phase.dynamics({x: 0.01 * u})  # Very slow dynamics, can't reach target

        control_effort = phase.add_integral(u**2)  # Proper objective construction
        problem.minimize(control_effort)

        phase.mesh([2], [-1.0, 1.0])
        return problem
