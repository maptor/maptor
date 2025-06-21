from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import numpy as np


if TYPE_CHECKING:
    from .solution import Solution


logger = logging.getLogger(__name__)


def print_comprehensive_solution_summary(solution: Solution) -> None:
    """
    Present comprehensive factual solution data for advanced control engineers.

    Args:
        solution: Solution object containing multiphase optimization results
    """
    print("\n" + "=" * 100)
    print("MAPTOR OPTIMAL CONTROL SOLUTION REPORT")
    print("=" * 100)

    _print_problem_configuration_data(solution)
    _print_optimization_status_data(solution)
    _print_solver_configuration_data(solution)
    _print_static_parameters_data(solution)
    _print_multiphase_timing_data(solution)
    _print_multiphase_variables_data(solution)
    _print_mesh_configuration_data(solution)
    _print_integral_results_data(solution)
    _print_adaptive_refinement_data(solution)
    _print_raw_solver_data(solution)

    print("=" * 100)
    print("END SOLUTION REPORT")
    print("=" * 100 + "\n")


def _print_problem_configuration_data(solution: Solution) -> None:
    print("\n" + "=" * 50)
    print("PROBLEM CONFIGURATION")
    print("=" * 50)

    if solution._problem is not None:
        problem_name = getattr(solution._problem, "name", "Optimal Control Problem")
        print(f"Problem Name: {problem_name}")
    else:
        print("Problem Name: Not available")

    phases = solution.phases
    print(f"Number of Phases: {len(phases)}")

    total_states = sum(phase_data["variables"]["num_states"] for phase_data in phases.values())
    total_controls = sum(phase_data["variables"]["num_controls"] for phase_data in phases.values())

    print(f"Total State Variables: {total_states}")
    print(f"Total Control Variables: {total_controls}")

    parameters = solution.parameters
    num_static_params = parameters["count"] if parameters else 0
    print(f"Static Parameters: {num_static_params}")


def _print_optimization_status_data(solution: Solution) -> None:
    print("\n" + "=" * 50)
    print("OPTIMIZATION STATUS")
    print("=" * 50)

    status = solution.status
    print(f"Solver Success: {status['success']}")
    print(f"Solver Message: {status['message']}")

    if status["success"]:
        print(f"Objective Value: {status['objective']:.16e}")
        print(f"Total Mission Duration: {status['total_mission_time']:.16e} time units")
    else:
        print("Objective Value: Not available (solver failed)")
        print("Total Mission Duration: Not available (solver failed)")


def _print_solver_configuration_data(solution: Solution) -> None:
    print("\n" + "=" * 50)
    print("SOLVER CONFIGURATION")
    print("=" * 50)

    if solution._problem is not None and hasattr(solution._problem, "solver_options"):
        solver_opts = solution._problem.solver_options
        if solver_opts:
            print("NLP Solver Options:")
            for key, value in solver_opts.items():
                print(f"  {key}: {value}")
        else:
            print("NLP Solver Options: Default IPOPT settings")
    else:
        print("NLP Solver Options: Not available")


def _print_static_parameters_data(solution: Solution) -> None:
    parameters = solution.parameters
    if parameters is None or parameters["count"] == 0:
        return

    print("\n" + "=" * 50)
    print("OPTIMIZED STATIC PARAMETERS")
    print("=" * 50)

    print(f"Parameter Count: {parameters['count']}")

    param_names = parameters["names"]
    param_values = parameters["values"]

    for i, value in enumerate(param_values):
        param_name = (
            f"parameter_{i + 1}" if param_names is None or i >= len(param_names) else param_names[i]
        )
        print(f"{param_name}: {value:.16e}")


def _print_multiphase_timing_data(solution: Solution) -> None:
    print("\n" + "=" * 50)
    print("PHASE TIMING DATA")
    print("=" * 50)

    phases = solution.phases
    if not phases:
        print("No timing data available")
        return

    for phase_id in sorted(phases.keys()):
        phase_data = phases[phase_id]
        times = phase_data["times"]

        print(f"\nPhase {phase_id}:")

        if not np.isnan(times["initial"]):
            print(f"  Initial Time: {times['initial']:.16e}")
            print(f"  Final Time: {times['final']:.16e}")
            print(f"  Duration: {times['duration']:.16e}")
        else:
            print("  Timing: Not available")


def _print_multiphase_variables_data(solution: Solution) -> None:
    print("\n" + "=" * 50)
    print("PHASE VARIABLES DATA")
    print("=" * 50)

    phases = solution.phases
    if not phases:
        print("No variable data available")
        return

    for phase_id in sorted(phases.keys()):
        phase_data = phases[phase_id]
        variables = phase_data["variables"]

        print(f"\nPhase {phase_id}:")
        print(f"  State Count: {variables['num_states']}")
        print(f"  Control Count: {variables['num_controls']}")

        if variables["state_names"]:
            print(f"  State Variables: {variables['state_names']}")

        if variables["control_names"]:
            print(f"  Control Variables: {variables['control_names']}")


def _print_mesh_configuration_data(solution: Solution) -> None:
    print("\n" + "=" * 50)
    print("MESH CONFIGURATION DATA")
    print("=" * 50)

    phases = solution.phases
    if not phases:
        print("No mesh data available")
        return

    total_intervals = 0
    total_collocation_points = 0

    for phase_id in sorted(phases.keys()):
        phase_data = phases[phase_id]
        mesh_data = phase_data["mesh"]

        print(f"\nPhase {phase_id}:")

        polynomial_degrees = mesh_data["polynomial_degrees"]
        mesh_nodes = mesh_data["mesh_nodes"]
        num_intervals = mesh_data["num_intervals"]

        total_intervals += num_intervals

        print(f"  Number of Intervals: {num_intervals}")

        if polynomial_degrees:
            print(f"  Polynomial Degrees: {polynomial_degrees}")
            phase_collocation_points = sum(polynomial_degrees)
            total_collocation_points += phase_collocation_points
            print(f"  Collocation Points: {phase_collocation_points}")
        else:
            print("  Polynomial Degrees: Not available")
            print("  Collocation Points: Not available")

        if mesh_nodes is not None and len(mesh_nodes) > 0:
            mesh_str = "[" + ", ".join(f"{node:.6f}" for node in mesh_nodes) + "]"
            print(f"  Mesh Node Distribution: {mesh_str}")
        else:
            print("  Mesh Node Distribution: Not available")

    print(f"\nTotal Intervals Across All Phases: {total_intervals}")
    print(f"Total Collocation Points Across All Phases: {total_collocation_points}")

    adaptive = solution.adaptive
    if adaptive is not None:
        print("\nAdaptive Mesh Refinement Applied:")
        print(f"  Final mesh represents iteration {adaptive['iterations']} configuration")
        print(f"  Target tolerance: {adaptive['target_tolerance']:.3e}")


def _print_integral_results_data(solution: Solution) -> None:
    phases = solution.phases
    has_integrals = any(phase_data["integrals"] is not None for phase_data in phases.values())

    if not has_integrals:
        return

    print("\n" + "=" * 50)
    print("INTEGRAL RESULTS DATA")
    print("=" * 50)

    for phase_id in sorted(phases.keys()):
        phase_data = phases[phase_id]
        integrals = phase_data["integrals"]

        if integrals is not None:
            print(f"\nPhase {phase_id}:")

            if isinstance(integrals, int | float):
                print(f"  Integral Value: {integrals:.16e}")
            elif hasattr(integrals, "__len__"):
                for i, integral_val in enumerate(integrals):
                    print(f"  Integral {i + 1}: {integral_val:.16e}")
            else:
                print(f"  Integral: {integrals}")


def _print_adaptive_refinement_data(solution: Solution) -> None:
    adaptive = solution.adaptive

    print("\n" + "=" * 50)
    print("ADAPTIVE REFINEMENT DATA")
    print("=" * 50)

    print(f"Algorithm Converged: {adaptive['converged']}")
    print(f"Total Iterations: {adaptive['iterations']}")
    print(f"Target Error Tolerance: {adaptive['target_tolerance']:.3e}")

    print("\nPer-Phase Convergence Status:")
    for phase_id in sorted(adaptive["phase_converged"].keys()):
        converged = adaptive["phase_converged"][phase_id]
        status_symbol = "✓" if converged else "✗"
        print(f"  Phase {phase_id}: {status_symbol} {converged}")

    print("\nFinal Error Estimates by Interval:")
    all_finite_errors = []

    for phase_id in sorted(adaptive["final_errors"].keys()):
        errors = adaptive["final_errors"][phase_id]
        print(f"  Phase {phase_id}:")

        phase_finite_errors = []
        for k, error in enumerate(errors):
            if np.isnan(error) or np.isinf(error):
                error_str = f"{error}"
            else:
                error_str = f"{error:.6e}"
                phase_finite_errors.append(error)
                all_finite_errors.append(error)
            print(f"    Interval {k}: {error_str}")

        if phase_finite_errors:
            phase_max = max(phase_finite_errors)
            phase_avg = np.mean(phase_finite_errors)
            print(f"    Phase Maximum Error: {phase_max:.6e}")
            print(f"    Phase Average Error: {phase_avg:.6e}")

    if all_finite_errors:
        global_max = max(all_finite_errors)
        global_avg = np.mean(all_finite_errors)
        print("\nGlobal Error Statistics:")
        print(f"  Maximum Error: {global_max:.6e}")
        print(f"  Average Error: {global_avg:.6e}")

        convergence_margin = (
            adaptive["target_tolerance"] / global_max if global_max > 0 else float("inf")
        )
        print(f"  Convergence Margin: {convergence_margin:.3f}x tolerance")

    gamma_factors = adaptive.get("gamma_factors", {})
    if gamma_factors:
        print(f"\nGamma Normalization Factors Available: {len(gamma_factors)} phases")


def _print_raw_solver_data(solution: Solution) -> None:
    print("\n" + "=" * 50)
    print("RAW SOLVER DATA ACCESS")
    print("=" * 50)

    print(f"Raw CasADi Solution Available: {solution.raw_solution is not None}")
    print(f"CasADi Opti Object Available: {solution.opti is not None}")

    if solution.raw_solution is not None:
        print("Raw solver data accessible for advanced analysis")

    phases = solution.phases
    trajectory_data_available = all(
        len(phase_data["time_arrays"]["states"]) > 0 for phase_data in phases.values()
    )

    print(f"Trajectory Data Extracted: {trajectory_data_available}")

    if trajectory_data_available:
        total_state_points = sum(
            len(phase_data["time_arrays"]["states"]) for phase_data in phases.values()
        )
        total_control_points = sum(
            len(phase_data["time_arrays"]["controls"]) for phase_data in phases.values()
        )
        print(f"Total State Trajectory Points: {total_state_points}")
        print(f"Total Control Trajectory Points: {total_control_points}")
