from __future__ import annotations

import logging
from typing import TYPE_CHECKING

import numpy as np


if TYPE_CHECKING:
    from .solution import Solution


logger = logging.getLogger(__name__)


def print_comprehensive_benchmark_summary(solution: Solution) -> None:
    """
    Present adaptive mesh refinement benchmark analysis.

    Args:
        solution: Solution object from adaptive mesh refinement solve
    """
    if not solution.adaptive:
        print("No adaptive refinement data available.")
        return

    print("\n" + "=" * 60)
    print("ADAPTIVE MESH REFINEMENT BENCHMARK")
    print("=" * 60)

    _print_performance_table(solution)
    _print_refinement_strategy_summary(solution)
    _print_efficiency_metrics(solution)

    print("=" * 60 + "\n")


def _print_performance_table(solution: Solution) -> None:
    adaptive_info = solution.adaptive

    # Use single source benchmark data
    benchmark_data = adaptive_info.get("benchmark")
    if benchmark_data is None:
        print("Benchmark data unavailable")
        return

    print(f"Status: {'CONVERGED' if adaptive_info['converged'] else 'NOT CONVERGED'}")
    print(f"Iterations: {adaptive_info['iterations']}")
    print(f"Tolerance: {adaptive_info['target_tolerance']:.1e}")

    print()
    print(f"{'Iteration':>9} | {'Error':>12} | {'Points':>8} | {'Intervals':>9}")
    print("-" * 45)

    for i in range(len(benchmark_data["mesh_iteration"])):
        iteration = benchmark_data["mesh_iteration"][i]
        error = benchmark_data["estimated_error"][i]
        points = benchmark_data["collocation_points"][i]
        intervals = benchmark_data["mesh_intervals"][i]

        iter_label = "Initial" if iteration == 0 else f"{iteration}"
        error_str = "N/A" if np.isnan(error) else f"{error:.3e}"

        print(f"{iter_label:>9} | {error_str:>12} | {points:8d} | {intervals:9d}")

    print("-" * 45)


def _print_refinement_strategy_summary(solution: Solution) -> None:
    adaptive_info = solution.adaptive
    if adaptive_info is None or "iteration_history" not in adaptive_info:
        return

    history = adaptive_info["iteration_history"]
    total_h = 0
    total_p = 0

    for iteration in sorted(history.keys()):
        if iteration == 0:
            continue

        data = history[iteration]
        for strategies in data["refinement_strategy"].values():
            total_h += sum(1 for s in strategies.values() if s == "h")
            total_p += sum(1 for s in strategies.values() if s == "p")

    if total_h > 0 or total_p > 0:
        print(f"\nRefinement Actions: {total_h} h-refinements, {total_p} p-refinements")


def _print_efficiency_metrics(solution: Solution) -> None:
    adaptive_info = solution.adaptive

    # Use single source benchmark data
    benchmark_data = adaptive_info.get("benchmark")
    if benchmark_data is None:
        return

    points = benchmark_data["collocation_points"]
    if len(points) < 2:
        return

    initial_points = points[0]
    final_points = points[-1]
    iterations = len(points) - 1

    errors = benchmark_data["estimated_error"]
    valid_errors = [e for e in errors[1:] if not (np.isnan(e) or np.isinf(e))]

    print(f"\nCollocation Points: {initial_points} → {final_points}")
    if iterations > 0:
        print(f"Points per Iteration: {(final_points - initial_points) / iterations:.1f}")

    if len(valid_errors) >= 2:
        error_reduction = valid_errors[0] / valid_errors[-1]
        print(f"Error Reduction: {error_reduction:.1e}x")
