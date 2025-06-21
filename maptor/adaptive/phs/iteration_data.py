from __future__ import annotations

import logging
from typing import Any

import numpy as np

from maptor.mtor_types import (
    IterationData,
    PhaseID,
    ProblemProtocol,
)


logger = logging.getLogger(__name__)


def _capture_iteration_metrics(
    iteration: int,
    solution: Any,
    problem: ProblemProtocol,
    adaptive_state: Any,
    phase_errors: dict[PhaseID, list[float]],
    refinement_actions: dict[PhaseID, dict[int, tuple[str, Any]]],
) -> IterationData:
    """Capture exact iteration metrics for research benchmarking."""
    phase_collocation_points = {}
    phase_mesh_intervals = {}
    refinement_strategy = {}
    total_collocation_points = 0

    for phase_id in problem._get_phase_ids():
        current_degrees = adaptive_state.phase_polynomial_degrees[phase_id]

        phase_colloc = sum(current_degrees)
        phase_collocation_points[phase_id] = phase_colloc
        total_collocation_points += phase_colloc

        phase_mesh_intervals[phase_id] = len(current_degrees)

        if phase_id in refinement_actions:
            strategy = {}
            for interval_idx, (strategy_type, _) in refinement_actions[phase_id].items():
                strategy[interval_idx] = strategy_type
            refinement_strategy[phase_id] = strategy
        else:
            refinement_strategy[phase_id] = {}

    all_finite_errors = []
    for _phase_id, errors in phase_errors.items():
        for error in errors:
            if not (np.isnan(error) or np.isinf(error)):
                all_finite_errors.append(error)

    max_error = max(all_finite_errors) if all_finite_errors else float("inf")

    return IterationData(
        iteration=iteration,
        phase_error_estimates={pid: errors.copy() for pid, errors in phase_errors.items()},
        phase_collocation_points=phase_collocation_points,
        phase_mesh_intervals=phase_mesh_intervals,
        phase_polynomial_degrees={
            pid: degrees.copy() for pid, degrees in adaptive_state.phase_polynomial_degrees.items()
        },
        phase_mesh_nodes={
            pid: nodes.copy() for pid, nodes in adaptive_state.phase_mesh_points.items()
        },
        refinement_strategy=refinement_strategy,
        total_collocation_points=total_collocation_points,
        max_error_all_phases=max_error,
        convergence_status=adaptive_state.phase_converged.copy(),
    )
