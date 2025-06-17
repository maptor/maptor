from __future__ import annotations

from dataclasses import dataclass, field
from typing import TypeAlias

import casadi as ca

from ..mtor_types import PhaseID


_PhaseIntervalBundle: TypeAlias = tuple[ca.MX, ca.MX | None]  # state_matrix, interior_nodes


@dataclass
class _PhaseVariable:
    """Container for optimization variable references for a single phase."""

    phase_id: PhaseID
    initial_time: ca.MX
    terminal_time: ca.MX
    state_at_mesh_nodes: list[ca.MX]
    control_variables: list[ca.MX]
    integral_variables: ca.MX | None
    state_matrices: list[ca.MX] = field(default_factory=list)
    interior_variables: list[ca.MX | None] = field(default_factory=list)


@dataclass
class _MultiPhaseVariable:
    """Container for optimization variable references for multiphase problems."""

    phase_variables: dict[PhaseID, _PhaseVariable] = field(default_factory=dict)
    static_parameters: ca.MX | None = None
