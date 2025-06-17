import logging

import numpy as np

from ..input_validation import _validate_mesh_configuration
from ..mtor_types import NumericArrayLike
from .state import PhaseDefinition


logger = logging.getLogger(__name__)


def _configure_phase_mesh(
    phase_def: PhaseDefinition, polynomial_degrees: list[int], mesh_points: NumericArrayLike
) -> None:
    mesh_array = np.asarray(mesh_points, dtype=np.float64)

    logger.debug(
        "Configuring mesh for phase %d: %d degrees, %d points",
        phase_def.phase_id,
        len(polynomial_degrees),
        len(mesh_array),
    )

    _validate_mesh_configuration(polynomial_degrees, mesh_array, len(polynomial_degrees))

    phase_def.collocation_points_per_interval = polynomial_degrees
    phase_def.global_normalized_mesh_nodes = mesh_array
    phase_def.mesh_configured = True

    logger.debug(
        "Mesh configuration complete for phase %d: intervals=%d, total_nodes=%d",
        phase_def.phase_id,
        len(polynomial_degrees),
        len(mesh_array),
    )
