from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import numpy as np

from maptor.input_validation import (
    _validate_array_numerical_integrity,
    _validate_integral_values,
)
from maptor.mtor_types import NumericArrayLike


def _validate_and_convert_arrays(arrays: Sequence[Any]) -> list[np.ndarray]:
    return [np.array(arr, dtype=np.float64) for arr in arrays]


def _process_single_or_multi_integral(
    integrals: float | NumericArrayLike, num_integrals: int
) -> float | np.ndarray:
    arr = np.array(integrals)
    if num_integrals == 1:
        return float(arr.item())
    return arr


def _validate_time_values(time_values: dict[int, float] | None, time_type: str) -> None:
    if time_values is not None:
        for phase_id, time_val in time_values.items():
            _validate_array_numerical_integrity(
                np.array([time_val]), f"Phase {phase_id} {time_type} time"
            )


def _set_phase_initial_guess(
    phase_def: Any,  # PhaseDefinition
    states: Sequence[NumericArrayLike] | None = None,
    controls: Sequence[NumericArrayLike] | None = None,
    initial_time: float | None = None,
    terminal_time: float | None = None,
    integrals: float | NumericArrayLike | None = None,
) -> None:
    # Reuse existing validation functions
    validated_states = None
    if states is not None:
        validated_states = _validate_and_convert_arrays(states)

    validated_controls = None
    if controls is not None:
        validated_controls = _validate_and_convert_arrays(controls)

    validated_integrals = None
    if integrals is not None:
        _validate_integral_values(integrals, phase_def.num_integrals)
        validated_integrals = _process_single_or_multi_integral(integrals, phase_def.num_integrals)

    # Validate time values using existing function
    if initial_time is not None:
        _validate_time_values({phase_def.phase_id: initial_time}, "initial")
    if terminal_time is not None:
        _validate_time_values({phase_def.phase_id: terminal_time}, "terminal")

    # Store in phase definition
    phase_def.guess_states = validated_states
    phase_def.guess_controls = validated_controls
    phase_def.guess_initial_time = initial_time
    phase_def.guess_terminal_time = terminal_time
    phase_def.guess_integrals = validated_integrals
