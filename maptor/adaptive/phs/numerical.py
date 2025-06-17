__all__ = [
    "_map_global_normalized_tau_to_local_interval_tau",
    "_map_local_interval_tau_to_global_normalized_tau",
    "_map_local_tau_from_interval_k_plus_1_to_equivalent_in_interval_k",
    "_map_local_tau_from_interval_k_to_equivalent_in_interval_k_plus_1",
]

from maptor.utils.constants import COORDINATE_PRECISION


def _compute_interval_parameters(global_start: float, global_end: float) -> tuple[float, float]:
    beta = (global_end - global_start) / 2.0
    beta0 = (global_end + global_start) / 2.0
    return beta, beta0


def _map_global_normalized_tau_to_local_interval_tau(
    global_tau: float, global_start: float, global_end: float
) -> float:
    beta, beta0 = _compute_interval_parameters(global_start, global_end)

    if abs(beta) < COORDINATE_PRECISION:
        return 0.0

    return (global_tau - beta0) / beta


def _map_local_interval_tau_to_global_normalized_tau(
    local_tau: float, global_start: float, global_end: float
) -> float:
    beta, beta0 = _compute_interval_parameters(global_start, global_end)
    return beta * local_tau + beta0


def _map_local_tau_from_interval_k_to_equivalent_in_interval_k_plus_1(
    local_tau_k: float, global_start_k: float, global_shared: float, global_end_kp1: float
) -> float:
    global_tau = _map_local_interval_tau_to_global_normalized_tau(
        local_tau_k, global_start_k, global_shared
    )
    return _map_global_normalized_tau_to_local_interval_tau(
        global_tau, global_shared, global_end_kp1
    )


def _map_local_tau_from_interval_k_plus_1_to_equivalent_in_interval_k(
    local_tau_kp1: float, global_start_k: float, global_shared: float, global_end_kp1: float
) -> float:
    global_tau = _map_local_interval_tau_to_global_normalized_tau(
        local_tau_kp1, global_shared, global_end_kp1
    )
    return _map_global_normalized_tau_to_local_interval_tau(
        global_tau, global_start_k, global_shared
    )
