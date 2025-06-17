import casadi as ca

from ..mtor_types import FloatArray


def _tau_to_time(
    tau: float | FloatArray | ca.MX,
    mesh_start: float,
    mesh_end: float,
    time_start: float | ca.MX,
    time_end: float | ca.MX,
) -> float | FloatArray | ca.MX:
    # Combined transformation: tau → global_tau → physical_time
    # physical_time = time_scale * (mesh_scale * tau + mesh_offset) + time_offset
    return (
        (time_end - time_start) * ((mesh_end - mesh_start) * tau + (mesh_end + mesh_start))
        + 2 * (time_end + time_start)
    ) / 4
