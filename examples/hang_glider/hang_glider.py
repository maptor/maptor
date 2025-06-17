import casadi as ca
import numpy as np

import maptor as mtor


# ============================================================================
# Physical Constants and Parameters
# ============================================================================

# Physical constants
M = 100.0
G = 9.80665
U_M = 2.5
R = 100.0
C0 = 0.034
K = 0.069662
S = 14.0
RHO = 1.13


# ============================================================================
# Scaling Factors
# ============================================================================


X_SCALE = 1000.0  # position scaling (km)
V_SCALE = 10.0  # velocity scaling (10 m/s)


# ============================================================================
# Initial and Target Conditions
# ============================================================================

# Scaled values
X0_SCALED = 0.0 / X_SCALE
Y0_SCALED = 1000.0 / X_SCALE
YF_SCALED = 900.0 / X_SCALE
VX0_SCALED = 13.2275675 / V_SCALE
VY0_SCALED = -1.28750052 / V_SCALE
VXF_SCALED = 13.2275675 / V_SCALE
VYF_SCALED = -1.28750052 / V_SCALE

# Scaled bounds
X_MAX_SCALED = 1500.0 / X_SCALE
Y_MAX_SCALED = 1100.0 / X_SCALE
VX_MAX_SCALED = 15.0 / V_SCALE
VY_MIN_SCALED = -4.0 / V_SCALE
VY_MAX_SCALED = 4.0 / V_SCALE


# ============================================================================
# Problem Setup
# ============================================================================

problem = mtor.Problem("Hang Glider")
phase = problem.set_phase(1)


# ============================================================================
# Variables
# ============================================================================

t = phase.time(initial=0.0, final=(0.1, 200.0))
x_s = phase.state("x_scaled", initial=X0_SCALED, boundary=(0.0, X_MAX_SCALED))
y_s = phase.state("y_scaled", initial=Y0_SCALED, final=YF_SCALED, boundary=(0.0, Y_MAX_SCALED))
vx_s = phase.state("vx_scaled", initial=VX0_SCALED, final=VXF_SCALED, boundary=(0.0, VX_MAX_SCALED))
vy_s = phase.state(
    "vy_scaled", initial=VY0_SCALED, final=VYF_SCALED, boundary=(VY_MIN_SCALED, VY_MAX_SCALED)
)
CL = phase.control("CL", boundary=(0.0, 1.4))


# ============================================================================
# Dynamics
# ============================================================================


def _define_scaled_dynamics():
    # Convert scaled variables to physical units
    x_phys = x_s * X_SCALE
    y_s * X_SCALE
    vx_phys = vx_s * V_SCALE
    vy_phys = vy_s * V_SCALE

    # Numerical safeguards
    vr_squared = vx_phys * vx_phys + vy_phys * vy_phys
    vr = ca.sqrt(ca.fmax(vr_squared, 1e-6))

    # Aerodynamic model
    CD = C0 + K * CL * CL
    D = 0.5 * CD * RHO * S * vr * vr
    L = 0.5 * CL * RHO * S * vr * vr

    # Updraft model with numerical bounds
    X_arg = x_phys / R - 2.5
    X = X_arg * X_arg
    X_safe = ca.fmin(X, 50.0)
    ua = U_M * (1.0 - X_safe) * ca.exp(-X_safe)

    Vy = vy_phys - ua
    sin_eta = Vy / vr
    cos_eta = vx_phys / vr
    W = M * G

    # Dynamics in physical units
    x_dot_phys = vx_phys
    y_dot_phys = vy_phys
    vx_dot_phys = (1.0 / M) * (-L * sin_eta - D * cos_eta)
    vy_dot_phys = (1.0 / M) * (L * cos_eta - D * sin_eta - W)

    # Manual scaling of derivatives (NO TIME SCALING)
    x_dot_scaled = x_dot_phys / X_SCALE
    y_dot_scaled = y_dot_phys / X_SCALE
    vx_dot_scaled = vx_dot_phys / V_SCALE
    vy_dot_scaled = vy_dot_phys / V_SCALE

    return {
        x_s: x_dot_scaled,
        y_s: y_dot_scaled,
        vx_s: vx_dot_scaled,
        vy_s: vy_dot_scaled,
    }


phase.dynamics(_define_scaled_dynamics())


# ============================================================================
# Objective
# ============================================================================

# Objective: maximize x(tf)
problem.minimize(-x_s.final)


# ============================================================================
# Mesh Configuration and Initial Guess
# ============================================================================

phase.mesh(
    [3, 3, 3, 3, 3, 3, 3, 3, 3, 3],  # Low degree polynomials
    np.linspace(-1.0, 1.0, 11),  # Fine, uniform spacing
)

# Initial guess
states_guess = []
controls_guess = []
for N in [3, 3, 3, 3, 3, 3, 3, 3, 3, 3]:
    tau = np.linspace(-1, 1, N + 1)
    t_norm = (tau + 1) / 2

    x_vals = (0.0 + 1500 * t_norm) / X_SCALE
    y_vals = (1000.0 + (900.0 - 1000.0) * t_norm) / X_SCALE
    vx_vals = (13.23 * np.ones(N + 1)) / V_SCALE
    vy_vals = (-1.288 * np.ones(N + 1)) / V_SCALE

    states_guess.append(np.vstack([x_vals, y_vals, vx_vals, vy_vals]))
    controls_guess.append(np.ones((1, N)) * 0.7)

phase.guess(
    states=states_guess,
    controls=controls_guess,
    terminal_time=105,
)


# ============================================================================
# Solve
# ============================================================================

solution = mtor.solve_adaptive(
    problem,
    error_tolerance=1e-6,
    max_iterations=30,
    min_polynomial_degree=3,
    max_polynomial_degree=8,
    ode_solver_tolerance=1e-5,
    ode_method="DOP853",
    nlp_options={
        "ipopt.max_iter": 2000,
        "ipopt.tol": 1e-6,
        "ipopt.constr_viol_tol": 1e-6,
        "ipopt.acceptable_tol": 1e-3,
        "ipopt.linear_solver": "mumps",
        "ipopt.print_level": 0,
    },
)


# ============================================================================
# Results Analysis
# ============================================================================

if solution.status["success"]:
    final_x_scaled = solution[(1, "x_scaled")][-1]
    final_x_physical = final_x_scaled * X_SCALE  #
    flight_time = solution.phases[1]["times"]["final"]

    print(f"MAPTOR result: {final_x_physical:.2f} m in {flight_time:.2f} s")
    print("Literature ref: 1248.26 m in 98.47 s")
    print(f"Range error: {abs(final_x_physical - 1248.26):.2f} m")
    print(f"Time error: {abs(flight_time - 98.47):.2f} s")

    # Final state verification in physical units
    x_final = solution[(1, "x_scaled")][-1] * X_SCALE
    y_final = solution[(1, "y_scaled")][-1] * X_SCALE
    vx_final = solution[(1, "vx_scaled")][-1] * V_SCALE
    vy_final = solution[(1, "vy_scaled")][-1] * V_SCALE

    print("Final states (physical units):")
    print(f"  x: {x_final:.2f} m")
    print(f"  y: {y_final:.2f} m (target: 900.0)")
    print(f"  vx: {vx_final:.6f} m/s (target: 13.2275675)")
    print(f"  vy: {vy_final:.6f} m/s (target: -1.28750052)")

    solution.plot()
else:
    print(f"Failed: {solution.status['message']}")
