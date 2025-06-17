import casadi as ca
import numpy as np

import maptor as mtor


# ============================================================================
# Constants and Scaling
# ============================================================================

# Physical constants
DEG2RAD = np.pi / 180.0
MU_EARTH = 0.14076539e17
R_EARTH = 20902900.0
S_REF = 2690.0
RHO0 = 0.002378
H_R = 23800.0
MASS = 203000.0 / 32.174
A0, A1 = -0.20704, 0.029244
B0, B1, B2 = 0.07854, -0.61592e-2, 0.621408e-3

# Scaling factors for numerical conditioning
H_SCALE = 1e5  # Altitude scaling
V_SCALE = 1e4  # Velocity scaling

# ============================================================================
# Problem Setup
# ============================================================================

problem = mtor.Problem("Shuttle Reentry")
phase = problem.set_phase(1)

# ============================================================================
# Variables
# ============================================================================

t = phase.time(initial=0.0)
h_s = phase.state("altitude_scaled", initial=2.6, final=0.8, boundary=(0, None))
phi = phase.state("longitude", initial=0.0)
theta = phase.state("latitude", initial=0.0, boundary=(-89 * DEG2RAD, 89 * DEG2RAD))
v_s = phase.state("velocity_scaled", initial=2.56, final=0.25, boundary=(1e-4, None))
gamma = phase.state(
    "flight_path_angle",
    initial=-1 * DEG2RAD,
    final=-5 * DEG2RAD,
    boundary=(-89 * DEG2RAD, 89 * DEG2RAD),
)
psi = phase.state("heading_angle", initial=90 * DEG2RAD)
alpha = phase.control("angle_of_attack", boundary=(-90 * DEG2RAD, 90 * DEG2RAD))
beta = phase.control("bank_angle", boundary=(-90 * DEG2RAD, 1 * DEG2RAD))

# ============================================================================
# Dynamics
# ============================================================================

# Physics calculations inline for exact numerical behavior
h = h_s * H_SCALE
v = v_s * V_SCALE
r = R_EARTH + h
rho = RHO0 * ca.exp(-h / H_R)
g = MU_EARTH / r**2
alpha_deg = alpha * 180.0 / np.pi
CL = A0 + A1 * alpha_deg
CD = B0 + B1 * alpha_deg + B2 * alpha_deg**2
q = 0.5 * rho * v**2
L = q * CL * S_REF
D = q * CD * S_REF
eps = 1e-10

# Dynamics with manual scaling
phase.dynamics(
    {
        h_s: (v * ca.sin(gamma)) / H_SCALE,
        phi: (v / r) * ca.cos(gamma) * ca.sin(psi) / (ca.cos(theta) + eps),
        theta: (v / r) * ca.cos(gamma) * ca.cos(psi),
        v_s: (-(D / MASS) - g * ca.sin(gamma)) / V_SCALE,
        gamma: (L / (MASS * v + eps)) * ca.cos(beta) + ca.cos(gamma) * (v / r - g / (v + eps)),
        psi: (L * ca.sin(beta) / (MASS * v * ca.cos(gamma) + eps))
        + (v / (r * (ca.cos(theta) + eps))) * ca.cos(gamma) * ca.sin(psi) * ca.sin(theta),
    }
)

# ============================================================================
# Objective
# ============================================================================

# Maximize crossrange (final latitude)
problem.minimize(-theta.final)

# ============================================================================
# Mesh and Guess
# ============================================================================

phase.mesh([8] * 3, np.linspace(-1.0, 1.0, 4))

# Initial guess
states_guess = []
controls_guess = []
for N in [8] * 3:
    t_norm = np.linspace(0, 1, N + 1)
    h_traj = 2.6 + (0.8 - 2.6) * t_norm
    phi_traj = np.zeros(N + 1)
    theta_traj = np.zeros(N + 1)
    v_traj = 2.56 + (0.25 - 2.56) * t_norm
    gamma_traj = -1 * DEG2RAD + (-5 * DEG2RAD - (-1 * DEG2RAD)) * t_norm
    psi_traj = 90 * DEG2RAD * np.ones(N + 1)

    # State order MUST match variable declaration: h_s, phi, theta, v_s, gamma, psi
    states_guess.append(np.vstack([h_traj, phi_traj, theta_traj, v_traj, gamma_traj, psi_traj]))
    controls_guess.append(np.vstack([np.zeros(N), -45 * DEG2RAD * np.ones(N)]))

phase.guess(
    states=states_guess,
    controls=controls_guess,
    terminal_time=2000.0,
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
# Results
# ============================================================================

if solution.status["success"]:
    crossrange_deg = -solution.status["objective"] * 180.0 / np.pi
    print(f"Final time: {solution.phases[1]['times']['final']:.1f} seconds")
    print(f"Crossrange: {crossrange_deg:.2f} degrees")
    solution.plot()
else:
    print(f"Failed: {solution.status['message']}")
