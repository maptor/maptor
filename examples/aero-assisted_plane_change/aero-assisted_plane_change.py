import casadi as ca
import numpy as np

import maptor as mtor


# Constants from Table 10.3
Re = 2.092643e7  # ft
m = 3.315e2  # slugs
rho0 = 3.3195e-5  # slug/ft³
h0 = 1e5  # ft
hr = 2.41388e4  # ft
CD0 = 0.032
k = 1.4
S = 1.2584e2  # ft²
mu = 1.40895e16  # ft³/sec²

DEG2RAD = np.pi / 180.0

# Derived parameters
rhos = rho0 * ca.exp(h0 / hr)
a0 = (S / (2 * m)) * ca.sqrt(CD0 / k)
a1 = (CD0 * S) / (2 * m)
vs = ca.sqrt(mu / Re)

# Problem setup
problem = mtor.Problem("Optimal Aero-Assisted Plane Change")
phase = problem.set_phase(1)

# Variables
t = phase.time(initial=0.0, final=(800, 2000))
phi = phase.state(
    "phi", initial=0.0, final=(-89 * DEG2RAD, 89 * DEG2RAD), boundary=(-89 * DEG2RAD, 89 * DEG2RAD)
)
h = phase.state("altitude", initial=365000.0, final=365000.0, boundary=(0, 400000))
v = phase.state("velocity", initial=25745.704, boundary=(20000, 28000))
gamma = phase.state(
    "gamma",
    initial=-0.55 * DEG2RAD,
    final=(-10 * DEG2RAD, 10 * DEG2RAD),
    boundary=(-10 * DEG2RAD, 10 * DEG2RAD),
)
psi = phase.state(
    "psi", initial=0.0, final=(-89 * DEG2RAD, 89 * DEG2RAD), boundary=(-89 * DEG2RAD, 89 * DEG2RAD)
)

CL = phase.control("lift_coeff", boundary=(0, 2))
beta = phase.control("bank_angle", boundary=(0, 180 * DEG2RAD))

# Additional variables
r = Re + h
rho = rho0 * ca.exp(-(h - h0) / hr)
M = (1 / (a0 * rho * r)) * (1 - mu / (r * v**2))
q_dot = 17600 * ca.sqrt(rho / rhos) * (v / vs) ** 3.15

# Dynamics
phi_dot = (v / r) * ca.cos(gamma) * ca.sin(psi)
h_dot = v * ca.sin(gamma)
v_dot = -a1 * rho * v**2 * (1 + CL**2) - (mu * ca.sin(gamma)) / r**2
gamma_dot = a0 * rho * v * (CL * ca.cos(beta) + M * ca.cos(gamma))
psi_dot = (a0 * rho * v * CL * ca.sin(beta)) / ca.cos(gamma) - (
    v * ca.cos(gamma) * ca.cos(psi) * ca.tan(phi)
) / r

phase.dynamics(
    {
        phi: phi_dot,
        h: h_dot,
        v: v_dot,
        gamma: gamma_dot,
        psi: psi_dot,
    }
)

# Path constraints: heat rate limits (applied at every collocation point)
phase.path_constraints(
    q_dot <= 800,  # Heat rate upper limit
    q_dot >= 0,  # Heat rate lower limit (physical constraint)
)

# Event constraint: final boundary condition (applied at final boundary only)
target_angle = ca.cos(18 * DEG2RAD)
phase.event_constraints(ca.cos(phi.final) * ca.cos(psi.final) == target_angle)

# Objective: Maximize final velocity
problem.minimize(-v.final)

# Mesh and guess
phase.mesh([12, 12, 12], [-1.0, -1 / 3, 1 / 3, 1.0])

states_guess = []
controls_guess = []
for N in [12, 12, 12]:
    tau = np.linspace(-1, 1, N + 1)
    t_norm = (tau + 1) / 2

    # Linear interpolation for states
    phi_vals = 0.0 + (0.0 - 0.0) * t_norm  # phi stays near 0
    h_vals = 365000.0 + (365000.0 - 365000.0) * t_norm  # altitude constant
    v_vals = 25715.704 + (25715.704 - 25715.704) * t_norm  # velocity constant
    gamma_vals = -0.55 * DEG2RAD + (0.0 - (-0.55 * DEG2RAD)) * t_norm
    psi_vals = 0.0 + (18 * DEG2RAD - 0.0) * t_norm  # approach target angle

    states_guess.append(np.vstack([phi_vals, h_vals, v_vals, gamma_vals, psi_vals]))

    # Control guess
    CL_vals = np.full(N, 1.0)  # Mid-range lift coefficient
    beta_vals = np.full(N, 45 * DEG2RAD)  # Mid-range bank angle
    controls_guess.append(np.vstack([CL_vals, beta_vals]))

phase.guess(
    states=states_guess,
    controls=controls_guess,
    terminal_time=800,
)

# Solve
solution = mtor.solve_adaptive(
    problem,
    error_tolerance=1e-6,
    max_iterations=50,
    min_polynomial_degree=4,
    max_polynomial_degree=15,
    nlp_options={
        "ipopt.print_level": 5,
        "ipopt.max_iter": 3000,
        "ipopt.tol": 1e-8,
        "ipopt.constr_viol_tol": 1e-7,
    },
)

# Results
if solution.status["success"]:
    final_velocity = -solution.status["objective"]  # Convert back from minimization
    print(f"Final velocity: {final_velocity:.4f} ft/sec")
    print(f"Final time: {solution.phases[1]['times']['final']:.4f} sec")
    print("Reference: v_f = 22043.5079 ft/sec, t_f = 1005.8778 sec")

    error_v = abs(final_velocity - 22043.5079) / 22043.5079 * 100
    print(f"Velocity error: {error_v:.2f}%")

    solution.plot()
else:
    print(f"Failed: {solution.status['message']}")
