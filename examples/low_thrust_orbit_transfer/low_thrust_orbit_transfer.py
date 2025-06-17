import casadi as ca
import numpy as np
from initial_guess_generator import generate_initial_guess

import maptor as mtor


# ============================================================================
# Physical Constants and Parameters
# ============================================================================

# Propulsion parameters
ISP = 450.0
G0 = 32.174
T_THRUST = 4.446618e-3

# Gravitational parameters
MU = 1.407645794e16
RE = 20925662.73
J2 = 1082.639e-6
J3 = -2.565e-6
J4 = -1.608e-6


# ============================================================================
# Scaling Factors
# ============================================================================

P_SCALE = 1e7
L_SCALE = np.pi
T_SCALE = 1e4


# ============================================================================
# Initial and Target Conditions
# ============================================================================

# Initial modified equinoctial elements
PTI = 21837080.052835
FTI = 0.0
GTI = 0.0
HTI = -0.25396764647494
KTI = 0.0
LTI = np.pi
WTI = 1.0

# Target conditions
PTF = 40007346.015232
EVENT_FINAL_9 = 0.73550320568829
EVENT_FINAL_10 = 0.61761258786099
EVENT_FINAL_11 = 0.0
EVENT_FINAL_12_LOWER = -10.0
EVENT_FINAL_12_UPPER = 0.0

# Numerical tolerance
EQ_TOL = 0.001


# ============================================================================
# Helper Functions
# ============================================================================


def cross_product(a, b):
    return ca.vertcat(
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def dot_product(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


# ============================================================================
# Problem Setup
# ============================================================================

problem = mtor.Problem("Low-Thrust Orbit Transfer")
phase = problem.set_phase(1)


# ============================================================================
# Variables
# ============================================================================

# Time variable (scaled)
t_s = phase.time(initial=0.0, final=(50000.0 / T_SCALE, 100000.0 / T_SCALE))

# State variables (modified equinoctial elements)
p_s = phase.state(
    "p_scaled",
    initial=PTI / P_SCALE,
    boundary=(10.0e6 / P_SCALE, 60.0e6 / P_SCALE),
)
f = phase.state("f", initial=FTI, boundary=(-0.20, 0.20))
g = phase.state("g", initial=GTI, boundary=(-0.10, 1.0))
h = phase.state("h", initial=HTI, boundary=(-1.0, 1.0))
k = phase.state("k", initial=KTI, boundary=(-0.20, 0.20))
L_s = phase.state(
    "L_scaled",
    initial=LTI / L_SCALE,
    boundary=(np.pi / L_SCALE, 20 * np.pi / L_SCALE),
)
w = phase.state("w", initial=WTI, boundary=(0.0, 2.0))

# Control variables (thrust direction components)
u1 = phase.control("u1", boundary=(-1.0, 1.0))
u2 = phase.control("u2", boundary=(-1.0, 1.0))
u3 = phase.control("u3", boundary=(-1.0, 1.0))

# Parameter (thrust efficiency factor)
tau = problem.parameter("tau", boundary=(-50.0, 0.0))


# ============================================================================
# Mathematical Formulation
# ============================================================================

# Convert scaled variables to physical units
t = t_s * T_SCALE
p = p_s * P_SCALE
L = L_s * L_SCALE

# Numerical safeguard
eps = 1e-12

# Modified equinoctial element calculations
q = 1.0 + f * ca.cos(L) + g * ca.sin(L)
r = p / ca.fmax(q, eps)
alpha2 = h * h - k * k
X = ca.sqrt(h * h + k * k)
s2 = 1 + X * X

# Position vector components
r1 = r / s2 * (ca.cos(L) + alpha2 * ca.cos(L) + 2 * h * k * ca.sin(L))
r2 = r / s2 * (ca.sin(L) - alpha2 * ca.sin(L) + 2 * h * k * ca.cos(L))
r3 = 2 * r / s2 * (h * ca.sin(L) - k * ca.cos(L))
rvec = ca.vertcat(r1, r2, r3)

# Velocity vector components
sqrt_mu_p = ca.sqrt(MU / ca.fmax(p, eps))
v1 = (
    -(1.0 / s2)
    * sqrt_mu_p
    * (ca.sin(L) + alpha2 * ca.sin(L) - 2 * h * k * ca.cos(L) + g - 2 * f * h * k + alpha2 * g)
)
v2 = (
    -(1.0 / s2)
    * sqrt_mu_p
    * (-ca.cos(L) + alpha2 * ca.cos(L) + 2 * h * k * ca.sin(L) - f + 2 * g * h * k + alpha2 * f)
)
v3 = (2.0 / s2) * sqrt_mu_p * (h * ca.cos(L) + k * ca.sin(L) + f * h + g * k)
vvec = ca.vertcat(v1, v2, v3)

# Reference frame vectors
rv = cross_product(rvec, vvec)
rvr = cross_product(rv, rvec)
norm_r = ca.sqrt(ca.fmax(dot_product(rvec, rvec), eps))
norm_rv = ca.sqrt(ca.fmax(dot_product(rv, rv), eps))

ir = rvec / norm_r
ith = rvr / (norm_rv * norm_r)
ih = rv / norm_rv

# Earth-fixed reference calculations
en = ca.vertcat(0.0, 0.0, 1.0)
enir = dot_product(en, ir)
in_vec = en - enir * ir
norm_in = ca.sqrt(ca.fmax(dot_product(in_vec, in_vec), eps))
in_normalized = in_vec / norm_in

# Gravitational perturbation calculations
sin_phi = rvec[2] / norm_r
sin_phi_clamped = ca.fmax(ca.fmin(sin_phi, 1.0 - eps), -1.0 + eps)
cos_phi = ca.sqrt(1.0 - sin_phi_clamped**2)

# J2 perturbation
P2 = 0.5 * (3.0 * sin_phi_clamped**2 - 1.0)
Pdash2 = 3.0 * sin_phi_clamped
r_safe = ca.fmax(r, RE / 100.0)
deltagn_j2 = -MU * cos_phi / (r_safe * r_safe) * (RE / r_safe) ** 2 * Pdash2 * J2
deltagr_j2 = -MU / (r_safe * r_safe) * 3.0 * (RE / r_safe) ** 2 * P2 * J2

# J3 perturbation
P3 = 0.5 * (5.0 * sin_phi_clamped**3 - 3.0 * sin_phi_clamped)
Pdash3 = 0.5 * (15.0 * sin_phi_clamped**2 - 3.0)
deltagn_j3 = -MU * cos_phi / (r_safe * r_safe) * (RE / r_safe) ** 3 * Pdash3 * J3
deltagr_j3 = -MU / (r_safe * r_safe) * 4.0 * (RE / r_safe) ** 3 * P3 * J3

# J4 perturbation
P4 = (1.0 / 8.0) * (35.0 * sin_phi_clamped**4 - 30.0 * sin_phi_clamped**2 + 3.0)
Pdash4 = (1.0 / 8.0) * (140.0 * sin_phi_clamped**3 - 60.0 * sin_phi_clamped)
deltagn_j4 = -MU * cos_phi / (r_safe * r_safe) * (RE / r_safe) ** 4 * Pdash4 * J4
deltagr_j4 = -MU / (r_safe * r_safe) * 5.0 * (RE / r_safe) ** 4 * P4 * J4

# Total gravitational perturbations
deltagn = deltagn_j2 + deltagn_j3 + deltagn_j4
deltagr = deltagr_j2 + deltagr_j3 + deltagr_j4
delta_g = deltagn * in_normalized - deltagr * ir

# Gravitational acceleration components
DELTA_g1 = dot_product(ir, delta_g)
DELTA_g2 = dot_product(ith, delta_g)
DELTA_g3 = dot_product(ih, delta_g)

# Thrust acceleration components
w_safe = ca.fmax(w, eps)
DELTA_T1 = G0 * T_THRUST * (1.0 + 0.01 * tau) * u1 / w_safe
DELTA_T2 = G0 * T_THRUST * (1.0 + 0.01 * tau) * u2 / w_safe
DELTA_T3 = G0 * T_THRUST * (1.0 + 0.01 * tau) * u3 / w_safe

# Total acceleration components
delta1 = DELTA_g1 + DELTA_T1
delta2 = DELTA_g2 + DELTA_T2
delta3 = DELTA_g3 + DELTA_T3


# ============================================================================
# Dynamics
# ============================================================================

# Gauss variational equations for modified equinoctial elements
sqrt_p_mu = ca.sqrt(ca.fmax(p, eps) / MU)
q_safe = ca.fmax(ca.fabs(q), eps)

pdot = 2 * p / q_safe * sqrt_p_mu * delta2
fdot = (
    sqrt_p_mu * ca.sin(L) * delta1
    + sqrt_p_mu * (1.0 / q_safe) * ((q + 1.0) * ca.cos(L) + f) * delta2
    - sqrt_p_mu * (g / q_safe) * (h * ca.sin(L) - k * ca.cos(L)) * delta3
)
gdot = (
    -sqrt_p_mu * ca.cos(L) * delta1
    + sqrt_p_mu * (1.0 / q_safe) * ((q + 1.0) * ca.sin(L) + g) * delta2
    + sqrt_p_mu * (f / q_safe) * (h * ca.sin(L) - k * ca.cos(L)) * delta3
)
hdot = sqrt_p_mu * s2 * ca.cos(L) / (2.0 * q_safe) * delta3
kdot = sqrt_p_mu * s2 * ca.sin(L) / (2.0 * q_safe) * delta3
Ldot = (
    sqrt_p_mu * (1.0 / q_safe) * (h * ca.sin(L) - k * ca.cos(L)) * delta3
    + ca.sqrt(MU * ca.fmax(p, eps)) * (q / ca.fmax(p, eps)) ** 2
)
wdot = -T_THRUST * (1.0 + 0.01 * tau) / ISP

# Scaled dynamics
phase.dynamics(
    {
        p_s: (pdot / P_SCALE) * T_SCALE,
        f: fdot * T_SCALE,
        g: gdot * T_SCALE,
        h: hdot * T_SCALE,
        k: kdot * T_SCALE,
        L_s: (Ldot / L_SCALE) * T_SCALE,
        w: wdot * T_SCALE,
    }
)


# ============================================================================
# Constraints
# ============================================================================

# Path constraint: Unit thrust vector magnitude
thrust_magnitude_squared = u1**2 + u2**2 + u3**2
phase.path_constraints(
    thrust_magnitude_squared >= 1.0 - EQ_TOL,
    thrust_magnitude_squared <= 1.0 + EQ_TOL,
)

# Event constraints: Final orbital element targets
phase.event_constraints(
    p_s.final == PTF / P_SCALE,
    ca.sqrt(f.final**2 + g.final**2) == EVENT_FINAL_9,
    ca.sqrt(h.final**2 + k.final**2) == EVENT_FINAL_10,
    f.final * h.final + g.final * k.final == EVENT_FINAL_11,
)

# Additional final constraint
gtf_htf_minus_ktf_ftf = g.final * h.final - k.final * f.final
phase.event_constraints(
    gtf_htf_minus_ktf_ftf >= EVENT_FINAL_12_LOWER,
    gtf_htf_minus_ktf_ftf <= EVENT_FINAL_12_UPPER,
)


# ============================================================================
# Objective
# ============================================================================

# Maximize final mass (minimize negative mass)
problem.minimize(-w.final)


# ============================================================================
# Mesh Configuration and Initial Guess
# ============================================================================

# Mesh configuration
phase.mesh(
    [8, 8, 8, 8, 8, 8, 8, 8],
    [-1.0, -6 / 7, -4 / 7, -2 / 7, 0, 2 / 7, 4 / 7, 6 / 7, 1.0],
)

# Generate initial guess from external module
states_guess, controls_guess, final_time_guess = generate_initial_guess()

phase.guess(
    states=states_guess,
    controls=controls_guess,
    terminal_time=final_time_guess,
)

problem.parameter_guess(tau=-25.0)

# ============================================================================
# Solve
# ============================================================================

solution = mtor.solve_adaptive(
    problem,
    error_tolerance=1e-4,
    max_iterations=30,
    min_polynomial_degree=3,
    max_polynomial_degree=8,
    ode_solver_tolerance=1e-4,
    nlp_options={
        "ipopt.print_level": 0,
        "ipopt.max_iter": 500,
        "ipopt.tol": 1e-4,
        "ipopt.constr_viol_tol": 1e-4,
        "ipopt.acceptable_tol": 1e-3,
        "ipopt.mu_strategy": "adaptive",
        "ipopt.linear_solver": "mumps",
    },
)


# ============================================================================
# Results
# ============================================================================

if solution.status["success"]:
    final_mass = solution[(1, "w")][-1]
    final_time_scaled = solution.phases[1]["times"]["final"]
    final_time = final_time_scaled * T_SCALE

    print(f"Final mass: {final_mass:.6f}")
    print(f"Final time: {final_time:.1f} seconds ({final_time / 3600:.2f} hours)")

    p_final_scaled = solution[(1, "p_scaled")][-1]
    p_final_physical = p_final_scaled * P_SCALE
    print(f"Final p (scaled): {p_final_scaled:.6f}")
    print(f"Final p (physical): {p_final_physical:.1f} ft")

    solution.plot()
else:
    print(f"Failed: {solution.status['message']}")
