import casadi as ca
import numpy as np

import maptor as mtor


# ============================================================================
# Physical Constants and Parameters
# ============================================================================

# Vehicle parameters (DJI Mavic 3 specifications)
m = 0.92
g = 9.81
l_arm = 0.19

# Inertia parameters (kg⋅m²)
Jx = 0.0123
Jy = 0.0123
Jz = 0.0246
Jr = 0.000030

# Aerodynamic coefficients
K_T = 2.5e-6
K_d = 6.3e-8

# Drag coefficients (linear damping)
K_dx = 0.15
K_dy = 0.15
K_dz = 0.20

# Motor constraints
omega_max = 1500.0
omega_min = 0.0

# Danger zone parameters (infinite height cylinder)
DANGER_ZONE_CENTER_X = 3.0
DANGER_ZONE_CENTER_Y = 3.0
DANGER_ZONE_RADIUS = 0.4


# ============================================================================
# Scaling Factors
# ============================================================================

POS_SCALE = 10.0
VEL_SCALE = 10.0
ANG_SCALE = 1.0
OMEGA_B_SCALE = 10.0
OMEGA_M_SCALE = 1000.0
TIME_SCALE = 1.0


# ============================================================================
# Problem Setup
# ============================================================================

problem = mtor.Problem("Quadrotor Minimum Time Trajectory")
phase = problem.set_phase(1)


# ============================================================================
# Variables
# ============================================================================

t = phase.time(initial=0.0)

# Position states (scaled)
X_s = phase.state("X_scaled", initial=1.0 / POS_SCALE, final=5.0 / POS_SCALE)
Y_s = phase.state("Y_scaled", initial=1.0 / POS_SCALE, final=5.0 / POS_SCALE)
Z_s = phase.state("Z_scaled", initial=1.0 / POS_SCALE, final=5.0 / POS_SCALE, boundary=(0.0, None))

# Velocity states (scaled)
X_dot_s = phase.state("X_dot_scaled", initial=0.0, final=0.0)
Y_dot_s = phase.state("Y_dot_scaled", initial=0.0, final=0.0)
Z_dot_s = phase.state("Z_dot_scaled", initial=0.0, final=0.0)

# Attitude states (scaled Euler angles)
phi_s = phase.state(
    "phi_scaled", initial=0.0, final=0.0, boundary=(-np.pi / 3 / ANG_SCALE, np.pi / 3 / ANG_SCALE)
)
theta_s = phase.state(
    "theta_scaled", initial=0.0, final=0.0, boundary=(-np.pi / 3 / ANG_SCALE, np.pi / 3 / ANG_SCALE)
)
psi_s = phase.state("psi_scaled", initial=0.0, boundary=(-np.pi / ANG_SCALE, np.pi / ANG_SCALE))

# Angular rate states (scaled)
p_s = phase.state(
    "p_scaled", initial=0.0, final=0.0, boundary=(-10.0 / OMEGA_B_SCALE, 10.0 / OMEGA_B_SCALE)
)
q_s = phase.state(
    "q_scaled", initial=0.0, final=0.0, boundary=(-10.0 / OMEGA_B_SCALE, 10.0 / OMEGA_B_SCALE)
)
r_s = phase.state(
    "r_scaled", initial=0.0, final=0.0, boundary=(-5.0 / OMEGA_B_SCALE, 5.0 / OMEGA_B_SCALE)
)

# Control inputs (scaled motor speeds)
omega1_s = phase.control(
    "omega1_scaled", boundary=(omega_min / OMEGA_M_SCALE, omega_max / OMEGA_M_SCALE)
)
omega2_s = phase.control(
    "omega2_scaled", boundary=(omega_min / OMEGA_M_SCALE, omega_max / OMEGA_M_SCALE)
)
omega3_s = phase.control(
    "omega3_scaled", boundary=(omega_min / OMEGA_M_SCALE, omega_max / OMEGA_M_SCALE)
)
omega4_s = phase.control(
    "omega4_scaled", boundary=(omega_min / OMEGA_M_SCALE, omega_max / OMEGA_M_SCALE)
)


# ============================================================================
# Dynamics
# ============================================================================

# Convert scaled variables to physical units
X_phys = X_s * POS_SCALE
Y_phys = Y_s * POS_SCALE
Z_phys = Z_s * POS_SCALE
X_dot_phys = X_dot_s * VEL_SCALE
Y_dot_phys = Y_dot_s * VEL_SCALE
Z_dot_phys = Z_dot_s * VEL_SCALE
phi_phys = phi_s * ANG_SCALE
theta_phys = theta_s * ANG_SCALE
psi_phys = psi_s * ANG_SCALE
p_phys = p_s * OMEGA_B_SCALE
q_phys = q_s * OMEGA_B_SCALE
r_phys = r_s * OMEGA_B_SCALE
omega1_phys = omega1_s * OMEGA_M_SCALE
omega2_phys = omega2_s * OMEGA_M_SCALE
omega3_phys = omega3_s * OMEGA_M_SCALE
omega4_phys = omega4_s * OMEGA_M_SCALE

# Total thrust and gyroscopic effect from motor speeds
F_T = K_T * (omega1_phys**2 + omega2_phys**2 + omega3_phys**2 + omega4_phys**2)
omega_prop = omega1_phys - omega2_phys + omega3_phys - omega4_phys

# Trigonometric functions for rotation matrices
cos_phi, sin_phi = ca.cos(phi_phys), ca.sin(phi_phys)
cos_theta, sin_theta = ca.cos(theta_phys), ca.sin(theta_phys)
cos_psi, sin_psi = ca.cos(psi_phys), ca.sin(psi_phys)
tan_theta = ca.tan(theta_phys)

# Position dynamics (simple integration of global velocities)
X_dynamics_phys = X_dot_phys
Y_dynamics_phys = Y_dot_phys
Z_dynamics_phys = Z_dot_phys

# Translational dynamics (global frame accelerations)
X_dot_dynamics_phys = (1 / m) * (
    -(cos_phi * cos_psi * sin_theta + sin_phi * sin_psi) * F_T - K_dx * X_dot_phys
)
Y_dot_dynamics_phys = (1 / m) * (
    -(cos_phi * sin_psi * sin_theta - cos_psi * sin_phi) * F_T - K_dy * Y_dot_phys
)
Z_dot_dynamics_phys = (1 / m) * (-cos_phi * cos_theta * F_T - K_dz * Z_dot_phys) + g

# Attitude dynamics
cos_theta_safe = ca.fmax(ca.fabs(cos_theta), 1e-6)
sec_theta = 1.0 / cos_theta_safe

phi_dynamics_phys = p_phys + sin_phi * tan_theta * q_phys + cos_phi * tan_theta * r_phys
theta_dynamics_phys = cos_phi * q_phys - sin_phi * r_phys
psi_dynamics_phys = (sin_phi * q_phys + cos_phi * r_phys) * sec_theta

# Angular acceleration dynamics
p_dynamics_phys = (1 / Jx) * (
    (Jy - Jz) * q_phys * r_phys
    - Jr * q_phys * omega_prop
    + l_arm * K_T * (omega1_phys**2 - omega3_phys**2)
)
q_dynamics_phys = (1 / Jy) * (
    (Jz - Jx) * p_phys * r_phys
    + Jr * p_phys * omega_prop
    + l_arm * K_T * (omega2_phys**2 - omega4_phys**2)
)
r_dynamics_phys = (1 / Jz) * (
    (Jx - Jy) * p_phys * q_phys
    + K_d * (omega1_phys**2 - omega2_phys**2 + omega3_phys**2 - omega4_phys**2)
)

phase.dynamics(
    {
        X_s: X_dynamics_phys / POS_SCALE,
        Y_s: Y_dynamics_phys / POS_SCALE,
        Z_s: Z_dynamics_phys / POS_SCALE,
        X_dot_s: X_dot_dynamics_phys / VEL_SCALE,
        Y_dot_s: Y_dot_dynamics_phys / VEL_SCALE,
        Z_dot_s: Z_dot_dynamics_phys / VEL_SCALE,
        phi_s: phi_dynamics_phys / ANG_SCALE,
        theta_s: theta_dynamics_phys / ANG_SCALE,
        psi_s: psi_dynamics_phys / ANG_SCALE,
        p_s: p_dynamics_phys / OMEGA_B_SCALE,
        q_s: q_dynamics_phys / OMEGA_B_SCALE,
        r_s: r_dynamics_phys / OMEGA_B_SCALE,
    }
)


# ============================================================================
# Constraints
# ============================================================================

# Cylindrical danger zone constraint (scaled coordinates)
danger_zone_center_x_scaled = DANGER_ZONE_CENTER_X / POS_SCALE
danger_zone_center_y_scaled = DANGER_ZONE_CENTER_Y / POS_SCALE
danger_zone_radius_scaled = DANGER_ZONE_RADIUS / POS_SCALE

safety_distance_scaled = 0.5 / POS_SCALE

distance_squared_to_danger = (X_s - danger_zone_center_x_scaled) ** 2 + (
    Y_s - danger_zone_center_y_scaled
) ** 2

phase.path_constraints(
    distance_squared_to_danger >= danger_zone_radius_scaled**2 + safety_distance_scaled**2
)


# ============================================================================
# Objective
# ============================================================================

omega_reg_scaled = omega1_s**2 + omega2_s**2 + omega3_s**2 + omega4_s**2
omega_integral_scaled = phase.add_integral(omega_reg_scaled)
problem.minimize(t.final + omega_integral_scaled)


# ============================================================================
# Mesh Configuration and Initial Guess
# ============================================================================

phase.mesh([8, 8, 8], [-1.0, -1 / 3, 1 / 3, 1.0])

states_guess = []
controls_guess = []

for N in [8, 8, 8]:
    tau = np.linspace(-1, 1, N + 1)
    t_norm = (tau + 1) / 2

    # Linear trajectory guess (scaled) - avoiding danger zone
    X_vals_scaled = (1.0 + 4.0 * t_norm) / POS_SCALE
    Y_vals_scaled = (1.0 + 4.0 * t_norm) / POS_SCALE
    Z_vals_scaled = (1.0 + 4.0 * t_norm) / POS_SCALE

    # Smooth velocity profile (scaled)
    X_dot_vals_scaled = (5.0 * np.sin(np.pi * t_norm)) / VEL_SCALE
    Y_dot_vals_scaled = (5.0 * np.sin(np.pi * t_norm)) / VEL_SCALE
    Z_dot_vals_scaled = (3.0 * np.sin(np.pi * t_norm)) / VEL_SCALE

    # Small attitude variations (scaled)
    phi_vals_scaled = (0.1 * np.sin(2 * np.pi * t_norm)) / ANG_SCALE
    theta_vals_scaled = (0.1 * np.cos(2 * np.pi * t_norm)) / ANG_SCALE
    psi_vals_scaled = (0.2 * t_norm) / ANG_SCALE

    # Angular rates near zero (scaled)
    p_vals_scaled = np.zeros(N + 1)
    q_vals_scaled = np.zeros(N + 1)
    r_vals_scaled = np.zeros(N + 1)

    states_guess.append(
        np.vstack(
            [
                X_vals_scaled,
                Y_vals_scaled,
                Z_vals_scaled,
                X_dot_vals_scaled,
                Y_dot_vals_scaled,
                Z_dot_vals_scaled,
                phi_vals_scaled,
                theta_vals_scaled,
                psi_vals_scaled,
                p_vals_scaled,
                q_vals_scaled,
                r_vals_scaled,
            ]
        )
    )

    # Hover motor speed baseline (scaled)
    hover_speed = np.sqrt(m * g / (4 * K_T))
    omega_vals_scaled = np.full(N, hover_speed / OMEGA_M_SCALE)

    controls_guess.append(
        np.vstack([omega_vals_scaled, omega_vals_scaled, omega_vals_scaled, omega_vals_scaled])
    )

phase.guess(
    states=states_guess,
    controls=controls_guess,
    terminal_time=8.0,
)


# ============================================================================
# Solve
# ============================================================================

solution = mtor.solve_adaptive(
    problem,
    error_tolerance=1e-2,
    max_iterations=30,
    min_polynomial_degree=3,
    max_polynomial_degree=8,
    ode_solver_tolerance=1e-3,
    nlp_options={
        "ipopt.max_iter": 1000,
        "ipopt.tol": 1e-6,
        "ipopt.constr_viol_tol": 1e-4,
        "ipopt.linear_solver": "mumps",
        "ipopt.print_level": 3,
        "ipopt.mu_strategy": "adaptive",
        "ipopt.acceptable_tol": 1e-4,
        "ipopt.acceptable_iter": 5,
    },
)


# ============================================================================
# Results
# ============================================================================

if solution.status["success"]:
    flight_time = solution.status["objective"]
    print(f"Objective: {flight_time:.3f} seconds")

    # Convert scaled final state to physical for verification
    X_final = solution["X_scaled"][-1] * POS_SCALE
    Y_final = solution["Y_scaled"][-1] * POS_SCALE
    Z_final = solution["Z_scaled"][-1] * POS_SCALE
    position_error = np.sqrt((X_final - 5.0) ** 2 + (Y_final - 5.0) ** 2 + (Z_final - 5.0) ** 2)

    print(f"Final position: ({X_final:.3f}, {Y_final:.3f}, {Z_final:.3f}) m")
    print(f"Position error: {position_error:.3f} m")

    # Flight envelope analysis (convert scaled to physical)
    X_dot_phys = solution["X_dot_scaled"] * VEL_SCALE
    Y_dot_phys = solution["Y_dot_scaled"] * VEL_SCALE
    Z_dot_phys = solution["Z_dot_scaled"] * VEL_SCALE
    max_speed = max(np.sqrt(X_dot_phys**2 + Y_dot_phys**2 + Z_dot_phys**2))

    phi_phys = solution["phi_scaled"] * ANG_SCALE
    theta_phys = solution["theta_scaled"] * ANG_SCALE
    max_phi = max(np.abs(phi_phys)) * 180 / np.pi
    max_theta = max(np.abs(theta_phys)) * 180 / np.pi

    print(f"Maximum speed: {max_speed:.2f} m/s")
    print(f"Maximum roll angle: {max_phi:.1f}°")
    print(f"Maximum pitch angle: {max_theta:.1f}°")

    # Motor usage analysis (convert scaled to physical)
    omega1_phys = solution["omega1_scaled"] * OMEGA_M_SCALE
    omega2_phys = solution["omega2_scaled"] * OMEGA_M_SCALE
    omega3_phys = solution["omega3_scaled"] * OMEGA_M_SCALE
    omega4_phys = solution["omega4_scaled"] * OMEGA_M_SCALE

    avg_motor_speed = (omega1_phys + omega2_phys + omega3_phys + omega4_phys) / 4
    max_motor_speed = max(np.maximum.reduce([omega1_phys, omega2_phys, omega3_phys, omega4_phys]))

    print(f"Average motor speed: {np.mean(avg_motor_speed):.1f} rad/s")
    print(f"Maximum motor speed: {max_motor_speed:.1f} rad/s")
    print(f"Motor utilization: {max_motor_speed / omega_max * 100:.1f}%")

    # Danger zone verification
    X_traj_phys = solution["X_scaled"] * POS_SCALE
    Y_traj_phys = solution["Y_scaled"] * POS_SCALE
    min_distance_to_danger = min(
        np.sqrt(
            (X_traj_phys - DANGER_ZONE_CENTER_X) ** 2 + (Y_traj_phys - DANGER_ZONE_CENTER_Y) ** 2
        )
    )
    print(
        f"Minimum distance to danger zone: {min_distance_to_danger:.3f} m (required: {DANGER_ZONE_RADIUS:.1f} m)"
    )

    solution.plot()

else:
    print(f"Optimization failed: {solution.status['message']}")
