import casadi as ca
import numpy as np

import maptor as mtor


# ============================================================================
# Physical Constants and Parameters
# ============================================================================

# Earth and gravitational parameters
OMEGA_EARTH = 7.29211585e-5  # Earth rotation rate (rad/s)
MU_EARTH = 3.986012e14  # Gravitational parameter (m^3/s^2)
RE_EARTH = 6378145.0  # Earth radius (m)
G0 = 9.80665  # Sea level gravity (m/s^2)

# Atmospheric parameters
RHO0 = 1.225  # Sea level density (kg/m^3)
H_SCALE_HEIGHT = 7200.0  # Density scale height (m)
CD = 0.5  # Drag coefficient
SA = 4 * np.pi  # Surface area (m^2)

# Vehicle mass parameters (kg)
M_TOT_SRB = 19290.0
M_PROP_SRB = 17010.0
M_DRY_SRB = M_TOT_SRB - M_PROP_SRB
M_TOT_FIRST = 104380.0
M_PROP_FIRST = 95550.0
M_DRY_FIRST = M_TOT_FIRST - M_PROP_FIRST
M_TOT_SECOND = 19300.0
M_PROP_SECOND = 16820.0
M_DRY_SECOND = M_TOT_SECOND - M_PROP_SECOND
M_PAYLOAD = 4164.0

# Thrust parameters (N)
THRUST_SRB = 628500.0
THRUST_FIRST = 1083100.0
THRUST_SECOND = 110094.0

# Mission timeline (s)
T_PHASE_1_END = 75.2
T_PHASE_2_END = 150.4
T_PHASE_3_END = 261.0
T_PHASE_4_END = 961.0

# Derived propulsion parameters
MDOT_SRB = M_PROP_SRB / T_PHASE_1_END
MDOT_FIRST = M_PROP_FIRST / T_PHASE_3_END
MDOT_SECOND = M_PROP_SECOND / 700.0
ISP_SRB = THRUST_SRB / (G0 * MDOT_SRB)
ISP_FIRST = THRUST_FIRST / (G0 * MDOT_FIRST)
ISP_SECOND = THRUST_SECOND / (G0 * MDOT_SECOND)

# Numerical scaling factors
R_SCALE = 1e6  # Position scaling (m)
V_SCALE = 1e3  # Velocity scaling (m/s)
M_SCALE = 1e4  # Mass scaling (kg)

# Earth rotation matrix
OMEGA_MATRIX = ca.MX(3, 3)
OMEGA_MATRIX[0, 0] = 0.0
OMEGA_MATRIX[0, 1] = -OMEGA_EARTH
OMEGA_MATRIX[0, 2] = 0.0
OMEGA_MATRIX[1, 0] = OMEGA_EARTH
OMEGA_MATRIX[1, 1] = 0.0
OMEGA_MATRIX[1, 2] = 0.0
OMEGA_MATRIX[2, 0] = 0.0
OMEGA_MATRIX[2, 1] = 0.0
OMEGA_MATRIX[2, 2] = 0.0


# ============================================================================
# Initial and Target Conditions
# ============================================================================

LAT_LAUNCH = 28.5 * np.pi / 180.0
X_LAUNCH = RE_EARTH * np.cos(LAT_LAUNCH)
Y_LAUNCH = 0.0
Z_LAUNCH = RE_EARTH * np.sin(LAT_LAUNCH)
R_INITIAL_SCALED = [X_LAUNCH / R_SCALE, Y_LAUNCH / R_SCALE, Z_LAUNCH / R_SCALE]

# Initial velocity due to Earth rotation
omega_matrix_np = np.array([[0.0, -OMEGA_EARTH, 0.0], [OMEGA_EARTH, 0.0, 0.0], [0.0, 0.0, 0.0]])
v_initial_phys = omega_matrix_np @ np.array([X_LAUNCH, Y_LAUNCH, Z_LAUNCH])
V_INITIAL_SCALED = [
    v_initial_phys[0] / V_SCALE,
    v_initial_phys[1] / V_SCALE,
    v_initial_phys[2] / V_SCALE,
]

# Target orbital elements
A_TARGET = 24361140.0  # Semi-major axis (m)
E_TARGET = 0.7308  # Eccentricity
INC_TARGET = 28.5 * np.pi / 180.0  # Inclination (rad)
RAAN_TARGET = 269.8 * np.pi / 180.0  # Right ascension of ascending node (rad)
AOP_TARGET = 130.5 * np.pi / 180.0  # Argument of periapsis (rad)

# Initial masses for each phase (scaled)
M_INITIAL_P1 = (M_PAYLOAD + M_TOT_SECOND + M_TOT_FIRST + 9 * M_TOT_SRB) / M_SCALE
M_FINAL_P1 = (M_INITIAL_P1 * M_SCALE - (6 * MDOT_SRB + MDOT_FIRST) * T_PHASE_1_END) / M_SCALE
M_INITIAL_P2 = (M_FINAL_P1 * M_SCALE - 6 * M_DRY_SRB) / M_SCALE
M_FINAL_P2 = (
    M_INITIAL_P2 * M_SCALE - (3 * MDOT_SRB + MDOT_FIRST) * (T_PHASE_2_END - T_PHASE_1_END)
) / M_SCALE
M_INITIAL_P3 = (M_FINAL_P2 * M_SCALE - 3 * M_DRY_SRB) / M_SCALE
M_FINAL_P3 = (M_INITIAL_P3 * M_SCALE - MDOT_FIRST * (T_PHASE_3_END - T_PHASE_2_END)) / M_SCALE
M_INITIAL_P4 = (M_FINAL_P3 * M_SCALE - M_DRY_FIRST) / M_SCALE
M_FINAL_P4 = M_PAYLOAD / M_SCALE

# State bounds (scaled)
R_MIN = -2 * RE_EARTH / R_SCALE
R_MAX = 2 * RE_EARTH / R_SCALE
V_MIN = -10000.0 / V_SCALE
V_MAX = 10000.0 / V_SCALE


# ============================================================================
# Helper Functions
# ============================================================================


def _oe2rv(oe, mu):
    # Convert orbital elements to position and velocity vectors.
    a, e, i, Om, om, nu = oe
    p = a * (1 - e * e)
    r = p / (1 + e * np.cos(nu))

    # Position in perifocal frame
    rv_pf = np.array([r * np.cos(nu), r * np.sin(nu), 0.0])

    # Velocity in perifocal frame
    vv_pf = np.array([-np.sin(nu), e + np.cos(nu), 0.0]) * np.sqrt(mu / p)

    # Rotation matrix from perifocal to inertial frame
    cO, sO = np.cos(Om), np.sin(Om)
    co, so = np.cos(om), np.sin(om)
    ci, si = np.cos(i), np.sin(i)

    R = np.array(
        [
            [cO * co - sO * so * ci, -cO * so - sO * co * ci, sO * si],
            [sO * co + cO * so * ci, -sO * so + cO * co * ci, -cO * si],
            [so * si, co * si, ci],
        ]
    )

    ri = R @ rv_pf
    vi = R @ vv_pf

    return ri, vi


def _cross_product(a, b):
    # Cross product of two 3D vectors
    return ca.vertcat(
        a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]
    )


def _dot_product(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _smooth_heaviside(x, a_eps=0.1):
    return 0.5 * (1 + ca.tanh(x / a_eps))


def _rv2oe(rv, vv, mu):
    # Convert position and velocity to orbital elements with smooth transitions
    eps = 1e-12

    K = ca.vertcat(0.0, 0.0, 1.0)
    hv = _cross_product(rv, vv)
    nv = _cross_product(K, hv)

    n = ca.sqrt(ca.fmax(_dot_product(nv, nv), eps))
    h2 = ca.fmax(_dot_product(hv, hv), eps)
    v2 = ca.fmax(_dot_product(vv, vv), eps)
    r = ca.sqrt(ca.fmax(_dot_product(rv, rv), eps))

    rv_dot_vv = _dot_product(rv, vv)

    ev = ca.vertcat(
        (1 / mu) * ((v2 - mu / r) * rv[0] - rv_dot_vv * vv[0]),
        (1 / mu) * ((v2 - mu / r) * rv[1] - rv_dot_vv * vv[1]),
        (1 / mu) * ((v2 - mu / r) * rv[2] - rv_dot_vv * vv[2]),
    )

    p = h2 / mu
    e = ca.sqrt(ca.fmax(_dot_product(ev, ev), eps))
    a = p / (1 - e * e)
    i = ca.acos(ca.fmax(ca.fmin(hv[2] / ca.sqrt(h2), 1.0 - eps), -1.0 + eps))

    # Smooth transitions for angular elements
    a_eps = 0.1
    nv_dot_ev = _dot_product(nv, ev)

    Om = _smooth_heaviside(nv[1] + eps, a_eps) * ca.acos(
        ca.fmax(ca.fmin(nv[0] / n, 1.0 - eps), -1.0 + eps)
    ) + _smooth_heaviside(-(nv[1] + eps), a_eps) * (
        2 * np.pi - ca.acos(ca.fmax(ca.fmin(nv[0] / n, 1.0 - eps), -1.0 + eps))
    )

    om = _smooth_heaviside(ev[2], a_eps) * ca.acos(
        ca.fmax(ca.fmin(nv_dot_ev / (n * e), 1.0 - eps), -1.0 + eps)
    ) + _smooth_heaviside(-ev[2], a_eps) * (
        2 * np.pi - ca.acos(ca.fmax(ca.fmin(nv_dot_ev / (n * e), 1.0 - eps), -1.0 + eps))
    )

    ev_dot_rv = _dot_product(ev, rv)
    nu = _smooth_heaviside(rv_dot_vv, a_eps) * ca.acos(
        ca.fmax(ca.fmin(ev_dot_rv / (e * r), 1.0 - eps), -1.0 + eps)
    ) + _smooth_heaviside(-rv_dot_vv, a_eps) * (
        2 * np.pi - ca.acos(ca.fmax(ca.fmin(ev_dot_rv / (e * r), 1.0 - eps), -1.0 + eps))
    )

    return ca.vertcat(a, e, i, Om, om, nu)


def _define_phase_dynamics(phase, r_vars_s, v_vars_s, m_var_s, u_vars, phase_num):
    # Convert scaled variables to physical units
    r_vec_s = ca.vertcat(*r_vars_s)
    v_vec_s = ca.vertcat(*v_vars_s)
    u_vec = ca.vertcat(*u_vars)

    r_vec = r_vec_s * R_SCALE
    v_vec = v_vec_s * V_SCALE
    m_phys = m_var_s * M_SCALE

    rad = ca.sqrt(ca.fmax(_dot_product(r_vec, r_vec), 1e-12))

    # Relative velocity accounting for Earth rotation
    vrel = v_vec - OMEGA_MATRIX @ r_vec
    speedrel = ca.sqrt(ca.fmax(_dot_product(vrel, vrel), 1e-12))

    # Atmospheric drag
    altitude = rad - RE_EARTH
    rho = RHO0 * ca.exp(-altitude / H_SCALE_HEIGHT)
    bc = (rho / (2 * m_phys)) * SA * CD
    bcspeed = bc * speedrel
    drag = -vrel * bcspeed

    # Gravitational acceleration
    muoverradcubed = MU_EARTH / (rad**3)
    grav = -muoverradcubed * r_vec

    # Phase-specific thrust and mass flow
    if phase_num == 1:
        T_tot = 6 * THRUST_SRB + THRUST_FIRST
        mdot = -(6 * THRUST_SRB / (G0 * ISP_SRB) + THRUST_FIRST / (G0 * ISP_FIRST))
    elif phase_num == 2:
        T_tot = 3 * THRUST_SRB + THRUST_FIRST
        mdot = -(3 * THRUST_SRB / (G0 * ISP_SRB) + THRUST_FIRST / (G0 * ISP_FIRST))
    elif phase_num == 3:
        T_tot = THRUST_FIRST
        mdot = -THRUST_FIRST / (G0 * ISP_FIRST)
    elif phase_num == 4:
        T_tot = THRUST_SECOND
        mdot = -THRUST_SECOND / (G0 * ISP_SECOND)

    # Thrust acceleration
    Toverm = T_tot / m_phys
    thrust = Toverm * u_vec

    # Total acceleration
    acceleration = thrust + drag + grav

    # Scaled derivatives
    r_dot_scaled = v_vec / R_SCALE
    v_dot_scaled = acceleration / V_SCALE
    m_dot_scaled = mdot / M_SCALE

    # Define dynamics
    dynamics_dict = {}
    for i in range(3):
        dynamics_dict[r_vars_s[i]] = r_dot_scaled[i]
        dynamics_dict[v_vars_s[i]] = v_dot_scaled[i]
    dynamics_dict[m_var_s] = m_dot_scaled

    phase.dynamics(dynamics_dict)

    # Unit thrust vector constraint
    thrust_magnitude_squared = _dot_product(u_vec, u_vec)
    phase.path_constraints(thrust_magnitude_squared == 1.0)


def _generate_phase_guess(N_list, r_init_s, v_init_s, m_init_s, m_final_s):
    states_guess = []
    controls_guess = []

    for N in N_list:
        # State guess (7 states × N+1 points)
        states = np.zeros((7, N + 1))
        for i in range(3):
            states[i, :] = r_init_s[i]  # Constant position
            states[i + 3, :] = v_init_s[i]  # Constant velocity
        states[6, :] = np.linspace(m_init_s, m_final_s, N + 1)  # Linear mass variation
        states_guess.append(states)

        # Control guess (3 controls × N points)
        controls = np.zeros((3, N))
        controls[0, :] = 1.0  # [1, 0, 0] unit vector
        controls_guess.append(controls)

    return states_guess, controls_guess


# ============================================================================
# Problem Setup
# ============================================================================

problem = mtor.Problem("Multiphase Vehicle Launch")


# ============================================================================
# Phase 1: Six Solid Rocket Boosters + First Stage
# ============================================================================

phase1 = problem.set_phase(1)
t_1 = phase1.time(initial=0.0, final=T_PHASE_1_END)
r1_s = [
    phase1.state(f"r{i}_scaled", initial=R_INITIAL_SCALED[i], boundary=(R_MIN, R_MAX))
    for i in range(3)
]
v1_s = [
    phase1.state(f"v{i}_scaled", initial=V_INITIAL_SCALED[i], boundary=(V_MIN, V_MAX))
    for i in range(3)
]
m1_s = phase1.state("mass_scaled", initial=M_INITIAL_P1, boundary=(M_FINAL_P1, M_INITIAL_P1))
u1 = [phase1.control(f"u{i}", boundary=(-1.0, 1.0)) for i in range(3)]

_define_phase_dynamics(phase1, r1_s, v1_s, m1_s, u1, 1)


# ============================================================================
# Phase 2: Three Solid Rocket Boosters + First Stage
# ============================================================================

phase2 = problem.set_phase(2)
t_2 = phase2.time(initial=T_PHASE_1_END, final=T_PHASE_2_END)
r2_s = [
    phase2.state(f"r{i}_scaled", initial=r1_s[i].final, boundary=(R_MIN, R_MAX)) for i in range(3)
]
v2_s = [
    phase2.state(f"v{i}_scaled", initial=v1_s[i].final, boundary=(V_MIN, V_MAX)) for i in range(3)
]
m2_s = phase2.state(
    "mass_scaled", initial=m1_s.final - 6 * M_DRY_SRB / M_SCALE, boundary=(M_FINAL_P2, M_INITIAL_P2)
)
u2 = [phase2.control(f"u{i}", boundary=(-1.0, 1.0)) for i in range(3)]

_define_phase_dynamics(phase2, r2_s, v2_s, m2_s, u2, 2)


# ============================================================================
# Phase 3: First Stage Only
# ============================================================================

phase3 = problem.set_phase(3)
t_3 = phase3.time(initial=T_PHASE_2_END, final=T_PHASE_3_END)
r3_s = [
    phase3.state(f"r{i}_scaled", initial=r2_s[i].final, boundary=(R_MIN, R_MAX)) for i in range(3)
]
v3_s = [
    phase3.state(f"v{i}_scaled", initial=v2_s[i].final, boundary=(V_MIN, V_MAX)) for i in range(3)
]
m3_s = phase3.state(
    "mass_scaled", initial=m2_s.final - 3 * M_DRY_SRB / M_SCALE, boundary=(M_FINAL_P3, M_INITIAL_P3)
)
u3 = [phase3.control(f"u{i}", boundary=(-1.0, 1.0)) for i in range(3)]

_define_phase_dynamics(phase3, r3_s, v3_s, m3_s, u3, 3)


# ============================================================================
# Phase 4: Second Stage Only
# ============================================================================

phase4 = problem.set_phase(4)
t_4 = phase4.time(initial=T_PHASE_3_END, final=(T_PHASE_3_END, T_PHASE_4_END))
r4_s = [
    phase4.state(f"r{i}_scaled", initial=r3_s[i].final, boundary=(R_MIN, R_MAX)) for i in range(3)
]
v4_s = [
    phase4.state(f"v{i}_scaled", initial=v3_s[i].final, boundary=(V_MIN, V_MAX)) for i in range(3)
]
m4_s = phase4.state(
    "mass_scaled", initial=m3_s.final - M_DRY_FIRST / M_SCALE, boundary=(M_FINAL_P4, M_INITIAL_P4)
)
u4 = [phase4.control(f"u{i}", boundary=(-1.0, 1.0)) for i in range(3)]

_define_phase_dynamics(phase4, r4_s, v4_s, m4_s, u4, 4)


# ============================================================================
# Target Orbital Elements Constraint
# ============================================================================

# Convert target orbital elements to position/velocity
target_oe = [A_TARGET, E_TARGET, INC_TARGET, RAAN_TARGET, AOP_TARGET, 0.0]
rout_phys, vout_phys = _oe2rv(target_oe, MU_EARTH)

# Convert scaled final state to physical units for constraint evaluation
r_final_s = ca.vertcat(*[r4_s[i].final for i in range(3)])
v_final_s = ca.vertcat(*[v4_s[i].final for i in range(3)])
r_final = r_final_s * R_SCALE
v_final = v_final_s * V_SCALE

# Compute final orbital elements
oe_final = _rv2oe(r_final, v_final, MU_EARTH)

# Constrain five orbital elements (true anomaly is free)
phase4.event_constraints(
    oe_final[0] == A_TARGET,  # Semi-major axis
    oe_final[1] == E_TARGET,  # Eccentricity
    oe_final[2] == INC_TARGET,  # Inclination
    oe_final[3] == RAAN_TARGET,  # Right ascension of ascending node
    oe_final[4] == AOP_TARGET,  # Argument of periapsis
)


# ============================================================================
# Objective Function
# ============================================================================

# Maximize final mass (minimize negative mass)
problem.minimize(-m4_s.final)


# ============================================================================
# Mesh Configuration and Initial Guess
# ============================================================================

# Mesh configuration
phase1.mesh([4, 4], [-1.0, 0.0, 1.0])
phase2.mesh([4, 4], [-1.0, 0.0, 1.0])
phase3.mesh([4, 4], [-1.0, 0.0, 1.0])
phase4.mesh([4, 4], [-1.0, 0.0, 1.0])

# Generate initial guesses for each phase
states_p1, controls_p1 = _generate_phase_guess(
    [4, 4], R_INITIAL_SCALED, V_INITIAL_SCALED, M_INITIAL_P1, M_FINAL_P1
)
states_p2, controls_p2 = _generate_phase_guess(
    [4, 4], R_INITIAL_SCALED, V_INITIAL_SCALED, M_INITIAL_P2, M_FINAL_P2
)
states_p3, controls_p3 = _generate_phase_guess(
    [4, 4], R_INITIAL_SCALED, V_INITIAL_SCALED, M_INITIAL_P3, M_FINAL_P3
)

# Target position and velocity for Phase 4 guess
rout_scaled = rout_phys / R_SCALE
vout_scaled = vout_phys / V_SCALE
states_p4, controls_p4 = _generate_phase_guess(
    [4, 4], rout_scaled, vout_scaled, M_INITIAL_P4, M_FINAL_P4
)

phase1.guess(states=states_p1, controls=controls_p1)
phase2.guess(states=states_p2, controls=controls_p2)
phase3.guess(states=states_p3, controls=controls_p3)
phase4.guess(states=states_p4, controls=controls_p4)


# ============================================================================
# Solve
# ============================================================================

solution = mtor.solve_adaptive(
    problem,
    max_iterations=15,
    min_polynomial_degree=3,
    max_polynomial_degree=10,
    nlp_options={
        "ipopt.max_iter": 1000,
        "ipopt.tol": 1e-6,
        "ipopt.constr_viol_tol": 1e-6,
        "ipopt.linear_solver": "mumps",
        "ipopt.print_level": 5,
    },
)


# ============================================================================
# Results Analysis
# ============================================================================

if solution.status["success"]:
    final_mass_scaled = -solution.status["objective"]
    final_mass = final_mass_scaled * M_SCALE
    mission_time = solution.status["total_mission_time"]

    print("Multiphase Vehicle Launch Solution:")
    print(f"Final mass: {final_mass:.1f} kg")
    print(f"Mission time: {mission_time:.1f} seconds")
    print(f"Payload fraction: {final_mass / (M_INITIAL_P1 * M_SCALE) * 100:.2f}%")

    # =============================================================================
    # COMPREHENSIVE BENCHMARK SHOWCASE
    # =============================================================================

    print("\n" + "=" * 70)
    print("MULTIPHASE ADAPTIVE MESH REFINEMENT BENCHMARK SHOWCASE")
    print("=" * 70)

    # Professional benchmark summary
    solution.print_benchmark_summary()

    # =============================================================================
    # MULTIPHASE BENCHMARK DATA ACCESS
    # =============================================================================

    print("\n" + "=" * 50)
    print("MULTIPHASE BENCHMARK DATA ACCESS")
    print("=" * 50)

    # Mission-wide benchmark arrays
    benchmark = solution.adaptive["benchmark"]
    print(f"Mission completed in {solution.adaptive['iterations']} adaptive iterations")
    print(f"Convergence: {'✓' if solution.adaptive['converged'] else '✗'}")

    # Phase-specific benchmark analysis
    phase_benchmarks = solution.adaptive["phase_benchmarks"]
    print("\nPhase-specific refinement patterns:")
    for phase_id in sorted(solution.phases.keys()):
        phase_data = phase_benchmarks[phase_id]
        final_error = phase_data["estimated_error"][-1]
        final_points = phase_data["collocation_points"][-1]
        final_intervals = phase_data["mesh_intervals"][-1]

        phase_name = {
            1: "6 SRBs + First",
            2: "3 SRBs + First",
            3: "First Stage",
            4: "Second Stage",
        }[phase_id]

        print(
            f"  Phase {phase_id} ({phase_name}): "
            f"Error={final_error:.2e}, Points={final_points}, Intervals={final_intervals}"
        )

    # =============================================================================
    # RESEARCH-GRADE DATA ANALYSIS
    # =============================================================================

    print("\n" + "=" * 50)
    print("RESEARCH-GRADE PERFORMANCE ANALYSIS")
    print("=" * 50)

    # Convergence analysis
    errors = benchmark["estimated_error"]
    points = benchmark["collocation_points"]

    finite_errors = [e for e in errors[1:] if not (np.isnan(e) or np.isinf(e))]
    if len(finite_errors) >= 2:
        convergence_rate = np.log(finite_errors[0] / finite_errors[-1]) / (len(finite_errors) - 1)
        efficiency = (finite_errors[0] / finite_errors[-1]) / (points[-1] / points[0])

        print("Algorithm Performance Metrics:")
        print(f"  Error reduction: {finite_errors[0] / finite_errors[-1]:.1e}x")
        print(f"  Convergence rate: {convergence_rate:.3f} per iteration")
        print(f"  Computational efficiency: {efficiency:.3f}")

    # Phase convergence comparison
    print("\nPhase Convergence Analysis:")
    for phase_id, converged in solution.adaptive["phase_converged"].items():
        status = "CONVERGED" if converged else "NEEDS REFINEMENT"
        print(f"  Phase {phase_id}: {status}")

    # =============================================================================
    # PROFESSIONAL VISUALIZATION
    # =============================================================================

    print("\n" + "=" * 50)
    print("PROFESSIONAL VISUALIZATION")
    print("=" * 50)

    try:
        # Mission trajectory visualization
        print("Generating mission trajectory plots...")
        solution.plot(show_phase_boundaries=True, figsize=(16, 10))

        # Mesh refinement history for complex phases
        print("Generating mesh refinement histories...")

        # Show refinement for most complex phase (Phase 4 - orbital insertion)
        solution.plot_refinement_history(
            phase_id=4, figsize=(14, 8), transform_domain=(T_PHASE_3_END, T_PHASE_4_END)
        )

        # Comparative refinement visualization
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle("Multiphase Adaptive Refinement Comparison", fontsize=16)

        # Error convergence per phase
        ax1 = axes[0, 0]
        for phase_id, phase_data in phase_benchmarks.items():
            phase_errors = [e for e in phase_data["estimated_error"][1:] if not np.isnan(e)]
            if phase_errors:
                ax1.semilogy(
                    range(1, len(phase_errors) + 1),
                    phase_errors,
                    "o-",
                    label=f"Phase {phase_id}",
                    linewidth=2,
                    markersize=6,
                )
        ax1.set_xlabel("Iteration")
        ax1.set_ylabel("Error Estimate")
        ax1.set_title("Phase Error Convergence")
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Computational cost per phase
        ax2 = axes[0, 1]
        phase_names = ["6 SRBs+First", "3 SRBs+First", "First Only", "Second Only"]
        final_points_per_phase = [
            phase_benchmarks[i]["collocation_points"][-1] for i in range(1, 5)
        ]
        bars = ax2.bar(
            phase_names, final_points_per_phase, color=["red", "orange", "blue", "green"]
        )
        ax2.set_ylabel("Final Collocation Points")
        ax2.set_title("Computational Cost by Phase")
        ax2.tick_params(axis="x", rotation=45)

        # Add value labels on bars
        for bar, value in zip(bars, final_points_per_phase, strict=False):
            ax2.text(
                bar.get_x() + bar.get_width() / 2,
                bar.get_height() + 1,
                str(value),
                ha="center",
                va="bottom",
            )

        # Mission-wide convergence
        ax3 = axes[1, 0]
        ax3.semilogy(
            benchmark["mesh_iteration"][1:],
            finite_errors,
            "bo-",
            linewidth=3,
            markersize=8,
            label="Mission Error",
        )
        ax3.axhline(
            y=solution.adaptive["target_tolerance"],
            color="r",
            linestyle="--",
            linewidth=2,
            label=f"Target: {solution.adaptive['target_tolerance']:.1e}",
        )
        ax3.set_xlabel("Iteration")
        ax3.set_ylabel("Maximum Error")
        ax3.set_title("Mission-Wide Error Convergence")
        ax3.legend()
        ax3.grid(True, alpha=0.3)

        # Total computational cost evolution
        ax4 = axes[1, 1]
        ax4.plot(
            benchmark["mesh_iteration"],
            benchmark["collocation_points"],
            "go-",
            linewidth=3,
            markersize=8,
        )
        ax4.set_xlabel("Iteration")
        ax4.set_ylabel("Total Collocation Points")
        ax4.set_title("Computational Cost Evolution")
        ax4.grid(True, alpha=0.3)

        plt.tight_layout()
        plt.show()

    except ImportError:
        print("matplotlib not available for visualization")

    # =============================================================================
    # DATA EXPORT FOR RESEARCH
    # =============================================================================

    print("\n" + "=" * 50)
    print("RESEARCH DATA EXPORT")
    print("=" * 50)

    # Mission-wide CSV export
    print("Mission-wide benchmark data (CSV format):")
    print("iteration,error,total_points,total_intervals,refinement_actions")
    for i in range(len(benchmark["mesh_iteration"])):
        iteration = benchmark["mesh_iteration"][i]
        error = benchmark["estimated_error"][i]
        points = benchmark["collocation_points"][i]
        intervals = benchmark["mesh_intervals"][i]
        actions = len(benchmark["refinement_strategy"][i])

        error_str = "NaN" if np.isnan(error) else f"{error:.6e}"
        print(f"{iteration},{error_str},{points},{intervals},{actions}")

    # Phase-specific data summary
    print("\nPhase-specific summary:")
    for phase_id in sorted(phase_benchmarks.keys()):
        phase_data = phase_benchmarks[phase_id]
        print(
            f"Phase {phase_id}: {len(phase_data['mesh_iteration'])} iterations, "
            f"final error {phase_data['estimated_error'][-1]:.2e}"
        )

else:
    print(f"Solution failed: {solution.status['message']}")
