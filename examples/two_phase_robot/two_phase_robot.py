import numpy as np

import maptor as mtor


# Problem setup
problem = mtor.Problem("Two Phase Path Tracking Robot")

# Phase 1: First tracking phase (0 to 1 second)
phase1 = problem.set_phase(1)
t1 = phase1.time(initial=0.0, final=1.0)
x1_1 = phase1.state("x1", initial=0.0, boundary=(-10.0, 10.0))
x2_1 = phase1.state("x2", initial=0.0, boundary=(-10.0, 10.0))
x3_1 = phase1.state("x3", initial=0.5, boundary=(-10.0, 10.0))
x4_1 = phase1.state("x4", initial=0.0, boundary=(-10.0, 10.0))
u1_1 = phase1.control("u1", boundary=(-10.0, 10.0))
u2_1 = phase1.control("u2", boundary=(-10.0, 10.0))

# Phase 1 dynamics: Simple integrator
phase1.dynamics({x1_1: x3_1, x2_1: x4_1, x3_1: u1_1, x4_1: u2_1})

# Phase 1 reference tracking objective
w1, w2, w3, w4 = 100.0, 100.0, 500.0, 500.0
x1ref_1 = t1 / 2.0
x2ref_1 = 0.0
x3ref_1 = 0.5
x4ref_1 = 0.0

integrand_1 = (
    w1 * (x1_1 - x1ref_1) ** 2
    + w2 * (x2_1 - x2ref_1) ** 2
    + w3 * (x3_1 - x3ref_1) ** 2
    + w4 * (x4_1 - x4ref_1) ** 2
)
integral_1 = phase1.add_integral(integrand_1)

# Phase 2: Second tracking phase (1 to 2 seconds) with automatic continuity
phase2 = problem.set_phase(2)
t2 = phase2.time(initial=1.0, final=2.0)
x1_2 = phase2.state("x1", initial=x1_1.final, final=0.5, boundary=(-10.0, 10.0))
x2_2 = phase2.state("x2", initial=x2_1.final, final=0.5, boundary=(-10.0, 10.0))
x3_2 = phase2.state("x3", initial=x3_1.final, final=0.0, boundary=(-10.0, 10.0))
x4_2 = phase2.state("x4", initial=x4_1.final, final=0.5, boundary=(-10.0, 10.0))
u1_2 = phase2.control("u1", boundary=(-10.0, 10.0))
u2_2 = phase2.control("u2", boundary=(-10.0, 10.0))

# Phase 2 dynamics: Same integrator
phase2.dynamics({x1_2: x3_2, x2_2: x4_2, x3_2: u1_2, x4_2: u2_2})

# Phase 2 reference tracking objective
x1ref_2 = 0.5
x2ref_2 = (t2 - 1.0) / 2.0
x3ref_2 = 0.0
x4ref_2 = 0.5

integrand_2 = (
    w1 * (x1_2 - x1ref_2) ** 2
    + w2 * (x2_2 - x2ref_2) ** 2
    + w3 * (x3_2 - x3ref_2) ** 2
    + w4 * (x4_2 - x4ref_2) ** 2
)
integral_2 = phase2.add_integral(integrand_2)

# Total objective: Sum of both phase costs
problem.minimize(integral_1 + integral_2)

# Mesh configuration
phase1.mesh([6, 6], [-1.0, 0.95, 1.0])
phase2.mesh([6, 6], [-1.0, -0.95, 1.0])


# Initial guess following PSOPT pattern
def _generate_phase_guess(N_intervals, t_start, t_end, x_start, x_end):
    """Generate initial guess for a phase matching C++ PSOPT format."""
    states_guess = []
    controls_guess = []

    for N in N_intervals:
        # State points (N+1 points for N degree polynomial)
        N_state_points = N + 1
        states = np.zeros((4, N_state_points))

        # Linear interpolation for states
        for i in range(4):
            states[i, :] = np.linspace(x_start[i], x_end[i], N_state_points)

        states_guess.append(states)

        # Control points (N points)
        controls = np.zeros((2, N))
        controls_guess.append(controls)

    return states_guess, controls_guess


# Phase 1 initial guess: from initial to midpoint
x_initial = [0.0, 0.0, 0.5, 0.0]
x_midpoint = [0.25, 0.25, 0.25, 0.25]  # (initial + final) / 2
states_p1, controls_p1 = _generate_phase_guess([6, 6], 0.0, 1.0, x_initial, x_midpoint)


# Phase 2 initial guess: from midpoint to final
x_final = [0.5, 0.5, 0.0, 0.5]
states_p2, controls_p2 = _generate_phase_guess([6, 6], 1.0, 2.0, x_midpoint, x_final)

phase1.guess(
    states=states_p1,
    controls=controls_p1,
    integrals=10.0,
)

phase2.guess(
    states=states_p2,
    controls=controls_p2,
    integrals=10.0,
)

# Solve
solution = mtor.solve_adaptive(
    problem,
    error_tolerance=1e-5,
    max_iterations=25,
    min_polynomial_degree=4,
    max_polynomial_degree=8,
    nlp_options={
        "ipopt.max_iter": 500,
        "ipopt.linear_solver": "mumps",
        "ipopt.constr_viol_tol": 1e-10,
        "ipopt.print_level": 0,
        "ipopt.nlp_scaling_method": "gradient-based",
        "ipopt.mu_strategy": "adaptive",
        "ipopt.tol": 1e-10,
    },
)

# Results
if solution.status["success"]:
    total_cost = solution.status["objective"]
    total_time = solution.status["total_mission_time"]

    print(f"Total tracking cost: {total_cost:.6f}")
    print(f"Total mission time: {total_time:.1f} seconds")

    # Final state verification
    print("\nFinal states:")
    print(f"  x1: {solution[(2, 'x1')][-1]:.6f} (target: 0.5)")
    print(f"  x2: {solution[(2, 'x2')][-1]:.6f} (target: 0.5)")
    print(f"  x3: {solution[(2, 'x3')][-1]:.6f} (target: 0.0)")
    print(f"  x4: {solution[(2, 'x4')][-1]:.6f} (target: 0.5)")

    # Show phase information
    print("\nPhase Information:")
    for phase_id, phase_data in solution.phases.items():
        times = phase_data["times"]
        variables = phase_data["variables"]
        print(f"  Phase {phase_id}:")
        print(f"    Duration: {times['duration']:.3f} seconds")
        print(f"    States: {variables['state_names']}")
        print(f"    Controls: {variables['control_names']}")

    solution.plot(show_phase_boundaries=True)

else:
    print(f"Failed: {solution.status['message']}")
