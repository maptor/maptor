import numpy as np

import maptor as mtor


# Problem setup
problem = mtor.Problem("Two-Phase Schwartz Problem")

# Phase 1
phase1 = problem.set_phase(1)
phase1.time(initial=0.0, final=1.0)
x0_1 = phase1.state("x0", initial=1.0, boundary=(-20.0, 10.0))
x1_1 = phase1.state("x1", initial=1.0, boundary=(-0.8, 10.0))
u1 = phase1.control("u", boundary=(-1.0, 1.0))

phase1.dynamics(
    {
        x0_1: x1_1,
        x1_1: u1 - 0.1 * (1 + 2 * x0_1**2) * x1_1,
    }
)

# Path constraint: feasible region outside ellipse
elliptical_constraint = 1 - 9 * (x0_1 - 1) ** 2 - ((x1_1 - 0.4) / 0.3) ** 2
phase1.path_constraints(elliptical_constraint <= 0)
phase1.mesh([20], [-1.0, 1.0])

# Phase 2
phase2 = problem.set_phase(2)
phase2.time(initial=1.0, final=2.9)
x0_2 = phase2.state("x0", initial=x0_1.final, boundary=(-20.0, 10.0))
x1_2 = phase2.state("x1", initial=x1_1.final, boundary=(-10.0, 10.0))
u2 = phase2.control("u", boundary=(-50.0, 50.0))

phase2.dynamics(
    {
        x0_2: x1_2,
        x1_2: u2 - 0.1 * (1 + 2 * x0_2**2) * x1_2,
    }
)
phase2.mesh([20], [-1.0, 1.0])

# Objective
objective_expr = 5 * (x0_2.final**2 + x1_2.final**2)
problem.minimize(objective_expr)

# Guess
states_p1 = []
controls_p1 = []
states_p2 = []
controls_p2 = []

for N in [20]:
    tau_states = np.linspace(-1, 1, N + 1)
    t_norm_states = (tau_states + 1) / 2
    x0_vals = 1.0 + 0.2 * t_norm_states
    x1_vals = 1.0 - 0.3 * t_norm_states
    states_p1.append(np.array([x0_vals, x1_vals]))

    t_norm_controls = np.linspace(0, 1, N)
    u_vals = 0.3 * np.sin(np.pi * t_norm_controls)
    controls_p1.append(np.array([u_vals]))

for N in [20]:
    tau_states = np.linspace(-1, 1, N + 1)
    t_norm_states = (tau_states + 1) / 2
    x0_end_p1 = 1.2
    x1_end_p1 = 0.7
    x0_vals = x0_end_p1 * (1 - 0.8 * t_norm_states)
    x1_vals = x1_end_p1 * (1 - 0.9 * t_norm_states)
    states_p2.append(np.array([x0_vals, x1_vals]))

    t_norm_controls = np.linspace(0, 1, N)
    u_vals = -1.0 + 0.5 * t_norm_controls
    controls_p2.append(np.array([u_vals]))

phase1.guess(
    states=states_p1,
    controls=controls_p1,
    initial_time=0.0,
    terminal_time=1.0,
)

phase2.guess(
    states=states_p2,
    controls=controls_p2,
    initial_time=1.0,
    terminal_time=2.9,
)

# Solve
solution = mtor.solve_adaptive(
    problem,
    error_tolerance=1e-5,
    max_iterations=30,
    min_polynomial_degree=3,
    max_polynomial_degree=20,
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


# Results
if solution.status["success"]:
    print(f"Objective: {solution.status['objective']:.8f}")
    print(f"Total mission time: {solution.status['total_mission_time']:.6f}")

    # Variable access
    x0_final = solution[(2, "x0")][-1]
    x1_final = solution[(2, "x1")][-1]
    print(f"Final state: x0={x0_final:.6f}, x1={x1_final:.6f}")

    # Plotting
    solution.plot()

else:
    print(f"Failed: {solution.status['message']}")
