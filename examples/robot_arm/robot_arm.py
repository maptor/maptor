import casadi as ca
import numpy as np

import maptor as mtor


# Problem parameters
L = 5

# Problem setup
problem = mtor.Problem("Robot Arm Control")
phase = problem.set_phase(1)

# Variables (free final time - minimum time problem)
t = phase.time(initial=0.0)
y1 = phase.state("y1", initial=9.0 / 2.0, final=9.0 / 2.0)
y2 = phase.state("y2", initial=0.0, final=0.0)
y3 = phase.state("y3", initial=0.0, final=2.0 * np.pi / 3.0)
y4 = phase.state("y4", initial=0.0, final=0.0)
y5 = phase.state("y5", initial=np.pi / 4.0, final=np.pi / 4.0)
y6 = phase.state("y6", initial=0.0, final=0.0)

u1 = phase.control("u1", boundary=(-1.0, 1.0))
u2 = phase.control("u2", boundary=(-1.0, 1.0))
u3 = phase.control("u3", boundary=(-1.0, 1.0))

# Inertia calculations (equations 10.832, 10.833)
I_phi = (1.0 / 3.0) * ((L - y1) ** 3 + y1**3)
I_theta = I_phi * (ca.sin(y5)) ** 2

# Dynamics (equations 10.826-10.831)
phase.dynamics({y1: y2, y2: u1 / L, y3: y4, y4: u2 / I_theta, y5: y6, y6: u3 / I_phi})

# Objective: minimize final time (minimum time problem)
problem.minimize(t.final)

# Mesh and guess
phase.mesh([8, 8, 8], [-1.0, -1 / 3, 1 / 3, 1.0])

states_guess = []
controls_guess = []
for N in [8, 8, 8]:
    tau = np.linspace(-1, 1, N + 1)
    t_norm = (tau + 1) / 2

    # Linear interpolation between initial and final conditions
    y1_vals = 9.0 / 2.0 + (9.0 / 2.0 - 9.0 / 2.0) * t_norm  # Constant
    y2_vals = 0.0 + (0.0 - 0.0) * t_norm  # Constant at zero
    y3_vals = 0.0 + (2.0 * np.pi / 3.0 - 0.0) * t_norm  # Linear increase
    y4_vals = 0.0 + (0.0 - 0.0) * t_norm  # Constant at zero
    y5_vals = np.pi / 4.0 + (np.pi / 4.0 - np.pi / 4.0) * t_norm  # Constant
    y6_vals = 0.0 + (0.0 - 0.0) * t_norm  # Constant at zero

    states_guess.append(np.vstack([y1_vals, y2_vals, y3_vals, y4_vals, y5_vals, y6_vals]))

    # Control guess (small values for smooth motion)
    u1_vals = np.zeros(N)
    u2_vals = 0.1 * np.sin(np.pi * np.linspace(0, 1, N))  # Smooth motion for y3
    u3_vals = np.zeros(N)
    controls_guess.append(np.vstack([u1_vals, u2_vals, u3_vals]))

phase.guess(
    states=states_guess,
    controls=controls_guess,
    terminal_time=9.0,
)

# Solve
solution = mtor.solve_adaptive(
    problem,
    min_polynomial_degree=3,
    max_polynomial_degree=12,
    nlp_options={
        "ipopt.print_level": 0,
        "ipopt.max_iter": 3000,
        "ipopt.tol": 1e-8,
        "ipopt.constr_viol_tol": 1e-7,
    },
)

# Results
if solution.status["success"]:
    final_time = solution.phases[1]["times"]["final"]
    print(f"Minimum time: {final_time:.8f}")
    print(f"Reference: 9.14093620 (Error: {abs(final_time - 9.14093620) / 9.14093620 * 100:.3f}%)")

    # Final state verification
    y1_final = solution[(1, "y1")][-1]
    y2_final = solution[(1, "y2")][-1]
    y3_final = solution[(1, "y3")][-1]
    y4_final = solution[(1, "y4")][-1]
    y5_final = solution[(1, "y5")][-1]
    y6_final = solution[(1, "y6")][-1]

    print("Final states:")
    print(f"  y1: {y1_final:.6f} (target: {9.0 / 2.0:.6f})")
    print(f"  y2: {y2_final:.6f} (target: 0.0)")
    print(f"  y3: {y3_final:.6f} (target: {2.0 * np.pi / 3.0:.6f})")
    print(f"  y4: {y4_final:.6f} (target: 0.0)")
    print(f"  y5: {y5_final:.6f} (target: {np.pi / 4.0:.6f})")
    print(f"  y6: {y6_final:.6f} (target: 0.0)")

    solution.plot()
else:
    print(f"Failed: {solution.status['message']}")
