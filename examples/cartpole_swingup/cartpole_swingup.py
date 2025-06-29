import casadi as ca
import numpy as np

import maptor as mtor


# ============================================================================
# Physical Parameters
# ============================================================================

# Cart-pole system parameters
M = 1.0  # Cart mass (kg)
m = 0.1  # Pole mass (kg)
l = 0.5  # Pole length (m)
g = 9.81  # Gravity (m/s²)


# ============================================================================
# Problem Setup
# ============================================================================

problem = mtor.Problem("Cartpole Swing-Up")
phase = problem.set_phase(1)


# ============================================================================
# Variables
# ============================================================================

# Time variable
t = phase.time(initial=0.0)

# State variables
x = phase.state("x", initial=0.0, final=0.0)  # Cart position
theta = phase.state("theta", initial=np.pi, final=0.0)  # Pole angle (inverted)
x_dot = phase.state("x_dot", initial=0.0, final=0.0)  # Cart velocity
theta_dot = phase.state("theta_dot", initial=0.0, final=0.0)  # Pole angular velocity

# Control variable
F = phase.control("F", boundary=(-10.0, 10.0))  # Applied force on cart


# ============================================================================
# Dynamics obtained from cartpole_dynamics.py
# ============================================================================

phase.dynamics(
    {
        x: x_dot,
        theta: theta_dot,
        x_dot: (
            4 * F + 3 * g * m * ca.sin(2 * theta) / 2 - 2 * l * m * ca.sin(theta) * theta_dot**2
        )
        / (4 * M + 3 * m * ca.sin(theta) ** 2 + m),
        theta_dot: 3
        * (
            2 * g * (M + m) * ca.sin(theta)
            + (2 * F - l * m * ca.sin(theta) * theta_dot**2) * ca.cos(theta)
        )
        / (l * (4 * M + 3 * m * ca.sin(theta) ** 2 + m)),
    }
)


# ============================================================================
# Objective
# ============================================================================

# Minimize final time
problem.minimize(t.final)


# ============================================================================
# Mesh Configuration and Initial Guess
# ============================================================================

phase.mesh([6, 6], [-1.0, 0.0, 1.0])

# Initial guess - linear interpolation from initial to final states
states_guess = []
controls_guess = []

for N in [6, 6]:
    tau = np.linspace(-1, 1, N + 1)
    t_norm = (tau + 1) / 2

    # Linear transition from inverted (π) to upright (0)
    x_vals = np.zeros(N + 1)
    theta_vals = np.pi * (1 - t_norm)
    x_dot_vals = np.zeros(N + 1)
    theta_dot_vals = np.zeros(N + 1)

    states_guess.append(np.vstack([x_vals, theta_vals, x_dot_vals, theta_dot_vals]))

    # Small control guess
    F_vals = np.ones(N) * 0.1
    controls_guess.append(np.array([F_vals]))

phase.guess(
    states=states_guess,
    controls=controls_guess,
    terminal_time=2.0,
)


# ============================================================================
# Solve
# ============================================================================

solution = mtor.solve_adaptive(
    problem,
    error_tolerance=1e-4,
    max_iterations=20,
    min_polynomial_degree=3,
    max_polynomial_degree=8,
    nlp_options={
        "ipopt.print_level": 0,
        "ipopt.max_iter": 500,
        "ipopt.tol": 1e-6,
    },
)


# ============================================================================
# Results
# ============================================================================

if solution.status["success"]:
    print(f"Objective: {solution.status['objective']:.6f}")
    print(f"Mission time: {solution.status['total_mission_time']:.3f} seconds")

    # Final states
    x_final = solution["x"][-1]
    theta_final = solution["theta"][-1]
    print(f"Final cart position: {x_final:.6f} m")
    print(f"Final pole angle: {theta_final:.6f} rad ({np.degrees(theta_final):.2f}°)")

    # Control statistics
    force_max = max(np.abs(solution["F"]))
    print(f"Maximum control force: {force_max:.3f} N")

    solution.plot()

else:
    print(f"Failed: {solution.status['message']}")
