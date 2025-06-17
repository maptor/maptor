import numpy as np

import maptor as mtor


# Problem setup
problem = mtor.Problem("LQR Problem")
phase = problem.set_phase(1)

# Variables
t = phase.time(initial=0, final=1)
x = phase.state("x", initial=1.0)
u = phase.control("u")

# Dynamics
phase.dynamics({x: 0.5 * x + u})

# Objective
integrand = 0.625 * x**2 + 0.5 * x * u + 0.5 * u**2
integral_var = phase.add_integral(integrand)
problem.minimize(integral_var)

# Mesh and guess
phase.mesh([4, 4], [-1.0, 0.0, 1.0])

# Initial guess - explicit arrays for each mesh interval
states_guess = []
controls_guess = []

# First interval: 5 state points, 4 control points
states_guess.append([[1.0, 0.8, 0.6, 0.4, 0.2]])
controls_guess.append([[-0.5, -0.4, -0.3, -0.2]])

# Second interval: 5 state points, 4 control points
states_guess.append([[0.2, 0.15, 0.1, 0.05, 0.0]])
controls_guess.append([[-0.1, -0.05, 0.0, 0.05]])

phase.guess(
    states=states_guess,
    controls=controls_guess,
    initial_time=0.0,
    terminal_time=1.0,
    integrals=0.5,
)

# Solve with adaptive mesh
solution = mtor.solve_adaptive(
    problem,
    error_tolerance=1e-6,
    max_iterations=20,
    min_polynomial_degree=3,
    max_polynomial_degree=10,
    nlp_options={"ipopt.print_level": 0, "ipopt.max_iter": 200},
)

# Results
if solution.status["success"]:
    print(f"Adaptive objective: {solution.status['objective']:.12f}")
    print("Literature reference: 0.380797077977481140")
    print(f"Error: {abs(solution.status['objective'] - 0.380797077977481140):.2e}")

    # Final state value
    x_final = solution["x"][-1]
    print(f"Final state: x(1) = {x_final:.6f}")

    solution.plot()

    # Compare with fixed mesh
    phase.mesh([8, 8], [-1.0, 0.0, 1.0])

    states_guess_fixed = []
    controls_guess_fixed = []

    # First interval: 9 state points, 8 control points
    x_vals_1 = np.linspace(1.0, 0.2, 9)
    states_guess_fixed.append(x_vals_1.reshape(1, -1))
    u_vals_1 = np.linspace(-0.5, -0.1, 8)
    controls_guess_fixed.append(u_vals_1.reshape(1, -1))

    # Second interval: 9 state points, 8 control points
    x_vals_2 = np.linspace(0.2, 0.0, 9)
    states_guess_fixed.append(x_vals_2.reshape(1, -1))
    u_vals_2 = np.linspace(-0.1, 0.05, 8)
    controls_guess_fixed.append(u_vals_2.reshape(1, -1))

    phase.guess(
        states=states_guess_fixed,
        controls=controls_guess_fixed,
        initial_time=0.0,
        terminal_time=1.0,
        integrals=0.5,
    )

    fixed_solution = mtor.solve_fixed_mesh(
        problem, nlp_options={"ipopt.print_level": 0, "ipopt.max_iter": 200}
    )

    if fixed_solution.status["success"]:
        print(f"Fixed mesh objective: {fixed_solution.status['objective']:.12f}")
        print(
            f"Difference: {abs(solution.status['objective'] - fixed_solution.status['objective']):.2e}"
        )
    else:
        print(f"Fixed mesh failed: {fixed_solution.status['message']}")

else:
    print(f"Failed: {solution.status['message']}")
