import numpy as np

import maptor as mtor


# Constants
p_rho = 0.01
c1 = 2.83374
c2 = -0.80865
c3 = 0.71265
c4 = 17.2656
c5 = 27.0756
t_final = 9.0

# Problem setup
problem = mtor.Problem("Container Crane")
phase = problem.set_phase(1)

# Variables
t = phase.time(initial=0.0, final=t_final)
x1 = phase.state("x1", initial=0.0, final=10.0)
x2 = phase.state("x2", initial=22.0, final=14.0)
x3 = phase.state("x3", initial=0.0, final=0.0)
x4 = phase.state("x4", initial=0, final=2.5, boundary=(-2.5, 2.5))
x5 = phase.state("x5", initial=-1, final=0.0, boundary=(-1.0, 1.0))
x6 = phase.state("x6", initial=0.0, final=0.0)
u1 = phase.control("u1", boundary=(-c1, c1))
u2 = phase.control("u2", boundary=(c2, c3))

# Dynamics
phase.dynamics(
    {
        x1: x4,
        x2: x5,
        x3: x6,
        x4: u1 + c4 * x3,
        x5: u2,
        x6: -(u1 + c5 * x3 + 2.0 * x5 * x6) / x2,
    }
)

# Objective
integrand = 0.5 * (x3**2 + x6**2 + p_rho * (u1**2 + u2**2))
integral_var = phase.add_integral(integrand)
problem.minimize(integral_var)

# Mesh and guess
phase.mesh([6, 6, 6], [-1.0, -1 / 3, 1 / 3, 1.0])

states_guess = []
controls_guess = []
for N in [6, 6, 6]:
    tau = np.linspace(-1, 1, N + 1)
    t_norm = (tau + 1) / 2
    x1_vals = 0.0 + (10.0 - 0.0) * t_norm
    x2_vals = 22.0 + (14.0 - 22.0) * t_norm
    x3_vals = 0.0 + (0.0 - 0.0) * t_norm
    x4_vals = 0.0 + (2.5 - 0.0) * t_norm
    x5_vals = -1.0 + (0.0 - (-1.0)) * t_norm
    x6_vals = 0.0 + (0.0 - 0.0) * t_norm
    states_guess.append(np.vstack([x1_vals, x2_vals, x3_vals, x4_vals, x5_vals, x6_vals]))

    u1_mid = (-c1 + c1) / 2.0
    u2_mid = (c2 + c3) / 2.0
    controls_guess.append(np.vstack([np.full(N, u1_mid), np.full(N, u2_mid)]))

phase.guess(states=states_guess, controls=controls_guess, integrals=0.1)

# Solve
solution = mtor.solve_adaptive(
    problem,
    error_tolerance=5e-7,
    ode_method="Radau",
    ode_solver_tolerance=1e-9,
    nlp_options={"ipopt.print_level": 0, "ipopt.max_iter": 2000, "ipopt.tol": 1e-7},
)

# Results
if solution.status["success"]:
    print(f"Objective: {solution.status['objective']:.8f}")
    print(
        f"Reference: 0.0375195 (Error: {abs(solution.status['objective'] - 0.0375195) / 0.0375195 * 100:.2f}%)"
    )
    solution.plot()
else:
    print(f"Failed: {solution.status['message']}")
