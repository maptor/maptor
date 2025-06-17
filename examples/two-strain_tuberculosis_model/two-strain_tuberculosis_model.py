import numpy as np

import maptor as mtor


# Problem parameters from Table 10.30
beta1 = 13
beta2 = 13
mu = 0.0143
d1 = 0
d2 = 0
k1 = 0.5
k2 = 1
r1 = 2
r2 = 1
p = 0.4
q = 0.1
N = 30000
beta_star = 0.029
B1 = 50
B2 = 500
Lambda = mu * N  # 429

# Problem setup
problem = mtor.Problem("Two-Strain Tuberculosis Model")
phase = problem.set_phase(1)

# Variables
t = phase.time(initial=0.0, final=5.0)
S = phase.state("S", initial=19000.0)  # 76*N/120
T = phase.state("T", initial=250.0)  # N/120
L1 = phase.state("L1", initial=9000.0)  # 36*N/120
I1 = phase.state("I1", initial=1000.0)  # 4*N/120
L2 = phase.state("L2", initial=500.0)  # 2*N/120
I2 = phase.state("I2", initial=250.0)  # N/120

u1 = phase.control("u1", boundary=(0.05, 0.95))
u2 = phase.control("u2", boundary=(0.05, 0.95))

# Dynamics (equations 10.1009-10.1014)
phase.dynamics(
    {
        S: Lambda - beta1 * S * (I1 / N) - beta_star * S * (I2 / N) - mu * S,
        T: (
            u1 * r1 * L1
            - mu * T
            + (1 - (1 - u2) * (p + q)) * r2 * I1
            - beta2 * T * (I1 / N)
            - beta_star * T * (I2 / N)
        ),
        L1: (
            beta1 * S * (I1 / N)
            - (mu + k1) * L1
            - u1 * r1 * L1
            + (1 - u2) * p * r2 * I1
            + beta2 * T * (I1 / N)
            - beta_star * L1 * (I2 / N)
        ),
        L2: ((1 - u2) * q * r2 * I1 - (mu + k2) * L2 + beta_star * (S + L1 + T) * (I2 / N)),
        I1: k1 * L1 - (mu + d1) * I1 - r2 * I1,
        I2: k2 * L2 - (mu + d2) * I2,
    }
)

# Objective: minimize L2 + I2 + 0.5*B1*u1^2 + 0.5*B2*u2^2
integrand = L2 + I2 + 0.5 * B1 * u1**2 + 0.5 * B2 * u2**2
integral_var = phase.add_integral(integrand)
problem.minimize(integral_var)

# Mesh and guess
phase.mesh([6, 6, 6], [-1.0, -1 / 3, 1 / 3, 1.0])

states_guess = []
controls_guess = []
for N_interval in [6, 6, 6]:
    tau = np.linspace(-1, 1, N_interval + 1)
    t_norm = (tau + 1) / 2

    # Linear interpolation for states
    S_vals = 19000.0 - 1000.0 * t_norm
    T_vals = 250.0 + 50.0 * t_norm
    L1_vals = 9000.0 - 500.0 * t_norm
    I1_vals = 1000.0 - 200.0 * t_norm
    L2_vals = 500.0 - 100.0 * t_norm
    I2_vals = 250.0 - 50.0 * t_norm

    states_guess.append(np.vstack([S_vals, T_vals, L1_vals, I1_vals, L2_vals, I2_vals]))

    # Control guess (mid-range values)
    u1_vals = np.full(N_interval, 0.5)
    u2_vals = np.full(N_interval, 0.5)
    controls_guess.append(np.vstack([u1_vals, u2_vals]))

phase.guess(states=states_guess, controls=controls_guess, integrals=5000.0)

# Solve
solution = mtor.solve_adaptive(
    problem,
    error_tolerance=1e-6,
    max_iterations=20,
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
    print(f"Objective: {solution.status['objective']:.5f}")
    print(
        f"Reference: 5152.07310 (Error: {abs(solution.status['objective'] - 5152.07310) / 5152.07310 * 100:.3f}%)"
    )

    # Final state values
    S_final = solution[(1, "S")][-1]
    T_final = solution[(1, "T")][-1]
    L1_final = solution[(1, "L1")][-1]
    I1_final = solution[(1, "I1")][-1]
    L2_final = solution[(1, "L2")][-1]
    I2_final = solution[(1, "I2")][-1]

    print("Result:")
    print(f"S: {S_final:.1f}")
    print(f"T: {T_final:.1f}")
    print(f"L1: {L1_final:.1f}")
    print(f"I1: {I1_final:.1f}")
    print(f"L2: {L2_final:.1f}")
    print(f"I2: {I2_final:.1f}")

    solution.plot()
else:
    print(f"Failed: {solution.status['message']}")
