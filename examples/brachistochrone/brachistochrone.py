import casadi as ca
import numpy as np

import maptor as mtor


# Problem setup
problem = mtor.Problem("Brachistochrone Problem")
phase = problem.set_phase(1)

# Variables
t = phase.time(initial=0.0)
x = phase.state("x", initial=0.0, final=2.0)
y = phase.state("y", initial=0.0, final=2.0)
v = phase.state("v", initial=0.0)
theta = phase.control("u", boundary=(0.0, np.pi / 2))

# Dynamics
g = 9.81
phase.dynamics({x: v * ca.sin(theta), y: v * ca.cos(theta), v: g * ca.cos(theta)})

# Objective
problem.minimize(t.final)

# Mesh and guess
phase.mesh([6], [-1.0, 1.0])

states_guess = [
    # Interval 1: 7 state points
    [
        [0.0, 1.0, 2.0, 3.0, 4.0, 4.5, 5.0],  # x
        [10.0, 9.5, 9.0, 8.5, 8.0, 7.5, 7.0],  # y
        [0.0, 1.0, 2.0, 2.5, 3.0, 3.5, 4.0],  # v
    ]
]

controls_guess = (
    [
        # Interval 1: 6 control points
        [0.8, 0.7, 0.6, 0.5, 0.4, 0.3]
    ],
)


phase.guess(
    states=states_guess,
    controls=controls_guess,
    terminal_time=1.0,
)
# Solve with adaptive mesh
solution = mtor.solve_adaptive(
    problem,
    error_tolerance=1e-6,
    max_iterations=30,
    min_polynomial_degree=3,
    max_polynomial_degree=12,
    nlp_options={"ipopt.print_level": 0, "ipopt.max_iter": 500},
)

# Results
if solution.status["success"]:
    print(f"Adaptive objective: {solution.status['objective']:.9f}")
    solution.plot()

    # Compare with fixed mesh
    phase.mesh([6], [-1.0, 1.0])

    states_guess = [
        # Interval 1: 7 state points
        [
            [0.0, 1.0, 2.0, 3.0, 4.0, 4.5, 5.0],  # x
            [10.0, 9.5, 9.0, 8.5, 8.0, 7.5, 7.0],  # y
            [0.0, 1.0, 2.0, 2.5, 3.0, 3.5, 4.0],  # v
        ]
    ]

    controls_guess = (
        [
            # Interval 1: 6 control points
            [0.8, 0.7, 0.6, 0.5, 0.4, 0.3]
        ],
    )

    phase.guess(
        states=states_guess,
        controls=controls_guess,
        terminal_time=1.0,
    )

    fixed_solution = mtor.solve_fixed_mesh(
        problem, nlp_options={"ipopt.print_level": 0, "ipopt.max_iter": 500}
    )

    if fixed_solution.status["success"]:
        print(f"Fixed mesh objective: {fixed_solution.status['objective']:.9f}")
        print(
            f"Difference: {abs(solution.status['objective'] - fixed_solution.status['objective']):.2e}"
        )
        fixed_solution.plot()
    else:
        print(f"Fixed mesh failed: {fixed_solution.status['message']}")

else:
    print(f"Failed: {solution.status['message']}")
