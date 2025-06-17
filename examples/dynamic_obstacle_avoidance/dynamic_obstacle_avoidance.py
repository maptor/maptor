import casadi as ca
import numpy as np

import maptor as mtor


# Obstacle trajectory definition
OBSTACLE_WAYPOINTS = np.array(
    [
        [5.0, 5.0, 0.0],
        [12.0, 12.0, 3.0],
        [15.0, 15.0, 6.0],
        [20.0, 20.0, 12.0],
    ]
)

# Create module-level interpolants for obstacle trajectory
_times = OBSTACLE_WAYPOINTS[:, 2]
_x_coords = OBSTACLE_WAYPOINTS[:, 0]
_y_coords = OBSTACLE_WAYPOINTS[:, 1]

_x_interpolant = ca.interpolant("obs_x_interp", "linear", [_times], _x_coords)
_y_interpolant = ca.interpolant("obs_y_interp", "linear", [_times], _y_coords)


def obstacle_position(current_time):
    """Get obstacle position at given time with clamped interpolation."""
    t_clamped = ca.fmax(_times[0], ca.fmin(_times[-1], current_time))
    return _x_interpolant(t_clamped), _y_interpolant(t_clamped)


# Vehicle physical parameters
m = 1412.0
I_z = 1536.7
l_f = 1.06
l_r = 1.85
k_f = 128916.0
k_r = 85944.0

# Safety parameters
vehicle_radius = 1.5
obstacle_radius = 2.5
min_separation = vehicle_radius + obstacle_radius
u_min = 0.05

# Problem setup
problem = mtor.Problem("Dynamic Bicycle Model - Forward Motion Only")
phase = problem.set_phase(1)

# Variables
t = phase.time(initial=0.0)
x = phase.state("x_position", initial=0.0, final=20.0)
y = phase.state("y_position", initial=0.0, final=20.0)
phi = phase.state("heading", initial=np.pi / 3)
u = phase.state("longitudinal_velocity", initial=5.0, boundary=(u_min, 20.0))
v = phase.state("lateral_velocity", initial=0.0, boundary=(-15.0, 15.0))
omega = phase.state("yaw_rate", initial=0.0, boundary=(-3.0, 3.0))
a = phase.control("acceleration", boundary=(-8.0, 8.0))
delta = phase.control("steering_angle", boundary=(-0.5, 0.5))

# Dynamics
u_safe = ca.if_else(u < u_min, u_min, u)
alpha_f = (v + l_f * omega) / u_safe - delta
alpha_r = (v - l_r * omega) / u_safe
F_Y1 = -k_f * alpha_f
F_Y2 = -k_r * alpha_r

phase.dynamics(
    {
        x: u * ca.cos(phi) - v * ca.sin(phi),
        y: u * ca.sin(phi) + v * ca.cos(phi),
        phi: omega,
        u: a + v * omega - (F_Y1 * ca.sin(delta)) / m,
        v: -u * omega + (F_Y1 * ca.cos(delta) + F_Y2) / m,
        omega: (l_f * F_Y1 * ca.cos(delta) - l_r * F_Y2) / I_z,
    }
)

# Constraints
obs_x, obs_y = obstacle_position(t)
distance_squared = (x - obs_x) ** 2 + (y - obs_y) ** 2

phase.path_constraints(
    distance_squared >= min_separation**2,
    x >= -40.0,
    x <= 40.0,
    y >= -40.0,
    y <= 40.0,
)

# Objective
control_effort = phase.add_integral(a**2 + delta**2)
problem.minimize(t.final + 0.01 * control_effort)

# Mesh and guess
phase.mesh([8, 8, 8], [-1.0, -1 / 3, 1 / 3, 1.0])

# Solve
solution = mtor.solve_adaptive(
    problem,
    error_tolerance=1e-1,
    max_iterations=30,
    min_polynomial_degree=3,
    max_polynomial_degree=10,
    ode_solver_tolerance=1e-1,
    nlp_options={
        "ipopt.max_iter": 2000,
        "ipopt.tol": 1e-8,
        "ipopt.constr_viol_tol": 1e-4,
        "ipopt.linear_solver": "mumps",
        "ipopt.print_level": 5,
    },
)

# Results
if solution.status["success"]:
    print(f"Forward-only model objective: {solution.status['objective']:.6f}")
    print(f"Mission time: {solution.status['total_mission_time']:.6f} seconds")

    x_final = solution[(1, "x_position")][-1]
    y_final = solution[(1, "y_position")][-1]
    u_final = solution[(1, "longitudinal_velocity")][-1]
    v_final = solution[(1, "lateral_velocity")][-1]
    omega_final = solution[(1, "yaw_rate")][-1]

    print("Final state verification:")
    print(f"  Position: ({x_final:.2f}, {y_final:.2f}) m")
    print("  Target position: (20.0, 20.0) m")
    print(f"  Position error: {np.sqrt((x_final - 20.0) ** 2 + (y_final - 20.0) ** 2):.3f} m")
    print(f"  Final velocities: u = {u_final:.2f} m/s, v = {v_final:.2f} m/s")
    print(f"  Final yaw rate: ω = {omega_final:.2f} rad/s")

    solution.plot()

else:
    print(f"Optimization failed: {solution.status['message']}")
