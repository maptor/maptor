import casadi as ca
import numpy as np
from scipy.interpolate import CubicSpline

import maptor as mtor


# ============================================================================
# Street Geometry
# ============================================================================

STREET_LEFT_BOUNDARY = -2.0
STREET_RIGHT_BOUNDARY = 20.0
STREET_BOTTOM = -10.0
STREET_TOP = 50.0

LANE_WIDTH = 3.6
STREET_CENTER = (STREET_LEFT_BOUNDARY + STREET_RIGHT_BOUNDARY) / 2
RIGHT_LANE_CENTER = STREET_CENTER + LANE_WIDTH / 2
LEFT_LANE_CENTER = STREET_CENTER - LANE_WIDTH / 2


# ============================================================================
# Obstacle Trajectories
# ============================================================================

AGENT_START = (RIGHT_LANE_CENTER, 0.0)
AGENT_END = (RIGHT_LANE_CENTER, 50.0)

OBSTACLE_1_WAYPOINTS = np.array(
    [
        [RIGHT_LANE_CENTER, 15.0, 0.0],
        [RIGHT_LANE_CENTER, 40.0, 5.0],
    ]
)

OBSTACLE_2_WAYPOINTS = np.array(
    [
        [LEFT_LANE_CENTER, 55.0, 0.0],
        [LEFT_LANE_CENTER, 0.0, 15.0],
    ]
)

# Create interpolants for obstacle motion
_times_1 = OBSTACLE_1_WAYPOINTS[:, 2]
_x_coords_1 = OBSTACLE_1_WAYPOINTS[:, 0]
_y_coords_1 = OBSTACLE_1_WAYPOINTS[:, 1]

_times_2 = OBSTACLE_2_WAYPOINTS[:, 2]
_x_coords_2 = OBSTACLE_2_WAYPOINTS[:, 0]
_y_coords_2 = OBSTACLE_2_WAYPOINTS[:, 1]

_x_interpolant_1 = ca.interpolant("obs_x_interp_1", "linear", [_times_1], _x_coords_1)
_y_interpolant_1 = ca.interpolant("obs_y_interp_1", "linear", [_times_1], _y_coords_1)
_x_interpolant_2 = ca.interpolant("obs_x_interp_2", "linear", [_times_2], _x_coords_2)
_y_interpolant_2 = ca.interpolant("obs_y_interp_2", "linear", [_times_2], _y_coords_2)


def obstacle_1_position(current_time):
    """Get first obstacle position with clamped interpolation."""
    t_clamped = ca.fmax(_times_1[0], ca.fmin(_times_1[-1], current_time))
    return _x_interpolant_1(t_clamped), _y_interpolant_1(t_clamped)


def obstacle_2_position(current_time):
    """Get second obstacle position with clamped interpolation."""
    t_clamped = ca.fmax(_times_2[0], ca.fmin(_times_2[-1], current_time))
    return _x_interpolant_2(t_clamped), _y_interpolant_2(t_clamped)


# ============================================================================
# Vehicle Parameters
# ============================================================================

m = 1412.0
I_z = 1536.7
l_f = 1.06
l_r = 1.85
k_f = 128916.0
k_r = 85944.0

vehicle_radius = 1.5
obstacle_radius = 1.5
min_separation = vehicle_radius + obstacle_radius + 0.5
u_min = 0.5


# ============================================================================
# Problem Setup
# ============================================================================

problem = mtor.Problem("Robust Street Overtaking")
phase = problem.set_phase(1)


# ============================================================================
# Variables
# ============================================================================

t = phase.time(initial=0.0)

x = phase.state("x_position", initial=AGENT_START[0], final=AGENT_END[0])
y = phase.state("y_position", initial=AGENT_START[1], final=AGENT_END[1])
phi = phase.state("heading", initial=np.pi / 2, final=np.pi / 2)

u = phase.state("longitudinal_velocity", initial=12.0, boundary=(u_min, 30.0))
v = phase.state("lateral_velocity", initial=0.0, boundary=(-5.0, 5.0))
omega = phase.state("yaw_rate", initial=0.0, boundary=(-2.5, 2.5))

a = phase.control("acceleration", boundary=(-6.0, 6.0))
delta = phase.control("steering_angle", boundary=(-0.4, 0.4))


# ============================================================================
# Dynamics
# ============================================================================

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


# ============================================================================
# Constraints
# ============================================================================

# Collision avoidance
obs_x_1, obs_y_1 = obstacle_1_position(t)
obs_x_2, obs_y_2 = obstacle_2_position(t)

distance_squared_1 = (x - obs_x_1) ** 2 + (y - obs_y_1) ** 2
distance_squared_2 = (x - obs_x_2) ** 2 + (y - obs_y_2) ** 2

phase.path_constraints(
    distance_squared_1 >= min_separation**2 - 1.0,
    distance_squared_2 >= min_separation**2 - 1.0,
)

# Workspace bounds
phase.path_constraints(
    x >= STREET_LEFT_BOUNDARY,
    x <= STREET_RIGHT_BOUNDARY,
    y >= STREET_BOTTOM,
    y <= STREET_TOP,
)


# ============================================================================
# Objective
# ============================================================================

time_scale = 5.0
control_scale = 10.0

normalized_time = t.final / time_scale
normalized_control = phase.add_integral((a / control_scale) ** 2 + (delta / 0.3) ** 2)

problem.minimize(normalized_time + 0.02 * normalized_control)


# ============================================================================
# Mesh Configuration
# ============================================================================

POLYNOMIAL_DEGREES = [4, 4, 4, 6, 7, 8, 6, 5, 4, 4, 4]

MESH_NODES = [-1.0, -0.7, -0.4, -0.2, -0.1, 0.0, 0.2, 0.4, 0.6, 0.8, 0.9, 1.0]

phase.mesh(POLYNOMIAL_DEGREES, MESH_NODES)


# ============================================================================
# Initial Guess Generation
# ============================================================================


def _generate_robust_initial_guess():
    """Generate weight-insensitive initial guess with conservative timing."""
    waypoints_t = np.array([0.0, 0.2, 0.35, 0.5, 0.65, 0.8, 1.0])

    waypoints_x = np.array(
        [
            RIGHT_LANE_CENTER,
            RIGHT_LANE_CENTER,
            (RIGHT_LANE_CENTER + LEFT_LANE_CENTER) / 2,
            LEFT_LANE_CENTER,
            (RIGHT_LANE_CENTER + LEFT_LANE_CENTER) / 2,
            RIGHT_LANE_CENTER,
            RIGHT_LANE_CENTER,
        ]
    )

    waypoints_y = np.array([0.0, 10.0, 20.0, 30.0, 40.0, 45.0, 50.0])
    waypoints_phi = np.array(
        [np.pi / 2, np.pi / 2, np.pi / 2 + 0.1, np.pi / 2, np.pi / 2 - 0.1, np.pi / 2, np.pi / 2]
    )
    waypoints_u = np.array([12.0, 14.0, 16.0, 18.0, 16.0, 14.0, 12.0])

    spline_x = CubicSpline(waypoints_t, waypoints_x, bc_type="natural")
    spline_y = CubicSpline(waypoints_t, waypoints_y, bc_type="natural")
    spline_phi = CubicSpline(waypoints_t, waypoints_phi, bc_type="natural")
    spline_u = CubicSpline(waypoints_t, waypoints_u, bc_type="natural")

    states_guess = []
    controls_guess = []

    for N in POLYNOMIAL_DEGREES:
        tau = np.linspace(-1, 1, N + 1)
        t_norm = (tau + 1) / 2

        x_vals = spline_x(t_norm)
        y_vals = spline_y(t_norm)
        phi_vals = spline_phi(t_norm)
        u_vals = spline_u(t_norm)

        v_vals = np.gradient(x_vals) * 1.5
        omega_vals = np.gradient(phi_vals) * 1.0

        v_vals = np.clip(v_vals, -10.0, 10.0)
        omega_vals = np.clip(omega_vals, -2.0, 2.0)

        states_guess.append(np.vstack([x_vals, y_vals, phi_vals, u_vals, v_vals, omega_vals]))

        a_vals = np.gradient(u_vals)[:N] * 1.0
        delta_vals = np.gradient(phi_vals)[:N] * 2.0

        a_vals = np.clip(a_vals, -4.0, 4.0)
        delta_vals = np.clip(delta_vals, -0.3, 0.3)

        controls_guess.append(np.vstack([a_vals, delta_vals]))

    return states_guess, controls_guess


# ============================================================================
# Initial Guess and Solve
# ============================================================================

states_guess, controls_guess = _generate_robust_initial_guess()

phase.guess(
    states=states_guess,
    controls=controls_guess,
    terminal_time=5.0,
)

solution = mtor.solve_adaptive(
    problem,
    error_tolerance=1e-3,
    max_iterations=15,
    min_polynomial_degree=3,
    max_polynomial_degree=8,
    ode_solver_tolerance=1e-1,
    nlp_options={
        "ipopt.max_iter": 1000,
        "ipopt.tol": 1e-6,
        "ipopt.constr_viol_tol": 1e-4,
        "ipopt.linear_solver": "mumps",
        "ipopt.print_level": 3,
        "ipopt.mu_strategy": "adaptive",
        "ipopt.acceptable_tol": 1e-4,
        "ipopt.acceptable_iter": 5,
    },
)


# ============================================================================
# Results
# ============================================================================

if solution.status["success"]:
    print(f"  Objective: {solution.status['objective']:.6f}")
    print(f"  Mission time: {solution.status['total_mission_time']:.3f} seconds")

    x_traj = solution["x_position"]
    y_traj = solution["y_position"]
    u_traj = solution["longitudinal_velocity"]
    v_traj = solution["lateral_velocity"]

    max_lateral_deviation = abs(x_traj - RIGHT_LANE_CENTER).max()
    max_speed = u_traj.max()
    max_lateral_velocity = abs(v_traj).max()

    print(f"  Max lateral deviation: {max_lateral_deviation:.2f} m")
    print(f"  Max speed: {max_speed:.1f} m/s")
    print(f"  Max lateral velocity: {max_lateral_velocity:.2f} m/s")
    print(f"  Successful overtaking: {'✓' if max_lateral_deviation > 3.0 else '✗'}")

    x_final = solution[(1, "x_position")][-1]
    y_final = solution[(1, "y_position")][-1]
    position_error = np.sqrt((x_final - AGENT_END[0]) ** 2 + (y_final - AGENT_END[1]) ** 2)

    print("\nFinal Position:")
    print(f"  Reached: ({x_final:.2f}, {y_final:.2f}) m")
    print(f"  Target: ({AGENT_END[0]:.2f}, {AGENT_END[1]:.2f}) m")
    print(f"  Error: {position_error:.3f} m")

    solution.plot()

else:
    print(f"✗ Optimization failed: {solution.status['message']}")


# ============================================================================
# Export for Animation
# ============================================================================

__all__ = [
    "AGENT_END",
    "AGENT_START",
    "LANE_WIDTH",
    "LEFT_LANE_CENTER",
    "OBSTACLE_1_WAYPOINTS",
    "OBSTACLE_2_WAYPOINTS",
    "RIGHT_LANE_CENTER",
    "STREET_BOTTOM",
    "STREET_CENTER",
    "STREET_LEFT_BOUNDARY",
    "STREET_RIGHT_BOUNDARY",
    "STREET_TOP",
    "solution",
]
