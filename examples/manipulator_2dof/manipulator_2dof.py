import casadi as ca
import numpy as np

import maptor as mtor


# ============================================================================
# Physical Parameters
# ============================================================================

m1 = 2.0
m2 = 1.5
m_payload = 5.0

l1 = 0.4
l2 = 0.4

lc1 = 0.20
lc2 = 0.20

I1 = m1 * l1**2 / 12
I2 = m2 * l2**2 / 12

g = 9.81


# ============================================================================
# End-Effector Position Specification
# ============================================================================

x_ee_initial = 0.3
y_ee_initial = 0.1

x_ee_final = -0.6
y_ee_final = 0.1


# ============================================================================
# Forward/Inverse Kinematics
# ============================================================================


def calculate_inverse_kinematics(x_target, y_target, l1, l2):
    r_total = np.sqrt(x_target**2 + y_target**2)

    max_reach = l1 + l2
    min_reach = abs(l1 - l2)

    if r_total > max_reach:
        raise ValueError(f"Target unreachable: distance {r_total:.3f} > max reach {max_reach:.3f}")
    if r_total < min_reach:
        raise ValueError(f"Target too close: distance {r_total:.3f} < min reach {min_reach:.3f}")

    cos_q2 = (r_total**2 - l1**2 - l2**2) / (2 * l1 * l2)
    cos_q2 = np.clip(cos_q2, -1, 1)

    alpha = np.arctan2(y_target, x_target)

    # Try elbow up configuration first
    q2_up = np.arccos(cos_q2)
    beta_up = np.arctan2(l2 * np.sin(q2_up), l1 + l2 * np.cos(q2_up))
    q1_up = alpha - beta_up

    # Try elbow down configuration
    q2_down = -np.arccos(cos_q2)
    beta_down = np.arctan2(l2 * np.sin(q2_down), l1 + l2 * np.cos(q2_down))
    q1_down = alpha - beta_down

    # Select configuration that keeps q1 >= 0 (above ground)
    if q1_up >= 0:
        return q1_up, q2_up
    elif q1_down >= 0:
        return q1_down, q2_down
    else:
        raise ValueError(
            f"Target ({x_target:.3f}, {y_target:.3f}) requires base link below ground. Both solutions: q1_up={np.degrees(q1_up):.1f}°, q1_down={np.degrees(q1_down):.1f}°"
        )


def verify_forward_kinematics(q1, q2, l1, l2):
    x_ee = l1 * np.cos(q1) + l2 * np.cos(q1 + q2)
    y_ee = l1 * np.sin(q1) + l2 * np.sin(q1 + q2)
    return x_ee, y_ee


q1_initial, q2_initial = calculate_inverse_kinematics(x_ee_initial, y_ee_initial, l1, l2)
q1_final, q2_final = calculate_inverse_kinematics(x_ee_final, y_ee_final, l1, l2)

x_check, y_check = verify_forward_kinematics(q1_final, q2_final, l1, l2)
position_error = np.sqrt((x_check - x_ee_final) ** 2 + (y_check - y_ee_final) ** 2)

print("=== INVERSE KINEMATICS VERIFICATION ===")
print(f"Target position: ({x_ee_final:.3f}, {y_ee_final:.3f}) m")
print(f"Calculated joint angles: q1={np.degrees(q1_final):.1f}°, q2={np.degrees(q2_final):.1f}°")
print(f"Forward kinematics check: ({x_check:.3f}, {y_check:.3f}) m")
print(f"Position error: {position_error:.6f} m")
print()


# ============================================================================
# Problem Setup
# ============================================================================

problem = mtor.Problem("Enhanced 2DOF Manipulator")
phase = problem.set_phase(1)


# ============================================================================
# Design Parameters
# ============================================================================

max_tau1 = problem.parameter("joint_1_torque", boundary=(5.0, 100.0))
max_tau2 = problem.parameter("joint_2_torque", boundary=(5.0, 100.0))


# ============================================================================
# Variables
# ============================================================================

t = phase.time(initial=0.0)

q1 = phase.state("q1", initial=q1_initial, final=q1_final, boundary=(0, np.pi))
q2 = phase.state("q2", initial=q2_initial, final=q2_final, boundary=(-np.pi, np.pi))

q1_dot = phase.state("q1_dot", initial=0.0, final=0.0, boundary=(-2.0, 2.0))
q2_dot = phase.state("q2_dot", initial=0.0, final=0.0, boundary=(-2.0, 2.0))

tau1 = phase.control("tau1")
tau2 = phase.control("tau2")


# ============================================================================
# Dynamics
# ============================================================================

phase.dynamics(
    {
        q1: q1_dot,
        q2: q2_dot,
        q1_dot: (
            (I2 + lc2**2 * m2)
            * (
                -g * l1 * m2 * ca.cos(q1)
                - g * l1 * m_payload * ca.cos(q1)
                - g * l2 * m_payload * ca.cos(q1 + q2)
                - g * lc1 * m1 * ca.cos(q1)
                - g * lc2 * m2 * ca.cos(q1 + q2)
                + l1 * lc2 * m2 * (2 * q1_dot + q2_dot) * ca.sin(q2) * q2_dot
                + tau1
            )
            - (I2 + l1 * lc2 * m2 * ca.cos(q2) + lc2**2 * m2)
            * (
                -g * l2 * m_payload * ca.cos(q1 + q2)
                - g * lc2 * m2 * ca.cos(q1 + q2)
                - l1 * lc2 * m2 * ca.sin(q2) * q1_dot**2
                + tau2
            )
        )
        / (
            I1 * I2
            + I1 * lc2**2 * m2
            + I2 * l1**2 * m2
            + I2 * lc1**2 * m1
            + l1**2 * lc2**2 * m2**2 * ca.sin(q2) ** 2
            + lc1**2 * lc2**2 * m1 * m2
        ),
        q2_dot: (
            -(I2 + l1 * lc2 * m2 * ca.cos(q2) + lc2**2 * m2)
            * (
                -g * l1 * m2 * ca.cos(q1)
                - g * l1 * m_payload * ca.cos(q1)
                - g * l2 * m_payload * ca.cos(q1 + q2)
                - g * lc1 * m1 * ca.cos(q1)
                - g * lc2 * m2 * ca.cos(q1 + q2)
                + l1 * lc2 * m2 * (2 * q1_dot + q2_dot) * ca.sin(q2) * q2_dot
                + tau1
            )
            + (
                -g * l2 * m_payload * ca.cos(q1 + q2)
                - g * lc2 * m2 * ca.cos(q1 + q2)
                - l1 * lc2 * m2 * ca.sin(q2) * q1_dot**2
                + tau2
            )
            * (I1 + I2 + l1**2 * m2 + 2 * l1 * lc2 * m2 * ca.cos(q2) + lc1**2 * m1 + lc2**2 * m2)
        )
        / (
            I1 * I2
            + I1 * lc2**2 * m2
            + I2 * l1**2 * m2
            + I2 * lc1**2 * m1
            + l1**2 * lc2**2 * m2**2 * ca.sin(q2) ** 2
            + lc1**2 * lc2**2 * m1 * m2
        ),
    }
)


# ============================================================================
# Constraints
# ============================================================================

y_ee = l1 * ca.sin(q1) + l2 * ca.sin(q1 + q2)
phase.path_constraints(y_ee >= 0.05)

phase.path_constraints(
    tau1 >= -max_tau1,
    tau1 <= max_tau1,
    tau2 >= -max_tau2,
    tau2 <= max_tau2,
)


# ============================================================================
# Objective
# ============================================================================

actuator_cost = max_tau1 * 0.08 + max_tau2 * 0.05
energy = phase.add_integral(tau1**2 + tau2**2)
problem.minimize(t.final + 0.1 * actuator_cost + 0.01 * energy)


# ============================================================================
# Mesh Configuration and Initial Guess
# ============================================================================

num_interval = 10
degree = [4]
final_mesh = degree * num_interval
phase.mesh(final_mesh, np.linspace(-1.0, 1.0, num_interval + 1))

states_guess = []
controls_guess = []

for N in final_mesh:
    tau = np.linspace(-1, 1, N + 1)
    t_norm = (tau + 1) / 2

    q1_vals = q1_initial + (q1_final - q1_initial) * t_norm
    q2_vals = q2_initial + (q2_final - q2_initial) * t_norm

    q1_dot_vals = (q1_final - q1_initial) * np.sin(np.pi * t_norm) * 0.5
    q2_dot_vals = (q2_final - q2_initial) * np.sin(np.pi * t_norm) * 0.5

    states_guess.append(np.vstack([q1_vals, q2_vals, q1_dot_vals, q2_dot_vals]))

    tau1_vals = np.ones(N) * 2.0
    tau2_vals = np.ones(N) * 2.0
    controls_guess.append(np.vstack([tau1_vals, tau2_vals]))

phase.guess(
    states=states_guess,
    controls=controls_guess,
    terminal_time=4.0,
)


# ============================================================================
# Solve
# ============================================================================

solution = mtor.solve_adaptive(
    problem,
    error_tolerance=1e-3,
    max_iterations=15,
    min_polynomial_degree=3,
    max_polynomial_degree=8,
    nlp_options={
        "ipopt.print_level": 0,
        "ipopt.max_iter": 1000,
        "ipopt.tol": 1e-6,
        "ipopt.constr_viol_tol": 1e-4,
    },
)


# ============================================================================
# Results
# ============================================================================

if solution.status["success"]:
    params = solution.parameters
    optimal_tau1 = params["values"][0]
    optimal_tau2 = params["values"][1]

    tau1_max = max(np.abs(solution["tau1"]))
    tau2_max = max(np.abs(solution["tau2"]))

    utilization1 = (tau1_max / optimal_tau1) * 100
    utilization2 = (tau2_max / optimal_tau2) * 100

    actuator_cost = optimal_tau1 * 0.08 + optimal_tau2 * 0.05

    q1_solved = solution["q1"][-1]
    q2_solved = solution["q2"][-1]
    x_ee_achieved = l1 * np.cos(q1_solved) + l2 * np.cos(q1_solved + q2_solved)
    y_ee_achieved = l1 * np.sin(q1_solved) + l2 * np.sin(q1_solved + q2_solved)
    position_error = np.sqrt((x_ee_achieved - x_ee_final) ** 2 + (y_ee_achieved - y_ee_final) ** 2)

    print("=== OPTIMAL ACTUATOR DESIGN ===")
    print(f"Joint 1 motor torque: {optimal_tau1:.1f} N⋅m (utilization: {utilization1:.1f}%)")
    print(f"Joint 2 motor torque: {optimal_tau2:.1f} N⋅m (utilization: {utilization2:.1f}%)")
    print(
        f"Total actuator cost: ${actuator_cost:.2f}. (This is just a made-up cost, not the actual price)"
    )
    print()
    print("=== SYSTEM PERFORMANCE ===")
    print(f"Mission time: {solution.status['total_mission_time']:.2f} seconds")
    print(f"Position accuracy: {position_error * 1000:.2f} mm error")
    print("5kg payload successfully transported")

    solution.plot()

else:
    print(f"Optimization failed: {solution.status['message']}")

"""
OUTPUT
=== OPTIMAL ACTUATOR DESIGN ===
Joint 1 motor torque: 60.7 N⋅m (utilization: 100.0%)
Joint 2 motor torque: 26.9 N⋅m (utilization: 100.0%)
Total actuator cost: $6.21. (This is just a made-up cost, not the actual price)

=== SYSTEM PERFORMANCE ===
Mission time: 1.95 seconds
Position accuracy: 0.00 mm error
5kg payload successfully transported
"""
