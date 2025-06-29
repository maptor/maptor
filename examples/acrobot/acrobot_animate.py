from pathlib import Path

import acrobot
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np


# ============================================================================
# Colors
# ============================================================================

COLORS = {
    "primary_red": "#991b1b",
    "background_dark": "#2d2d2d",
    "text_light": "#e5e7eb",
    "blue": "#3b82f6",
    "green": "#10b981",
    "orange": "#f59e0b",
    "grey": "#6b7280",
}


# ============================================================================
# Geometry Creation Functions
# ============================================================================


def _create_acrobot_geometry(theta1, theta2, l1=1.0, l2=1.0):
    """Create acrobot geometry from joint angles."""
    # Shoulder joint (fixed at origin)
    shoulder_pos = np.array([0.0, 0.0])

    # Elbow joint (end of upper arm)
    # theta1 = 0: upper arm points down, theta1 = pi: upper arm points up
    elbow_pos = shoulder_pos + np.array([l1 * np.sin(theta1), -l1 * np.cos(theta1)])

    # End effector (end of forearm)
    # theta2 is relative to upper arm
    end_effector_pos = elbow_pos + np.array(
        [l2 * np.sin(theta1 + theta2), -l2 * np.cos(theta1 + theta2)]
    )

    return shoulder_pos, elbow_pos, end_effector_pos


def _create_workspace_boundary(l1=1.0, l2=1.0, num_points=100):
    """Create acrobot workspace boundary."""
    theta = np.linspace(0, 2 * np.pi, num_points)
    max_reach = l1 + l2
    min_reach = abs(l1 - l2)

    outer_boundary_x = max_reach * np.cos(theta)
    outer_boundary_y = max_reach * np.sin(theta)
    inner_boundary_x = min_reach * np.cos(theta)
    inner_boundary_y = min_reach * np.sin(theta)

    return (outer_boundary_x, outer_boundary_y), (inner_boundary_x, inner_boundary_y)


# ============================================================================
# Animation Function
# ============================================================================


def animate_acrobot_swingup(solution, save_filename="acrobot_swingup.mp4"):
    """
    Animate acrobot swing-up trajectory with real-time duration.

    Args:
        solution: MAPTOR solution object
        save_filename: Output video filename

    Returns:
        matplotlib animation object
    """
    if not solution.status["success"]:
        raise ValueError("Cannot animate a failed solution.")

    # Extract and clean data
    time_states = solution["time_states"]
    theta1_traj = solution["theta1"]
    theta2_traj = solution["theta2"]
    theta1_dot_traj = solution["theta1_dot"]
    theta2_dot_traj = solution["theta2_dot"]

    # Remove duplicate time points
    unique_indices = np.unique(time_states, return_index=True)[1]
    time_sol = time_states[unique_indices]
    theta1_sol = theta1_traj[unique_indices]
    theta2_sol = theta2_traj[unique_indices]
    theta1_dot_sol = theta1_dot_traj[unique_indices]
    theta2_dot_sol = theta2_dot_traj[unique_indices]

    # Real-time animation setup
    final_time = solution.status["total_mission_time"]
    fps = 30
    total_frames = int(final_time * fps)
    animation_time = np.linspace(0, final_time, total_frames)

    # Interpolate trajectories
    theta1_anim = np.interp(animation_time, time_sol, theta1_sol)
    theta2_anim = np.interp(animation_time, time_sol, theta2_sol)
    theta1_dot_anim = np.interp(animation_time, time_sol, theta1_dot_sol)
    theta2_dot_anim = np.interp(animation_time, time_sol, theta2_dot_sol)

    # Control data interpolation
    time_controls = solution["time_controls"]
    unique_control_indices = np.unique(time_controls, return_index=True)[1]
    time_control_sol = time_controls[unique_control_indices]
    tau_sol = solution["tau"][unique_control_indices]
    tau_anim = np.interp(animation_time, time_control_sol, tau_sol)

    # Setup figure and axes
    plt.style.use("dark_background")
    fig, (ax_main, ax_torque) = plt.subplots(
        1,
        2,
        figsize=(16, 8),
        facecolor=COLORS["background_dark"],
        gridspec_kw={"width_ratios": [2, 1]},
    )

    # Configure main plot
    ax_main.set_facecolor(COLORS["background_dark"])
    workspace_limit = 2.5
    ax_main.set_xlim(-workspace_limit, workspace_limit)
    ax_main.set_ylim(-workspace_limit, workspace_limit)
    ax_main.set_aspect("equal")
    ax_main.grid(True, alpha=0.3)
    ax_main.set_title("Acrobot Swing-Up Motion", color=COLORS["text_light"], fontsize=14)
    ax_main.set_xlabel("X Position (m)", color=COLORS["text_light"])
    ax_main.set_ylabel("Y Position (m)", color=COLORS["text_light"])
    ax_main.tick_params(colors=COLORS["text_light"])

    # Configure torque plot
    ax_torque.set_facecolor(COLORS["background_dark"])
    ax_torque.set_xlim(0, final_time)
    torque_min = min(tau_sol) - 2
    torque_max = max(tau_sol) + 2
    ax_torque.set_ylim(torque_min, torque_max)
    ax_torque.grid(True, alpha=0.3)
    ax_torque.set_title("Elbow Torque", color=COLORS["text_light"], fontsize=12)
    ax_torque.set_xlabel("Time (s)", color=COLORS["text_light"])
    ax_torque.set_ylabel("Torque (N⋅m)", color=COLORS["text_light"])
    ax_torque.tick_params(colors=COLORS["text_light"])

    # Plot torque trajectory
    ax_torque.plot(
        time_control_sol,
        tau_sol,
        color=COLORS["orange"],
        linewidth=2,
        alpha=0.7,
        label="τ (Elbow Torque)",
    )
    ax_torque.legend(
        facecolor=COLORS["background_dark"],
        edgecolor=COLORS["text_light"],
        labelcolor=COLORS["text_light"],
    )

    # Add workspace boundaries
    outer_boundary, inner_boundary = _create_workspace_boundary()
    ax_main.plot(
        outer_boundary[0],
        outer_boundary[1],
        color=COLORS["grey"],
        linewidth=1,
        alpha=0.3,
        linestyle="--",
    )
    ax_main.plot(
        inner_boundary[0],
        inner_boundary[1],
        color=COLORS["grey"],
        linewidth=1,
        alpha=0.3,
        linestyle="--",
    )

    # Add reference lines for initial and final positions
    ax_main.axhline(y=0, color=COLORS["text_light"], linewidth=1, alpha=0.3)
    ax_main.axvline(x=0, color=COLORS["text_light"], linewidth=1, alpha=0.3)

    # Initialize animated elements
    # Joint markers
    shoulder_marker = ax_main.scatter(
        [],
        [],
        s=200,
        c=COLORS["primary_red"],
        marker="o",
        zorder=10,
        edgecolor=COLORS["text_light"],
        linewidth=2,
        label="Shoulder",
    )
    elbow_marker = ax_main.scatter(
        [],
        [],
        s=150,
        c=COLORS["orange"],
        marker="o",
        zorder=10,
        edgecolor=COLORS["text_light"],
        linewidth=2,
        label="Elbow",
    )
    end_effector_marker = ax_main.scatter(
        [],
        [],
        s=120,
        c=COLORS["green"],
        marker="s",
        zorder=10,
        edgecolor=COLORS["text_light"],
        linewidth=2,
        label="End Effector",
    )

    # Links
    (upper_arm_line,) = ax_main.plot(
        [], [], color=COLORS["blue"], linewidth=8, solid_capstyle="round", alpha=0.9
    )
    (forearm_line,) = ax_main.plot(
        [], [], color=COLORS["orange"], linewidth=6, solid_capstyle="round", alpha=0.9
    )

    # End-effector trail
    (end_effector_trail,) = ax_main.plot(
        [], [], color=COLORS["green"], linewidth=2, alpha=0.8, label="End-effector path"
    )

    # Torque marker
    (tau_marker,) = ax_torque.plot([], [], "o", color=COLORS["orange"], markersize=8)

    # State information text
    state_text = ax_main.text(
        0.02,
        0.98,
        "",
        transform=ax_main.transAxes,
        fontsize=10,
        color=COLORS["text_light"],
        bbox={"boxstyle": "round,pad=0.3", "facecolor": COLORS["background_dark"], "alpha": 0.8},
        verticalalignment="top",
    )

    ax_main.legend(
        loc="upper right",
        facecolor=COLORS["background_dark"],
        edgecolor=COLORS["text_light"],
        labelcolor=COLORS["text_light"],
    )

    def animate(frame):
        current_time = animation_time[frame]

        # Get acrobot geometry
        shoulder_pos, elbow_pos, end_effector_pos = _create_acrobot_geometry(
            theta1_anim[frame], theta2_anim[frame]
        )

        # Update joint markers
        shoulder_marker.set_offsets([shoulder_pos])
        elbow_marker.set_offsets([elbow_pos])
        end_effector_marker.set_offsets([end_effector_pos])

        # Update links
        upper_arm_line.set_data([shoulder_pos[0], elbow_pos[0]], [shoulder_pos[1], elbow_pos[1]])
        forearm_line.set_data(
            [elbow_pos[0], end_effector_pos[0]], [elbow_pos[1], end_effector_pos[1]]
        )

        # Update end-effector trail (2-second window)
        trail_frames = min(frame + 1, int(2.0 * fps))
        trail_start = max(0, frame + 1 - trail_frames)
        trail_positions = [
            _create_acrobot_geometry(theta1_anim[i], theta2_anim[i])[2]
            for i in range(trail_start, frame + 1)
        ]
        if trail_positions:
            trail_x = [pos[0] for pos in trail_positions]
            trail_y = [pos[1] for pos in trail_positions]
            end_effector_trail.set_data(trail_x, trail_y)

        # Update torque marker
        tau_marker.set_data([current_time], [tau_anim[frame]])

        # Update state information
        state_info = (
            f"Time: {current_time:.2f}s / {final_time:.2f}s\n"
            f"Shoulder (θ₁): {theta1_anim[frame]:.3f} rad ({np.degrees(theta1_anim[frame]):+6.1f}°)\n"
            f"Elbow (θ₂): {theta2_anim[frame]:.3f} rad ({np.degrees(theta2_anim[frame]):+6.1f}°)\n"
            f"Angular velocities: {theta1_dot_anim[frame]:+6.3f}, {theta2_dot_anim[frame]:+6.3f} rad/s\n"
            f"End-effector: ({end_effector_pos[0]:+5.3f}, {end_effector_pos[1]:+5.3f}) m\n"
            f"Elbow torque: {tau_anim[frame]:+6.2f} N⋅m\n"
            f"Configuration: {'Hanging Down' if theta1_anim[frame] < np.pi / 2 else 'Swinging Up' if theta1_anim[frame] < 3 * np.pi / 4 else 'Nearly Inverted'}"
        )
        state_text.set_text(state_info)

        return (
            shoulder_marker,
            elbow_marker,
            end_effector_marker,
            upper_arm_line,
            forearm_line,
            end_effector_trail,
            tau_marker,
            state_text,
        )

    # Create animation
    anim = animation.FuncAnimation(
        fig, animate, frames=total_frames, interval=1000 / fps, blit=True
    )

    plt.tight_layout()

    # Save animation
    try:
        anim.save(save_filename, writer="ffmpeg", fps=fps, bitrate=2000)
        print(f"Animation saved to {Path(save_filename).resolve()}")
    except Exception as e:
        print(f"Could not save video file ({e}). Displaying animation instead.")

    return anim


# ============================================================================
# Main Execution
# ============================================================================

if __name__ == "__main__":
    solution = acrobot.solution

    if solution.status["success"]:
        print("Creating acrobot swing-up animation...")

        script_dir = Path(__file__).parent
        output_file = script_dir / "acrobot_swingup.mp4"

        anim = animate_acrobot_swingup(solution, str(output_file))
        plt.show()
    else:
        print("Cannot animate: solution failed")
        print(f"Failure message: {solution.status['message']}")
