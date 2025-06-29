from pathlib import Path

import manipulator_2dof
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle


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
    "payload_color": "#dc2626",
}


# ============================================================================
# 2D Geometry Creation Functions
# ============================================================================


def _create_manipulator_2d_geometry(q1, q2, l1=manipulator_2dof.l1, l2=manipulator_2dof.l2):
    base_pos = np.array([0.0, 0.0])

    joint1_pos = np.array([l1 * np.cos(q1), l1 * np.sin(q1)])

    end_effector_pos = joint1_pos + np.array([l2 * np.cos(q1 + q2), l2 * np.sin(q1 + q2)])

    return base_pos, joint1_pos, end_effector_pos


def _create_payload_square(center_pos, square_size=0.08):
    half_size = square_size / 2

    corners = np.array(
        [
            [center_pos[0] - half_size, center_pos[1] - half_size],
            [center_pos[0] + half_size, center_pos[1] - half_size],
            [center_pos[0] + half_size, center_pos[1] + half_size],
            [center_pos[0] - half_size, center_pos[1] + half_size],
        ]
    )

    return corners


def _create_workspace_boundary(l1=manipulator_2dof.l1, l2=manipulator_2dof.l2, num_points=100):
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


def animate_manipulator_2dof(solution, save_filename="manipulator_2dof.mp4"):
    if not solution.status["success"]:
        raise ValueError("Cannot animate a failed solution.")

    time_states = solution["time_states"]
    q1_traj = solution["q1"]
    q2_traj = solution["q2"]

    unique_indices = np.unique(time_states, return_index=True)[1]
    time_sol = time_states[unique_indices]
    q1_sol = q1_traj[unique_indices]
    q2_sol = q2_traj[unique_indices]

    final_time = solution.status["total_mission_time"]
    fps = 30
    total_frames = int(final_time * fps)
    animation_time = np.linspace(0, final_time, total_frames)

    q1_anim = np.interp(animation_time, time_sol, q1_sol)
    q2_anim = np.interp(animation_time, time_sol, q2_sol)

    plt.style.use("dark_background")
    fig, ax = plt.subplots(figsize=(12, 12), facecolor=COLORS["background_dark"])

    ax.set_facecolor(COLORS["background_dark"])
    workspace_limit = 1.0
    ax.set_xlim([-workspace_limit, workspace_limit])
    ax.set_ylim([-0.2, workspace_limit * 1.8])
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)", color=COLORS["text_light"], fontsize=14)
    ax.set_ylabel("Y (m)", color=COLORS["text_light"], fontsize=14)
    ax.tick_params(colors=COLORS["text_light"])

    # Remove spines for cleaner look
    for spine in ax.spines.values():
        spine.set_visible(False)

    # Add workspace boundaries
    outer_boundary, inner_boundary = _create_workspace_boundary()
    ax.plot(
        outer_boundary[0],
        outer_boundary[1],
        color=COLORS["grey"],
        linewidth=1,
        alpha=0.3,
        linestyle="--",
        label="Workspace",
    )
    ax.plot(
        inner_boundary[0],
        inner_boundary[1],
        color=COLORS["grey"],
        linewidth=1,
        alpha=0.3,
        linestyle="--",
    )

    # Initialize animated elements
    (base_marker,) = ax.plot(
        [],
        [],
        "o",
        color=COLORS["primary_red"],
        markersize=15,
        markeredgecolor=COLORS["text_light"],
        markeredgewidth=2,
        zorder=10,
        label="Base Joint",
    )
    (joint1_marker,) = ax.plot(
        [],
        [],
        "o",
        color=COLORS["blue"],
        markersize=12,
        markeredgecolor=COLORS["text_light"],
        markeredgewidth=2,
        zorder=10,
        label="Elbow Joint",
    )

    (link1_line,) = ax.plot([], [], color=COLORS["blue"], linewidth=10, solid_capstyle="round")
    (link2_line,) = ax.plot([], [], color=COLORS["green"], linewidth=8, solid_capstyle="round")

    # Payload square
    payload_square = Rectangle(
        (0, 0),
        0,
        0,
        facecolor=COLORS["payload_color"],
        edgecolor=COLORS["text_light"],
        linewidth=2,
        alpha=0.9,
        zorder=15,
    )
    ax.add_patch(payload_square)

    # End-effector trail
    (end_effector_trail,) = ax.plot(
        [], [], color=COLORS["payload_color"], linewidth=3, alpha=0.8, label="End-effector path"
    )

    # Professional text displays
    payload_text = ax.text(
        0.02,
        0.95,
        f"{manipulator_2dof.m_payload:.0f}kg Payload",
        transform=ax.transAxes,
        fontsize=18,
        color=COLORS["payload_color"],
        bbox={"boxstyle": "round,pad=0.5", "facecolor": COLORS["background_dark"], "alpha": 0.9},
        fontweight="bold",
    )

    time_text = ax.text(
        0.02,
        0.05,
        "",
        transform=ax.transAxes,
        fontsize=14,
        color=COLORS["text_light"],
        fontweight="bold",
    )

    # Performance metrics
    params = solution.parameters
    optimal_tau1 = params["values"][0]
    optimal_tau2 = params["values"][1]

    metrics_text = ax.text(
        0.98,
        0.95,
        f"Motor 1: {optimal_tau1:.1f} N⋅m\nMotor 2: {optimal_tau2:.1f} N⋅m",
        transform=ax.transAxes,
        fontsize=12,
        color=COLORS["text_light"],
        bbox={"boxstyle": "round,pad=0.3", "facecolor": COLORS["background_dark"], "alpha": 0.8},
        horizontalalignment="right",
        verticalalignment="top",
    )

    ax.legend(
        loc="upper left",
        bbox_to_anchor=(0.02, 0.85),
        facecolor=COLORS["background_dark"],
        edgecolor=COLORS["text_light"],
        labelcolor=COLORS["text_light"],
        fontsize=10,
    )

    def animate(frame):
        current_time = animation_time[frame]

        base_pos, joint1_pos, end_effector_pos = _create_manipulator_2d_geometry(
            q1_anim[frame], q2_anim[frame]
        )

        # Update joint markers
        base_marker.set_data([base_pos[0]], [base_pos[1]])
        joint1_marker.set_data([joint1_pos[0]], [joint1_pos[1]])

        # Update links
        link1_line.set_data([base_pos[0], joint1_pos[0]], [base_pos[1], joint1_pos[1]])
        link2_line.set_data(
            [joint1_pos[0], end_effector_pos[0]], [joint1_pos[1], end_effector_pos[1]]
        )

        # Update payload square
        payload_corners = _create_payload_square(end_effector_pos)
        payload_square.set_xy(payload_corners[0])
        payload_square.set_width(payload_corners[1, 0] - payload_corners[0, 0])
        payload_square.set_height(payload_corners[2, 1] - payload_corners[1, 1])

        # Update end-effector trail (2-second window)
        trail_frames = min(frame + 1, int(2.0 * fps))
        trail_start = max(0, frame + 1 - trail_frames)
        trail_positions = [
            _create_manipulator_2d_geometry(q1_anim[i], q2_anim[i])[2]
            for i in range(trail_start, frame + 1)
        ]
        if trail_positions:
            trail_x = [pos[0] for pos in trail_positions]
            trail_y = [pos[1] for pos in trail_positions]
            end_effector_trail.set_data(trail_x, trail_y)

        # Update time display
        time_text.set_text(f"t = {current_time:.1f}s")

        return (
            base_marker,
            joint1_marker,
            link1_line,
            link2_line,
            payload_square,
            end_effector_trail,
            payload_text,
            time_text,
            metrics_text,
        )

    anim = animation.FuncAnimation(
        fig, animate, frames=total_frames, interval=1000 / fps, blit=True
    )

    plt.tight_layout()

    try:
        anim.save(save_filename, writer="ffmpeg", fps=fps, bitrate=3000)
        print(f"Animation saved to {Path(save_filename).resolve()}")
    except Exception as e:
        print(f"Could not save video file ({e}). Displaying animation instead.")

    return anim


# ============================================================================
# Main Execution
# ============================================================================

if __name__ == "__main__":
    solution = manipulator_2dof.solution

    if solution.status["success"]:
        print("Creating professional 2DOF manipulator animation...")

        script_dir = Path(__file__).parent
        output_file = script_dir / "manipulator_2dof.mp4"

        anim = animate_manipulator_2dof(solution, str(output_file))

        plt.show()
    else:
        print("Cannot animate: solution failed")
        print(f"Failure message: {solution.status['message']}")
