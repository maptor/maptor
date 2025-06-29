from pathlib import Path

import manipulator_3dof
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
    "box_color": "#dc2626",
}


# ============================================================================
# 3D Geometry Creation Functions
# ============================================================================


def _create_manipulator_3d_geometry(
    q1, q2, q3, l1=manipulator_3dof.l1, l2=manipulator_3dof.l2, l3=manipulator_3dof.l3
):
    base_pos = np.array([0.0, 0.0, 0.0])

    joint1_pos = np.array([0.0, 0.0, l1])

    joint2_pos = joint1_pos + np.array(
        [l2 * np.cos(q2) * np.cos(q1), l2 * np.cos(q2) * np.sin(q1), l2 * np.sin(q2)]
    )

    end_effector_pos = joint2_pos + np.array(
        [l3 * np.cos(q2 + q3) * np.cos(q1), l3 * np.cos(q2 + q3) * np.sin(q1), l3 * np.sin(q2 + q3)]
    )

    return base_pos, joint1_pos, joint2_pos, end_effector_pos


def _create_box_wireframe(center_pos, box_size=0.12):
    half_size = box_size / 2

    vertices = (
        np.array(
            [
                [-half_size, -half_size, -half_size],
                [half_size, -half_size, -half_size],
                [half_size, half_size, -half_size],
                [-half_size, half_size, -half_size],
                [-half_size, -half_size, half_size],
                [half_size, -half_size, half_size],
                [half_size, half_size, half_size],
                [-half_size, half_size, half_size],
            ]
        )
        + center_pos
    )

    edges = [
        [0, 1],
        [1, 2],
        [2, 3],
        [3, 0],
        [4, 5],
        [5, 6],
        [6, 7],
        [7, 4],
        [0, 4],
        [1, 5],
        [2, 6],
        [3, 7],
    ]

    return vertices, edges


# ============================================================================
# Animation Function
# ============================================================================


def animate_manipulator_3dof(solution, save_filename="manipulator_3dof.mp4"):
    if not solution.status["success"]:
        raise ValueError("Cannot animate a failed solution.")

    time_states = solution["time_states"]
    q1_traj = solution["q1"]
    q2_traj = solution["q2"]
    q3_traj = solution["q3"]

    unique_indices = np.unique(time_states, return_index=True)[1]
    time_sol = time_states[unique_indices]
    q1_sol = q1_traj[unique_indices]
    q2_sol = q2_traj[unique_indices]
    q3_sol = q3_traj[unique_indices]

    final_time = solution.status["total_mission_time"]
    fps = 30
    total_frames = int(final_time * fps)
    animation_time = np.linspace(0, final_time, total_frames)

    q1_anim = np.interp(animation_time, time_sol, q1_sol)
    q2_anim = np.interp(animation_time, time_sol, q2_sol)
    q3_anim = np.interp(animation_time, time_sol, q3_sol)

    plt.style.use("dark_background")
    fig = plt.figure(figsize=(12, 12), facecolor=COLORS["background_dark"])

    ax_main = fig.add_subplot(111, projection="3d", facecolor=COLORS["background_dark"])

    workspace_limit = 0.6
    ax_main.set_xlim([-workspace_limit, workspace_limit])
    ax_main.set_ylim([-workspace_limit, workspace_limit])
    ax_main.set_zlim([0, 2 * workspace_limit])
    ax_main.set_xlabel("X (m)", color=COLORS["text_light"])
    ax_main.set_ylabel("Y (m)", color=COLORS["text_light"])
    ax_main.set_zlabel("Z (m)", color=COLORS["text_light"])
    ax_main.tick_params(colors=COLORS["text_light"])
    ax_main.view_init(elev=10, azim=108)

    (base_marker,) = ax_main.plot(
        [],
        [],
        [],
        "o",
        color=COLORS["primary_red"],
        markersize=12,
        markeredgecolor=COLORS["text_light"],
        markeredgewidth=2,
        zorder=10,
    )
    (joint1_marker,) = ax_main.plot(
        [],
        [],
        [],
        "o",
        color=COLORS["blue"],
        markersize=10,
        markeredgecolor=COLORS["text_light"],
        markeredgewidth=2,
        zorder=10,
    )
    (joint2_marker,) = ax_main.plot(
        [],
        [],
        [],
        "o",
        color=COLORS["green"],
        markersize=8,
        markeredgecolor=COLORS["text_light"],
        markeredgewidth=2,
        zorder=10,
    )

    (base_link_line,) = ax_main.plot([], [], [], color=COLORS["primary_red"], linewidth=8)
    (upper_arm_line,) = ax_main.plot([], [], [], color=COLORS["blue"], linewidth=6)
    (forearm_line,) = ax_main.plot([], [], [], color=COLORS["green"], linewidth=4)

    box_lines = []
    for _ in range(12):
        (line,) = ax_main.plot([], [], [], color=COLORS["box_color"], linewidth=3, alpha=0.9)
        box_lines.append(line)

    (end_effector_trail,) = ax_main.plot(
        [], [], [], color=COLORS["box_color"], linewidth=3, alpha=0.8
    )

    payload_text = ax_main.text2D(
        0.02,
        0.95,
        f"{manipulator_3dof.m_box:.0f}kg Payload",
        transform=ax_main.transAxes,
        fontsize=18,
        color=COLORS["box_color"],
        bbox={"boxstyle": "round,pad=0.5", "facecolor": COLORS["background_dark"], "alpha": 0.9},
        fontweight="bold",
    )

    time_text = ax_main.text2D(
        0.02,
        0.05,
        "",
        transform=ax_main.transAxes,
        fontsize=14,
        color=COLORS["text_light"],
        fontweight="bold",
    )

    def animate(frame):
        current_time = animation_time[frame]

        base_pos, joint1_pos, joint2_pos, end_effector_pos = _create_manipulator_3d_geometry(
            q1_anim[frame], q2_anim[frame], q3_anim[frame]
        )

        base_marker.set_data_3d([base_pos[0]], [base_pos[1]], [base_pos[2]])
        joint1_marker.set_data_3d([joint1_pos[0]], [joint1_pos[1]], [joint1_pos[2]])
        joint2_marker.set_data_3d([joint2_pos[0]], [joint2_pos[1]], [joint2_pos[2]])

        base_link_line.set_data_3d(
            [base_pos[0], joint1_pos[0]], [base_pos[1], joint1_pos[1]], [base_pos[2], joint1_pos[2]]
        )
        upper_arm_line.set_data_3d(
            [joint1_pos[0], joint2_pos[0]],
            [joint1_pos[1], joint2_pos[1]],
            [joint1_pos[2], joint2_pos[2]],
        )
        forearm_line.set_data_3d(
            [joint2_pos[0], end_effector_pos[0]],
            [joint2_pos[1], end_effector_pos[1]],
            [joint2_pos[2], end_effector_pos[2]],
        )

        vertices, edges = _create_box_wireframe(end_effector_pos)
        for i, edge in enumerate(edges):
            start_vertex = vertices[edge[0]]
            end_vertex = vertices[edge[1]]
            box_lines[i].set_data_3d(
                [start_vertex[0], end_vertex[0]],
                [start_vertex[1], end_vertex[1]],
                [start_vertex[2], end_vertex[2]],
            )

        trail_frames = min(frame + 1, int(2.0 * fps))
        trail_start = max(0, frame + 1 - trail_frames)
        trail_positions = [
            _create_manipulator_3d_geometry(q1_anim[i], q2_anim[i], q3_anim[i])[3]
            for i in range(trail_start, frame + 1)
        ]
        if trail_positions:
            trail_x = [pos[0] for pos in trail_positions]
            trail_y = [pos[1] for pos in trail_positions]
            trail_z = [pos[2] for pos in trail_positions]
            end_effector_trail.set_data_3d(trail_x, trail_y, trail_z)

        time_text.set_text(f"t = {current_time:.1f}s")

        return (
            base_marker,
            joint1_marker,
            joint2_marker,
            *box_lines,
            base_link_line,
            upper_arm_line,
            forearm_line,
            end_effector_trail,
            payload_text,
            time_text,
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
    solution = manipulator_3dof.solution

    if solution.status["success"]:
        print("Creating social media optimized 3DOF manipulator animation...")

        script_dir = Path(__file__).parent
        output_file = script_dir / "manipulator_3dof.mp4"

        anim = animate_manipulator_3dof(solution, str(output_file))

        plt.show()
    else:
        print("Cannot animate: solution failed")
        print(f"Failure message: {solution.status['message']}")
