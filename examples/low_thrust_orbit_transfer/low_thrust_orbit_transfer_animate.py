from pathlib import Path

import low_thrust_orbit_transfer
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np


# ============================================================================
# Constants
# ============================================================================

MU = 1.407645794e16
RE = 20925662.73
P_SCALE = 1e7
L_SCALE = np.pi
T_SCALE = 1e4

COLORS = {
    "primary_red": "#991b1b",
    "background_dark": "#2d2d2d",
    "text_light": "#e5e7eb",
    "agent_blue": "#3b82f6",
    "obstacle_green": "#10b981",
    "lane_guides": "#6b7280",
}


# ============================================================================
# Core Functions
# ============================================================================


def modified_equinoctial_to_cartesian(p_scaled, f, g, h, k, L_scaled):
    p = p_scaled * P_SCALE
    L = L_scaled * L_SCALE
    q = 1.0 + f * np.cos(L) + g * np.sin(L)
    r = p / q
    alpha2 = h * h - k * k
    s2 = 1 + h * h + k * k

    r1 = r / s2 * (np.cos(L) + alpha2 * np.cos(L) + 2 * h * k * np.sin(L))
    r2 = r / s2 * (np.sin(L) - alpha2 * np.sin(L) + 2 * h * k * np.cos(L))
    r3 = 2 * r / s2 * (h * np.sin(L) - k * np.cos(L))

    return np.array([r1, r2, r3])


def generate_orbit_points(p_scaled, f, g, h, k, num_points=100):
    L_values = np.linspace(0, 2 * np.pi, num_points)
    orbit_points = []
    for L in L_values:
        L_scaled = L / L_SCALE
        pos = modified_equinoctial_to_cartesian(p_scaled, f, g, h, k, L_scaled)
        orbit_points.append(pos)
    return np.array(orbit_points)


def animate_low_thrust_orbit_transfer(solution, save_filename="orbit_transfer.mp4"):
    if not solution.status["success"]:
        raise ValueError("Cannot animate a failed solution.")

    # Extract solution data
    time_states = solution["time_states"]
    unique_indices = np.unique(time_states, return_index=True)[1]
    time_sol = time_states[unique_indices]

    solution_data = {
        "p_scaled": solution["p_scaled"][unique_indices],
        "f": solution["f"][unique_indices],
        "g": solution["g"][unique_indices],
        "h": solution["h"][unique_indices],
        "k": solution["k"][unique_indices],
        "L_scaled": solution["L_scaled"][unique_indices],
    }

    # Animation parameters (CORRECTED - higher resolution for smooth orbital motion)
    animation_duration_seconds = 4
    final_time = solution.status["total_mission_time"]
    fps = 30  # Standard frame rate for DaVinci Resolve compatibility
    total_frames = int(animation_duration_seconds * fps)  # 120 animation frames
    trajectory_resolution = 1000  # High resolution for smooth orbital paths
    animation_time = np.linspace(0, final_time, trajectory_resolution)

    # Interpolate trajectories onto high-resolution time grid
    p_smooth = np.interp(animation_time, time_sol, solution_data["p_scaled"])
    f_smooth = np.interp(animation_time, time_sol, solution_data["f"])
    g_smooth = np.interp(animation_time, time_sol, solution_data["g"])
    h_smooth = np.interp(animation_time, time_sol, solution_data["h"])
    k_smooth = np.interp(animation_time, time_sol, solution_data["k"])
    L_smooth = np.interp(animation_time, time_sol, solution_data["L_scaled"])

    trajectory_points = []
    for i in range(trajectory_resolution):  # Create 1000 smooth trajectory points
        pos = modified_equinoctial_to_cartesian(
            p_smooth[i], f_smooth[i], g_smooth[i], h_smooth[i], k_smooth[i], L_smooth[i]
        )
        trajectory_points.append(pos)
    trajectory_points = np.array(trajectory_points)

    # Generate orbits
    initial_orbit = generate_orbit_points(
        solution_data["p_scaled"][0],
        solution_data["f"][0],
        solution_data["g"][0],
        solution_data["h"][0],
        solution_data["k"][0],
    )
    final_orbit = generate_orbit_points(
        solution_data["p_scaled"][-1],
        solution_data["f"][-1],
        solution_data["g"][-1],
        solution_data["h"][-1],
        solution_data["k"][-1],
    )

    # Setup plot - MODIFIED: Larger scale by tighter bounds
    plt.style.use("dark_background")
    fig = plt.figure(figsize=(12, 12), facecolor=COLORS["background_dark"])
    ax = fig.add_subplot(111, projection="3d", facecolor=COLORS["background_dark"])

    max_radius = (
        max(
            np.max(np.linalg.norm(initial_orbit, axis=1)),
            np.max(np.linalg.norm(final_orbit, axis=1)),
        )
        * 0.6
    )

    ax.set_xlim([-max_radius, max_radius])
    ax.set_ylim([-max_radius, max_radius])
    ax.set_zlim([-max_radius, max_radius])

    # REMOVE GUI elements completely
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_zticks([])
    ax.grid(False)
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    ax.xaxis.pane.set_edgecolor("none")
    ax.yaxis.pane.set_edgecolor("none")
    ax.zaxis.pane.set_edgecolor("none")

    # Draw Earth
    u = np.linspace(0, 2 * np.pi, 30)
    v = np.linspace(0, np.pi, 30)
    earth_x = RE * np.outer(np.cos(u), np.sin(v))
    earth_y = RE * np.outer(np.sin(u), np.sin(v))
    earth_z = RE * np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(earth_x, earth_y, earth_z, color=COLORS["agent_blue"], alpha=0.8, zorder=1)
    ax.set_box_aspect([1, 1, 1])

    # Draw orbits
    ax.plot(
        initial_orbit[:, 0],
        initial_orbit[:, 1],
        initial_orbit[:, 2],
        color=COLORS["lane_guides"],
        alpha=0.4,
        linewidth=2,
        linestyle="--",
    )
    ax.plot(
        final_orbit[:, 0],
        final_orbit[:, 1],
        final_orbit[:, 2],
        color=COLORS["obstacle_green"],
        alpha=0.6,
        linewidth=3,
    )

    # Animated elements
    (trajectory_line,) = ax.plot(
        [], [], [], color=COLORS["primary_red"], linewidth=1, alpha=0.9, zorder=10
    )
    (spacecraft_point,) = ax.plot(
        [], [], [], "o", color=COLORS["primary_red"], markersize=8, zorder=10
    )

    def animate(frame):
        # Map animation frame to trajectory index for smooth motion
        trajectory_index = int(frame * (trajectory_resolution - 1) / (total_frames - 1))

        trail_points = trajectory_points[: trajectory_index + 1]
        if len(trail_points) > 1:
            trajectory_line.set_data_3d(trail_points[:, 0], trail_points[:, 1], trail_points[:, 2])

        current_pos = trajectory_points[trajectory_index]
        spacecraft_point.set_data_3d([current_pos[0]], [current_pos[1]], [current_pos[2]])

        return trajectory_line, spacecraft_point

    ax.view_init(elev=15, azim=45)

    anim = animation.FuncAnimation(
        fig, animate, frames=total_frames, interval=1000 / fps, blit=True
    )

    try:
        anim.save(save_filename, writer="ffmpeg", fps=fps, bitrate=3000)
        print(f"Animation saved to {Path(save_filename).resolve()}")
    except Exception as e:
        print(f"Could not save video file: {e}")

    plt.close(fig)  # Close figure to prevent GUI display
    return anim


if __name__ == "__main__":
    solution = low_thrust_orbit_transfer.solution
    if solution.status["success"]:
        script_dir = Path(__file__).parent
        output_file = script_dir / "orbit_transfer.mp4"
        anim = animate_low_thrust_orbit_transfer(solution, str(output_file))
        plt.show()
