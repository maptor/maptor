from pathlib import Path

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import overtaking_maneuver
from matplotlib.patches import Polygon


# ============================================================================
# Colors
# ============================================================================

COLORS = {
    "primary_red": "#991b1b",
    "primary_red_dark": "#7f1d1d",
    "primary_red_light": "#f87171",
    "background_dark": "#2d2d2d",
    "text_light": "#e5e7eb",
    "agent_blue": "#3b82f6",
    "obstacle_green": "#10b981",
    "obstacle_orange": "#f59e0b",
    "road_markings": "#e5e7eb",
    "lane_guides": "#6b7280",
}


# ============================================================================
# Vehicle Visualization
# ============================================================================


def create_vehicle_rectangle(x, y, theta, length=3.0, width=1.5):
    corners_local = np.array(
        [
            [-length / 2, -width / 2],
            [length / 2, -width / 2],
            [length / 2, width / 2],
            [-length / 2, width / 2],
        ]
    )

    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    rotation_matrix = np.array([[cos_theta, -sin_theta], [sin_theta, cos_theta]])

    corners_global = corners_local @ rotation_matrix.T + np.array([x, y])
    return corners_global


# ============================================================================
# Obstacle Trajectory Generation
# ============================================================================


def create_obstacle_trajectories_numpy(time_array):
    waypoints_1 = overtaking_maneuver.OBSTACLE_1_WAYPOINTS
    waypoints_2 = overtaking_maneuver.OBSTACLE_2_WAYPOINTS

    obs_x_1 = np.zeros_like(time_array)
    obs_y_1 = np.zeros_like(time_array)
    obs_x_2 = np.zeros_like(time_array)
    obs_y_2 = np.zeros_like(time_array)

    for i, t in enumerate(time_array):
        # Obstacle 1 trajectory
        t_clamped_1 = np.clip(t, waypoints_1[0, 2], waypoints_1[-1, 2])
        if t_clamped_1 >= waypoints_1[-1, 2]:
            obs_x_1[i] = waypoints_1[-1, 0]
            obs_y_1[i] = waypoints_1[-1, 1]
        else:
            alpha_1 = (t_clamped_1 - waypoints_1[0, 2]) / (waypoints_1[-1, 2] - waypoints_1[0, 2])
            obs_x_1[i] = (1 - alpha_1) * waypoints_1[0, 0] + alpha_1 * waypoints_1[-1, 0]
            obs_y_1[i] = (1 - alpha_1) * waypoints_1[0, 1] + alpha_1 * waypoints_1[-1, 1]

        # Obstacle 2 trajectory
        t_clamped_2 = np.clip(t, waypoints_2[0, 2], waypoints_2[-1, 2])
        if t_clamped_2 >= waypoints_2[-1, 2]:
            obs_x_2[i] = waypoints_2[-1, 0]
            obs_y_2[i] = waypoints_2[-1, 1]
        else:
            alpha_2 = (t_clamped_2 - waypoints_2[0, 2]) / (waypoints_2[-1, 2] - waypoints_2[0, 2])
            obs_x_2[i] = (1 - alpha_2) * waypoints_2[0, 0] + alpha_2 * waypoints_2[-1, 0]
            obs_y_2[i] = (1 - alpha_2) * waypoints_2[0, 1] + alpha_2 * waypoints_2[-1, 1]

    return obs_x_1, obs_y_1, obs_x_2, obs_y_2


# ============================================================================
# Animation Setup
# ============================================================================


def animate_overtaking_maneuver(solution, save_filename="overtaking_maneuver.mp4"):
    if not solution.status["success"]:
        raise ValueError("Cannot animate a failed solution.")

    # Extract and clean data
    time_states = solution["time_states"]
    x_vehicle = solution["x_position"]
    y_vehicle = solution["y_position"]
    theta_vehicle = solution["heading"]

    # Remove duplicate time points
    unique_indices = np.unique(time_states, return_index=True)[1]
    time_sol = time_states[unique_indices]
    x_sol = x_vehicle[unique_indices]
    y_sol = y_vehicle[unique_indices]
    theta_sol = theta_vehicle[unique_indices]

    # Create animation grid using actual solution timing
    final_time = solution.status["total_mission_time"]
    fps = 30
    total_frames = int(final_time * fps)
    animation_time = np.linspace(0, final_time, total_frames)

    # Interpolate trajectories
    x_anim = np.interp(animation_time, time_sol, x_sol)
    y_anim = np.interp(animation_time, time_sol, y_sol)
    theta_anim = np.interp(animation_time, time_sol, theta_sol)

    # Create obstacle trajectories
    obs_x_1_anim, obs_y_1_anim, obs_x_2_anim, obs_y_2_anim = create_obstacle_trajectories_numpy(
        animation_time
    )

    return _setup_and_run_animation(
        x_anim,
        y_anim,
        theta_anim,
        obs_x_1_anim,
        obs_y_1_anim,
        obs_x_2_anim,
        obs_y_2_anim,
        fps,
        total_frames,
        save_filename,
    )


# ============================================================================
# Animation Execution
# ============================================================================


def _setup_and_run_animation(
    x_anim,
    y_anim,
    theta_anim,
    obs_x_1_anim,
    obs_y_1_anim,
    obs_x_2_anim,
    obs_y_2_anim,
    fps,
    total_frames,
    save_filename,
):
    plt.style.use("dark_background")
    fig, ax = plt.subplots(figsize=(12, 16), facecolor=COLORS["background_dark"])
    ax.set_facecolor(COLORS["background_dark"])

    ax.set_xlim(
        overtaking_maneuver.STREET_LEFT_BOUNDARY - 1,
        overtaking_maneuver.STREET_RIGHT_BOUNDARY + 1,
    )
    ax.set_ylim(overtaking_maneuver.STREET_BOTTOM - 2, overtaking_maneuver.STREET_TOP + 2)
    ax.set_aspect("equal")

    ax.grid(False)
    ax.set_xlabel("Lateral Position (m)", fontsize=12, color=COLORS["text_light"])
    ax.set_ylabel("Longitudinal Position (m)", fontsize=12, color=COLORS["text_light"])
    ax.tick_params(colors=COLORS["text_light"], labelbottom=False, labelleft=False)

    # Set spine colors
    for spine in ax.spines.values():
        spine.set_color(COLORS["primary_red"])
        spine.set_linewidth(2)

    # Draw street infrastructure
    center_line = overtaking_maneuver.STREET_CENTER

    # Street boundaries
    ax.axvline(
        x=overtaking_maneuver.STREET_LEFT_BOUNDARY,
        color=COLORS["primary_red"],
        linewidth=4,
    )
    ax.axvline(
        x=overtaking_maneuver.STREET_RIGHT_BOUNDARY, color=COLORS["primary_red"], linewidth=4
    )

    # Lane markings
    y_range = np.arange(overtaking_maneuver.STREET_BOTTOM, overtaking_maneuver.STREET_TOP, 6)
    for y_mark in y_range:
        ax.plot(
            [center_line, center_line],
            [y_mark, y_mark + 3],
            color=COLORS["road_markings"],
            linewidth=2,
            alpha=0.3,
        )

    # Start/end markers
    ax.scatter(
        *overtaking_maneuver.AGENT_START,
        c=COLORS["obstacle_green"],
        s=200,
        marker="s",
        zorder=10,
        edgecolor=COLORS["text_light"],
        linewidth=2,
    )
    ax.scatter(
        *overtaking_maneuver.AGENT_END,
        c=COLORS["agent_blue"],
        s=250,
        marker="*",
        zorder=10,
        edgecolor=COLORS["text_light"],
        linewidth=2,
    )

    # Dynamic trailing trajectories
    trail_length_seconds = 3.0
    trail_frames = int(trail_length_seconds * fps)

    # Agent trail
    (agent_trail,) = ax.plot([], [], color=COLORS["agent_blue"], alpha=1, linewidth=3, zorder=5)

    # Obstacle trails
    (obstacle_1_trail,) = ax.plot(
        [],
        [],
        color=COLORS["obstacle_green"],
        alpha=0.6,
        linewidth=2,
        linestyle="--",
        zorder=4,
    )
    (obstacle_2_trail,) = ax.plot(
        [],
        [],
        color=COLORS["obstacle_orange"],
        alpha=0.6,
        linewidth=2,
        linestyle="--",
        zorder=4,
    )

    # Initialize vehicles
    agent_vehicle = Polygon(
        [[0, 0], [0, 0], [0, 0], [0, 0]],
        facecolor=COLORS["agent_blue"],
        edgecolor=COLORS["text_light"],
        alpha=1,
        linewidth=2,
    )
    ax.add_patch(agent_vehicle)

    obstacle_1 = Polygon(
        [[0, 0], [0, 0], [0, 0], [0, 0]],
        facecolor=COLORS["obstacle_green"],
        edgecolor=COLORS["text_light"],
        alpha=1,
        linewidth=2,
    )
    ax.add_patch(obstacle_1)

    obstacle_2 = Polygon(
        [[0, 0], [0, 0], [0, 0], [0, 0]],
        facecolor=COLORS["obstacle_orange"],
        edgecolor=COLORS["text_light"],
        alpha=1,
        linewidth=2,
    )
    ax.add_patch(obstacle_2)

    def animate(frame):
        # Update agent vehicle
        agent_corners = create_vehicle_rectangle(
            x_anim[frame], y_anim[frame], theta_anim[frame], 4.0, 2.0
        )
        agent_vehicle.set_xy(agent_corners)

        # Update trails
        trail_start = max(0, frame - trail_frames)

        # Agent trail
        trail_x = x_anim[trail_start : frame + 1]
        trail_y = y_anim[trail_start : frame + 1]
        agent_trail.set_data(trail_x, trail_y)

        # Obstacle trails
        obs1_trail_x = obs_x_1_anim[trail_start : frame + 1]
        obs1_trail_y = obs_y_1_anim[trail_start : frame + 1]
        obstacle_1_trail.set_data(obs1_trail_x, obs1_trail_y)

        obs2_trail_x = obs_x_2_anim[trail_start : frame + 1]
        obs2_trail_y = obs_y_2_anim[trail_start : frame + 1]
        obstacle_2_trail.set_data(obs2_trail_x, obs2_trail_y)

        # Update obstacles
        obs1_corners = create_vehicle_rectangle(
            obs_x_1_anim[frame], obs_y_1_anim[frame], np.pi / 2, 4.0, 2.0
        )
        obstacle_1.set_xy(obs1_corners)

        obs2_corners = create_vehicle_rectangle(
            obs_x_2_anim[frame], obs_y_2_anim[frame], -np.pi / 2, 4.0, 2.0
        )
        obstacle_2.set_xy(obs2_corners)

        return (
            agent_vehicle,
            agent_trail,
            obstacle_1_trail,
            obstacle_2_trail,
            obstacle_1,
            obstacle_2,
        )

    # Create and save animation
    anim = animation.FuncAnimation(
        fig, animate, frames=total_frames, interval=1000 / fps, blit=True
    )

    plt.tight_layout()

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
    solution = overtaking_maneuver.solution

    if solution.status["success"]:
        print("Creating MAPTOR street animation...")

        script_dir = Path(__file__).parent
        output_file = script_dir / "overtaking_maneuver.mp4"

        anim = animate_overtaking_maneuver(solution, str(output_file))
        plt.show()
    else:
        print("Cannot animate: solution failed")
        print(f"Failure message: {solution.status['message']}")
