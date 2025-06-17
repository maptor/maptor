from pathlib import Path

# Import the solution by running the main problem
import dynamic_obstacle_avoidance
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Polygon


def create_obstacle_trajectory_numpy(time_array):
    waypoints = dynamic_obstacle_avoidance.OBSTACLE_WAYPOINTS

    obs_x = np.zeros_like(time_array)
    obs_y = np.zeros_like(time_array)

    for i, t in enumerate(time_array):
        # Clamp time to waypoint bounds
        t_clamped = np.clip(t, waypoints[0, 2], waypoints[-1, 2])

        # Find segment and interpolate
        if t_clamped >= waypoints[-1, 2]:
            obs_x[i] = waypoints[-1, 0]
            obs_y[i] = waypoints[-1, 1]
        else:
            for j in range(len(waypoints) - 1):
                t1, t2 = waypoints[j, 2], waypoints[j + 1, 2]
                if t1 <= t_clamped < t2:
                    alpha = (t_clamped - t1) / (t2 - t1)
                    obs_x[i] = (1 - alpha) * waypoints[j, 0] + alpha * waypoints[j + 1, 0]
                    obs_y[i] = (1 - alpha) * waypoints[j, 1] + alpha * waypoints[j + 1, 1]
                    break

    return obs_x, obs_y


def create_vehicle_triangle(x, y, theta, size=1.0):
    front = np.array([x + size * np.cos(theta), y + size * np.sin(theta)])
    left_rear = np.array(
        [
            x - 0.5 * size * np.cos(theta) - 0.5 * size * np.sin(theta),
            y - 0.5 * size * np.sin(theta) + 0.5 * size * np.cos(theta),
        ]
    )
    right_rear = np.array(
        [
            x - 0.5 * size * np.cos(theta) + 0.5 * size * np.sin(theta),
            y - 0.5 * size * np.sin(theta) - 0.5 * size * np.cos(theta),
        ]
    )

    return np.array([front, left_rear, right_rear])


def animate_dynamic_obstacle_avoidance(solution, save_filename="dynamic_obstacle_avoidance.mp4"):
    if not solution.status["success"]:
        raise ValueError("Cannot animate a failed solution.")

    # Extract and clean data - updated for dynamic bicycle model
    time_states = solution["time_states"]
    x_vehicle = solution["x_position"]
    y_vehicle = solution["y_position"]
    theta_vehicle = solution["heading"]
    v_x_vehicle = solution["longitudinal_velocity"]
    v_y_vehicle = solution["lateral_velocity"]
    r_vehicle = solution["yaw_rate"]

    unique_indices = np.unique(time_states, return_index=True)[1]
    time_sol = time_states[unique_indices]
    x_sol = x_vehicle[unique_indices]
    y_sol = y_vehicle[unique_indices]
    theta_sol = theta_vehicle[unique_indices]
    v_x_sol = v_x_vehicle[unique_indices]
    v_y_sol = v_y_vehicle[unique_indices]
    r_sol = r_vehicle[unique_indices]

    final_time = time_sol[-1]
    fps = 30
    total_frames = int(final_time * fps)
    animation_time = np.linspace(0, final_time, total_frames)

    x_anim = np.interp(animation_time, time_sol, x_sol)
    y_anim = np.interp(animation_time, time_sol, y_sol)
    theta_anim = np.interp(animation_time, time_sol, theta_sol)
    v_x_anim = np.interp(animation_time, time_sol, v_x_sol)
    v_y_anim = np.interp(animation_time, time_sol, v_y_sol)
    r_anim = np.interp(animation_time, time_sol, r_sol)

    obs_x_anim, obs_y_anim = create_obstacle_trajectory_numpy(animation_time)

    fig = plt.figure(figsize=(16, 10))

    ax_main = plt.subplot(2, 2, (1, 3))
    ax_main.set_xlim(-5, 25)
    ax_main.set_ylim(-5, 25)
    ax_main.set_aspect("equal")
    ax_main.grid(True, alpha=0.3)
    ax_main.set_title("Dynamic Bicycle Model - Obstacle Avoidance")
    ax_main.set_xlabel("X Position (m)")
    ax_main.set_ylabel("Y Position (m)")

    ax_vel = plt.subplot(2, 2, 2)
    ax_vel.set_xlim(0, final_time)
    ax_vel.set_ylim(min(v_x_sol.min(), v_y_sol.min()) - 2, max(v_x_sol.max(), v_y_sol.max()) + 2)
    ax_vel.grid(True, alpha=0.3)
    ax_vel.set_title("Velocity Components")
    ax_vel.set_xlabel("Time (s)")
    ax_vel.set_ylabel("Velocity (m/s)")

    ax_yaw = plt.subplot(2, 2, 4)
    ax_yaw.set_xlim(0, final_time)
    ax_yaw.set_ylim(r_sol.min() - 0.5, r_sol.max() + 0.5)
    ax_yaw.grid(True, alpha=0.3)
    ax_yaw.set_title("Yaw Rate")
    ax_yaw.set_xlabel("Time (s)")
    ax_yaw.set_ylabel("Yaw Rate (rad/s)")

    ax_main.scatter(0, 0, c="g", s=100, marker="s", label="Start", zorder=5)
    ax_main.scatter(20, 20, c="b", s=150, marker="*", label="Goal", zorder=5)
    ax_main.plot(x_sol, y_sol, "b-", alpha=0.3, label="Vehicle Path", linewidth=2)
    ax_main.plot(
        *create_obstacle_trajectory_numpy(time_sol),
        "r-",
        alpha=0.3,
        label="Obstacle Path",
        linewidth=2,
    )

    # Plot velocity trajectories
    ax_vel.plot(time_sol, v_x_sol, "b-", label="Longitudinal (v_x)", linewidth=2)
    ax_vel.plot(time_sol, v_y_sol, "r-", label="Lateral (v_y)", linewidth=2)
    ax_vel.legend()

    # Plot yaw rate trajectory
    ax_yaw.plot(time_sol, r_sol, "g-", label="Yaw Rate", linewidth=2)
    ax_yaw.legend()

    # Define radii from the problem
    vehicle_radius = 1.5
    obstacle_radius = 2.5

    # Initialize animated artists for main plot
    vehicle_triangle = Polygon([[0, 0], [0, 0], [0, 0]], facecolor="blue", alpha=0.8)
    ax_main.add_patch(vehicle_triangle)

    safety_circle = Circle(
        (0, 0),
        vehicle_radius,
        fill=False,
        edgecolor="blue",
        linestyle="--",
        alpha=0.6,
    )
    ax_main.add_patch(safety_circle)

    obstacle_circle = Circle((0, 0), obstacle_radius, facecolor="red", alpha=0.7)
    ax_main.add_patch(obstacle_circle)

    # Time indicators
    time_text = ax_main.text(0.02, 0.95, "", transform=ax_main.transAxes, fontsize=12)

    # Vehicle state text
    state_text = ax_main.text(
        0.02,
        0.85,
        "",
        transform=ax_main.transAxes,
        fontsize=10,
        bbox={"boxstyle": "round,pad=0.3", "facecolor": "white", "alpha": 0.8},
    )

    # Time markers on velocity plots
    (time_marker_vel,) = ax_vel.plot([], [], "ko", markersize=8)
    (time_marker_yaw,) = ax_yaw.plot([], [], "ko", markersize=8)

    ax_main.legend(loc="upper right")

    def animate(frame):
        current_time = animation_time[frame]

        triangle_verts = create_vehicle_triangle(
            x_anim[frame], y_anim[frame], theta_anim[frame], size=1.8
        )
        vehicle_triangle.set_xy(triangle_verts)

        safety_circle.center = (x_anim[frame], y_anim[frame])

        obstacle_circle.center = (obs_x_anim[frame], obs_y_anim[frame])

        time_text.set_text(f"Time: {current_time:.2f}s")

        v_total = np.sqrt(v_x_anim[frame] ** 2 + v_y_anim[frame] ** 2)
        slip_angle = np.arctan2(v_y_anim[frame], max(abs(v_x_anim[frame]), 0.1))

        state_info = (
            f"Speed: {v_total:.1f} m/s\n"
            f"v_x: {v_x_anim[frame]:.1f} m/s\n"
            f"v_y: {v_y_anim[frame]:.1f} m/s\n"
            f"Slip: {slip_angle * 180 / np.pi:.1f}°\n"
            f"Yaw rate: {r_anim[frame]:.2f} rad/s"
        )
        state_text.set_text(state_info)

        time_marker_vel.set_data([current_time], [np.interp(current_time, time_sol, v_x_sol)])
        time_marker_yaw.set_data([current_time], [np.interp(current_time, time_sol, r_sol)])

        return (
            vehicle_triangle,
            safety_circle,
            obstacle_circle,
            time_text,
            state_text,
            time_marker_vel,
            time_marker_yaw,
        )

    anim = animation.FuncAnimation(
        fig,
        animate,
        frames=total_frames,
        interval=1000 / fps,
        blit=True,
    )

    plt.tight_layout()

    try:
        anim.save(save_filename, writer="ffmpeg", fps=fps)
        print(f"Animation saved to {Path(save_filename).resolve()}")
    except Exception as e:
        print(f"Could not save video file ({e}). Displaying animation instead.")

    return anim


if __name__ == "__main__":
    solution = dynamic_obstacle_avoidance.solution

    if solution.status["success"]:
        print("Creating enhanced animation for dynamic bicycle model...")

        # Save mp4 in the same directory as this script
        script_dir = Path(__file__).parent
        output_file = script_dir / "dynamic_obstacle_avoidance.mp4"

        anim = animate_dynamic_obstacle_avoidance(solution, str(output_file))

        # Print dynamic behavior summary
        v_x_traj = solution["longitudinal_velocity"]
        v_y_traj = solution["lateral_velocity"]
        r_traj = solution["yaw_rate"]

        print("\nDynamic Behavior Summary:")
        print(f"  Longitudinal velocity range: [{v_x_traj.min():.2f}, {v_x_traj.max():.2f}] m/s")
        print(f"  Max lateral velocity: {abs(v_y_traj).max():.2f} m/s")
        print(f"  Max yaw rate: {abs(r_traj).max():.2f} rad/s")
        print(f"  Reverse motion: {'Yes' if v_x_traj.min() < 0 else 'No'}")
        print(f"  Significant lateral motion: {'Yes' if abs(v_y_traj).max() > 1.0 else 'No'}")

        # Calculate slip angles for analysis
        slip_angles = []
        for i in range(len(v_x_traj)):
            if abs(v_x_traj[i]) > 0.1:
                slip_angle = np.arctan2(v_y_traj[i], v_x_traj[i]) * 180 / np.pi
                slip_angles.append(abs(slip_angle))

        if slip_angles:
            print(f"  Max slip angle: {max(slip_angles):.1f}°")
            print(f"  Drifting behavior: {'Yes' if max(slip_angles) > 10 else 'No'}")

        plt.show()
    else:
        print("Cannot animate: solution failed")
        print(f"Failure message: {solution.status['message']}")
