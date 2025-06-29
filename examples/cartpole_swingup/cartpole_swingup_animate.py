from pathlib import Path

import cartpole_swingup
import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle


# ============================================================================ #
# Colors
# ============================================================================ #

COLORS = {
    "primary_red": "#991b1b",
    "background_dark": "#2d2d2d",
    "text_light": "#e5e7eb",
    "blue": "#3b82f6",
    "green": "#10b981",
    "orange": "#f59e0b",
}


# ============================================================================ #
# Geometry Creation Functions
# ============================================================================ #


def _create_cart_rectangle(x_pos, cart_width=0.3, cart_height=0.2):
    """Create cart rectangle centered at x_pos."""
    corners = np.array(
        [
            [x_pos - cart_width / 2, -cart_height / 2],
            [x_pos + cart_width / 2, -cart_height / 2],
            [x_pos + cart_width / 2, cart_height / 2],
            [x_pos - cart_width / 2, cart_height / 2],
        ]
    )
    return corners


def _create_pole_line(x_pos, theta, pole_length=0.5, return_midpoint=False):
    """Create pole line from cart position at given angle.
    If return_midpoint=True, returns start, end, and midpoint (COM).
    """
    dx = -pole_length * np.sin(theta)
    dy = pole_length * np.cos(theta)

    start = np.array([x_pos, 0])
    end = start + np.array([dx, dy])

    if return_midpoint:
        midpoint = start + 0.5 * np.array([dx, dy])
        return start, end, midpoint
    else:
        return np.array([start, end])


# ============================================================================ #
# Animation Function
# ============================================================================ #


def animate_cartpole_swingup(solution, save_filename="cartpole_swingup.mp4"):
    """
    Animate cartpole swingup trajectory.

    Args:
        solution: MAPTOR solution object
        save_filename: Output video filename

    Returns:
        matplotlib animation object
    """
    if not solution.status["success"]:
        raise ValueError("Cannot animate a failed solution.")

    time_states = solution["time_states"]
    x_cart = solution["x"]
    theta_pole = solution["theta"]
    x_dot_cart = solution["x_dot"]
    force = solution["F"]

    unique_indices = np.unique(time_states, return_index=True)[1]
    time_sol = time_states[unique_indices]
    x_sol = x_cart[unique_indices]
    theta_sol = theta_pole[unique_indices]
    x_dot_sol = x_dot_cart[unique_indices]

    final_time = solution.status["total_mission_time"]
    fps = 30
    total_frames = int(final_time * fps)
    animation_time = np.linspace(0, final_time, total_frames)

    x_anim = np.interp(animation_time, time_sol, x_sol)
    theta_anim = np.interp(animation_time, time_sol, theta_sol)
    x_dot_anim = np.interp(animation_time, time_sol, x_dot_sol)

    time_controls = solution["time_controls"]
    unique_control_indices = np.unique(time_controls, return_index=True)[1]
    time_control_sol = time_controls[unique_control_indices]
    force_sol = force[unique_control_indices]
    force_anim = np.interp(animation_time, time_control_sol, force_sol)

    plt.style.use("dark_background")
    fig, (ax_main, ax_force) = plt.subplots(
        2,
        1,
        figsize=(12, 10),
        facecolor=COLORS["background_dark"],
        gridspec_kw={"height_ratios": [3, 1]},
    )

    ax_main.set_facecolor(COLORS["background_dark"])
    ax_main.set_xlim(-2, 2)
    ax_main.set_ylim(-1, 1)
    ax_main.set_aspect("equal")
    ax_main.grid(True, alpha=0.3)
    ax_main.set_title("Cartpole Swingup", color=COLORS["text_light"], fontsize=14)
    ax_main.set_xlabel("Position (m)", color=COLORS["text_light"])
    ax_main.set_ylabel("Height (m)", color=COLORS["text_light"])
    ax_main.tick_params(colors=COLORS["text_light"])

    ax_force.set_facecolor(COLORS["background_dark"])
    ax_force.set_xlim(0, final_time)
    ax_force.set_ylim(min(force_sol) - 1, max(force_sol) + 1)
    ax_force.grid(True, alpha=0.3)
    ax_force.set_title("Applied Force", color=COLORS["text_light"], fontsize=12)
    ax_force.set_xlabel("Time (s)", color=COLORS["text_light"])
    ax_force.set_ylabel("Force (N)", color=COLORS["text_light"])
    ax_force.tick_params(colors=COLORS["text_light"])

    ax_force.plot(
        time_control_sol,
        force_sol,
        color=COLORS["blue"],
        linewidth=2,
        alpha=0.7,
        label="Force trajectory",
    )
    ax_force.legend(
        facecolor=COLORS["background_dark"],
        edgecolor=COLORS["text_light"],
        labelcolor=COLORS["text_light"],
    )

    ax_main.axhline(y=-0.5, color=COLORS["text_light"], linewidth=2, alpha=0.5)

    cart_patch = Rectangle(
        (0, 0), 0, 0, facecolor=COLORS["blue"], edgecolor=COLORS["text_light"], linewidth=2
    )
    ax_main.add_patch(cart_patch)

    (pole_line,) = ax_main.plot([], [], color=COLORS["primary_red"], linewidth=4)
    (pole_com_dot,) = ax_main.plot([], [], "o", color=COLORS["orange"], markersize=8)

    (cart_trail_line,) = ax_main.plot(
        [], [], color=COLORS["green"], linewidth=2, alpha=0.6, label="Cart trajectory"
    )
    (pole_trail_line,) = ax_main.plot(
        [], [], color=COLORS["orange"], linewidth=2, alpha=0.8, label="Pole COM trajectory"
    )

    (force_marker,) = ax_force.plot([], [], "o", color=COLORS["primary_red"], markersize=8)

    state_text = ax_main.text(
        0.02,
        0.95,
        "",
        transform=ax_main.transAxes,
        fontsize=10,
        color=COLORS["text_light"],
        bbox={"boxstyle": "round,pad=0.3", "facecolor": COLORS["background_dark"], "alpha": 0.8},
    )

    ax_main.legend(
        loc="upper right",
        facecolor=COLORS["background_dark"],
        edgecolor=COLORS["text_light"],
        labelcolor=COLORS["text_light"],
    )

    def animate(frame):
        current_time = animation_time[frame]

        cart_corners = _create_cart_rectangle(x_anim[frame])
        cart_patch.set_xy(cart_corners[0])
        cart_patch.set_width(cart_corners[1, 0] - cart_corners[0, 0])
        cart_patch.set_height(cart_corners[2, 1] - cart_corners[1, 1])

        start, end, midpoint = _create_pole_line(
            x_anim[frame], theta_anim[frame], return_midpoint=True
        )
        pole_line.set_data([start[0], end[0]], [start[1], end[1]])
        pole_com_dot.set_data([midpoint[0]], [midpoint[1]])

        cart_trail_x = x_anim[: frame + 1]
        cart_trail_y = np.zeros_like(cart_trail_x)
        cart_trail_line.set_data(cart_trail_x, cart_trail_y)

        pole_trail_x = x_anim[: frame + 1] - 0.25 * np.sin(theta_anim[: frame + 1])
        pole_trail_y = 0.25 * np.cos(theta_anim[: frame + 1])
        pole_trail_line.set_data(pole_trail_x, pole_trail_y)

        force_marker.set_data([current_time], [force_anim[frame]])

        state_info = (
            f"Time: {current_time:.2f}s\n"
            f"Cart position: {x_anim[frame]:.3f} m\n"
            f"Pole angle: {theta_anim[frame]:.3f} rad ({np.degrees(theta_anim[frame]):.1f}°)\n"
            f"Cart velocity: {x_dot_anim[frame]:.3f} m/s\n"
            f"Applied force: {force_anim[frame]:.2f} N"
        )
        state_text.set_text(state_info)

        return (
            cart_patch,
            pole_line,
            pole_com_dot,
            cart_trail_line,
            pole_trail_line,
            force_marker,
            state_text,
        )

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


# ============================================================================ #
# Main Execution
# ============================================================================ #

if __name__ == "__main__":
    solution = cartpole_swingup.solution

    if solution.status["success"]:
        print("Creating cartpole swingup animation...")

        script_dir = Path(__file__).parent
        output_file = script_dir / "cartpole_swingup.mp4"

        anim = animate_cartpole_swingup(solution, str(output_file))

        plt.show()
    else:
        print("Cannot animate: solution failed")
        print(f"Failure message: {solution.status['message']}")
