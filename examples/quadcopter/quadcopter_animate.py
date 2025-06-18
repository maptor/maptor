from pathlib import Path

import matplotlib.animation as animation
import matplotlib.pyplot as plt
import numpy as np
import quadcopter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


# ============================================================================
# Colors
# ============================================================================

COLORS = {
    "primary_red": "#991b1b",
    "background_dark": "#2d2d2d",
    "text_light": "#e5e7eb",
    "agent_blue": "#3b82f6",
    "obstacle_green": "#10b981",
    "danger_red": "#dc2626",
    "danger_red_alpha": "#dc262640",
}


# ============================================================================
# Geometry Creation Functions
# ============================================================================


def _create_rotation_matrix(phi, theta, psi):
    R_phi = np.array([[1, 0, 0], [0, np.cos(phi), np.sin(phi)], [0, -np.sin(phi), np.cos(phi)]])

    R_theta = np.array(
        [[np.cos(theta), 0, -np.sin(theta)], [0, 1, 0], [np.sin(theta), 0, np.cos(theta)]]
    )

    R_psi = np.array([[np.cos(psi), np.sin(psi), 0], [-np.sin(psi), np.cos(psi), 0], [0, 0, 1]])

    return R_phi @ R_theta @ R_psi


def _create_detailed_quadcopter_geometry(x, y, z, phi, theta, psi):
    evtol_size = 2 * quadcopter.l_arm
    arm_length = evtol_size / 2
    rotor_radius = arm_length / 3
    body_radius = arm_length / 2
    body_height = arm_length / 3

    # Create octagonal body (eVTOL style)
    n_sides = 8
    angles = np.linspace(0, 2 * np.pi, n_sides, endpoint=False)
    body_points = []
    for angle in angles:
        body_points.append([body_radius * np.cos(angle), body_radius * np.sin(angle), 0])

    # Tapered body design
    taper_ratio = 0.8
    body_points_top = [
        [x[0] * taper_ratio, x[1] * taper_ratio, -body_height / 2] for x in body_points
    ]
    body_points_bottom = [
        [x[0] * taper_ratio, x[1] * taper_ratio, body_height / 2] for x in body_points
    ]

    # Arm dimensions
    arm_width = arm_length / 10
    arm_height = arm_width / 2

    all_vertices = []
    all_vertices.extend(body_points_top)
    all_vertices.extend(body_points_bottom)
    faces = []

    def create_arm_vertices(start_point, end_point, width, height):
        direction = np.array(end_point) - np.array(start_point)
        length = np.linalg.norm(direction)
        direction = direction / length

        up = np.array([0, 0, 1])
        right = np.cross(direction, up)
        right = right / np.linalg.norm(right)

        vertices = []
        # Bottom vertices
        vertices.append(start_point - right * width / 2 - up * height / 2)
        vertices.append(start_point + right * width / 2 - up * height / 2)
        vertices.append(end_point + right * (width / 2 * 0.7) - up * height / 2)
        vertices.append(end_point - right * (width / 2 * 0.7) - up * height / 2)

        # Top vertices
        vertices.append(start_point - right * width / 2 + up * height / 2)
        vertices.append(start_point + right * width / 2 + up * height / 2)
        vertices.append(end_point + right * (width / 2 * 0.7) + up * height / 2)
        vertices.append(end_point - right * (width / 2 * 0.7) + up * height / 2)

        return vertices

    # X-configuration arm positions (matching quadcopter.py motor layout)
    arm_positions = [
        (
            [0, 0, 0],
            [arm_length * np.cos(np.pi / 4), arm_length * np.sin(np.pi / 4), 0],
        ),
        (
            [0, 0, 0],
            [arm_length * np.cos(3 * np.pi / 4), arm_length * np.sin(3 * np.pi / 4), 0],
        ),
        (
            [0, 0, 0],
            [arm_length * np.cos(-np.pi / 4), arm_length * np.sin(-np.pi / 4), 0],
        ),
        (
            [0, 0, 0],
            [arm_length * np.cos(-3 * np.pi / 4), arm_length * np.sin(-3 * np.pi / 4), 0],
        ),
    ]

    # Create arms
    arm_vertex_offset = len(all_vertices)
    for i, (start, end) in enumerate(arm_positions):
        arm_vertices = create_arm_vertices(np.array(start), np.array(end), arm_width, arm_height)
        all_vertices.extend(arm_vertices)

        idx = arm_vertex_offset + i * 8

        arm_faces = [
            [idx, idx + 1, idx + 2, idx + 3],
            [idx + 4, idx + 5, idx + 6, idx + 7],
            [idx, idx + 1, idx + 5, idx + 4],
            [idx + 2, idx + 3, idx + 7, idx + 6],
            [idx, idx + 3, idx + 7, idx + 4],
            [idx + 1, idx + 2, idx + 6, idx + 5],
        ]
        faces.extend(arm_faces)

    # Create detailed rotors
    rotors = []
    n_rotor_points = 16
    rotor_height = arm_height / 2

    for _start, end in arm_positions:
        x_center, y_center = end[0], end[1]

        hub_radius = rotor_radius * 0.2
        for j in range(n_rotor_points):
            rotor_angle = j * 2 * np.pi / n_rotor_points
            rotors.extend(
                [
                    [
                        x_center + rotor_radius * np.cos(rotor_angle),
                        y_center + rotor_radius * np.sin(rotor_angle),
                        rotor_height,
                    ],
                    [
                        x_center + rotor_radius * np.cos(rotor_angle),
                        y_center + rotor_radius * np.sin(rotor_angle),
                        -rotor_height,
                    ],
                    [
                        x_center + hub_radius * np.cos(rotor_angle),
                        y_center + hub_radius * np.sin(rotor_angle),
                        rotor_height * 1.5,
                    ],
                    [
                        x_center + hub_radius * np.cos(rotor_angle),
                        y_center + hub_radius * np.sin(rotor_angle),
                        -rotor_height * 1.5,
                    ],
                ]
            )

    all_vertices.extend(rotors)

    # Body faces
    faces.append(list(range(n_sides, 2 * n_sides)))
    faces.append(list(range(n_sides)))

    # Body side faces
    for i in range(n_sides):
        faces.append([i, (i + 1) % n_sides, ((i + 1) % n_sides) + n_sides, i + n_sides])

    # Rotor faces
    rotor_vertex_offset = len(all_vertices) - len(rotors)
    points_per_rotor = n_rotor_points * 4

    for rotor_idx in range(4):
        current_rotor_idx = rotor_vertex_offset + rotor_idx * points_per_rotor

        for j in range(n_rotor_points):
            next_j = (j + 1) % n_rotor_points
            faces.append(
                [
                    current_rotor_idx + j * 2,
                    current_rotor_idx + next_j * 2,
                    current_rotor_idx + next_j * 2 + 1,
                    current_rotor_idx + j * 2 + 1,
                ]
            )
            hub_start = current_rotor_idx + n_rotor_points * 2
            faces.append(
                [
                    hub_start + j * 2,
                    hub_start + next_j * 2,
                    hub_start + next_j * 2 + 1,
                    hub_start + j * 2 + 1,
                ]
            )

    # Transform all vertices using rotation matrix
    all_vertices = np.array(all_vertices)
    R = _create_rotation_matrix(phi, theta, psi)
    transformed_points = (R @ all_vertices.T).T + np.array([x, y, z])

    transformed_faces = [[transformed_points[idx] for idx in face] for face in faces]

    # Create front direction indicator
    front_direction = np.array(
        [[0, 0, 0], [arm_length * np.cos(np.pi / 4), arm_length * np.sin(np.pi / 4), 0]]
    )
    transformed_front = (R @ front_direction.T).T + np.array([x, y, z])

    return transformed_faces, transformed_front


def _create_danger_zone_cylinder(
    center_x, center_y, radius, height_bottom, height_top, n_points=32
):
    theta = np.linspace(0, 2 * np.pi, n_points, endpoint=False)

    # Bottom circle
    bottom_circle = np.column_stack(
        [
            center_x + radius * np.cos(theta),
            center_y + radius * np.sin(theta),
            np.full(n_points, height_bottom),
        ]
    )

    # Top circle
    top_circle = np.column_stack(
        [
            center_x + radius * np.cos(theta),
            center_y + radius * np.sin(theta),
            np.full(n_points, height_top),
        ]
    )

    # Create cylindrical surface faces
    faces = []
    for i in range(n_points):
        next_i = (i + 1) % n_points
        face = [bottom_circle[i], bottom_circle[next_i], top_circle[next_i], top_circle[i]]
        faces.append(face)

    return faces


# ============================================================================
# Animation Function
# ============================================================================


def animate_quadcopter_flight(solution, save_filename="quadcopter_flight.mp4"):
    if not solution.status["success"]:
        raise ValueError("Cannot animate failed solution")

    # Extract and clean data
    time_states = solution["time_states"]
    unique_indices = np.unique(time_states, return_index=True)[1]
    time_sol = time_states[unique_indices]

    # Convert scaled solution data to physical (raw data points)
    X_sol = solution["X_scaled"][unique_indices] * quadcopter.POS_SCALE
    Y_sol = solution["Y_scaled"][unique_indices] * quadcopter.POS_SCALE
    Z_sol = solution["Z_scaled"][unique_indices] * quadcopter.POS_SCALE
    phi_sol = solution["phi_scaled"][unique_indices] * quadcopter.ANG_SCALE
    theta_sol = solution["theta_scaled"][unique_indices] * quadcopter.ANG_SCALE
    psi_sol = solution["psi_scaled"][unique_indices] * quadcopter.ANG_SCALE

    # Animation parameters (fixed viewing duration)
    animation_duration_seconds = 3
    final_time = solution.status["total_mission_time"]
    total_frames = 200
    fps = total_frames / animation_duration_seconds
    animation_time = np.linspace(0, final_time, total_frames)

    # Interpolate trajectories onto regular time grid for smooth animation
    X_phys = np.interp(animation_time, time_sol, X_sol)
    Y_phys = np.interp(animation_time, time_sol, Y_sol)
    Z_phys = np.interp(animation_time, time_sol, Z_sol)
    phi_phys = np.interp(animation_time, time_sol, phi_sol)
    theta_phys = np.interp(animation_time, time_sol, theta_sol)
    psi_phys = np.interp(animation_time, time_sol, psi_sol)

    # Setup minimal 3D plot
    plt.style.use("dark_background")
    fig = plt.figure(figsize=(12, 12), dpi=200, facecolor=COLORS["background_dark"])
    ax = fig.add_subplot(111, projection="3d", facecolor=COLORS["background_dark"])

    # Set bounds with padding for 10m x 10m visualization
    bound_min, bound_max = 0.0, 6.0
    ax.set_xlim([bound_min, bound_max])
    ax.set_ylim([bound_min, bound_max])
    ax.set_zlim([bound_min, bound_max])

    # Remove GUI elements completely
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

    # Start/end markers
    ax.scatter(X_phys[0], Y_phys[0], Z_phys[0], c=COLORS["obstacle_green"], s=200, marker="s")
    ax.scatter(X_phys[-1], Y_phys[-1], Z_phys[-1], c=COLORS["agent_blue"], s=300, marker="*")

    # Create and add danger zone visualization
    danger_zone_faces = _create_danger_zone_cylinder(
        quadcopter.DANGER_ZONE_CENTER_X,
        quadcopter.DANGER_ZONE_CENTER_Y,
        quadcopter.DANGER_ZONE_RADIUS,
        height_bottom=bound_min,
        height_top=bound_max,
        n_points=24,
    )

    danger_zone_mesh = Poly3DCollection(
        danger_zone_faces,
        facecolor=COLORS["danger_red_alpha"],
        alpha=0.1,
        edgecolor=COLORS["danger_red"],
        linewidth=1.5,
    )
    ax.add_collection3d(danger_zone_mesh)

    # Add danger zone boundary circle at ground level for clarity
    theta_circle = np.linspace(0, 2 * np.pi, 50)
    circle_x = quadcopter.DANGER_ZONE_CENTER_X + quadcopter.DANGER_ZONE_RADIUS * np.cos(
        theta_circle
    )
    circle_y = quadcopter.DANGER_ZONE_CENTER_Y + quadcopter.DANGER_ZONE_RADIUS * np.sin(
        theta_circle
    )
    circle_z = np.full_like(circle_x, bound_min)
    ax.plot(circle_x, circle_y, circle_z, color=COLORS["danger_red"], linewidth=3, alpha=0.1)

    # Animated detailed quadcopter
    quadcopter_mesh = Poly3DCollection(
        [], facecolor=COLORS["agent_blue"], alpha=0.9, edgecolor=COLORS["text_light"], linewidth=0.5
    )
    ax.add_collection3d(quadcopter_mesh)

    # Front direction indicator
    (front_line,) = ax.plot([], [], [], color="yellow", linewidth=3, alpha=0.9)

    # Progressive trail (builds up as quadcopter moves)
    (trail_line,) = ax.plot([], [], [], color=COLORS["agent_blue"], linewidth=3, alpha=0.9)

    def animate(frame):
        # Update detailed quadcopter geometry
        faces, front_nose_line = _create_detailed_quadcopter_geometry(
            X_phys[frame],
            Y_phys[frame],
            Z_phys[frame],
            phi_phys[frame],
            theta_phys[frame],
            psi_phys[frame],
        )
        quadcopter_mesh.set_verts(faces)
        front_line.set_data_3d(front_nose_line[:, 0], front_nose_line[:, 1], front_nose_line[:, 2])

        # Update progressive trail (from start to current frame)
        trail_x = X_phys[0 : frame + 1]
        trail_y = Y_phys[0 : frame + 1]
        trail_z = Z_phys[0 : frame + 1]
        trail_line.set_data_3d(trail_x, trail_y, trail_z)

        return quadcopter_mesh, front_line, trail_line

    # Custom view angle for optimal visualization
    ax.view_init(elev=22, azim=-140)
    ax.set_box_aspect([1, 1, 1])

    anim = animation.FuncAnimation(
        fig, animate, frames=total_frames, interval=1000 / fps, blit=True
    )

    try:
        anim.save(save_filename, writer="ffmpeg", fps=fps, bitrate=8000)
        print(f"Animation saved to {Path(save_filename).resolve()}")
    except Exception as e:
        print(f"Could not save video: {e}")

    plt.show()
    return anim


# ============================================================================
# Main Execution
# ============================================================================

if __name__ == "__main__":
    solution = quadcopter.solution
    if solution.status["success"]:
        script_dir = Path(__file__).parent
        output_file = script_dir / "quadcopter_flight.mp4"
        anim = animate_quadcopter_flight(solution, str(output_file))

        # Print danger zone avoidance verification (using original solution data)
        X_traj_phys = solution["X_scaled"] * quadcopter.POS_SCALE
        Y_traj_phys = solution["Y_scaled"] * quadcopter.POS_SCALE
        distances_to_danger = np.sqrt(
            (X_traj_phys - quadcopter.DANGER_ZONE_CENTER_X) ** 2
            + (Y_traj_phys - quadcopter.DANGER_ZONE_CENTER_Y) ** 2
        )
        min_distance = np.min(distances_to_danger)
        print(f"Minimum distance to danger zone: {min_distance:.3f} m")
        print(f"Required clearance: {quadcopter.DANGER_ZONE_RADIUS:.1f} m")
        print(f"Safety margin: {min_distance - quadcopter.DANGER_ZONE_RADIUS:.3f} m")

        plt.show()
    else:
        print("Cannot animate: solution failed")
        print(f"Failure message: {solution.status['message']}")
