import logging
from typing import TYPE_CHECKING

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.axes import Axes as MplAxes
from matplotlib.figure import Figure as MplFigure

from .mtor_types import FloatArray, PhaseID
from .utils.constants import (
    DEFAULT_FIGURE_SIZE,
    DEFAULT_GRID_ALPHA,
    DEFAULT_PHASE_BOUNDARY_ALPHA,
    DEFAULT_PHASE_BOUNDARY_LINEWIDTH,
    TIME_PRECISION,
)


if TYPE_CHECKING:
    from .solution import Solution


logger = logging.getLogger(__name__)


def plot_multiphase_solution(
    solution: "Solution",
    phase_id: PhaseID | None = None,
    variable_names: tuple[str, ...] = (),
    figsize: tuple[float, float] | None = None,
    show_phase_boundaries: bool = True,
) -> None:
    if figsize is None:
        figsize = DEFAULT_FIGURE_SIZE
    """
    Plot multiphase trajectories with interval coloring and phase boundaries.

    Args:
        solution: Solution object containing multiphase trajectory data
        phase_id: Specific phase to plot (None plots all phases)
        variable_names: Optional specific variable names to plot
        figsize: Figure size for each window
        show_phase_boundaries: Whether to show vertical lines at phase boundaries

    Examples:
        >>> plot_multiphase_solution(solution)  # Plot all phases
        >>> plot_multiphase_solution(solution, 1)  # Plot only phase 1
        >>> plot_multiphase_solution(solution, None, ("position", "velocity"))
    """
    if not solution.status["success"]:
        logger.warning("Cannot plot: Solution not successful")
        return

    if phase_id is not None:
        if phase_id not in solution.phases:
            raise ValueError(f"Phase {phase_id} not found in solution")
        _plot_single_phase(solution, phase_id, variable_names, figsize)
    else:
        if variable_names:
            _plot_multiphase_variables(solution, variable_names, figsize, show_phase_boundaries)
        else:
            _plot_multiphase_default(solution, figsize, show_phase_boundaries)


def _plot_single_phase(
    solution: "Solution",
    phase_id: PhaseID,
    variable_names: tuple[str, ...],
    figsize: tuple[float, float],
) -> None:
    phase_data = solution.phases[phase_id]
    state_names = phase_data["variables"]["state_names"]
    control_names = phase_data["variables"]["control_names"]

    if variable_names:
        _create_variable_plot(
            solution,
            f"Phase {phase_id} Variables",
            [(phase_id, var) for var in variable_names],
            figsize,
        )
    else:
        figures_created = []

        if state_names:
            fig = _create_variable_plot(
                solution,
                f"Phase {phase_id} States",
                [(phase_id, var) for var in state_names],
                figsize,
                show_immediately=False,
            )
            figures_created.append(fig)

        if control_names:
            fig = _create_variable_plot(
                solution,
                f"Phase {phase_id} Controls",
                [(phase_id, var) for var in control_names],
                figsize,
                show_immediately=False,
            )
            figures_created.append(fig)

        for fig in figures_created:
            plt.figure(fig.number)  # type: ignore[attr-defined]
            plt.show(block=False)

        if figures_created:
            plt.figure(figures_created[-1].number)  # type: ignore[attr-defined]
            plt.show()


def _plot_multiphase_variables(
    solution: "Solution",
    variable_names: tuple[str, ...],
    figsize: tuple[float, float],
    show_phase_boundaries: bool,
) -> None:
    phase_var_pairs = []
    for var_name in variable_names:
        for phase_id in solution.phases.keys():
            if (phase_id, var_name) in solution:
                phase_var_pairs.append((phase_id, var_name))

    if not phase_var_pairs:
        logger.warning("None of the requested variables found in any phase")
        return

    _create_multiphase_variable_plot(
        solution, "Multiphase Variables", phase_var_pairs, figsize, show_phase_boundaries
    )


def _plot_multiphase_default(
    solution: "Solution", figsize: tuple[float, float], show_phase_boundaries: bool
) -> None:
    figures_created = []

    all_state_vars = set()
    all_control_vars = set()

    for phase_data in solution.phases.values():
        all_state_vars.update(phase_data["variables"]["state_names"])
        all_control_vars.update(phase_data["variables"]["control_names"])

    if all_state_vars:
        state_pairs = []
        for var_name in sorted(all_state_vars):
            for phase_id in solution.phases.keys():
                if (phase_id, var_name) in solution:
                    state_pairs.append((phase_id, var_name))

        if state_pairs:
            fig = _create_multiphase_variable_plot(
                solution,
                "States",
                state_pairs,
                figsize,
                show_phase_boundaries,
                show_immediately=False,
            )
            figures_created.append(fig)

    if all_control_vars:
        control_pairs = []
        for var_name in sorted(all_control_vars):
            for phase_id in solution.phases.keys():
                if (phase_id, var_name) in solution:
                    control_pairs.append((phase_id, var_name))

        if control_pairs:
            fig = _create_multiphase_variable_plot(
                solution,
                "Controls",
                control_pairs,
                figsize,
                show_phase_boundaries,
                show_immediately=False,
            )
            figures_created.append(fig)

    for fig in figures_created:
        plt.figure(fig.number)  # type: ignore[attr-defined]
        plt.show(block=False)

    if figures_created:
        plt.figure(figures_created[-1].number)  # type: ignore[attr-defined]
        plt.show()


def _create_variable_plot(
    solution: "Solution",
    title: str,
    phase_var_pairs: list[tuple[PhaseID, str]],
    figsize: tuple[float, float],
    show_immediately: bool = True,
) -> MplFigure:
    if not phase_var_pairs:
        return plt.figure()

    var_groups: dict[str, list[PhaseID]] = {}
    for phase_id, var_name in phase_var_pairs:
        if var_name not in var_groups:
            var_groups[var_name] = []
        var_groups[var_name].append(phase_id)

    num_vars = len(var_groups)
    if num_vars == 0:
        return plt.figure()

    rows, cols = _determine_subplot_layout(num_vars)
    fig, axes = plt.subplots(rows, cols, figsize=figsize, sharex=False)
    fig.suptitle(title)

    if num_vars == 1:
        axes = [axes]
    elif isinstance(axes, np.ndarray):
        axes = axes.flatten()

    for i, (var_name, phase_ids) in enumerate(var_groups.items()):
        ax = axes[i]

        for phase_id in phase_ids:
            try:
                _plot_single_variable_with_intervals(solution, ax, phase_id, var_name)
            except KeyError:
                continue

        ax.set_ylabel(var_name)
        ax.set_xlabel("Time")
        ax.grid(True, alpha=DEFAULT_GRID_ALPHA)

    for i in range(num_vars, len(axes)):
        axes[i].set_visible(False)

    plt.tight_layout()

    if show_immediately:
        plt.show()

    return fig


def _create_multiphase_variable_plot(
    solution: "Solution",
    title: str,
    phase_var_pairs: list[tuple[PhaseID, str]],
    figsize: tuple[float, float],
    show_phase_boundaries: bool,
    show_immediately: bool = True,
) -> MplFigure:
    fig = _create_variable_plot(solution, title, phase_var_pairs, figsize, show_immediately=False)

    if show_phase_boundaries and len(solution.phases) > 1:
        phase_ids = sorted(solution.phases.keys())
        for ax in fig.get_axes():
            if ax.get_visible():
                for phase_id in phase_ids[:-1]:
                    final_time = solution.phases[phase_id]["times"]["final"]
                    ax.axvline(
                        final_time,
                        color="red",
                        linestyle="--",
                        alpha=DEFAULT_PHASE_BOUNDARY_ALPHA,
                        linewidth=DEFAULT_PHASE_BOUNDARY_LINEWIDTH,
                    )

    if show_immediately:
        plt.show()

    return fig


def _plot_single_variable_with_intervals(
    solution: "Solution", ax: MplAxes, phase_id: PhaseID, var_name: str
) -> None:
    phase_data = solution.phases[phase_id]

    if var_name in phase_data["variables"]["state_names"]:
        time_data = solution[(phase_id, "time_states")]
        var_data = solution[(phase_id, var_name)]
    elif var_name in phase_data["variables"]["control_names"]:
        time_data = solution[(phase_id, "time_controls")]
        var_data = solution[(phase_id, var_name)]
    else:
        return

    if len(time_data) == 0:
        return

    interval_colors = _get_phase_interval_colors(solution, phase_id)
    interval_boundaries = _get_phase_mesh_intervals(solution, phase_id)

    if interval_colors is None or len(interval_boundaries) == 0:
        _plot_state_linear_simple(ax, time_data, var_data)
    else:
        _plot_state_linear_intervals(
            ax, time_data, var_data, interval_boundaries, interval_colors, phase_id
        )


def _plot_state_linear_intervals(
    ax: MplAxes,
    time_array: FloatArray,
    values_array: FloatArray,
    intervals: list[tuple[float, float]],
    colors: np.ndarray,
    phase_id: PhaseID,
) -> None:
    if len(time_array) == 0:
        return

    for k, (t_start, t_end) in enumerate(intervals):
        mask = (time_array >= t_start - 1e-10) & (time_array <= t_end + TIME_PRECISION)
        if not np.any(mask):
            continue

        color = colors[k % len(colors)]

        ax.plot(
            time_array[mask],
            values_array[mask],
            color=color,
            marker=".",
            linestyle="-",
            linewidth=1.5,
            markersize=7,
        )


def _plot_state_linear_simple(
    ax: MplAxes, time_array: FloatArray, values_array: FloatArray
) -> None:
    ax.plot(time_array, values_array, ".-", linewidth=1.5, markersize=3)


def _get_phase_interval_colors(solution: "Solution", phase_id: PhaseID) -> np.ndarray | None:
    phase_data = solution.phases[phase_id]
    num_intervals = phase_data["mesh"]["num_intervals"]

    if num_intervals <= 1:
        return None

    colormap = plt.get_cmap("viridis")
    color_values = np.linspace(0, 1, num_intervals, dtype=np.float64)
    colors = colormap(color_values)
    return colors


def _get_phase_mesh_intervals(solution: "Solution", phase_id: PhaseID) -> list[tuple[float, float]]:
    phase_data = solution.phases[phase_id]

    mesh_nodes = phase_data["mesh"]["mesh_nodes"]
    initial_time = phase_data["times"]["initial"]
    terminal_time = phase_data["times"]["final"]

    if (
        mesh_nodes is None
        or len(mesh_nodes) == 0
        or np.isnan(initial_time)
        or np.isnan(terminal_time)
    ):
        return []

    # Convert normalized mesh nodes to physical time
    alpha = (terminal_time - initial_time) / 2.0
    alpha_0 = (terminal_time + initial_time) / 2.0
    mesh_phys = alpha * mesh_nodes + alpha_0

    return [(mesh_phys[i], mesh_phys[i + 1]) for i in range(len(mesh_phys) - 1)]


def _determine_subplot_layout(num_plots: int) -> tuple[int, int]:
    if num_plots <= 1:
        return (1, 1)

    rows = int(np.ceil(np.sqrt(num_plots)))
    cols = int(np.ceil(num_plots / rows))
    return (rows, cols)
