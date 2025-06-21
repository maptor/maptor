import logging
from typing import TYPE_CHECKING, Any

import numpy as np

from .mtor_types import BenchmarkData, FloatArray, OptimalControlSolution, PhaseID, ProblemProtocol


if TYPE_CHECKING:
    pass

logger = logging.getLogger(__name__)


class Solution:
    """
    Optimal control solution with comprehensive data access and analysis capabilities.

    Provides unified interface for accessing optimization results, trajectories,
    solver diagnostics, mesh information, and adaptive refinement data. Supports
    both single-phase and multiphase problems with automatic data concatenation.

    **Data Access Patterns:**

    **Mission-wide access (concatenates all phases):**
    - `solution["variable_name"]` - Variable across all phases
    - `solution["time_states"]` - State time points across all phases
    - `solution["time_controls"]` - Control time points across all phases

    **Phase-specific access:**
    - `solution[(phase_id, "variable_name")]` - Variable in specific phase
    - `solution[(phase_id, "time_states")]` - State times in specific phase
    - `solution[(phase_id, "time_controls")]` - Control times in specific phase

    **Existence checking:**
    - `"variable_name" in solution` - Check mission-wide variable
    - `(phase_id, "variable") in solution` - Check phase-specific variable

    Examples:
        Basic solution workflow:

        >>> solution = mtor.solve_adaptive(problem)
        >>> if solution.status["success"]:
        ...     print(f"Objective: {solution.status['objective']:.6f}")
        ...     solution.plot()

        Mission-wide data access:

        >>> altitude_all = solution["altitude"]       # All phases concatenated
        >>> velocity_all = solution["velocity"]       # All phases concatenated
        >>> state_times_all = solution["time_states"] # All phase state times

        Phase-specific data access:

        >>> altitude_p1 = solution[(1, "altitude")]   # Phase 1 only
        >>> velocity_p2 = solution[(2, "velocity")]   # Phase 2 only
        >>> state_times_p1 = solution[(1, "time_states")]

        Data extraction patterns:

        >>> # Final/initial values
        >>> final_altitude = solution["altitude"][-1]
        >>> initial_velocity = solution["velocity"][0]
        >>> final_mass_p1 = solution[(1, "mass")][-1]
        >>>
        >>> # Extrema
        >>> max_altitude = max(solution["altitude"])
        >>> min_thrust_p2 = min(solution[(2, "thrust")])

        Variable existence checking:

        >>> if "altitude" in solution:
        ...     altitude_data = solution["altitude"]
        >>> if (2, "thrust") in solution:
        ...     thrust_p2 = solution[(2, "thrust")]

        Phase information access:

        >>> for phase_id, phase_data in solution.phases.items():
        ...     duration = phase_data["times"]["duration"]
        ...     state_names = phase_data["variables"]["state_names"]

        Solution validation:

        >>> status = solution.status
        >>> if status["success"]:
        ...     objective = status["objective"]
        ...     mission_time = status["total_mission_time"]
        ... else:
        ...     print(f"Failed: {status['message']}")
    """

    def __init__(
        self,
        raw_solution: OptimalControlSolution | None,
        problem: ProblemProtocol | None,
        auto_summary: bool = True,
    ) -> None:
        """
        Initialize solution wrapper from raw multiphase optimization results.

        Args:
            raw_solution: Raw optimization results from solver
            problem: Problem protocol instance
            auto_summary: Whether to automatically display comprehensive summary (default: True)
        """
        # Store raw data for internal use and direct CasADi access
        self._raw_solution = raw_solution
        self._problem = problem

        # Store raw CasADi objects for advanced users
        self.raw_solution = raw_solution.raw_solution if raw_solution else None
        self.opti = raw_solution.opti_object if raw_solution else None

        # Build variable name mappings for dictionary access
        if problem is not None:
            self._phase_state_names = {}
            self._phase_control_names = {}
            for phase_id in problem._get_phase_ids():
                self._phase_state_names[phase_id] = problem._get_phase_ordered_state_names(phase_id)
                self._phase_control_names[phase_id] = problem._get_phase_ordered_control_names(
                    phase_id
                )
        else:
            self._phase_state_names = {}
            self._phase_control_names = {}

        if auto_summary:
            self._show_comprehensive_summary()

    def _show_comprehensive_summary(self) -> None:
        try:
            from .summary import print_comprehensive_solution_summary

            print_comprehensive_solution_summary(self)
        except ImportError as e:
            logger.warning(f"Could not import comprehensive summary: {e}")
        except Exception as e:
            logger.warning(f"Error in comprehensive summary: {e}")

    @property
    def status(self) -> dict[str, Any]:
        """
        Complete solution status and optimization results.

        Provides comprehensive optimization outcome information including
        success status, objective value, and mission timing. Essential
        for solution validation and performance assessment.

        Returns:
            Dictionary containing complete status information:

            - **success** (bool): Optimization success status
            - **message** (str): Detailed solver status message
            - **objective** (float): Final objective function value
            - **total_mission_time** (float): Sum of all phase durations

        Examples:
            Success checking:

            >>> if solution.status["success"]:
            ...     print("Optimization successful")

            Objective extraction:

            >>> objective = solution.status["objective"]
            >>> mission_time = solution.status["total_mission_time"]

            Error handling:

            >>> status = solution.status
            >>> if not status["success"]:
            ...     print(f"Failed: {status['message']}")
            ...     print(f"Objective: {status['objective']}")  # May be NaN

            Status inspection:

            >>> print(f"Success: {solution.status['success']}")
            >>> print(f"Message: {solution.status['message']}")
            >>> print(f"Objective: {solution.status['objective']:.6e}")
            >>> print(f"Mission time: {solution.status['total_mission_time']:.3f}")
        """
        if self._raw_solution is None:
            return {
                "success": False,
                "message": "No solution available",
                "objective": float("nan"),
                "total_mission_time": float("nan"),
            }

        # Calculate total mission time
        if self._raw_solution.phase_initial_times and self._raw_solution.phase_terminal_times:
            earliest_start = min(self._raw_solution.phase_initial_times.values())
            latest_end = max(self._raw_solution.phase_terminal_times.values())
            total_time = latest_end - earliest_start
        else:
            total_time = float("nan")

        return {
            "success": self._raw_solution.success,
            "message": self._raw_solution.message,
            "objective": self._raw_solution.objective
            if self._raw_solution.objective is not None
            else float("nan"),
            "total_mission_time": total_time,
        }

    @property
    def phases(self) -> dict[PhaseID, dict[str, Any]]:
        """
        Comprehensive phase information and data organization.

        Provides detailed data for each phase including timing, variables,
        mesh configuration, and trajectory arrays. Essential for understanding
        multiphase structure and accessing phase-specific information.

        Returns:
            Dictionary mapping phase IDs to phase data:

            **Phase data structure:**

            - **times** (dict): Phase timing
                - initial (float): Phase start time
                - final (float): Phase end time
                - duration (float): Phase duration
            - **variables** (dict): Variable information
                - state_names (list): State variable names
                - control_names (list): Control variable names
                - num_states (int): Number of states
                - num_controls (int): Number of controls
            - **mesh** (dict): Mesh configuration
                - polynomial_degrees (list): Polynomial degree per interval
                - mesh_nodes (FloatArray): Mesh node locations
                - num_intervals (int): Total intervals
            - **time_arrays** (dict): Time coordinates
                - states (FloatArray): State time points
                - controls (FloatArray): Control time points
            - **integrals** (float | FloatArray | None): Integral values

        Examples:
            Phase iteration:

            >>> for phase_id, phase_data in solution.phases.items():
            ...     print(f"Phase {phase_id}")

            Timing information:

            >>> phase_1 = solution.phases[1]
            >>> duration = phase_1["times"]["duration"]
            >>> start_time = phase_1["times"]["initial"]
            >>> end_time = phase_1["times"]["final"]

            Variable information:

            >>> variables = solution.phases[1]["variables"]
            >>> state_names = variables["state_names"]     # ["x", "y", "vx", "vy"]
            >>> control_names = variables["control_names"] # ["thrust_x", "thrust_y"]
            >>> num_states = variables["num_states"]       # 4
            >>> num_controls = variables["num_controls"]   # 2

            Mesh information:

            >>> mesh = solution.phases[1]["mesh"]
            >>> degrees = mesh["polynomial_degrees"]       # [6, 8, 6]
            >>> intervals = mesh["num_intervals"]          # 3
            >>> nodes = mesh["mesh_nodes"]                 # [-1, -0.5, 0.5, 1]

            Time arrays:

            >>> time_arrays = solution.phases[1]["time_arrays"]
            >>> state_times = time_arrays["states"]        # State time coordinates
            >>> control_times = time_arrays["controls"]    # Control time coordinates

            Integral values:

            >>> integrals = solution.phases[1]["integrals"]
            >>> if isinstance(integrals, float):
            ...     single_integral = integrals             # Single integral
            >>> else:
            ...     multiple_integrals = integrals          # Array of integrals
        """
        if self._raw_solution is None:
            return {}

        phases_data = {}

        for phase_id in self._get_phase_ids():
            # Time information
            initial_time = self._raw_solution.phase_initial_times.get(phase_id, float("nan"))
            final_time = self._raw_solution.phase_terminal_times.get(phase_id, float("nan"))
            duration = (
                final_time - initial_time
                if not (np.isnan(initial_time) or np.isnan(final_time))
                else float("nan")
            )

            # Variable information
            state_names = self._phase_state_names.get(phase_id, [])
            control_names = self._phase_control_names.get(phase_id, [])

            # Mesh information
            polynomial_degrees = self._raw_solution.phase_mesh_intervals.get(phase_id, [])
            mesh_nodes = self._raw_solution.phase_mesh_nodes.get(
                phase_id, np.array([], dtype=np.float64)
            )

            # Time arrays
            time_states = self._raw_solution.phase_time_states.get(
                phase_id, np.array([], dtype=np.float64)
            )
            time_controls = self._raw_solution.phase_time_controls.get(
                phase_id, np.array([], dtype=np.float64)
            )

            # Integrals
            integrals = self._raw_solution.phase_integrals.get(phase_id, None)

            phases_data[phase_id] = {
                "times": {"initial": initial_time, "final": final_time, "duration": duration},
                "variables": {
                    "state_names": state_names.copy(),
                    "control_names": control_names.copy(),
                    "num_states": len(state_names),
                    "num_controls": len(control_names),
                },
                "mesh": {
                    "polynomial_degrees": polynomial_degrees.copy() if polynomial_degrees else [],
                    "mesh_nodes": mesh_nodes.copy()
                    if mesh_nodes.size > 0
                    else np.array([], dtype=np.float64),
                    "num_intervals": len(polynomial_degrees) if polynomial_degrees else 0,
                },
                "time_arrays": {"states": time_states.copy(), "controls": time_controls.copy()},
                "integrals": integrals,
            }

        return phases_data

    @property
    def parameters(self) -> dict[str, Any]:
        """
        Static parameter optimization results and information.

        Always returns valid parameter dictionary with empty structure
        if no parameters were defined

        Returns:
            Parameter information dictionary:

            - **values** (FloatArray): Optimized parameter values (empty array if no parameters)
            - **names** (list[str] | None): Parameter names if available
            - **count** (int): Number of static parameters (0 if no parameters)

        Examples:
            Parameter existence check:

            >>> if solution.parameters["count"] > 0:
            ...     print("Problem has static parameters")

            Parameter access:

            >>> params = solution.parameters
            >>> if params["count"] > 0:
            ...     values = params["values"]        # [500.0, 1500.0, 0.1]
            ...     count = params["count"]          # 3
            ...     names = params["names"]          # ["mass", "thrust", "drag"] or None

            Named parameter access:

            >>> params = solution.parameters
            >>> if params["names"] and params["count"] > 0:
            ...     for name, value in zip(params["names"], params["values"]):
            ...         print(f"{name}: {value:.6f}")

            Unnamed parameter access:

            >>> params = solution.parameters
            >>> for i in range(params["count"]):
            ...     value = params["values"][i]
            ...     print(f"Parameter {i}: {value:.6f}")

            No parameters case:

            >>> if solution.parameters["count"] == 0:
            ...     print("No static parameters in problem")
        """
        if self._raw_solution is None or self._raw_solution.static_parameters is None:
            return {
                "values": np.array([], dtype=np.float64),
                "names": None,
                "count": 0,
            }

        # Try to get parameter names if available
        param_names = None
        if self._problem is not None and hasattr(self._problem, "_static_parameters"):
            try:
                static_params = self._problem._static_parameters
                if hasattr(static_params, "parameter_names"):
                    param_names = static_params.parameter_names.copy()
            except (AttributeError, IndexError):
                pass

        return {
            "values": self._raw_solution.static_parameters.copy(),
            "names": param_names,
            "count": len(self._raw_solution.static_parameters),
        }

    def _extract_mission_benchmark_arrays(self) -> BenchmarkData:
        if self._raw_solution is None or self._raw_solution.adaptive_data is None:
            return {
                "mesh_iteration": [],
                "estimated_error": [],
                "collocation_points": [],
                "mesh_intervals": [],
                "polynomial_degrees": [],
                "refinement_strategy": [],
            }

        history = self._raw_solution.adaptive_data.iteration_history
        if not history:
            return {
                "mesh_iteration": [],
                "estimated_error": [],
                "collocation_points": [],
                "mesh_intervals": [],
                "polynomial_degrees": [],
                "refinement_strategy": [],
            }

        iterations = sorted(history.keys())

        # Single pass extraction
        mesh_iterations = iterations
        estimated_errors = [history[i].max_error_all_phases for i in iterations]
        collocation_points = [history[i].total_collocation_points for i in iterations]
        mesh_intervals = [sum(history[i].phase_mesh_intervals.values()) for i in iterations]

        polynomial_degrees = []
        refinement_strategy = []
        for i in iterations:
            data = history[i]
            combined_degrees = []
            for phase_degrees in data.phase_polynomial_degrees.values():
                combined_degrees.extend(phase_degrees)
            polynomial_degrees.append(combined_degrees)

            combined_strategy = {}
            for phase_strategy in data.refinement_strategy.values():
                combined_strategy.update(phase_strategy)
            refinement_strategy.append(combined_strategy)

        return {
            "mesh_iteration": mesh_iterations,
            "estimated_error": estimated_errors,
            "collocation_points": collocation_points,
            "mesh_intervals": mesh_intervals,
            "polynomial_degrees": polynomial_degrees,
            "refinement_strategy": refinement_strategy,
        }

    def _extract_phase_benchmark_arrays(self) -> dict[PhaseID, BenchmarkData]:
        if self._raw_solution is None or self._raw_solution.adaptive_data is None:
            return {}

        history = self._raw_solution.adaptive_data.iteration_history
        if not history:
            return {}

        iterations = sorted(history.keys())
        first_data = history[iterations[0]]
        available_phases = list(first_data.phase_error_estimates.keys())

        phase_benchmarks: dict[PhaseID, BenchmarkData] = {}

        # Single pass extraction per phase
        for phase_id in available_phases:
            mesh_iterations = iterations
            estimated_errors = []
            collocation_points = []
            mesh_intervals = []
            polynomial_degrees = []
            refinement_strategy = []

            for iteration in iterations:
                data = history[iteration]

                # Error estimate for this phase
                if phase_id in data.phase_error_estimates:
                    phase_errors = data.phase_error_estimates[phase_id]
                    max_error = max(phase_errors) if phase_errors else float("inf")
                    estimated_errors.append(max_error)
                else:
                    estimated_errors.append(float("inf"))

                # Phase metrics
                collocation_points.append(data.phase_collocation_points.get(phase_id, 0))
                mesh_intervals.append(data.phase_mesh_intervals.get(phase_id, 0))
                polynomial_degrees.append(data.phase_polynomial_degrees.get(phase_id, []).copy())
                refinement_strategy.append(data.refinement_strategy.get(phase_id, {}).copy())

            phase_benchmarks[phase_id] = {
                "mesh_iteration": mesh_iterations,
                "estimated_error": estimated_errors,
                "collocation_points": collocation_points,
                "mesh_intervals": mesh_intervals,
                "polynomial_degrees": polynomial_degrees,
                "refinement_strategy": refinement_strategy,
            }

        return phase_benchmarks

    @property
    def adaptive(self) -> dict[str, Any]:
        """
        Comprehensive adaptive algorithm performance data and benchmarking metrics.

        Provides complete adaptive mesh refinement data including convergence status,
        error estimates, refinement statistics, and iteration-by-iteration benchmarking
        arrays. Only available for adaptive solver solutions.

        Returns:
            Adaptive algorithm data dictionary with comprehensive benchmarking arrays:

            **Algorithm Status:**
            - **converged** (bool): Algorithm convergence status
            - **iterations** (int): Total refinement iterations performed
            - **target_tolerance** (float): Target error tolerance
            - **phase_converged** (dict): Per-phase convergence status
            - **final_errors** (dict): Final error estimates per phase per interval
            - **gamma_factors** (dict): Normalization factors per phase

            **Complete Benchmarking Data:**
            - **iteration_history** (dict): Raw per-iteration algorithm state
            - **benchmark** (dict): Processed mission-wide benchmark arrays
            - **phase_benchmarks** (dict): Per-phase benchmark arrays

            **Benchmark Array Structure** (both mission-wide and per-phase):
            - **mesh_iteration** (list[int]): Iteration numbers [0, 1, 2, ...]
            - **estimated_error** (list[float]): Error estimates [1e-2, 1e-3, 1e-5, ...]
            - **collocation_points** (list[int]): Total collocation points [50, 75, 100, ...]
            - **mesh_intervals** (list[int]): Total mesh intervals [10, 15, 20, ...]
            - **polynomial_degrees** (list[list[int]]): Polynomial degrees per interval
            - **refinement_strategy** (list[dict]): Refinement actions per iteration

        Raises:
            RuntimeError: If no adaptive data available. This typically means
                solve_fixed_mesh() was used instead of solve_adaptive().

        Examples:
            Safe adaptive data access:

            >>> try:
            ...     adaptive = solution.adaptive
            ...     converged = adaptive["converged"]
            ...     iterations = adaptive["iterations"]
            ... except RuntimeError:
            ...     print("Fixed mesh solution - no adaptive data available")

            Complete benchmark arrays:

            >>> adaptive = solution.adaptive  # May raise RuntimeError
            >>> benchmark = adaptive["benchmark"]
            >>> iterations = benchmark["mesh_iteration"]         # [0, 1, 2, 3]
            >>> errors = benchmark["estimated_error"]            # [1e-2, 1e-3, 1e-5, 1e-7]
            >>> points = benchmark["collocation_points"]         # [50, 75, 100, 150]

            Phase-specific benchmark data:

            >>> phase_benchmarks = solution.adaptive["phase_benchmarks"]
            >>> phase1_data = phase_benchmarks[1]
            >>> phase1_errors = phase1_data["estimated_error"]

            Built-in analysis methods:

            >>> try:
            ...     solution.print_benchmark_summary()
            ...     solution.plot_refinement_history(phase_id=1)
            ... except RuntimeError:
            ...     print("No adaptive data for analysis")
        """
        if self._raw_solution is None or self._raw_solution.adaptive_data is None:
            raise RuntimeError(
                "Adaptive data not available. This solution was created with solve_fixed_mesh(). "
                "Use solve_adaptive() to obtain adaptive mesh refinement data."
            )

        adaptive_data = self._raw_solution.adaptive_data
        result = {
            "converged": adaptive_data.converged,
            "iterations": adaptive_data.total_iterations,
            "target_tolerance": adaptive_data.target_tolerance,
            "phase_converged": adaptive_data.phase_converged,
            "final_errors": adaptive_data.final_phase_error_estimates,
            "gamma_factors": adaptive_data.phase_gamma_factors,
        }

        if hasattr(adaptive_data, "iteration_history") and adaptive_data.iteration_history:
            iteration_history: dict[int, dict[str, Any]] = {}
            for iteration, data in adaptive_data.iteration_history.items():
                iteration_history[iteration] = {
                    "iteration": data.iteration,
                    "phase_error_estimates": data.phase_error_estimates,
                    "phase_collocation_points": data.phase_collocation_points,
                    "phase_mesh_intervals": data.phase_mesh_intervals,
                    "phase_polynomial_degrees": data.phase_polynomial_degrees,
                    "phase_mesh_nodes": data.phase_mesh_nodes,
                    "refinement_strategy": data.refinement_strategy,
                    "total_collocation_points": data.total_collocation_points,
                    "max_error_all_phases": data.max_error_all_phases,
                    "convergence_status": data.convergence_status,
                }
            result["iteration_history"] = iteration_history
            result["benchmark"] = self._extract_mission_benchmark_arrays()
            result["phase_benchmarks"] = self._extract_phase_benchmark_arrays()

        return result

    def _get_phase_ids(self) -> list[PhaseID]:
        if self._raw_solution is None:
            return []
        return sorted(self._raw_solution.phase_initial_times.keys())

    def __getitem__(self, key: str | tuple[PhaseID, str]) -> FloatArray:
        if not self.status["success"]:
            raise RuntimeError(
                f"Cannot access variable '{key}': Solution failed with message: {self.status['message']}"
            )

        if isinstance(key, tuple):
            return self._get_by_tuple_key(key)
        elif isinstance(key, str):
            return self._get_by_string_key(key)
        else:
            raise KeyError(
                f"Invalid key type: {type(key)}. Use string or (phase_id, variable_name) tuple"
            )

    def _get_by_tuple_key(self, key: tuple[PhaseID, str]) -> FloatArray:
        if len(key) != 2:
            raise KeyError("Tuple key must have exactly 2 elements: (phase_id, variable_name)")

        if self._raw_solution is None:
            raise RuntimeError("Cannot access variable: No solution data available")

        phase_id, var_name = key

        if phase_id not in self._get_phase_ids():
            raise KeyError(f"Phase {phase_id} not found in solution")

        if var_name == "time_states":
            return self._raw_solution.phase_time_states.get(
                phase_id, np.array([], dtype=np.float64)
            )
        elif var_name == "time_controls":
            return self._raw_solution.phase_time_controls.get(
                phase_id, np.array([], dtype=np.float64)
            )

        if phase_id in self._phase_state_names and var_name in self._phase_state_names[phase_id]:
            var_index = self._phase_state_names[phase_id].index(var_name)
            if phase_id in self._raw_solution.phase_states and var_index < len(
                self._raw_solution.phase_states[phase_id]
            ):
                return self._raw_solution.phase_states[phase_id][var_index]

        if (
            phase_id in self._phase_control_names
            and var_name in self._phase_control_names[phase_id]
        ):
            var_index = self._phase_control_names[phase_id].index(var_name)
            if phase_id in self._raw_solution.phase_controls and var_index < len(
                self._raw_solution.phase_controls[phase_id]
            ):
                return self._raw_solution.phase_controls[phase_id][var_index]

        raise KeyError(f"Variable '{var_name}' not found in phase {phase_id}")

    def _get_by_string_key(self, key: str) -> FloatArray:
        matching_arrays = []

        for phase_id in self._get_phase_ids():
            try:
                phase_data = self[(phase_id, key)]
                matching_arrays.append(phase_data)
            except KeyError:
                continue

        if not matching_arrays:
            all_vars = []
            for phase_id in self._get_phase_ids():
                phase_vars = (
                    self._phase_state_names.get(phase_id, [])
                    + self._phase_control_names.get(phase_id, [])
                    + ["time_states", "time_controls"]
                )
                all_vars.extend([f"({phase_id}, '{var}')" for var in phase_vars])

            raise KeyError(f"Variable '{key}' not found in any phase. Available: {all_vars}")

        if len(matching_arrays) == 1:
            return matching_arrays[0]

        return np.concatenate(matching_arrays, dtype=np.float64)

    def __contains__(self, key: str | tuple[PhaseID, str]) -> bool:
        try:
            self[key]
            return True
        except KeyError:
            return False

    def plot(
        self,
        phase_id: PhaseID | None = None,
        *variable_names: str,
        figsize: tuple[float, float] = (12.0, 8.0),
        show_phase_boundaries: bool = True,
    ) -> None:
        """
        Plot solution trajectories with comprehensive customization options.

        Creates trajectory plots with automatic formatting, phase boundaries,
        and flexible variable selection. Supports both single-phase and
        multiphase visualization with professional styling.

        Args:
            phase_id: Phase selection:
                - None: Plot all phases (default)
                - int: Plot specific phase only

            variable_names: Variable selection:
                - Empty: Plot all variables
                - Specified: Plot only named variables

            figsize: Figure size tuple (width, height)

            show_phase_boundaries: Display vertical lines at phase transitions

        Examples:
            Basic plotting:

            >>> solution.plot()  # All variables, all phases

            Specific phase:

            >>> solution.plot(phase_id=1)  # Phase 1 only

            Selected variables:

            >>> solution.plot(phase_id=None, "altitude", "velocity", "thrust")

            Custom formatting:

            >>> solution.plot(
            ...     figsize=(16, 10),
            ...     show_phase_boundaries=True
            ... )

            Phase-specific variables:

            >>> solution.plot(1, "x_position", "y_position")  # Phase 1 positions

            No phase boundaries:

            >>> solution.plot(show_phase_boundaries=False)
        """
        from .plot import plot_multiphase_solution

        plot_multiphase_solution(self, phase_id, variable_names, figsize, show_phase_boundaries)

    def summary(self, comprehensive: bool = True) -> None:
        """
        Display solution summary with comprehensive details and diagnostics.

        Prints detailed overview including solver status, phase information,
        mesh details, and adaptive algorithm results. Essential for solution
        validation and performance analysis.

        Args:
            comprehensive: Summary detail level:
                - True: Full detailed summary (default)
                - False: Concise key information only

        Examples:
            Full summary:

            >>> solution.summary()  # Comprehensive details

            Concise summary:

            >>> solution.summary(comprehensive=False)  # Key information only

            Manual summary control:

            >>> # Solve without automatic summary
            >>> solution = mtor.solve_adaptive(problem, show_summary=False)
            >>> # Display summary when needed
            >>> solution.summary()

            Conditional summary:

            >>> if solution.status["success"]:
            ...     solution.summary()
            ... else:
            ...     solution.summary(comprehensive=False)  # Brief failure info
        """
        if comprehensive:
            try:
                from .summary import print_comprehensive_solution_summary

                print_comprehensive_solution_summary(self)
            except ImportError as e:
                logger.warning(f"Could not import comprehensive summary: {e}")
            except Exception as e:
                logger.warning(f"Error in comprehensive summary: {e}")
        else:
            # Simple summary
            print(f"Solution Status: {self.status['success']}")
            print(f"Objective: {self.status['objective']:.6e}")
            print(f"Total Mission Time: {self.status['total_mission_time']:.6f}")
            print(f"Phases: {len(self.phases)}")
            if self.adaptive:
                print(f"Adaptive: Converged in {self.adaptive['iterations']} iterations")

    def plot_refinement_history(
        self,
        phase_id: PhaseID,
        figsize: tuple[float, float] = (12, 6),
        transform_domain: tuple[float, float] | None = None,
    ) -> None:
        """
        Visualize adaptive mesh refinement history for research analysis.

        Creates mesh point distribution evolution plot showing both mesh interval
        boundaries (red circles) and interior collocation points (black dots).

        Args:
            phase_id: Phase to visualize
            figsize: Figure dimensions (width, height)
            transform_domain: Transform from [-1,1] to physical domain (min, max)
        """
        if not self.adaptive or "iteration_history" not in self.adaptive:
            raise ValueError("No adaptive iteration history available for plotting")

        history = self.adaptive["iteration_history"]
        if not history:
            raise ValueError("Adaptive iteration history is empty")

        first_iteration = next(iter(history.values()))
        if phase_id not in first_iteration["phase_mesh_nodes"]:
            available_phases = list(first_iteration["phase_mesh_nodes"].keys())
            raise ValueError(f"Phase {phase_id} not found. Available phases: {available_phases}")

        try:
            import matplotlib.pyplot as plt

            from maptor.radau import _compute_radau_collocation_components
        except ImportError as e:
            raise ImportError("matplotlib required for mesh refinement plotting") from e

        fig, ax = plt.subplots(figsize=figsize)

        for iteration in sorted(history.keys()):
            data = history[iteration]
            mesh_nodes = data["phase_mesh_nodes"][phase_id].copy()
            polynomial_degrees = data["phase_polynomial_degrees"][phase_id]

            # Transform domain if requested
            if transform_domain is not None:
                domain_min, domain_max = transform_domain
                mesh_nodes = domain_min + (mesh_nodes + 1) * (domain_max - domain_min) / 2

            y_position = iteration + 1

            # Plot mesh interval boundaries (red circles)
            ax.scatter(
                mesh_nodes,
                [y_position] * len(mesh_nodes),
                s=60,
                marker="o",
                facecolors="none",
                edgecolors="red",
                linewidth=2,
            )

            # Plot interior collocation points (black dots)
            collocation_points = []
            for interval_idx in range(len(polynomial_degrees)):
                degree = polynomial_degrees[interval_idx]
                if degree > 0:
                    radau_components = _compute_radau_collocation_components(degree)
                    radau_points = radau_components.collocation_nodes

                    # Transform to current interval
                    interval_start = mesh_nodes[interval_idx]
                    interval_end = mesh_nodes[interval_idx + 1]
                    interval_colloc_points = (
                        interval_start + (radau_points + 1) * (interval_end - interval_start) / 2
                    )
                    collocation_points.extend(interval_colloc_points)

            if collocation_points:
                ax.scatter(
                    collocation_points,
                    [y_position] * len(collocation_points),
                    s=25,
                    marker="o",
                    color="black",
                )

        domain_label = "Mesh Point Location"
        if transform_domain is not None:
            domain_label += f" [{transform_domain[0]}, {transform_domain[1]}]"

        ax.set_xlabel(domain_label)
        ax.set_ylabel("Mesh State")
        ax.set_title(f"MAPTOR Mesh Refinement History - Phase {phase_id}")
        ax.grid(True, alpha=0.3)
        ax.set_ylim(0.5, len(history) + 0.5)

        iterations = sorted(history.keys())
        y_positions = [iter_num + 1 for iter_num in iterations]
        y_labels = []
        for iter_num in iterations:
            if iter_num == 0:
                y_labels.append("Initial")
            else:
                y_labels.append(f"Iter {iter_num}")

        ax.set_yticks(y_positions)
        ax.set_yticklabels(y_labels)

        from matplotlib.lines import Line2D

        legend_elements = [
            Line2D(
                [0],
                [0],
                marker="o",
                color="w",
                markerfacecolor="none",
                markeredgecolor="red",
                markersize=8,
                linewidth=2,
                label="Mesh boundaries",
            ),
            Line2D(
                [0],
                [0],
                marker="o",
                color="black",
                markersize=6,
                linewidth=0,
                label="Collocation points",
            ),
        ]
        ax.legend(handles=legend_elements, loc="upper right")

        plt.tight_layout()
        plt.show()

    def print_benchmark_summary(self) -> None:
        """
        Display professional adaptive mesh refinement benchmark analysis.

        Provides comprehensive performance metrics, convergence analysis, refinement
        strategies, and research integrity verification suitable for academic
        comparison with established pseudospectral optimal control methods.

        Examples:
            Complete benchmark analysis:

            >>> solution = mtor.solve_adaptive(problem)
            >>> solution.print_benchmark_summary()

            Access raw benchmark data:

            >>> benchmark_data = solution.adaptive["benchmark"]
            >>> iterations = benchmark_data["mesh_iteration"]
            >>> errors = benchmark_data["estimated_error"]

            Phase-specific analysis:

            >>> phase_data = solution.adaptive["phase_benchmarks"][1]
            >>> solution.plot_refinement_history(phase_id=1)
        """
        from .summary_benchmark import print_comprehensive_benchmark_summary

        print_comprehensive_benchmark_summary(self)
