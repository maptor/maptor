Working with Solution Data
==========================

This comprehensive tutorial covers all of MAPTOR's solution data extraction capabilities. After completing this guide, you'll understand how to access any optimization result using MAPTOR's unified solution interface.

**Prerequisites:** Complete the :doc:`../quickstart` guide first to understand the basic problem-solving workflow.

**Scope:** This tutorial covers the complete solution access API, from basic trajectory extraction to comprehensive adaptive algorithm diagnostics.

Solution Access Framework
--------------------------

MAPTOR provides a **unified solution interface** for accessing optimization results across all problem types. You systematically extract trajectories, metadata, and diagnostics using consistent access patterns that work for both single-phase and multiphase problems.

.. code-block:: python

    import maptor as mtor

    # Solve any optimal control problem
    solution = mtor.solve_adaptive(problem)

    # Access results through unified interface
    objective = solution.status["objective"]
    trajectory = solution["position"]
    final_value = trajectory[-1]

A **solution** contains all optimization results with automatic data organization, trajectory concatenation, and comprehensive metadata for analysis and visualization.

Status Validation
------------------

Every solution provides complete optimization outcome information through the status property:

**Basic Status Access:**

.. code-block:: python

    # Essential validation
    success = solution.status["success"]
    objective = solution.status["objective"]
    message = solution.status["message"]

**Complete Status Information:**

.. code-block:: python

    # All status components
    status = solution.status
    success = status["success"]           # bool: Optimization success
    objective = status["objective"]       # float: Final objective value
    total_time = status["total_mission_time"]  # float: Complete mission duration
    message = status["message"]           # str: Detailed solver message

**Status Validation Patterns:**

.. code-block:: python

    # Success checking
    if solution.status["success"]:
        print("Optimization succeeded")

    # Objective extraction
    optimal_cost = solution.status["objective"]

    # Mission timing
    duration = solution.status["total_mission_time"]

    # Solver diagnostics
    solver_info = solution.status["message"]

The status property provides immediate access to optimization outcomes and is essential for determining whether to proceed with data extraction.

Mission-Wide Data Access
-------------------------

Use string keys to automatically access complete mission trajectories with automatic phase concatenation:

**Basic Trajectory Access:**

.. code-block:: python

    # Complete mission trajectories
    time_states = solution["time_states"]       # All state time points
    time_controls = solution["time_controls"]   # All control time points
    position = solution["position"]             # Complete position trajectory
    velocity = solution["velocity"]             # Complete velocity trajectory
    thrust = solution["thrust"]                 # Complete control trajectory

**Time Coordinate Access:**

.. code-block:: python

    # State and control time arrays
    state_times = solution["time_states"]
    control_times = solution["time_controls"]

    # Time span analysis
    mission_start = state_times[0]
    mission_end = state_times[-1]
    total_duration = mission_end - mission_start

**Variable Trajectory Access:**

.. code-block:: python

    # State trajectories
    altitude = solution["altitude"]
    mass = solution["mass"]
    angle = solution["angle"]

    # Control trajectories
    throttle = solution["throttle"]
    steering = solution["steering"]
    power = solution["power"]

**Final Value Extraction:**

.. code-block:: python

    # Mission endpoints
    final_position = solution["position"][-1]
    final_velocity = solution["velocity"][-1]
    initial_mass = solution["mass"][0]
    final_mass = solution["mass"][-1]

String key access automatically concatenates data from all phases containing the specified variable, providing seamless mission-wide analysis.

Phase-Specific Data Access
---------------------------

Use tuple keys for granular control over individual phase data:

**Single Phase Access:**

.. code-block:: python

    # Phase-specific trajectories
    phase1_position = solution[(1, "position")]
    phase1_velocity = solution[(1, "velocity")]
    phase1_thrust = solution[(1, "thrust")]

    # Phase-specific time coordinates
    phase1_state_times = solution[(1, "time_states")]
    phase1_control_times = solution[(1, "time_controls")]

**Multi-Phase Access:**

.. code-block:: python

    # Access each phase individually
    ascent_altitude = solution[(1, "altitude")]
    coast_altitude = solution[(2, "altitude")]
    descent_altitude = solution[(3, "altitude")]

    # Phase-specific controls
    ascent_thrust = solution[(1, "thrust")]
    coast_thrust = solution[(2, "thrust")]    # May be zero
    descent_thrust = solution[(3, "thrust")]

**Phase Boundary Analysis:**

.. code-block:: python

    # Phase transition values
    phase1_final_mass = solution[(1, "mass")][-1]
    phase2_initial_mass = solution[(2, "mass")][0]

    # Continuity verification
    altitude_transition = solution[(1, "altitude")][-1]
    altitude_continuation = solution[(2, "altitude")][0]

**Phase Comparison:**

.. code-block:: python

    # Compare phase characteristics
    phase1_duration = len(solution[(1, "time_states")])
    phase2_duration = len(solution[(2, "time_states")])

    # Phase-specific extrema
    max_thrust_p1 = max(solution[(1, "thrust")])
    max_thrust_p2 = max(solution[(2, "thrust")])

Tuple access pattern ``(phase_id, variable_name)`` provides complete control over which phase data to extract and enables detailed phase-specific analysis.

Variable Existence Validation
------------------------------

Safely validate variable availability before accessing solution data:

**Basic Existence Checking:**

.. code-block:: python

    # String key validation
    if "altitude" in solution:
        altitude_data = solution["altitude"]

    # Tuple key validation
    if (1, "thrust") in solution:
        thrust_data = solution[(1, "thrust")]

**Multiple Variable Validation:**

.. code-block:: python

    # Check multiple variables
    required_vars = ["position", "velocity", "thrust"]
    available_vars = [var for var in required_vars if var in solution]

    # Conditional access
    if "fuel_mass" in solution:
        fuel_trajectory = solution["fuel_mass"]
    else:
        print("Fuel mass not tracked in this problem")

**Phase-Specific Validation:**

.. code-block:: python

    # Phase variable existence
    if (2, "steering") in solution:
        steering_profile = solution[(2, "steering")]

    # Multi-phase validation
    phases_with_thrust = []
    for phase_id in [1, 2, 3]:
        if (phase_id, "thrust") in solution:
            phases_with_thrust.append(phase_id)

**Safe Access Patterns:**

.. code-block:: python

    # Conditional trajectory extraction
    trajectories = {}
    for var_name in ["x", "y", "z", "vx", "vy", "vz"]:
        if var_name in solution:
            trajectories[var_name] = solution[var_name]

    # Phase-conditional access
    phase_data = {}
    for phase_id in range(1, 4):
        if (phase_id, "altitude") in solution:
            phase_data[phase_id] = solution[(phase_id, "altitude")]

The ``in`` operator works with both string and tuple keys, enabling robust solution processing workflows.

Phase Information Analysis
--------------------------

The ``phases`` property provides comprehensive metadata for detailed mission analysis:

**Basic Phase Information:**

.. code-block:: python

    # Available phases
    phase_ids = list(solution.phases.keys())
    num_phases = len(solution.phases)

    # Single phase data
    phase_data = solution.phases[1]

**Timing Information:**

.. code-block:: python

    # Phase timing
    for phase_id, phase_data in solution.phases.items():
        times = phase_data["times"]
        initial_time = times["initial"]
        final_time = times["final"]
        duration = times["duration"]

**Variable Information:**

.. code-block:: python

    # Phase variables
    for phase_id, phase_data in solution.phases.items():
        variables = phase_data["variables"]
        state_names = variables["state_names"]
        control_names = variables["control_names"]
        num_states = variables["num_states"]
        num_controls = variables["num_controls"]

**Mesh Configuration:**

.. code-block:: python

    # Mesh details
    for phase_id, phase_data in solution.phases.items():
        mesh = phase_data["mesh"]
        polynomial_degrees = mesh["polynomial_degrees"]
        mesh_nodes = mesh["mesh_nodes"]
        num_intervals = mesh["num_intervals"]

**Time Array Access:**

.. code-block:: python

    # Direct time array access
    for phase_id, phase_data in solution.phases.items():
        time_arrays = phase_data["time_arrays"]
        state_times = time_arrays["states"]
        control_times = time_arrays["controls"]

**Integral Values:**

.. code-block:: python

    # Phase integral extraction
    for phase_id, phase_data in solution.phases.items():
        integrals = phase_data["integrals"]
        if integrals is not None:
            if isinstance(integrals, float):
                single_integral = integrals
            else:
                multiple_integrals = integrals

Each phase provides complete timing, variable, mesh, and integral information for comprehensive mission analysis.

Static Parameter Access
-----------------------

Extract optimized design parameters that remain constant throughout the mission:

**Parameter Availability:**

.. code-block:: python

    # Check parameter existence
    if solution.parameters["count"] > 0:
        print("Problem includes static parameters")
    else:
        print("No static parameters")

**Basic Parameter Access:**

.. code-block:: python

    # Parameter extraction - always available
    params = solution.parameters
    param_values = params["values"]        # Always valid (empty array if none)
    param_count = params["count"]          # Always valid (0 if none)
    param_names = params["names"]          # Always valid (None if none)

**Named Parameter Access:**

.. code-block:: python

    # With parameter names
    params = solution.parameters
    if params["names"] and params["count"] > 0:
        for name, value in zip(params["names"], params["values"]):
            print(f"{name}: {value}")

**Unnamed Parameter Access:**

.. code-block:: python

    # Without parameter names
    params = solution.parameters
    for i in range(params["count"]):
        value = params["values"][i]
        print(f"Parameter {i}: {value}")

**Parameter Value Extraction:**

.. code-block:: python

    # Direct value access (when parameters exist)
    params = solution.parameters
    if params["count"] >= 3:
        optimized_mass = params["values"][0]
        optimized_thrust = params["values"][1]
        design_parameter = params["values"][2]

Static parameters represent optimization variables that remain constant throughout the mission but are determined by the solver.

Adaptive Algorithm Data Access
-------------------------------

Access comprehensive adaptive mesh refinement performance data and benchmarking metrics for adaptive solutions:

**Algorithm Status:**

.. code-block:: python

    # Adaptive solution check
    try:
        adaptive = solution.adaptive
        print("Adaptive solution available")
    except RuntimeError:
        print("Fixed mesh solution")

**Basic Algorithm Information:**

.. code-block:: python

    # Algorithm status data
    try:
        adaptive = solution.adaptive
        converged = adaptive["converged"]
        iterations = adaptive["iterations"]
        tolerance = adaptive["target_tolerance"]
    except RuntimeError:
        print("No adaptive data available")

**Per-Phase Convergence Status:**

.. code-block:: python

    # Phase convergence information
    try:
        adaptive = solution.adaptive
        phase_converged = adaptive["phase_converged"]
        for phase_id, status in phase_converged.items():
            print(f"Phase {phase_id}: {'Converged' if status else 'Not converged'}")
    except RuntimeError:
        print("No adaptive data available")

**Final Error Estimates:**

.. code-block:: python

    # Final error estimates per phase per interval
    try:
        adaptive = solution.adaptive
        final_errors = adaptive["final_errors"]
        for phase_id, errors in final_errors.items():
            for interval_idx, error in enumerate(errors):
                print(f"Phase {phase_id} interval {interval_idx}: {error:.2e}")
    except RuntimeError:
        print("No adaptive data available")

**Gamma Normalization Factors:**

.. code-block:: python

    # Algorithm normalization factors
    try:
        adaptive = solution.adaptive
        gamma_factors = adaptive["gamma_factors"]
        for phase_id, factors in gamma_factors.items():
            if factors is not None:
                print(f"Phase {phase_id} gamma factors available")
    except RuntimeError:
        print("No adaptive data available")

Benchmark Array Data Access
----------------------------

Access structured benchmark arrays for performance analysis and research comparison:

**Mission-Wide Benchmark Arrays:**

.. code-block:: python

    # Complete benchmark data structure
    try:
        adaptive = solution.adaptive
        benchmark = adaptive["benchmark"]

        # All six benchmark arrays
        iterations = benchmark["mesh_iteration"]         # [0, 1, 2, 3, ...]
        errors = benchmark["estimated_error"]            # [1e-2, 1e-3, 1e-5, 1e-7, ...]
        points = benchmark["collocation_points"]         # [50, 75, 100, 150, ...]
        intervals = benchmark["mesh_intervals"]          # [10, 15, 20, 30, ...]
        degrees = benchmark["polynomial_degrees"]        # [[4,4,4], [4,6,4], [6,6,6], ...]
        strategies = benchmark["refinement_strategy"]    # [{0:'p', 1:'h'}, {2:'p'}, ...]
    except RuntimeError:
        print("Fixed mesh solution - no benchmark data available")

**Phase-Specific Benchmark Arrays:**

.. code-block:: python

    # Individual phase benchmark data
    try:
        adaptive = solution.adaptive
        phase_benchmarks = adaptive["phase_benchmarks"]

        # Access specific phase data
        phase1_data = phase_benchmarks[1]
        phase1_iterations = phase1_data["mesh_iteration"]      # [0, 1, 2, ...]
        phase1_errors = phase1_data["estimated_error"]         # [1e-2, 1e-4, 1e-6, ...]
        phase1_points = phase1_data["collocation_points"]      # [25, 35, 50, ...]
        phase1_intervals = phase1_data["mesh_intervals"]       # [5, 7, 10, ...]
        phase1_degrees = phase1_data["polynomial_degrees"]     # [[4,4], [4,6], [6,6], ...]
        phase1_strategies = phase1_data["refinement_strategy"] # [{0:'p'}, {1:'h'}, ...]
    except RuntimeError:
        print("Fixed mesh solution - no benchmark data available")

**Benchmark Array Definitions:**

.. code-block:: python

    # Array content definitions
    try:
        adaptive = solution.adaptive
        benchmark = adaptive["benchmark"]

        # mesh_iteration: Iteration sequence numbers starting from 0
        # estimated_error: Maximum error estimate across all intervals
        # collocation_points: Total collocation points used in mesh
        # mesh_intervals: Total number of mesh intervals
        # polynomial_degrees: List of polynomial degrees for each interval per iteration
        # refinement_strategy: Dictionary mapping interval index to strategy ('p' or 'h')
    except RuntimeError:
        print("Fixed mesh solution - no benchmark data available")

**Benchmark Data Iteration:**

.. code-block:: python

    # Process all benchmark iterations
    try:
        adaptive = solution.adaptive
        benchmark = adaptive["benchmark"]

        for i in range(len(benchmark["mesh_iteration"])):
            iteration = benchmark["mesh_iteration"][i]
            error = benchmark["estimated_error"][i]
            points = benchmark["collocation_points"][i]
            intervals = benchmark["mesh_intervals"][i]
            degrees = benchmark["polynomial_degrees"][i]
            strategy = benchmark["refinement_strategy"][i]

            print(f"Iteration {iteration}: Error={error:.2e}, Points={points}")
    except RuntimeError:
        print("Fixed mesh solution - no benchmark data available")

**Multi-Phase Benchmark Comparison:**

.. code-block:: python

    # Compare benchmark data across phases
    try:
        adaptive = solution.adaptive
        phase_benchmarks = adaptive["phase_benchmarks"]

        # Compare final iteration across phases
        for phase_id, phase_data in phase_benchmarks.items():
            final_error = phase_data["estimated_error"][-1]
            final_points = phase_data["collocation_points"][-1]
            print(f"Phase {phase_id}: Final error={final_error:.2e}, Points={final_points}")
    except RuntimeError:
        print("Fixed mesh solution - no benchmark data available")

Raw Iteration History Access
-----------------------------

Access complete algorithm state for each refinement iteration:

**Iteration History Structure:**

.. code-block:: python

    # Complete iteration-by-iteration algorithm state
    try:
        adaptive = solution.adaptive
        history = adaptive["iteration_history"]

        # Access specific iteration data
        iteration_data = history[2]  # Third iteration (0-indexed)

        # All available fields per iteration
        iteration_num = iteration_data["iteration"]
        phase_errors = iteration_data["phase_error_estimates"]
        total_points = iteration_data["total_collocation_points"]
        phase_points = iteration_data["phase_collocation_points"]
        phase_intervals = iteration_data["phase_mesh_intervals"]
        phase_degrees = iteration_data["phase_polynomial_degrees"]
        mesh_nodes = iteration_data["phase_mesh_nodes"]
        refinement_actions = iteration_data["refinement_strategy"]
        max_error = iteration_data["max_error_all_phases"]
        convergence_status = iteration_data["convergence_status"]
    except RuntimeError:
        print("Fixed mesh solution - no iteration history available")

**Iteration History Processing:**

.. code-block:: python

    # Process all iterations sequentially
    try:
        adaptive = solution.adaptive
        history = adaptive["iteration_history"]

        for iteration in sorted(history.keys()):
            data = history[iteration]
            print(f"Iteration {iteration}:")
            print(f"  Total points: {data['total_collocation_points']}")
            print(f"  Max error: {data['max_error_all_phases']:.2e}")

            # Phase-specific data for this iteration
            for phase_id in data["phase_error_estimates"].keys():
                phase_errors = data["phase_error_estimates"][phase_id]
                phase_points = data["phase_collocation_points"][phase_id]
                print(f"  Phase {phase_id}: {len(phase_errors)} intervals, {phase_points} points")
    except RuntimeError:
        print("Fixed mesh solution - no iteration history available")

**Mesh Evolution Tracking:**

.. code-block:: python

    # Track mesh node evolution
    try:
        adaptive = solution.adaptive
        history = adaptive["iteration_history"]

        for iteration in sorted(history.keys()):
            data = history[iteration]

            for phase_id, mesh_nodes in data["phase_mesh_nodes"].items():
                degrees = data["phase_polynomial_degrees"][phase_id]
                print(f"Iteration {iteration}, Phase {phase_id}:")
                print(f"  Mesh nodes: {len(mesh_nodes)} points")
                print(f"  Polynomial degrees: {degrees}")
    except RuntimeError:
        print("Fixed mesh solution - no mesh evolution data")

Built-in Analysis Methods
-------------------------

Access professional analysis and visualization capabilities:

**Comprehensive Benchmark Summary:**

.. code-block:: python

    # Professional benchmark analysis
    try:
        solution.print_benchmark_summary()
    except RuntimeError:
        print("No adaptive data for benchmark summary")

    # Concise benchmark information
    try:
        solution.print_benchmark_summary()
    except RuntimeError:
        print("No adaptive data for benchmark summary")

**Mesh Refinement Visualization:**

.. code-block:: python

    # Basic mesh evolution plot
    try:
        solution.plot_refinement_history(phase_id=1)
    except RuntimeError:
        print("No adaptive data for mesh refinement plot")

    # Custom mesh visualization
    try:
        solution.plot_refinement_history(
            phase_id=1,
            figsize=(16, 10),
            transform_domain=(0.0, 100.0)  # Transform from [-1,1] to [0,100]
        )
    except RuntimeError:
        print("No adaptive data for mesh refinement plot")

**Method Parameters:**

.. code-block:: python

    # plot_refinement_history parameters:
    # - phase_id: Phase to visualize (required)
    # - figsize: Figure dimensions (width, height), default (12, 6)
    # - transform_domain: Transform from [-1,1] to physical domain (min, max), default None

    # print_benchmark_summary parameters:
    # - comprehensive: Detail level (bool), default True

**Detecting Solution Type:**

.. code-block:: python

    # Handle solutions without adaptive data
    try:
        adaptive = solution.adaptive
        benchmark_available = "benchmark" in adaptive
        print(f"Adaptive solution - benchmark arrays available: {benchmark_available}")
    except RuntimeError:
        print("Fixed mesh solution - no benchmark data available")

Data Export Patterns
--------------------

Extract benchmark data for external analysis and research comparison:

**CSV Export Pattern:**

.. code-block:: python

    # Export benchmark data to CSV format
    try:
        adaptive = solution.adaptive
        benchmark = adaptive["benchmark"]

        print("iteration,error,points,intervals")
        for i in range(len(benchmark["mesh_iteration"])):
            iteration = benchmark["mesh_iteration"][i]
            error = benchmark["estimated_error"][i]
            points = benchmark["collocation_points"][i]
            intervals = benchmark["mesh_intervals"][i]

            error_str = "NaN" if np.isnan(error) else f"{error:.6e}"
            print(f"{iteration},{error_str},{points},{intervals}")
    except RuntimeError:
        print("Fixed mesh solution - no benchmark data to export")

**NumPy Array Conversion:**

.. code-block:: python

    # Convert to numpy arrays for analysis
    try:
        adaptive = solution.adaptive
        benchmark = adaptive["benchmark"]

        import numpy as np
        points_array = np.array(benchmark["collocation_points"])
        error_array = np.array(benchmark["estimated_error"])

        # Filter valid errors for analysis
        valid_errors = error_array[~np.isnan(error_array)]
    except RuntimeError:
        print("Fixed mesh solution - no benchmark data for conversion")

**Research Data Package:**

.. code-block:: python

    # Extract complete research dataset
    try:
        adaptive = solution.adaptive
        research_data = {
            "algorithm_status": {
                "converged": adaptive["converged"],
                "iterations": adaptive["iterations"],
                "tolerance": adaptive["target_tolerance"]
            },
            "mission_benchmark": adaptive["benchmark"],
            "phase_benchmarks": adaptive["phase_benchmarks"],
            "convergence_history": adaptive["iteration_history"]
        }
    except RuntimeError:
        print("Fixed mesh solution - no research data available")

Next Steps
----------

* **Problem Definition**: Study :doc:`problem_definition` to understand how solution structure relates to problem formulation
* **Complete Examples**: Explore :doc:`../examples/index` for solution analysis in context of specific problems
* **API Reference**: Use :doc:`../api/index` for detailed method signatures and advanced options
