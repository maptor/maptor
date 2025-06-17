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
    if solution.parameters is not None:
        print("Problem includes static parameters")
    else:
        print("No static parameters")

**Basic Parameter Access:**

.. code-block:: python

    # Parameter extraction
    if solution.parameters:
        param_values = solution.parameters["values"]
        param_count = solution.parameters["count"]
        param_names = solution.parameters["names"]

**Named Parameter Access:**

.. code-block:: python

    # With parameter names
    params = solution.parameters
    if params and params["names"]:
        for name, value in zip(params["names"], params["values"]):
            print(f"{name}: {value}")

**Unnamed Parameter Access:**

.. code-block:: python

    # Without parameter names
    params = solution.parameters
    if params:
        for i, value in enumerate(params["values"]):
            print(f"Parameter {i}: {value}")

**Parameter Value Extraction:**

.. code-block:: python

    # Direct value access
    if solution.parameters:
        optimized_mass = solution.parameters["values"][0]
        optimized_thrust = solution.parameters["values"][1]
        design_parameter = solution.parameters["values"][2]

Static parameters represent optimization variables that remain constant throughout the mission but are determined by the solver.

Adaptive Algorithm Analysis
---------------------------

Examine convergence and refinement performance for adaptive solutions:

**Algorithm Status:**

.. code-block:: python

    # Adaptive solution check
    if solution.adaptive:
        print("Adaptive solution available")
    else:
        print("Fixed mesh solution")

**Convergence Information:**

.. code-block:: python

    # Basic convergence data
    if solution.adaptive:
        converged = solution.adaptive["converged"]
        iterations = solution.adaptive["iterations"]
        tolerance = solution.adaptive["target_tolerance"]

**Phase-Specific Convergence:**

.. code-block:: python

    # Per-phase convergence status
    if solution.adaptive:
        phase_converged = solution.adaptive["phase_converged"]
        for phase_id, status in phase_converged.items():
            print(f"Phase {phase_id}: {'Converged' if status else 'Not converged'}")

**Error Analysis:**

.. code-block:: python

    # Error estimates
    if solution.adaptive:
        final_errors = solution.adaptive["final_errors"]
        for phase_id, errors in final_errors.items():
            if errors:
                max_error = max(errors)
                mean_error = sum(errors) / len(errors)

**Refinement Factors:**

.. code-block:: python

    # Algorithm parameters
    if solution.adaptive:
        gamma_factors = solution.adaptive["gamma_factors"]
        for phase_id, factors in gamma_factors.items():
            if factors is not None:
                refinement_data = factors

**Complete Adaptive Analysis:**

.. code-block:: python

    # Full adaptive diagnostics
    adaptive = solution.adaptive
    if adaptive:
        algorithm_converged = adaptive["converged"]
        total_iterations = adaptive["iterations"]
        target_accuracy = adaptive["target_tolerance"]
        phase_status = adaptive["phase_converged"]
        error_estimates = adaptive["final_errors"]
        normalization_factors = adaptive["gamma_factors"]

The ``adaptive`` property provides comprehensive algorithm performance data for understanding solution quality and refinement behavior.

Built-in Solution Methods
--------------------------

MAPTOR provides convenient methods for common solution analysis tasks:

**Comprehensive Plotting:**

.. code-block:: python

    # Default plotting (all variables, all phases)
    solution.plot()

    # Specific phase plotting
    solution.plot(phase_id=1)

    # Selected variables
    solution.plot("altitude", "velocity", "thrust")

    # Phase-specific variables
    solution.plot(1, "position", "velocity")

    # Custom formatting
    solution.plot(figsize=(16, 10), show_phase_boundaries=True)

**Solution Summaries:**

.. code-block:: python

    # Comprehensive summary (default)
    solution.summary()

    # Concise summary
    solution.summary(comprehensive=False)

**Plot Customization:**

.. code-block:: python

    # All phases with boundaries
    solution.plot(show_phase_boundaries=True)

    # No phase boundaries
    solution.plot(show_phase_boundaries=False)

    # Large figure size
    solution.plot(figsize=(20, 12))

**Summary Control:**

.. code-block:: python

    # Detailed diagnostics
    solution.summary(comprehensive=True)

    # Key information only
    solution.summary(comprehensive=False)

Built-in methods provide immediate visualization and analysis capabilities without requiring custom plotting code.

Next Steps
----------

* **Problem Definition**: Study :doc:`problem_definition` to understand how solution structure relates to problem formulation
* **Complete Examples**: Explore :doc:`../examples/index` for solution analysis in context of specific problems
* **API Reference**: Use :doc:`../api/index` for detailed method signatures and advanced options
