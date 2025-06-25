Complete Problem Definition Guide
=================================

This comprehensive tutorial covers all of MAPTOR's problem definition capabilities. After completing this guide, you'll understand how to construct any type of optimal control problem using MAPTOR's unified framework.

**Prerequisites:** Complete the :doc:`../quickstart` guide first to understand the basic workflow.

**Scope:** This tutorial covers the complete problem definition API, from simple single-phase problems to complex multiphase missions with design optimization.

Problem Construction Framework
------------------------------

MAPTOR uses a **builder pattern** for constructing optimal control problems. You incrementally define phases, variables, dynamics, and constraints before solving. This systematic approach handles everything from simple trajectory optimization to complex multiphase missions.

.. code-block:: python

    import maptor as mtor

    # Create the problem container
    problem = mtor.Problem("My Optimal Control Problem")

    # Define a phase (trajectory segment)
    phase = problem.set_phase(1)

A **phase** represents one segment of your trajectory with its own time domain, dynamics, and constraints. Simple problems have one phase; complex missions have multiple phases with automatic linking.

Time Variable Definition
------------------------

Each phase needs a time coordinate with complete constraint flexibility for both initial and final times:

**Initial Time Constraints:**

.. code-block:: python

    # Fixed initial time
    t = phase.time(initial=0.0)

    # Bounded initial time range
    t = phase.time(initial=(0.0, 5.0))

    # Upper bounded initial time
    t = phase.time(initial=(None, 10.0))

    # Lower bounded initial time
    t = phase.time(initial=(2.0, None))

    # Unconstrained initial time (optimization variable)
    t = phase.time(initial=None)

**Final Time Constraints:**

.. code-block:: python

    # Fixed final time
    t = phase.time(initial=0.0, final=10.0)

    # Free final time (optimization variable)
    t = phase.time(initial=0.0, final=None)

    # Bounded final time range
    t = phase.time(initial=0.0, final=(8.0, 12.0))

    # Upper bounded final time
    t = phase.time(initial=0.0, final=(None, 15.0))

    # Lower bounded final time
    t = phase.time(initial=0.0, final=(5.0, None))

**Combined Time Constraints:**

.. code-block:: python

    # Both endpoints free with bounds
    t = phase.time(initial=(0.0, 2.0), final=(8.0, 12.0))

    # Mixed constraint types
    t = phase.time(initial=0.0, final=(None, 20.0))

The time variable ``t`` provides ``.initial`` and ``.final`` properties for use in objectives and constraints. Free final time enables minimum-time problems via ``problem.minimize(t.final)``.

State Variable Specification
----------------------------

States represent quantities that evolve according to differential equations, with comprehensive constraint specification:

**Initial State Constraints:**

.. code-block:: python

    # Fixed initial value
    altitude = phase.state("altitude", initial=0.0)

    # Bounded initial range (uncertainty in starting condition)
    position = phase.state("position", initial=(0.0, 5.0))

    # Upper bounded initial
    fuel = phase.state("fuel", initial=(None, 1000.0))

    # Lower bounded initial
    speed = phase.state("speed", initial=(50.0, None))

    # Unconstrained initial (optimization variable)
    free_start = phase.state("free_start", initial=None)

**Final State Constraints:**

.. code-block:: python

    # Fixed final value (exact target)
    x_target = phase.state("x_position", final=1000.0)

    # Bounded final range (target region)
    landing_zone = phase.state("landing_x", final=(990.0, 1010.0))

    # Upper bounded final (maximum allowed)
    max_altitude = phase.state("altitude", final=(None, 5000.0))

    # Lower bounded final (minimum required)
    min_speed = phase.state("speed", final=(100.0, None))

    # Unconstrained final
    free_end = phase.state("free_end", final=None)

**Path Constraints (Continuous Bounds):**

.. code-block:: python

    # Two-sided path bounds
    altitude = phase.state("altitude", boundary=(0.0, 10000.0))

    # Upper path bound only
    speed = phase.state("speed", boundary=(None, 250.0))

    # Lower path bound only
    fuel_mass = phase.state("fuel", boundary=(0.0, None))

**Complete Constraint Combinations:**

.. code-block:: python

    # All constraint types together
    comprehensive_state = phase.state("state",
        initial=(0.0, 5.0),        # Start in range [0, 5]
        final=(95.0, 105.0),       # End in range [95, 105]
        boundary=(0.0, 150.0)      # Stay in range [0, 150] throughout
    )

    # Mixed constraint types
    mixed_state = phase.state("mixed",
        initial=0.0,               # Fixed start
        final=(None, 100.0),       # Upper bounded end
        boundary=(0.0, None)       # Lower bounded path
    )

Control Variable Specification
------------------------------

Controls represent actuator inputs with comprehensive bound specification:

.. code-block:: python

    # Unconstrained control (any value allowed)
    force = phase.control("force")

    # Two-sided bounds (most common)
    thrust = phase.control("thrust", boundary=(0.0, 2000.0))

    # Symmetric bounds
    steering_angle = phase.control("steering", boundary=(-0.5, 0.5))

    # Upper bound only
    power = phase.control("power", boundary=(None, 1500.0))

    # Lower bound only
    heating = phase.control("heating", boundary=(0.0, None))

    # Fixed control value (constant throughout)
    constant_thrust = phase.control("constant", boundary=1000.0)

**Control Bound Examples:**

.. code-block:: python

    # Asymmetric bounds (different positive/negative limits)
    braking_force = phase.control("brake", boundary=(-500.0, 100.0))

    # Large dynamic range
    throttle_percent = phase.control("throttle", boundary=(0.0, 100.0))

    # Actuator saturation limits
    elevator_deflection = phase.control("elevator", boundary=(-30.0, 30.0))

Controls are optimization variables that the solver adjusts to minimize your objective while satisfying all constraints.

System Dynamics Definition
--------------------------

Define how your system evolves over time using ordinary differential equations with complete expression flexibility:

**Basic Dynamics Patterns:**

.. code-block:: python

    # Simple integrator dynamics
    position = phase.state("position", initial=0.0)
    velocity = phase.state("velocity", initial=0.0)
    acceleration = phase.control("acceleration", boundary=(-10, 10))

    phase.dynamics({
        position: velocity,                    # dx/dt = v
        velocity: acceleration                 # dv/dt = a
    })

**Complex Dynamics with Mathematical Expressions:**

.. code-block:: python

    import casadi as ca

    # Nonlinear dynamics with trigonometry
    x = phase.state("x_position", initial=0.0)
    y = phase.state("y_position", initial=0.0)
    heading = phase.state("heading", initial=0.0)
    speed = phase.state("speed", initial=5.0)

    steering = phase.control("steering", boundary=(-0.5, 0.5))
    acceleration = phase.control("acceleration", boundary=(-3, 3))

    wheelbase = 2.5  # Vehicle parameter

    phase.dynamics({
        x: speed * ca.cos(heading),
        y: speed * ca.sin(heading),
        heading: speed * ca.tan(steering) / wheelbase,
        speed: acceleration - 0.1 * speed  # With drag
    })

**Dynamics with Time Dependence:**

.. code-block:: python

    # Time-varying dynamics
    phase.dynamics({
        position: velocity,
        velocity: thrust - 9.81 * (1 + 0.01 * t),  # Time-varying gravity
        mass: -thrust * fuel_flow_rate * ca.sin(t)  # Time-dependent consumption
    })

**Dynamics with Control Coupling:**

.. code-block:: python

    # Multiple controls affecting single state
    phase.dynamics({
        velocity: thrust_main + thrust_aux - drag_force,
        attitude: torque_x + torque_y + torque_z,
        energy: -power_main - power_aux - power_avionics
    })

You must provide dynamics for every state variable in your phase. The expressions can involve any combination of states, controls, time, parameters, and mathematical functions.

Comprehensive Constraint System
-------------------------------

MAPTOR provides multiple constraint types for complete problem specification:

**Path Constraints (Applied Continuously):**

.. code-block:: python

    # Safety and performance bounds
    phase.path_constraints(
        altitude >= 100,                    # Minimum altitude
        velocity <= 250,                    # Speed limit
        acceleration**2 <= 25,              # Acceleration magnitude
        fuel_mass >= 0                      # Physical constraint
    )

    # Complex geometric constraints
    phase.path_constraints(
        (x - 50)**2 + (y - 50)**2 >= 100,  # Avoid circular obstacle
        ca.sqrt(vx**2 + vy**2) <= v_max,   # Vector magnitude limit
        ca.atan2(y, x) >= min_angle         # Angular constraint
    )

    # State-dependent constraints
    phase.path_constraints(
        thrust <= max_thrust * (fuel_mass / initial_mass),  # Thrust-to-weight ratio
        lift_force <= 0.5 * rho * v**2 * wing_area * cl_max  # Aerodynamic limit
    )

**Event Constraints (Applied at Boundaries):**

.. code-block:: python

    # Integral term constraints
    fuel_consumed = phase.add_integral(thrust * fuel_flow_rate)
    energy_cost = phase.add_integral(power**2)

    phase.event_constraints(
        fuel_consumed <= 100.0,             # Fuel budget
        energy_cost <= 50.0                 # Energy budget
    )

    # Complex final conditions
    phase.event_constraints(
        x.final**2 + y.final**2 >= 100,    # Final distance constraint
        vx.final * ca.cos(heading.final) >= 5.0,  # Velocity component
        ca.sqrt(x.final**2 + y.final**2 + z.final**2) <= target_radius
    )

    # Multi-variable relationships
    phase.event_constraints(
        altitude.final / velocity.final <= glide_ratio,
        (mass.initial - mass.final) / fuel_capacity >= 0.8
    )

Mesh Configuration
------------------

The mesh controls how accurately your problem is discretized. Higher-order polynomials and more intervals give better accuracy but cost more computation:

.. code-block:: python

    # Single interval, 8th-order polynomial
    phase.mesh([8], [-1.0, 1.0])

    # Three intervals with different polynomial orders
    phase.mesh([6, 10, 6], [-1.0, -0.3, 0.3, 1.0])

    # Fine discretization for complex dynamics
    phase.mesh([4, 4, 4, 4, 4], [-1.0, -0.6, -0.2, 0.2, 0.6, 1.0])

The mesh points define interval boundaries in the normalized domain [-1, 1]. MAPTOR automatically maps these to your actual phase time span.

Choosing a Solver
-----------------

MAPTOR provides two solving approaches:

.. code-block:: python

    # Fixed mesh: Use your exact mesh specification
    solution = mtor.solve_fixed_mesh(problem)

    # Adaptive mesh: Automatically refine for high accuracy
    solution = mtor.solve_adaptive(
        problem,
        error_tolerance=1e-6,     # Target accuracy
        max_iterations=20         # Refinement limit
    )

Adaptive solving automatically adds mesh points and increases polynomial degrees until your target accuracy is achieved. Use it for high-precision solutions.

Analyzing Solutions
-------------------

Once solved, extract trajectories and analyze results:

.. code-block:: python

    # Check if optimization succeeded
    if solution.status["success"]:
        print(f"Objective value: {solution.status['objective']}")
        print(f"Total mission time: {solution.status['total_mission_time']}")

        # Get complete trajectories
        time_points = solution["time_states"]
        position_trajectory = solution["position"]
        velocity_trajectory = solution["velocity"]
        force_trajectory = solution["force"]

        # Analyze specific values
        final_position = position_trajectory[-1]
        max_speed = max(velocity_trajectory)

        # Built-in plotting
        solution.plot()
    else:
        print(f"Optimization failed: {solution.status['message']}")

The solution object provides comprehensive access to all trajectories, final values, and solver diagnostics.

Integral Terms and Accumulated Quantities
-----------------------------------------

Many optimal control problems involve accumulated quantities over the trajectory duration:

**Basic Integral Definition:**

.. code-block:: python

    # Fuel consumption
    fuel_flow = phase.add_integral(thrust * fuel_consumption_rate)

    # Energy expenditure
    energy_cost = phase.add_integral(power**2)

    # Tracking error accumulation
    tracking_error = phase.add_integral((position - reference_trajectory)**2)

    # Distance traveled
    distance = phase.add_integral(ca.sqrt(vx**2 + vy**2))

**Complex Integral Expressions:**

.. code-block:: python

    # Time-varying integrands
    time_weighted_cost = phase.add_integral(t * thrust**2)

    # State-dependent integrands
    altitude_penalty = phase.add_integral(ca.exp(-altitude/1000) * drag_force)

    # Multi-variable integrands
    combined_cost = phase.add_integral(
        fuel_weight * fuel_flow +
        time_weight * 1.0 +
        control_weight * (thrust**2 + steering**2)
    )

**Using Integrals in Objectives:**

.. code-block:: python

    # Minimize single integral
    problem.minimize(fuel_flow)

    # Weighted combinations
    problem.minimize(10*t.final + fuel_flow + 0.1*tracking_error)

    # Complex multi-objective formulations
    performance = altitude.final + range.final
    cost = fuel_flow + maintenance_cost
    problem.minimize(cost - performance_weight * performance)

**Using Integrals in Constraints:**

.. code-block:: python

    # Resource budget constraints
    phase.event_constraints(
        fuel_flow <= fuel_capacity,
        energy_cost <= battery_capacity,
        heat_generated <= thermal_limit
    )

Static Parameter Optimization
-----------------------------

Optimize design parameters that remain constant throughout the mission:

**Basic Parameter Definition:**

.. code-block:: python

    # Unconstrained parameters
    design_var = problem.parameter("design_variable")

    # Bounded parameters
    vehicle_mass = problem.parameter("mass", boundary=(100.0, 500.0))
    wing_area = problem.parameter("wing_area", boundary=(10.0, 50.0))

    # Upper bounded only
    max_power = problem.parameter("power_limit", boundary=(None, 5000.0))

    # Lower bounded only
    min_thrust = problem.parameter("min_thrust", boundary=(1000.0, None))

    # Fixed parameter value - numeric constant
    gravity_constant = problem.parameter("gravity", fixed=9.81)

    # Fixed parameter value - symbolic relationship
    mass2 = problem.parameter("mass2", fixed=mass1 * 2.0)

**Parameters in Dynamics and Constraints:**

.. code-block:: python

    # Use parameters in dynamics
    phase.dynamics({
        velocity: thrust / vehicle_mass - drag_coefficient * velocity**2,
        altitude: velocity * ca.sin(flight_path_angle),
        fuel_mass: -thrust / (specific_impulse * gravity_constant)
    })

    # Use parameters in constraints
    phase.path_constraints(
        thrust <= max_power / propulsion_efficiency,
        lift_force <= 0.5 * air_density * velocity**2 * wing_area * max_cl
    )

    # Use parameters in objectives
    total_cost = vehicle_mass * cost_per_kg + wing_area * manufacturing_cost
    mission_performance = altitude.final + range.final
    problem.minimize(total_cost - performance_weight * mission_performance)

**Parameter Optimization Examples:**

.. code-block:: python

    # Vehicle design optimization
    engine_thrust = problem.parameter("engine_thrust", boundary=(1000, 5000))
    fuel_capacity = problem.parameter("fuel_tank", boundary=(100, 1000))
    aerodynamic_efficiency = problem.parameter("l_over_d", boundary=(5, 20))

    # Trajectory and design co-optimization
    structural_mass = vehicle_mass - fuel_capacity
    problem.minimize(
        structural_mass +                    # Minimize vehicle mass
        fuel_consumed +                      # Minimize fuel usage
        0.1 * (t.final - target_time)**2    # Minimize time deviation
    )

**Parameter Initial Guesses:**

For complex optimization problems with static parameters, providing good initial guesses significantly improves solver convergence and solution quality:

.. code-block:: python

    # Define parameters with bounds
    vehicle_mass = problem.parameter("mass", boundary=(100, 500))
    drag_coefficient = problem.parameter("drag", boundary=(0, 0.1))
    wing_area = problem.parameter("wing_area", boundary=(10, 50))

    # Set initial guesses using parameter names
    problem.parameter_guess(
        mass=300.0,        # Start optimization near middle of range
        drag=0.05,         # Reasonable aerodynamic estimate
        wing_area=25.0     # Initial design point
    )

**Advanced Parameter Guess Strategies:**

.. code-block:: python

    # Physics-based guess selection
    thrust_to_weight = 1.2  # Desired T/W ratio
    estimated_mass = 1000.0
    required_thrust = thrust_to_weight * estimated_mass * 9.81

    max_thrust = problem.parameter("max_thrust", boundary=(5000, 15000))
    problem.parameter_guess(max_thrust=required_thrust)

    # Engineering constraint-based guesses
    max_heat_rate = problem.parameter("max_q_dot", boundary=(500, 1000))
    safety_margin = 0.8  # Conservative initial design
    problem.parameter_guess(max_q_dot=1000 * safety_margin)

**Parameter Guess Validation:**

.. code-block:: python

    # The system validates parameter names exist
    mass = problem.parameter("vehicle_mass", boundary=(100, 1000))

    # This works - parameter exists
    problem.parameter_guess(vehicle_mass=500.0)

    # This raises ConfigurationError - parameter name doesn't exist
    # problem.parameter_guess(mass=500.0)  # Wrong name

    # Parameters without explicit guesses default to 0.0
    wing_span = problem.parameter("wing_span", boundary=(5, 15))
    # No guess provided - solver starts with 0.0

Objective Function Specification
--------------------------------

Define scalar optimization objectives with complete expression flexibility:

**Single Objective Types:**

.. code-block:: python

    # Minimum time
    problem.minimize(t.final)

    # Maximum final altitude
    problem.minimize(-altitude.final)  # Negative for maximization

    # Minimum fuel consumption
    fuel_used = phase.add_integral(fuel_flow_rate * thrust)
    problem.minimize(fuel_used)

    # Minimum control effort
    control_effort = phase.add_integral(thrust**2 + steering**2)
    problem.minimize(control_effort)

**Multi-Objective Formulations:**

.. code-block:: python

    # Weighted combination
    performance = range.final + altitude.final
    cost = fuel_used + time_penalty * t.final
    problem.minimize(cost - 10*performance)

    # Trade-off objectives
    problem.minimize(
        fuel_weight * fuel_consumed +
        time_weight * t.final +
        accuracy_weight * tracking_error +
        smoothness_weight * control_effort
    )

**Complex Objective Expressions:**

.. code-block:: python

    # Nonlinear objective combinations
    efficiency_metric = range.final / fuel_consumed
    safety_margin = ca.sqrt(x.final**2 + y.final**2) - obstacle_radius
    problem.minimize(-efficiency_metric - safety_margin)

    # Parameter-dependent objectives
    total_system_mass = structural_mass + fuel_mass.final
    payload_fraction = payload_mass / total_system_mass
    problem.minimize(-payload_fraction + cost_per_kg * total_system_mass)

Providing Initial Guesses
--------------------------

For complex problems, providing good initial guesses helps the solver converge. MAPTOR provides a phase-specific guess system through the ``phase.guess()`` method.

**Simple Time Guesses**

For most problems, guessing final times is sufficient:

.. code-block:: python

    # Simple time guess
    phase.guess(terminal_time=10.0)


**Complete Trajectory Guesses**

For challenging problems, provide complete trajectory guesses. Arrays must match your mesh configuration exactly:

.. code-block:: python

    # First, set up your mesh
    phase.mesh([4, 6], [-1.0, 0.0, 1.0])  # Two intervals: degree 4 and degree 6

This creates two mesh intervals with specific array requirements:

* **State arrays**: For polynomial degree N, you need **N+1 points**
* **Control arrays**: For polynomial degree N, you need **N points**
* **Array structure**: ``[num_variables, num_points_per_interval]``

.. code-block:: python

    import numpy as np

    # Example with 2 states, 1 control, mesh [4, 6]
    states_guess = []
    controls_guess = []

    # Interval 1: degree 4 polynomial needs 5 state points, 4 control points
    states_interval_1 = np.array([
        [0.0, 0.2, 0.4, 0.6, 0.8],    # State 1: 5 points
        [0.0, 0.1, 0.3, 0.6, 1.0]     # State 2: 5 points
    ])
    controls_interval_1 = np.array([
        [1.0, 0.8, 0.6, 0.4]          # Control 1: 4 points
    ])

    # Interval 2: degree 6 polynomial needs 7 state points, 6 control points
    states_interval_2 = np.array([
        [0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4],  # State 1: 7 points
        [1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4]   # State 2: 7 points
    ])
    controls_interval_2 = np.array([
        [0.4, 0.2, 0.0, -0.2, -0.4, -0.6]      # Control 1: 6 points
    ])

    # Collect intervals in order
    states_guess = [states_interval_1, states_interval_2]
    controls_guess = [controls_interval_1, controls_interval_2]

    # Provide complete guess
    phase.guess(
        states=states_guess,
        controls=controls_guess,
        terminal_time=12.0,
        integrals=5.0  # If you have integral terms
    )

**Easy Guess Generation**

For simple trajectories, use linear interpolation:

.. code-block:: python

    def generate_linear_guess(mesh_degrees, initial_values, final_values):
        states_guess = []

        for degree in mesh_degrees:
            num_points = degree + 1
            # Linear interpolation from initial to final
            states = np.zeros((len(initial_values), num_points))
            for i, (init_val, final_val) in enumerate(zip(initial_values, final_values)):
                states[i, :] = np.linspace(init_val, final_val, num_points)
            states_guess.append(states)

        return states_guess

    # Usage
    phase.mesh([4, 4], [-1.0, 0.0, 1.0])
    states_guess = generate_linear_guess(
        mesh_degrees=[4, 4],
        initial_values=[0.0, 0.0],  # [position, velocity]
        final_values=[1.0, 0.0]     # [position, velocity]
    )
    phase.guess(states=states_guess, terminal_time=8.0)

**Multiphase Guesses**

For multiphase problems, provide guesses for each phase individually:

.. code-block:: python

    # Phase 1 guess
    phase1.guess(
        states=states_guess_phase1,
        controls=controls_guess_phase1,
        terminal_time=5.0
    )

    # Phase 2 guess
    phase2.guess(
        states=states_guess_phase2,
        controls=controls_guess_phase2,
        terminal_time=10.0
    )

**When to Use Detailed Guesses**

* **Simple problems**: Just guess terminal times
* **Complex dynamics**: Provide trajectory guesses that roughly follow expected behavior
* **Multiple local minima**: Good guesses help find the right solution
* **Convergence issues**: Detailed guesses often resolve solver failures

Multiphase Problems
-------------------

Complex missions often require multiple phases with different dynamics or operating modes:

.. code-block:: python

    problem = mtor.Problem("Rocket Launch Mission")

    # Phase 1: Powered ascent
    ascent = problem.set_phase(1)
    t1 = ascent.time(initial=0.0, final=120.0)
    h1 = ascent.state("altitude", initial=0.0)
    v1 = ascent.state("velocity", initial=0.0)
    m1 = ascent.state("mass", initial=1000.0)
    thrust1 = ascent.control("thrust", boundary=(0, 5000))

    ascent.dynamics({
        h1: v1,
        v1: thrust1/m1 - 9.81,
        m1: -thrust1 * 0.001  # Fuel consumption
    })

    # Phase 2: Coasting flight (automatically linked)
    coast = problem.set_phase(2)
    t2 = coast.time(initial=t1.final, final=300.0)    # Continuous time
    h2 = coast.state("altitude", initial=h1.final)   # Continuous altitude
    v2 = coast.state("velocity", initial=v1.final)   # Continuous velocity
    m2 = coast.state("mass", initial=m1.final)       # Continuous mass

    coast.dynamics({
        h2: v2,
        v2: -9.81,  # No thrust, only gravity
        m2: 0       # No fuel consumption
    })

When you reference ``phase1_variable.final`` in ``phase2_variable.initial``, MAPTOR automatically creates the mathematical linkage between phases.

**Different Phase Configurations:**

.. code-block:: python

    # Free transition time
    t2 = coast.time(initial=t1.final)  # Time continues, final time optimized

    # Fixed phase duration
    t2 = coast.time(initial=t1.final, final=t1.final + 200.0)

    # Bounded transition time
    t2 = coast.time(initial=t1.final, final=(250, 350))

Each phase can have completely different dynamics, constraints, and mesh configurations.

Advanced Problem Types
----------------------

**Minimum Time Problems:**

.. code-block:: python

    t = phase.time(initial=0.0)  # Free final time
    # ... define states, controls, dynamics
    problem.minimize(t.final)    # Minimize mission duration

**Fuel-Optimal Problems:**

.. code-block:: python

    fuel_used = phase.add_integral(thrust * fuel_flow_rate)
    problem.minimize(fuel_used)

**Tracking Problems:**

.. code-block:: python

    # Define reference trajectory
    reference_position = t * desired_speed

    # Minimize tracking error
    tracking_error = phase.add_integral((position - reference_position)**2)
    problem.minimize(tracking_error)

**Design Optimization:**

.. code-block:: python

    # Optimize both trajectory and design
    wing_span = problem.parameter("wing_span", boundary=(5, 15))
    engine_power = problem.parameter("power", boundary=(100, 500))

    # Multi-objective: maximize performance, minimize cost
    performance = altitude.final + range.final
    cost = wing_span**2 + engine_power
    problem.minimize(cost - 10*performance)

Common Patterns and Tips
------------------------

**Scaling for Numerical Stability:**

.. code-block:: python

    # Scale large values
    altitude_km = phase.state("altitude_km", initial=0.0)  # km instead of m
    velocity_kmh = phase.state("velocity_kmh", initial=0.0) # km/h instead of m/s

    # Use in dynamics with appropriate scaling
    phase.dynamics({
        altitude_km: velocity_kmh / 3.6,  # Convert km/h to km/s
        # ...
    })

**State Bounds for Physical Realism:**

.. code-block:: python

    # Prevent nonphysical solutions
    altitude = phase.state("altitude", boundary=(0, None))    # Can't go underground
    speed = phase.state("speed", boundary=(0, None))          # Can't go backwards
    fuel = phase.state("fuel", boundary=(0, 1000))            # Fuel limits

**Control Smoothing:**

.. code-block:: python

    # Add control rate penalty for smoother solutions
    control_effort = phase.add_integral(thrust**2)
    control_smoothness = phase.add_integral((thrust - previous_thrust)**2)
    problem.minimize(control_effort + 0.1*control_smoothness)

Solver Options
--------------

Fine-tune solver behavior for challenging problems:

.. code-block:: python

    # Fixed mesh with custom solver settings
    solution = mtor.solve_fixed_mesh(
        problem,
        nlp_options={
            "ipopt.print_level": 5,        # Verbose output
            "ipopt.max_iter": 3000,        # More iterations
            "ipopt.tol": 1e-8              # Tighter tolerance
        }
    )

    # Adaptive with custom accuracy
    solution = mtor.solve_adaptive(
        problem,
        error_tolerance=1e-8,           # High accuracy
        max_iterations=30,              # More refinement cycles
        min_polynomial_degree=4,        # Start with higher order
        max_polynomial_degree=15        # Allow very high order
    )

Summary: Complete MAPTOR Capabilities
-------------------------------------

You now have comprehensive knowledge of MAPTOR's problem definition capabilities:

**Core Framework:**
- **Problem Construction**: Incremental building with phases, variables, dynamics
- **Variable System**: Time, states, and controls with unified constraint specification
- **Constraint Architecture**: Bounds, paths, events, and automatic multiphase linking

**Advanced Capabilities:**
- **Static Parameters**: Design optimization alongside trajectory optimization
- **Integral Terms**: Accumulated quantities for objectives and constraints
- **Multiphase Problems**: Complex missions with automatic phase continuity
- **Initial Guess System**: From simple time guesses to detailed trajectory specifications

**Problem Types Covered:**
- Minimum-time problems
- Fuel-optimal trajectories
- Tracking and regulation
- Design optimization
- Multiphase missions

**Practical Knowledge:**
- Numerical scaling strategies
- Physical constraint enforcement
- Control smoothing techniques
- Solver configuration options
- Mesh design principles

Building Your Problem-Solving Expertise
---------------------------------------

**Recommended Learning Progression:**

1. **Start Simple**: Begin with single-phase minimum-time problems to solidify the basic workflow
2. **Add Constraints**: Practice with path constraints and bounded controls
3. **Include Integrals**: Implement fuel-optimal and tracking problems
4. **Go Multiphase**: Design missions with multiple operating phases
5. **Optimize Designs**: Combine trajectory and parameter optimization

**Next Learning Resources:**

* **Solution Analysis**: Study :doc:`solution_access` to master working with optimization results
* **Complete Examples**: Explore :doc:`../examples/index` for fully-worked problems with mathematical formulations
* **API Reference**: Use :doc:`../api/index` for detailed function documentation and advanced options

**When You're Ready for Real Problems:**

You now understand all the tools needed to formulate any optimal control problem in MAPTOR. The key to success is starting with the simplest version of your problem, getting it working, then gradually adding complexity as you verify each component.
