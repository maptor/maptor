Getting Started
===============

MAPTOR solves **optimal control problems** - finding the best way to control dynamic systems over time. Whether you're planning spacecraft trajectories, robot paths, or vehicle maneuvers, MAPTOR optimizes performance while respecting physical constraints.

Your First Problem: Minimum Time
---------------------------------

Let's solve a classic problem: reach a target in minimum time with limited thrust.

.. code-block:: python

    import maptor as mtor

    # 1. Create problem and phase
    problem = mtor.Problem("Minimum Time to Target")
    phase = problem.set_phase(1)

    # 2. Define variables
    t = phase.time(initial=0.0)                          # Free final time
    pos = phase.state("position", initial=0.0, final=1.0)   # Start at 0, reach 1
    vel = phase.state("velocity", initial=0.0, final=0.0)   # Start and end at rest
    force = phase.control("force", boundary=(-2.0, 2.0))    # Limited thrust

    # 3. Define system dynamics
    phase.dynamics({
        pos: vel,        # position changes at velocity rate
        vel: force       # velocity changes at force rate (unit mass)
    })

    # 4. Set objective
    problem.minimize(t.final)  # Minimize time to reach target

    # 5. Configure solution method
    phase.mesh([8], [-1.0, 1.0])  # 8th-order polynomial, single interval

    # 6. Solve
    solution = mtor.solve_adaptive(problem)

    # 7. Analyze results
    if solution.status["success"]:
        print(f"Minimum time: {solution.status['objective']:.3f} seconds")

        # Get trajectories
        time = solution["time_states"]
        position = solution["position"]
        velocity = solution["velocity"]
        thrust = solution["force"]

        # Plot results
        solution.plot()
    else:
        print(f"Failed: {solution.status['message']}")

**Run this code** and you'll see the optimal trajectory: maximum thrust forward, then maximum thrust backward to arrive at rest.

Design Optimization Capability
----------------------------------------------

Beyond basic trajectory optimization, MAPTOR handles problems where you need to determine optimal system design parameters alongside trajectories:

.. code-block:: python

    import maptor as mtor


    # Engine sizing optimization with mass penalty
    problem = mtor.Problem("Engine Sizing Optimization")
    phase = problem.set_phase(1)

    # Design parameter: maximum engine thrust capability
    max_thrust = problem.parameter("max_thrust", boundary=(1000, 5000))

    # Physical parameters
    base_mass = 100.0  # kg (vehicle dry mass)
    engine_mass_factor = 0.05  # kg per Newton (engine specific mass)
    gravity = 9.81  # m/s²

    # Mission variables
    t = phase.time(initial=0.0)
    altitude = phase.state("altitude", initial=0.0, final=1000.0)
    velocity = phase.state("velocity", initial=0.0, final=0.0)
    thrust = phase.control("thrust", boundary=(0, None))

    # Engine cannot exceed design capability
    phase.path_constraints(thrust <= max_thrust)

    # Total vehicle mass increases with engine size
    total_mass = base_mass + max_thrust * engine_mass_factor

    # Vertical flight dynamics with gravity
    phase.dynamics({altitude: velocity, velocity: thrust / total_mass - gravity})

    # Objective: minimize mission time + engine mass penalty
    engine_mass_cost = max_thrust * engine_mass_factor * 0.1  # Cost per kg of engine
    problem.minimize(t.final + engine_mass_cost)

    # Mesh configuration
    phase.mesh([6], [-1.0, 1.0])


    phase.guess(terminal_time=50.0)

    # Solve with adaptive mesh refinement
    solution = mtor.solve_adaptive(problem)

    # Results
    if solution.status["success"]:
        optimal_thrust = solution.parameters["values"][0]
        engine_mass = optimal_thrust * engine_mass_factor
        total_vehicle_mass = base_mass + engine_mass
        mission_time = solution.status["objective"] - engine_mass * 0.1

        print("Optimal Engine Design:")
        print(f"  Max thrust capability: {optimal_thrust:.0f} N")
        print(f"  Engine mass: {engine_mass:.1f} kg")
        print(f"  Total vehicle mass: {total_vehicle_mass:.1f} kg")
        print(f"  Mission time: {mission_time:.1f} seconds")
        print(f"  Thrust-to-weight ratio: {optimal_thrust / (total_vehicle_mass * gravity):.2f}")

        solution.plot()
    else:
        print(f"Optimization failed: {solution.status['message']}")

    #Output
    #Optimal Engine Design:
    #  Max thrust capability: 3535 N
    #  Engine mass: 176.7 kg
    #  Total vehicle mass: 276.7 kg
    #  Mission time: 29.6 seconds
    #  Thrust-to-weight ratio: 1.30

**Run this code** and you'll see MAPTOR simultaneously optimized both the engine design and the flight trajectory.

**Engineering Workflow:** Instead of guessing engine size, optimizing trajectory, then iterating, MAPTOR determines optimal engine size AND trajectory together.

Understanding the Workflow
---------------------------

Every MAPTOR problem follows the same pattern:

1. **Create Problem**: Container for your optimization
2. **Set Phase(s)**: Trajectory segments with different dynamics
3. **Define Variables**: Time, states (what evolves), controls (what you choose), parameters (design variables)
4. **Set Dynamics**: How your system evolves (differential equations)
5. **Add Constraints**: Safety limits, performance bounds, target conditions
6. **Set Objective**: What to minimize (time, fuel, error, etc.)
7. **Configure Mesh**: Numerical discretization for solution
8. **Solve**: Fixed mesh (fast) or adaptive mesh (high accuracy)

**Key Concepts:**

- **States**: Quantities that evolve over time (position, velocity, mass, etc.)
- **Controls**: Inputs you optimize (thrust, steering, power, etc.)
- **Parameters**: Design variables that remain constant (engine size, battery capacity, etc.)
- **Constraints**: Limits and requirements (bounds, targets, safety limits)
- **Phases**: Trajectory segments that can link together automatically

Common Problem Types
---------------------

**Minimum Time**: Reach target fastest

.. code-block:: python

    t = phase.time(initial=0.0)  # Free final time
    problem.minimize(t.final)

**Fuel Optimal**: Use least fuel

.. code-block:: python

    fuel_used = phase.add_integral(thrust * consumption_rate)
    problem.minimize(fuel_used)

**Tracking**: Follow reference trajectory

.. code-block:: python

    error = phase.add_integral((position - reference)**2)
    problem.minimize(error)

**Multiphase**: Complex missions with automatic linking

.. code-block:: python

    # Phase 1: Ascent
    ascent = problem.set_phase(1)
    h1 = ascent.state("altitude", initial=0.0)

    # Phase 2: Coast (automatically linked)
    coast = problem.set_phase(2)
    h2 = coast.state("altitude", initial=h1.final)  # Continuous altitude

**Design Optimization**: Optimize system parameters and trajectory together

.. code-block:: python

    # Design parameter: engine capability
    max_thrust = problem.parameter("max_thrust", boundary=(1000, 5000))

    # Use parameter in constraints and objective
    phase.path_constraints(thrust <= max_thrust)
    total_mass = base_mass + max_thrust * engine_mass_factor

    # Trade-off: mission time vs. engine mass
    problem.minimize(t.final + max_thrust * mass_penalty)

Adding Constraints
------------------

**Path Constraints** (enforced continuously):

.. code-block:: python

    phase.path_constraints(
        altitude >= 0,           # Stay above ground
        velocity <= 100,         # Speed limit
        thrust**2 <= max_thrust  # Thrust limit
    )

**Variable Bounds** (during variable definition):

.. code-block:: python

    # State bounds
    fuel = phase.state("fuel", boundary=(0, 1000))  # Fuel limits

    # Control bounds
    steering = phase.control("steering", boundary=(-30, 30))  # Steering limits

Solution Methods
----------------

**Fixed Mesh**: Use your exact discretization

.. code-block:: python

    solution = mtor.solve_fixed_mesh(problem)

**Adaptive Mesh**: Automatically refine for accuracy

.. code-block:: python

    solution = mtor.solve_adaptive(
        problem,
        error_tolerance=1e-6     # Target accuracy
    )

Working with Solutions
----------------------

.. code-block:: python

    # Check success
    if solution.status["success"]:
        print(f"Objective: {solution.status['objective']}")

        # Get complete trajectories
        time_points = solution["time_states"]
        position_traj = solution["position"]
        velocity_traj = solution["velocity"]

        # Get final values
        final_position = position_traj[-1]
        max_velocity = max(velocity_traj)

        # Built-in visualization
        solution.plot()

Learning Path
-------------

**Start Here**: Run the minimum-time example above to see MAPTOR in action.

**Next Steps**:

1. **Complete Problem Definition**: Study :doc:`tutorials/problem_definition` for comprehensive coverage of all MAPTOR capabilities
2. **Solution Analysis**: Learn :doc:`tutorials/solution_access` for working with optimization results
3. **Real Examples**: Explore :doc:`examples/index` for complete problems with mathematical formulations
4. **API Reference**: Use :doc:`api/index` for detailed function documentation
