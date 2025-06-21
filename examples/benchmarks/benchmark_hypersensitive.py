import numpy as np

import maptor as mtor


# Problem setup
problem = mtor.Problem("Hypersensitive Problem")
phase = problem.set_phase(1)

# Variables
t = phase.time(initial=0, final=10000)
x = phase.state("x", initial=1.5, final=1.0)
u = phase.control("u")

# Dynamics
phase.dynamics({x: -(x**3) + u})

# Objective
integrand = 0.5 * (x**2 + u**2)
integral_var = phase.add_integral(integrand)
problem.minimize(integral_var)

# Mesh and guess
phase.mesh([2] * 10, np.linspace(-1.0, 1.0, 11))

# Solve with adaptive mesh
solution = mtor.solve_adaptive(
    problem,
    error_tolerance=7.5e-7,
    min_polynomial_degree=3,
    max_polynomial_degree=8,
    ode_solver_tolerance=1e-8,
    nlp_options={"ipopt.print_level": 0, "ipopt.max_iter": 500},
    show_summary=True,
)

if not solution.status["success"]:
    print(f"Solution failed: {solution.status['message']}")
    exit(1)

print("✓ Solution converged successfully")

# =============================================================================
# PROFESSIONAL BENCHMARK SUMMARY
# =============================================================================

# Display professional benchmark analysis
solution.print_benchmark_summary()

# =============================================================================
# CUSTOM ANALYSIS WITH SINGLE SOURCE RAW DATA
# =============================================================================

print("\n" + "=" * 60)
print("CUSTOM ANALYSIS - SINGLE SOURCE DATA")
print("=" * 60)

# Extract benchmark data from single source
benchmark_data = solution.adaptive["benchmark"]

# Phase-specific analysis using single source
if len(solution.phases) > 1:
    print("\nPhase-specific analysis:")
    phase_benchmarks = solution.adaptive["phase_benchmarks"]
    for phase_id in solution.phases.keys():
        phase_benchmark = phase_benchmarks[phase_id]
        final_error = phase_benchmark["estimated_error"][-1]
        final_points = phase_benchmark["collocation_points"][-1]
        print(f"  Phase {phase_id}: Final error={final_error:.3e}, Points={final_points}")

# Custom calculations using extracted arrays
iterations = benchmark_data["mesh_iteration"]
errors = benchmark_data["estimated_error"]
points = benchmark_data["collocation_points"]

# Same data access pattern
valid_errors = [e for e in errors[1:] if not (np.isnan(e) or np.isinf(e))]
if len(valid_errors) >= 2:
    convergence_rate = np.log(valid_errors[0] / valid_errors[-1]) / (len(valid_errors) - 1)
    print(f"\nCustom Metric - Convergence Rate: {convergence_rate:.2f} per iteration")

# =============================================================================
# VISUALIZATION
# =============================================================================

print("\n" + "=" * 60)
print("VISUALIZATION")
print("=" * 60)

try:
    # Plot mesh refinement history - uses iteration_history (single source)
    print("\nGenerating mesh refinement history plot...")
    solution.plot_refinement_history(phase_id=1, figsize=(12, 8))

    # Create convergence plot using extracted benchmark data
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))

    # Error convergence
    valid_indices = [i for i, e in enumerate(errors) if not np.isnan(e)]
    valid_iterations = [iterations[i] for i in valid_indices]
    valid_errors_plot = [errors[i] for i in valid_indices]

    if valid_errors_plot:
        ax1.semilogy(valid_iterations, valid_errors_plot, "bo-", linewidth=2, markersize=8)
        ax1.axhline(
            y=solution.adaptive["target_tolerance"],
            color="r",
            linestyle="--",
            label=f"Target tolerance: {solution.adaptive['target_tolerance']:.1e}",
        )
        ax1.set_xlabel("Mesh Iteration")
        ax1.set_ylabel("Estimated Error")
        ax1.set_title("Error Convergence")
        ax1.grid(True, alpha=0.3)
        ax1.legend()

    # Computational cost
    ax2.plot(iterations, points, "ro-", linewidth=2, markersize=8)
    ax2.set_xlabel("Mesh Iteration")
    ax2.set_ylabel("Collocation Points")
    ax2.set_title("Computational Cost")
    ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

except ImportError:
    print("matplotlib not available for plotting")

# =============================================================================
# RAW DATA ACCESS
# =============================================================================

print("\n" + "=" * 60)
print("RAW DATA ACCESS")
print("=" * 60)

print("Raw benchmark data available from single source:")
print(f"  Iterations: {len(benchmark_data['mesh_iteration'])}")
print(f"  Data keys: {list(benchmark_data.keys())}")

#  Data consistency validation - all extracted from iteration_history
raw_history = solution.adaptive["iteration_history"]
print(f"\nValidation: Raw iteration history has {len(raw_history)} iterations")
print(f"Benchmark arrays have {len(benchmark_data['mesh_iteration'])} iterations")
print(f"✓ Data consistency: {len(raw_history) == len(benchmark_data['mesh_iteration'])}")

# Example: Export to CSV
print("\nExample: Manual CSV export from single source")
print("iteration,error,points,intervals")
for i in range(len(benchmark_data["mesh_iteration"])):
    iteration = benchmark_data["mesh_iteration"][i]
    error = benchmark_data["estimated_error"][i]
    points = benchmark_data["collocation_points"][i]
    intervals = benchmark_data["mesh_intervals"][i]
    error_str = "NaN" if np.isnan(error) else f"{error:.6e}"
    print(f"{iteration},{error_str},{points},{intervals}")
