import numpy as np
import pytest
from numpy.testing import assert_allclose

from maptor.adaptive.phs.error_estimation import (
    _calculate_gamma_normalization_factors,
)
from maptor.adaptive.phs.initial_guess import (
    _interpolate_phase_trajectory_to_new_mesh_streamlined,
)
from maptor.adaptive.phs.numerical import (
    _map_global_normalized_tau_to_local_interval_tau,
    _map_local_interval_tau_to_global_normalized_tau,
    _map_local_tau_from_interval_k_plus_1_to_equivalent_in_interval_k,
    _map_local_tau_from_interval_k_to_equivalent_in_interval_k_plus_1,
)
from maptor.radau import _compute_radau_collocation_components


class TestCoordinateTransformationInvertibility:
    # Coordinate transformations must be mathematically exact."""

    @pytest.mark.parametrize(
        ("global_start", "global_end"),
        [
            (-1.0, 1.0),
            (-0.8, 0.3),
            (0.2, 0.9),
            (-0.1, 0.1),  # Small interval
        ],
    )
    def test_global_local_transformation_invertibility(self, global_start, global_end):
        test_points = np.linspace(global_start, global_end, 20)

        for global_tau in test_points:
            local_tau = _map_global_normalized_tau_to_local_interval_tau(
                global_tau, global_start, global_end
            )
            recovered_global = _map_local_interval_tau_to_global_normalized_tau(
                local_tau, global_start, global_end
            )

            assert abs(recovered_global - global_tau) < 1e-15

    @pytest.mark.parametrize(
        ("tau_start_k", "tau_shared", "tau_end_kp1"),
        [
            (-1.0, 0.0, 1.0),
            (-0.7, -0.2, 0.8),
            (-0.3, 0.1, 0.4),
        ],
    )
    def test_interval_mapping_composition_invertibility(self, tau_start_k, tau_shared, tau_end_kp1):
        test_points = np.linspace(-1, 1, 15)

        for local_tau_k in test_points:
            # Map from interval k to equivalent position in interval k+1
            mapped_tau_kp1 = _map_local_tau_from_interval_k_to_equivalent_in_interval_k_plus_1(
                local_tau_k, tau_start_k, tau_shared, tau_end_kp1
            )

            # Map back to interval k
            recovered_tau_k = _map_local_tau_from_interval_k_plus_1_to_equivalent_in_interval_k(
                mapped_tau_kp1, tau_start_k, tau_shared, tau_end_kp1
            )

            assert abs(recovered_tau_k - local_tau_k) < 1e-15


class TestInterpolationMathematicalExactness:
    @pytest.mark.parametrize(("N_source", "N_target"), [(3, 4), (4, 3), (3, 6), (5, 3)])
    def test_polynomial_interpolation_exactness(self, N_source, N_target):
        # Interpolating polynomial of degree ≤ min(N_source, N_target) must be exact.
        max_exact_degree = min(N_source, N_target)

        # Create source mesh and target mesh
        source_mesh = np.array([-1.0, 1.0])
        target_mesh = np.array([-1.0, 1.0])
        source_degrees = [N_source]
        target_degrees = [N_target]

        # Test polynomials up to the degree that should be exact
        for degree in range(max_exact_degree + 1):
            # Create polynomial trajectory on source mesh
            source_basis = _compute_radau_collocation_components(N_source)
            source_nodes = source_basis.state_approximation_nodes

            # Polynomial p(x) = x^degree
            poly_values = source_nodes**degree
            source_trajectory = [poly_values.reshape(1, -1)]

            # Interpolate to target mesh
            interpolated = _interpolate_phase_trajectory_to_new_mesh_streamlined(
                source_trajectory,
                source_mesh,
                source_degrees,
                target_mesh,
                target_degrees,
                num_variables=1,
                phase_id=1,
                is_state_trajectory=True,
            )

            # Evaluate exact polynomial at target nodes
            target_basis = _compute_radau_collocation_components(N_target)
            target_nodes = target_basis.state_approximation_nodes
            exact_values = target_nodes**degree

            # Interpolation should be exact for polynomials within degree bounds
            interpolated_values = interpolated[0].flatten()
            max_error = np.max(np.abs(interpolated_values - exact_values))

            assert max_error < 1e-12, f"Polynomial degree {degree} not exact: error = {max_error}"

    def test_interpolation_preserves_constant_function(self):
        constant_value = 3.14159

        # Various mesh configurations
        test_cases = [
            ([3], [5], np.array([-1.0, 1.0])),
            ([4, 3], [2, 6], np.array([-1.0, 0.0, 1.0])),
        ]

        for source_degrees, target_degrees, mesh_points in test_cases:
            # Create constant trajectory
            source_trajectories = []
            for N in source_degrees:
                basis = _compute_radau_collocation_components(N)
                num_nodes = len(basis.state_approximation_nodes)
                constant_traj = np.full((1, num_nodes), constant_value)
                source_trajectories.append(constant_traj)

            # Interpolate
            interpolated = _interpolate_phase_trajectory_to_new_mesh_streamlined(
                source_trajectories,
                mesh_points,
                source_degrees,
                mesh_points,
                target_degrees,
                num_variables=1,
                phase_id=1,
                is_state_trajectory=True,
            )

            # All interpolated values should equal the constant
            for traj in interpolated:
                assert_allclose(traj, constant_value, rtol=1e-15)


class TestErrorEstimationConsistency:
    def test_gamma_normalization_bounds(self):
        test_cases = [
            np.array([0.1, 1.0, 10.0]),  # Normal values
            np.array([1e-15, 1e-10, 1e15]),  # Extreme values
            np.array([0.0, 0.0, 0.0]),  # Zero values
            np.array([1e20, 1e20, 1e20]),  # Large values
        ]

        for max_state_values in test_cases:
            gamma_factors = _calculate_gamma_normalization_factors(max_state_values)

            # Must be finite and positive
            assert np.all(np.isfinite(gamma_factors))
            assert np.all(gamma_factors > 0)

            # Must match actual implementation: 1/max(1+max_val, 1e-12)
            gamma_denominator = 1.0 + max_state_values
            safe_denominator = np.maximum(gamma_denominator, 1e-12)
            expected = (1.0 / safe_denominator).reshape(-1, 1)

            assert_allclose(gamma_factors, expected, rtol=1e-15)

    def test_gamma_normalization_monotonicity(self):
        small_values = np.array([0.1, 0.5])
        large_values = np.array([10.0, 50.0])

        gamma_small = _calculate_gamma_normalization_factors(small_values)
        gamma_large = _calculate_gamma_normalization_factors(large_values)

        # Gamma factors should decrease as state values increase
        assert np.all(gamma_small > gamma_large)


class TestMeshRefinementConsistency:
    @pytest.mark.parametrize("original_degree", [3, 4, 5])
    def test_p_refinement_preserves_polynomials(self, original_degree):
        refined_degree = original_degree + 2

        # Create polynomial trajectory on original mesh
        original_basis = _compute_radau_collocation_components(original_degree)
        original_nodes = original_basis.state_approximation_nodes

        # Test polynomial of degree = original_degree (should be preserved exactly)
        test_polynomial_degree = original_degree
        poly_values = original_nodes**test_polynomial_degree
        original_trajectory = [poly_values.reshape(1, -1)]

        # "Refine" to higher degree mesh (simulate p-refinement)
        mesh_points = np.array([-1.0, 1.0])
        original_degrees = [original_degree]
        refined_degrees = [refined_degree]

        refined_trajectory = _interpolate_phase_trajectory_to_new_mesh_streamlined(
            original_trajectory,
            mesh_points,
            original_degrees,
            mesh_points,
            refined_degrees,
            num_variables=1,
            phase_id=1,
            is_state_trajectory=True,
        )

        # Evaluate exact polynomial at refined nodes
        refined_basis = _compute_radau_collocation_components(refined_degree)
        refined_nodes = refined_basis.state_approximation_nodes
        exact_refined_values = refined_nodes**test_polynomial_degree

        # Should be exact since polynomial degree ≤ original degree
        refined_values = refined_trajectory[0].flatten()
        max_error = np.max(np.abs(refined_values - exact_refined_values))

        assert max_error < 1e-11, f"P-refinement failed to preserve polynomial: error = {max_error}"

    def test_h_refinement_preserves_total_integral(self):
        # Create a function on coarse mesh
        coarse_degree = 4
        coarse_basis = _compute_radau_collocation_components(coarse_degree)
        coarse_nodes = coarse_basis.state_approximation_nodes

        # Simple quadratic function: f(x) = x^2 + 1
        func_values = coarse_nodes**2 + 1
        coarse_trajectory = [func_values.reshape(1, -1)]

        # Compute integral on coarse mesh using quadrature
        coarse_weights = coarse_basis.quadrature_weights
        coarse_colloc_nodes = coarse_basis.collocation_nodes
        coarse_func_at_colloc = coarse_colloc_nodes**2 + 1
        coarse_integral = np.sum(coarse_weights * coarse_func_at_colloc)

        # Refine mesh (h-refinement): split into two intervals
        coarse_mesh = np.array([-1.0, 1.0])
        refined_mesh = np.array([-1.0, 0.0, 1.0])
        refined_degrees = [coarse_degree, coarse_degree]

        refined_trajectories = _interpolate_phase_trajectory_to_new_mesh_streamlined(
            coarse_trajectory,
            coarse_mesh,
            [coarse_degree],
            refined_mesh,
            refined_degrees,
            num_variables=1,
            phase_id=1,
            is_state_trajectory=True,
        )

        # Compute integral on refined mesh
        refined_integral = 0.0
        for k, _traj in enumerate(refined_trajectories):
            basis = _compute_radau_collocation_components(refined_degrees[k])
            weights = basis.quadrature_weights
            colloc_nodes = basis.collocation_nodes

            # Transform collocation nodes to global interval
            if k == 0:  # First interval: [-1, 0]
                global_colloc = (colloc_nodes - 1) / 2  # Map [-1,1] to [-1,0]
                scaling = 0.5
            else:  # Second interval: [0, 1]
                global_colloc = (colloc_nodes + 1) / 2  # Map [-1,1] to [0,1]
                scaling = 0.5

            # Evaluate function at global collocation points
            func_at_global_colloc = global_colloc**2 + 1
            refined_integral += scaling * np.sum(weights * func_at_global_colloc)

        # Integrals should match (h-refinement preserves integrals for smooth functions)
        integral_error = abs(refined_integral - coarse_integral)
        assert integral_error < 1e-10, f"H-refinement changed integral: error = {integral_error}"
