import numpy as np
import pytest
from numpy.testing import assert_allclose

from maptor.radau import (
    _compute_barycentric_weights,
    _compute_radau_collocation_components,
    _evaluate_lagrange_polynomial_at_point,
)


class TestRadauMathematicalCorrectness:
    @pytest.mark.parametrize("N", [1, 2, 3, 4, 5])
    def test_quadrature_exactness(self, N):
        components = _compute_radau_collocation_components(N)
        nodes = components.collocation_nodes
        weights = components.quadrature_weights

        for degree in range(2 * N - 1):
            poly_values = nodes**degree
            radau_integral = np.sum(weights * poly_values)

            exact_integral = 0.0 if degree % 2 == 1 else 2.0 / (degree + 1)

            assert abs(radau_integral - exact_integral) < 1e-14

    @pytest.mark.parametrize("N", [2, 3, 4, 5])
    def test_differentiation_matrix_accuracy(self, N):
        components = _compute_radau_collocation_components(N)
        state_nodes = components.state_approximation_nodes
        colloc_nodes = components.collocation_nodes
        diff_matrix = components.differentiation_matrix

        test_polynomials = [
            (lambda x: np.ones_like(x), lambda x: np.zeros_like(x)),
            (lambda x: x, lambda x: np.ones_like(x)),
        ]

        if N >= 2:
            test_polynomials.append((lambda x: x**2, lambda x: 2 * x))
        if N >= 3:
            test_polynomials.append((lambda x: x**3, lambda x: 3 * x**2))
        if N >= 4:
            test_polynomials.append((lambda x: x**4, lambda x: 4 * x**3))

        for func, dfunc in test_polynomials:
            func_values = func(state_nodes)
            computed_derivatives = diff_matrix @ func_values
            exact_derivatives = dfunc(colloc_nodes)

            max_error = np.max(np.abs(computed_derivatives - exact_derivatives))
            assert max_error < 1e-12

    def test_lagrange_interpolation_exactness(self):
        nodes = np.array([-1.0, -0.5, 0.0, 0.5, 1.0])
        weights = _compute_barycentric_weights(nodes)

        test_functions = [
            lambda x: 1.0,
            lambda x: x,
            lambda x: x**2,
            lambda x: x**3,
            lambda x: x**4,
        ]

        test_points = np.linspace(-0.9, 0.9, 20)

        for func in test_functions:
            func_values = np.array([func(x) for x in nodes])

            for tau in test_points:
                lagrange_coeffs = _evaluate_lagrange_polynomial_at_point(nodes, weights, tau)
                interpolated_value = np.dot(lagrange_coeffs, func_values)
                exact_value = func(tau)

                assert abs(interpolated_value - exact_value) < 1e-12

    def test_partition_of_unity(self):
        test_node_sets = [
            np.array([-1.0, 1.0]),
            np.array([-1.0, 0.0, 1.0]),
            np.array([-1.0, -0.5, 0.5, 1.0]),
        ]

        for nodes in test_node_sets:
            weights = _compute_barycentric_weights(nodes)
            test_points = np.linspace(-0.9, 0.9, 15)

            for tau in test_points:
                lagrange_vals = _evaluate_lagrange_polynomial_at_point(nodes, weights, tau)
                partition_sum = np.sum(lagrange_vals)
                assert abs(partition_sum - 1.0) < 1e-12

    def test_cache_consistency(self):
        N = 5
        comp1 = _compute_radau_collocation_components(N)
        comp2 = _compute_radau_collocation_components(N)

        assert comp1 is comp2
        assert_allclose(comp1.collocation_nodes, comp2.collocation_nodes, rtol=1e-15)
        assert_allclose(comp1.quadrature_weights, comp2.quadrature_weights, rtol=1e-15)
        assert_allclose(comp1.differentiation_matrix, comp2.differentiation_matrix, rtol=1e-15)

    def test_numerical_stability(self):
        close_nodes = np.array([-1.0, -0.999999999, 1.0])
        weights = _compute_barycentric_weights(close_nodes)

        assert np.all(np.isfinite(weights))

        test_point = -0.9999999999
        lagrange_vals = _evaluate_lagrange_polynomial_at_point(close_nodes, weights, test_point)

        assert np.all(np.isfinite(lagrange_vals))
        assert abs(np.sum(lagrange_vals) - 1.0) < 1e-10
