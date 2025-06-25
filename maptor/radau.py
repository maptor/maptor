import functools
from dataclasses import dataclass, field
from typing import Literal, cast, overload

import numpy as np
from scipy.special import roots_jacobi as _scipy_roots_jacobi

from .input_validation import _validate_positive_integer
from .mtor_types import FloatArray
from .utils.constants import NUMERICAL_ZERO


@dataclass
class RadauBasisComponents:
    """Components for Radau pseudospectral method basis functions."""

    state_approximation_nodes: FloatArray = field(
        default_factory=lambda: np.array([], dtype=np.float64)
    )
    collocation_nodes: FloatArray = field(default_factory=lambda: np.array([], dtype=np.float64))
    quadrature_weights: FloatArray = field(default_factory=lambda: np.array([], dtype=np.float64))
    differentiation_matrix: FloatArray = field(
        default_factory=lambda: np.empty((0, 0), dtype=np.float64)
    )
    barycentric_weights_for_state_nodes: FloatArray = field(
        default_factory=lambda: np.array([], dtype=np.float64)
    )
    barycentric_weights_for_collocation_nodes: FloatArray = field(
        default_factory=lambda: np.array([], dtype=np.float64)
    )
    lagrange_at_tau_plus_one: FloatArray = field(
        default_factory=lambda: np.array([], dtype=np.float64)
    )


@dataclass
class RadauNodesAndWeights:
    state_approximation_nodes: FloatArray
    collocation_nodes: FloatArray
    quadrature_weights: FloatArray


@overload
def _roots_jacobi(
    n: int, alpha: float, beta: float, mu: Literal[False]
) -> tuple[FloatArray, FloatArray]: ...


@overload
def _roots_jacobi(
    n: int, alpha: float, beta: float, mu: Literal[True]
) -> tuple[FloatArray, FloatArray, float]: ...


def _roots_jacobi(
    n: int, alpha: float, beta: float, mu: bool = False
) -> tuple[FloatArray, FloatArray] | tuple[FloatArray, FloatArray, float]:
    if mu:
        result = _scipy_roots_jacobi(n, alpha, beta, mu=True)
        x_val = result[0]
        w_val = result[1]
        mu_val: float = result[2]
        return (
            x_val.astype(np.float64),
            w_val.astype(np.float64),
            float(mu_val),
        )
    else:
        result = _scipy_roots_jacobi(n, alpha, beta, mu=False)
        x_val = result[0]
        w_val = result[1]
        return (
            x_val.astype(np.float64),
            w_val.astype(np.float64),
        )


def _compute_legendre_gauss_radau_nodes_and_weights(
    num_collocation_nodes: int,
) -> RadauNodesAndWeights:
    collocation_nodes_list: list[float] = [-1.0]

    if num_collocation_nodes == 1:
        quadrature_weights_list: list[float] = [2.0]
    else:
        num_interior_roots = num_collocation_nodes - 1
        interior_roots, jacobi_weights, _ = _roots_jacobi(num_interior_roots, 0.0, 1.0, mu=True)
        interior_weights = jacobi_weights / (np.add(1.0, interior_roots))
        left_endpoint_weight = 2.0 / (num_collocation_nodes**2)
        collocation_nodes_list.extend(interior_roots.tolist())
        quadrature_weights_list = [left_endpoint_weight, *interior_weights.tolist()]

    final_collocation_nodes = np.array(collocation_nodes_list, dtype=np.float64)
    final_quadrature_weights = np.array(quadrature_weights_list, dtype=np.float64)

    state_approximation_nodes_temp = np.concatenate(
        [final_collocation_nodes, np.array([1.0], dtype=np.float64)]
    )
    state_approximation_nodes = np.unique(state_approximation_nodes_temp)

    return RadauNodesAndWeights(
        state_approximation_nodes=state_approximation_nodes,
        collocation_nodes=final_collocation_nodes,
        quadrature_weights=final_quadrature_weights,
    )


def _compute_barycentric_weights(nodes: FloatArray) -> FloatArray:
    num_nodes = len(nodes)
    if num_nodes == 1:
        return np.array([1.0], dtype=np.float64)

    nodes_col = nodes[:, np.newaxis]
    nodes_row = nodes[np.newaxis, :]
    differences_matrix = nodes_col - nodes_row

    diagonal_mask = np.eye(num_nodes, dtype=bool)

    near_zero_mask = np.abs(differences_matrix) < NUMERICAL_ZERO
    perturbation = np.sign(differences_matrix) * NUMERICAL_ZERO
    perturbation[perturbation == 0] = NUMERICAL_ZERO

    off_diagonal_near_zero = near_zero_mask & ~diagonal_mask
    differences_matrix = np.where(off_diagonal_near_zero, perturbation, differences_matrix)

    differences_matrix[diagonal_mask] = 1.0

    products = np.prod(differences_matrix, axis=1, dtype=np.float64)

    small_product_mask = np.abs(products) < NUMERICAL_ZERO**2
    safe_products = np.where(
        small_product_mask,
        np.where(products == 0, 1.0 / (NUMERICAL_ZERO**2), np.sign(products) / (NUMERICAL_ZERO**2)),
        1.0 / products,
    )

    return safe_products.astype(np.float64)


def _evaluate_lagrange_polynomial_at_point(
    polynomial_definition_nodes: FloatArray,
    barycentric_weights: FloatArray,
    evaluation_point_tau: float,
) -> FloatArray:
    num_nodes = len(polynomial_definition_nodes)

    differences = np.abs(evaluation_point_tau - polynomial_definition_nodes)
    coincident_mask = differences < NUMERICAL_ZERO

    if np.any(coincident_mask):
        lagrange_values = np.zeros(num_nodes, dtype=np.float64)
        lagrange_values[coincident_mask] = 1.0
        return lagrange_values

    diffs = evaluation_point_tau - polynomial_definition_nodes

    near_zero_mask = np.abs(diffs) < NUMERICAL_ZERO
    safe_diffs = np.where(
        near_zero_mask, np.where(diffs == 0, NUMERICAL_ZERO, np.sign(diffs) * NUMERICAL_ZERO), diffs
    )

    terms = barycentric_weights / safe_diffs
    sum_terms = np.sum(terms)

    if abs(sum_terms) < NUMERICAL_ZERO:
        return np.zeros(num_nodes, dtype=np.float64)

    normalized_terms = terms / sum_terms
    return cast(FloatArray, normalized_terms)


def _compute_lagrange_derivative_coefficients_at_point(
    polynomial_definition_nodes: FloatArray,
    barycentric_weights: FloatArray,
    evaluation_point_tau: float,
) -> FloatArray:
    num_nodes = len(polynomial_definition_nodes)

    differences = np.abs(evaluation_point_tau - polynomial_definition_nodes)
    matched_indices = np.where(differences < NUMERICAL_ZERO)[0]

    if len(matched_indices) == 0:
        return np.zeros(num_nodes, dtype=np.float64)

    matched_node_idx_k = matched_indices[0]
    derivatives = np.zeros(num_nodes, dtype=np.float64)

    node_diffs = polynomial_definition_nodes[matched_node_idx_k] - polynomial_definition_nodes

    near_zero_mask = np.abs(node_diffs) < NUMERICAL_ZERO
    safe_diffs = np.where(
        near_zero_mask,
        np.where(node_diffs == 0, NUMERICAL_ZERO, np.sign(node_diffs) * NUMERICAL_ZERO),
        node_diffs,
    )

    non_diagonal_mask = np.arange(num_nodes) != matched_node_idx_k

    if abs(barycentric_weights[matched_node_idx_k]) < NUMERICAL_ZERO:
        derivatives[non_diagonal_mask] = 0.0
    else:
        weight_ratios = barycentric_weights / barycentric_weights[matched_node_idx_k]
        derivatives[non_diagonal_mask] = (
            weight_ratios[non_diagonal_mask] / safe_diffs[non_diagonal_mask]
        )

    derivatives[matched_node_idx_k] = np.sum(1.0 / safe_diffs[non_diagonal_mask])

    return derivatives


@functools.lru_cache(maxsize=32)
def _compute_radau_collocation_components(
    num_collocation_nodes: int,
) -> RadauBasisComponents:
    _validate_positive_integer(num_collocation_nodes, "collocation nodes")

    lgr_data = _compute_legendre_gauss_radau_nodes_and_weights(num_collocation_nodes)

    state_nodes = lgr_data.state_approximation_nodes
    collocation_nodes = lgr_data.collocation_nodes
    quadrature_weights = lgr_data.quadrature_weights

    num_state_nodes = len(state_nodes)
    num_actual_collocation_nodes = len(collocation_nodes)

    bary_weights_state_nodes = _compute_barycentric_weights(state_nodes)
    bary_weights_collocation_nodes = _compute_barycentric_weights(collocation_nodes)

    diff_matrix = np.zeros((num_actual_collocation_nodes, num_state_nodes), dtype=np.float64)
    for i in range(num_actual_collocation_nodes):
        tau_c_i = collocation_nodes[i]
        diff_matrix[i, :] = _compute_lagrange_derivative_coefficients_at_point(
            state_nodes, bary_weights_state_nodes, tau_c_i
        )

    lagrange_at_tau_plus_one = _evaluate_lagrange_polynomial_at_point(
        state_nodes, bary_weights_state_nodes, 1.0
    )

    return RadauBasisComponents(
        state_approximation_nodes=state_nodes,
        collocation_nodes=collocation_nodes,
        quadrature_weights=quadrature_weights,
        differentiation_matrix=diff_matrix,
        barycentric_weights_for_state_nodes=bary_weights_state_nodes,
        barycentric_weights_for_collocation_nodes=bary_weights_collocation_nodes,
        lagrange_at_tau_plus_one=lagrange_at_tau_plus_one,
    )


def _evaluate_lagrange_interpolation_at_points(
    nodes: FloatArray,
    barycentric_weights: FloatArray,
    values: FloatArray,
    eval_points: float | FloatArray,
) -> FloatArray:
    is_scalar = np.isscalar(eval_points)
    eval_array = np.atleast_1d(eval_points)
    values_2d = np.atleast_2d(values)
    num_vars = values_2d.shape[0]

    result = np.zeros((num_vars, len(eval_array)), dtype=np.float64)

    for i, zeta in enumerate(eval_array):
        L_j = _evaluate_lagrange_polynomial_at_point(nodes, barycentric_weights, zeta)
        result[:, i] = np.dot(values_2d, L_j)

    return result[:, 0] if is_scalar else result
