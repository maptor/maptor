import numpy as np
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d


ISP = 450.0
MU = 1.407645794e16
G0 = 32.174
T_THRUST = 4.446618e-3
RE = 20925662.73
J2 = 1082.639e-6
J3 = -2.565e-6
J4 = -1.608e-6

PTI = 21837080.052835
FTI = 0.0
GTI = 0.0
HTI = -0.25396764647494
KTI = 0.0
LTI = np.pi
WTI = 1.0

P_SCALE = 1e7
L_SCALE = np.pi
T_SCALE = 1e4


def generate_initial_guess():
    def legendre_polynomial(x, n):
        if n == 2:
            return 0.5 * (3.0 * x**2 - 1.0)
        elif n == 3:
            return 0.5 * (5.0 * x**3 - 3.0 * x)
        elif n == 4:
            return (1.0 / 8.0) * (35.0 * x**4 - 30.0 * x**2 + 3.0)
        else:
            raise ValueError(f"Legendre polynomial not implemented for n={n}")

    def legendre_polynomial_derivative(x, n):
        if n == 2:
            return 0.5 * (2.0 * 3.0 * x)
        elif n == 3:
            return 0.5 * (3.0 * 5.0 * x**2 - 3.0)
        elif n == 4:
            return (1.0 / 8.0) * (4.0 * 35.0 * x**3 - 2.0 * 30.0 * x)
        else:
            raise ValueError(f"Legendre polynomial derivative not implemented for n={n}")

    def cross_product(a, b):
        return np.array(
            [
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0],
            ]
        )

    def dot_product(a, b):
        return np.sum(a * b)

    def orbital_dynamics(t, state):
        p, f, g, h, k, L, w = state

        tau = -25.0

        eps = 1e-12
        p = max(p, eps)
        w = max(w, eps)

        q = 1.0 + f * np.cos(L) + g * np.sin(L)
        q = max(abs(q), eps) * np.sign(q) if q != 0 else eps
        r = p / q
        alpha2 = h * h - k * k
        X = np.sqrt(h * h + k * k)
        s2 = 1 + X * X

        r1 = r / s2 * (np.cos(L) + alpha2 * np.cos(L) + 2 * h * k * np.sin(L))
        r2 = r / s2 * (np.sin(L) - alpha2 * np.sin(L) + 2 * h * k * np.cos(L))
        r3 = 2 * r / s2 * (h * np.sin(L) - k * np.cos(L))
        rvec = np.array([r1, r2, r3])

        sqrt_mu_p = np.sqrt(MU / max(p, eps))
        v1 = (
            -(1.0 / s2)
            * sqrt_mu_p
            * (
                np.sin(L)
                + alpha2 * np.sin(L)
                - 2 * h * k * np.cos(L)
                + g
                - 2 * f * h * k
                + alpha2 * g
            )
        )
        v2 = (
            -(1.0 / s2)
            * sqrt_mu_p
            * (
                -np.cos(L)
                + alpha2 * np.cos(L)
                + 2 * h * k * np.sin(L)
                - f
                + 2 * g * h * k
                + alpha2 * f
            )
        )
        v3 = (2.0 / s2) * sqrt_mu_p * (h * np.cos(L) + k * np.sin(L) + f * h + g * k)
        vvec = np.array([v1, v2, v3])

        rv = cross_product(rvec, vvec)
        rvr = cross_product(rv, rvec)
        norm_r = np.sqrt(max(dot_product(rvec, rvec), eps))
        norm_rv = np.sqrt(max(dot_product(rv, rv), eps))

        ir = rvec / norm_r
        ith = rvr / (norm_rv * norm_r)
        ih = rv / norm_rv

        en = np.array([0.0, 0.0, 1.0])
        enir = dot_product(en, ir)
        in_vec = en - enir * ir
        norm_in = np.sqrt(max(dot_product(in_vec, in_vec), eps))
        in_normalized = in_vec / norm_in

        r_safe = max(r, RE / 100.0)
        sin_phi = rvec[2] / norm_r
        sin_phi = max(min(sin_phi, 1.0 - eps), -1.0 + eps)
        cos_phi = np.sqrt(1.0 - sin_phi**2)

        deltagn = 0.0
        deltagr = 0.0
        for j in [2, 3, 4]:
            J_coeff = [0, 0, J2, J3, J4][j]
            P_j = legendre_polynomial(sin_phi, j)
            Pdash_j = legendre_polynomial_derivative(sin_phi, j)
            deltagn += -MU * cos_phi / (r_safe * r_safe) * (RE / r_safe) ** j * Pdash_j * J_coeff
            deltagr += -MU / (r_safe * r_safe) * (j + 1) * (RE / r_safe) ** j * P_j * J_coeff

        delta_g = deltagn * in_normalized - deltagr * ir

        DELTA_g1 = dot_product(ir, delta_g)
        DELTA_g2 = dot_product(ith, delta_g)
        DELTA_g3 = dot_product(ih, delta_g)

        v_norm = np.sqrt(max(dot_product(vvec, vvec), eps))
        v_unit = vvec / v_norm
        u1 = dot_product(ir, v_unit)
        u2 = dot_product(ith, v_unit)
        u3 = dot_product(ih, v_unit)

        DELTA_T1 = G0 * T_THRUST * (1.0 + 0.01 * tau) * u1 / w
        DELTA_T2 = G0 * T_THRUST * (1.0 + 0.01 * tau) * u2 / w
        DELTA_T3 = G0 * T_THRUST * (1.0 + 0.01 * tau) * u3 / w

        delta1 = DELTA_g1 + DELTA_T1
        delta2 = DELTA_g2 + DELTA_T2
        delta3 = DELTA_g3 + DELTA_T3

        sqrt_p_mu = np.sqrt(max(p, eps) / MU)

        pdot = 2 * p / q * sqrt_p_mu * delta2
        fdot = (
            sqrt_p_mu * np.sin(L) * delta1
            + sqrt_p_mu * (1.0 / q) * ((q + 1.0) * np.cos(L) + f) * delta2
            - sqrt_p_mu * (g / q) * (h * np.sin(L) - k * np.cos(L)) * delta3
        )
        gdot = (
            -sqrt_p_mu * np.cos(L) * delta1
            + sqrt_p_mu * (1.0 / q) * ((q + 1.0) * np.sin(L) + g) * delta2
            + sqrt_p_mu * (f / q) * (h * np.sin(L) - k * np.cos(L)) * delta3
        )
        hdot = sqrt_p_mu * s2 * np.cos(L) / (2.0 * q) * delta3
        kdot = sqrt_p_mu * s2 * np.sin(L) / (2.0 * q) * delta3
        Ldot = (
            sqrt_p_mu * (1.0 / q) * (h * np.sin(L) - k * np.cos(L)) * delta3
            + np.sqrt(MU * max(p, eps)) * (q / max(p, eps)) ** 2
        )
        wdot = -T_THRUST * (1.0 + 0.01 * tau) / ISP

        return np.array([pdot, fdot, gdot, hdot, kdot, Ldot, wdot])

    state0 = np.array([PTI, FTI, GTI, HTI, KTI, LTI, WTI])
    t_final = 9e4

    print("Integrating orbital dynamics with advanced control law...")

    sol = solve_ivp(
        orbital_dynamics,
        [0, t_final],
        state0,
        method="DOP853",
        rtol=1e-8,
        atol=1e-10,
        dense_output=True,
    )

    if not sol.success:
        raise RuntimeError(f"ODE integration failed: {sol.message}")

    t_integration = sol.t
    states_integration = sol.y

    print(f"Integration completed: {len(t_integration)} time points")
    print(f"Final mass: {states_integration[6, -1]:.6f}")
    print(f"Final longitude: {states_integration[5, -1] / np.pi:.2f} π")

    mesh_intervals = 8
    polynomial_degrees = [8] * mesh_intervals

    states_guess = []
    controls_guess = []

    for interval_idx in range(mesh_intervals):
        N = polynomial_degrees[interval_idx]

        tau_interval = np.linspace(-1, 1, N + 1)
        t_interval_start = (interval_idx / mesh_intervals) * t_final
        t_interval_end = ((interval_idx + 1) / mesh_intervals) * t_final
        t_interval_physical = (tau_interval + 1) / 2 * (
            t_interval_end - t_interval_start
        ) + t_interval_start

        states_interpolated = np.zeros((7, N + 1))
        for state_idx in range(7):
            interp_func = interp1d(
                t_integration,
                states_integration[state_idx, :],
                kind="cubic",
                bounds_error=False,
                fill_value="extrapolate",  # type: ignore
            )
            states_interpolated[state_idx, :] = interp_func(t_interval_physical)

        p_s_vals = states_interpolated[0, :] / P_SCALE
        f_vals = states_interpolated[1, :]
        g_vals = states_interpolated[2, :]
        h_vals = states_interpolated[3, :]
        k_vals = states_interpolated[4, :]
        L_s_vals = states_interpolated[5, :] / L_SCALE
        w_vals = states_interpolated[6, :]

        states_guess.append(np.vstack([p_s_vals, f_vals, g_vals, h_vals, k_vals, L_s_vals, w_vals]))

        t_control_physical = (
            np.linspace(0, 1, N) * (t_interval_end - t_interval_start) + t_interval_start
        )

        controls_interpolated = np.zeros((3, N))
        for control_idx in range(N):
            t_ctrl = t_control_physical[control_idx]

            state_at_t = np.zeros(7)
            for state_idx in range(7):
                interp_func = interp1d(
                    t_integration,
                    states_integration[state_idx, :],
                    kind="cubic",
                    bounds_error=False,
                    fill_value="extrapolate",  # type: ignore
                )
                state_at_t[state_idx] = interp_func(t_ctrl)

            p, f, g, h, k, L, w = state_at_t
            eps = 1e-12
            q = 1.0 + f * np.cos(L) + g * np.sin(L)
            r = p / max(abs(q), eps)
            alpha2 = h * h - k * k
            X = np.sqrt(h * h + k * k)
            s2 = 1 + X * X

            sqrt_mu_p = np.sqrt(MU / max(p, eps))
            v1 = (
                -(1.0 / s2)
                * sqrt_mu_p
                * (
                    np.sin(L)
                    + alpha2 * np.sin(L)
                    - 2 * h * k * np.cos(L)
                    + g
                    - 2 * f * h * k
                    + alpha2 * g
                )
            )
            v2 = (
                -(1.0 / s2)
                * sqrt_mu_p
                * (
                    -np.cos(L)
                    + alpha2 * np.cos(L)
                    + 2 * h * k * np.sin(L)
                    - f
                    + 2 * g * h * k
                    + alpha2 * f
                )
            )
            v3 = (2.0 / s2) * sqrt_mu_p * (h * np.cos(L) + k * np.sin(L) + f * h + g * k)
            vvec = np.array([v1, v2, v3])

            r1 = r / s2 * (np.cos(L) + alpha2 * np.cos(L) + 2 * h * k * np.sin(L))
            r2 = r / s2 * (np.sin(L) - alpha2 * np.sin(L) + 2 * h * k * np.cos(L))
            r3 = 2 * r / s2 * (h * np.sin(L) - k * np.cos(L))
            rvec = np.array([r1, r2, r3])

            rv = cross_product(rvec, vvec)
            rvr = cross_product(rv, rvec)
            norm_r = np.sqrt(max(dot_product(rvec, rvec), eps))
            norm_rv = np.sqrt(max(dot_product(rv, rv), eps))

            ir = rvec / norm_r
            ith = rvr / (norm_rv * norm_r)
            ih = rv / norm_rv

            v_norm = np.sqrt(max(dot_product(vvec, vvec), eps))
            v_unit = vvec / v_norm
            u1 = dot_product(ir, v_unit)
            u2 = dot_product(ith, v_unit)
            u3 = dot_product(ih, v_unit)

            controls_interpolated[:, control_idx] = [u1, u2, u3]

        controls_guess.append(controls_interpolated)

    final_time_guess = t_final / T_SCALE

    print("Generated initial guess:")
    print(f"  Final time (scaled): {final_time_guess:.1f}")
    print(f"  Final mass: {states_guess[-1][6, -1]:.6f}")
    print(f"  Final longitude (scaled): {states_guess[-1][5, -1]:.2f}")

    return states_guess, controls_guess, final_time_guess
