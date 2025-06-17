# Low-Thrust Orbit Transfer

## Mathematical Formulation

### Problem Statement

Find the optimal thrust direction $\mathbf{u}(t) = [u_1, u_2, u_3]^T$ and thrust efficiency parameter $\tau$ that maximize the final spacecraft mass:

$$J = -w(t_f)$$ (maximizing mass by minimizing negative mass)

Subject to the Gauss variational equations in modified equinoctial elements:

$$\frac{dp}{dt} = \frac{2p}{q}\sqrt{\frac{p}{\mu}} \delta_2$$

$$\frac{df}{dt} = \sqrt{\frac{p}{\mu}} \left[ \sin L \cdot \delta_1 + \frac{1}{q}((q+1)\cos L + f) \delta_2 - \frac{g}{q}(h \sin L - k \cos L) \delta_3 \right]$$

$$\frac{dg}{dt} = \sqrt{\frac{p}{\mu}} \left[ -\cos L \cdot \delta_1 + \frac{1}{q}((q+1)\sin L + g) \delta_2 + \frac{f}{q}(h \sin L - k \cos L) \delta_3 \right]$$

$$\frac{dh}{dt} = \sqrt{\frac{p}{\mu}} \frac{s^2 \cos L}{2q} \delta_3$$

$$\frac{dk}{dt} = \sqrt{\frac{p}{\mu}} \frac{s^2 \sin L}{2q} \delta_3$$

$$\frac{dL}{dt} = \sqrt{\frac{\mu p}{p^3}} q^2 + \sqrt{\frac{p}{\mu}} \frac{1}{q}(h \sin L - k \cos L) \delta_3$$

$$\frac{dw}{dt} = -\frac{T(1 + 0.01\tau)}{I_{sp}}$$

Where:
- $q = 1 + f \cos L + g \sin L$
- $s^2 = 1 + h^2 + k^2$
- $\delta_i = \Delta g_i + \Delta T_i$ (gravitational + thrust accelerations)

### Thrust and Gravitational Accelerations

**Thrust accelerations**:
$$\Delta T_1 = \frac{g_0 T (1 + 0.01\tau)}{w} u_1$$
$$\Delta T_2 = \frac{g_0 T (1 + 0.01\tau)}{w} u_2$$
$$\Delta T_3 = \frac{g_0 T (1 + 0.01\tau)}{w} u_3$$

**Gravitational perturbations** (J2, J3, J4):
$$\Delta g_1, \Delta g_2, \Delta g_3$$ computed from Earth's gravitational harmonics

### Terminal Constraints

$$p(t_f) = p_{tf}$$
$$\sqrt{f^2(t_f) + g^2(t_f)} = e_{tf}$$
$$\sqrt{h^2(t_f) + k^2(t_f)} = \tan(i_{tf}/2)$$
$$f(t_f) h(t_f) + g(t_f) k(t_f) = 0$$
$$-10 \leq g(t_f) h(t_f) - k(t_f) f(t_f) \leq 0$$

### Boundary Conditions

- **Initial conditions**: $p(0) = 21837080.052835$ ft, $f(0) = 0$, $g(0) = 0$, $h(0) = -0.25396764647494$, $k(0) = 0$, $L(0) = \pi$, $w(0) = 1$
- **Final conditions**: Determined by terminal constraints
- **Control bounds**: $u_1^2 + u_2^2 + u_3^2 = 1$ (unit thrust vector)
- **Parameter bounds**: $-50 \leq \tau \leq 0$

### Physical Parameters

- Specific impulse: $I_{sp} = 450$ s
- Earth gravitational parameter: $\mu = 1.407645794 \times 10^{16}$ ft³/s²
- Standard gravity: $g_0 = 32.174$ ft/s²
- Thrust magnitude: $T = 4.446618 \times 10^{-3}$ lbf
- Earth radius: $R_E = 20925662.73$ ft
- Gravitational harmonics: $J_2 = 1082.639 \times 10^{-6}$, $J_3 = -2.565 \times 10^{-6}$, $J_4 = -1.608 \times 10^{-6}$

### State Variables

- $p(t)$: Semi-latus rectum (ft)
- $f(t), g(t)$: Eccentricity vector components
- $h(t), k(t)$: Inclination vector components
- $L(t)$: True longitude (rad)
- $w(t)$: Spacecraft mass (normalized)

### Control Variables

- $u_1(t), u_2(t), u_3(t)$: Thrust direction unit vector components

### Static Parameter

- $\tau$: Thrust efficiency parameter

### Notes

This problem uses modified equinoctial elements to avoid singularities in classical orbital elements and includes Earth's gravitational harmonics (J2, J3, J4) for high-fidelity orbital mechanics.

## Running This Example

```bash
cd examples/low_thrust_orbit_transfer
python low_thrust_orbit_transfer.py
```
