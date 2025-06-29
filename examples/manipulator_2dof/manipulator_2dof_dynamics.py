import sympy as sm
import sympy.physics.mechanics as me

from maptor.mechanics import lagrangian_to_maptor_dynamics


# ============================================================================
# Physical Parameters
# ============================================================================

m1, m2 = sm.symbols("m1 m2")
m_payload = sm.symbols("m_payload")
l1, l2 = sm.symbols("l1 l2")
lc1, lc2 = sm.symbols("lc1 lc2")
I1, I2 = sm.symbols("I1 I2")
g = sm.symbols("g")
tau1, tau2 = sm.symbols("tau1 tau2")

q1, q2 = me.dynamicsymbols("q1 q2")
q1d, q2d = me.dynamicsymbols("q1 q2", 1)


# ============================================================================
# Reference Frames
# ============================================================================

N = me.ReferenceFrame("N")
A = N.orientnew("A", "Axis", (q1, N.z))
B = A.orientnew("B", "Axis", (q2, A.z))


# ============================================================================
# Points and Velocities
# ============================================================================

O = me.Point("O")
O.set_vel(N, 0)

P1 = O.locatenew("P1", l1 * A.x)
P1.v2pt_theory(O, N, A)

G1 = O.locatenew("G1", lc1 * A.x)
G1.v2pt_theory(O, N, A)

G2 = P1.locatenew("G2", lc2 * B.x)
G2.v2pt_theory(P1, N, B)

P2 = P1.locatenew("P2", l2 * B.x)
P2.v2pt_theory(P1, N, B)


# ============================================================================
# Rigid Bodies
# ============================================================================

I1_dyadic = I1 * me.inertia(A, 0, 0, 1)
link1_body = me.RigidBody("link1", G1, A, m1, (I1_dyadic, G1))

I2_dyadic = I2 * me.inertia(B, 0, 0, 1)
link2_body = me.RigidBody("link2", G2, B, m2, (I2_dyadic, G2))


# ============================================================================
# Forces (Passive Only)
# ============================================================================

loads = [
    (G1, -m1 * g * N.y),  # Gravity on link 1 COM
    (G2, -m2 * g * N.y),  # Gravity on link 2 COM
    (P2, -m_payload * g * N.y),  # Gravity on payload at end effector
]


# ============================================================================
# Lagrangian Mechanics
# ============================================================================

L = me.Lagrangian(N, link1_body, link2_body)
LM = me.LagrangesMethod(L, [q1, q2], forcelist=loads, frame=N)


# ============================================================================
# Control Forces
# ============================================================================

# Joint torques acting on each generalized coordinate
control_forces = sm.Matrix([tau1, tau2])


# ============================================================================
# Convert to MAPTOR Format
# ============================================================================

lagrangian_to_maptor_dynamics(LM, [q1, q2], control_forces, "2dof_dynamics.txt")

"""
Output ready to be copy-pasted:

State variables:
q1 = phase.state('q1')
q2 = phase.state('q2')
q1_dot = phase.state('q1_dot')
q2_dot = phase.state('q2_dot')

Control variables:
tau1 = phase.control('tau1')
tau2 = phase.control('tau2')

MAPTOR dynamics dictionary:
phase.dynamics(
    {
        q1: q1_dot,
        q2: q2_dot,
        q1_dot: (
            (I2 + lc2**2 * m2)
            * (
                -g * l1 * m2 * ca.cos(q1)
                - g * l1 * m_payload * ca.cos(q1)
                - g * l2 * m_payload * ca.cos(q1 + q2)
                - g * lc1 * m1 * ca.cos(q1)
                - g * lc2 * m2 * ca.cos(q1 + q2)
                + l1 * lc2 * m2 * (2 * q1_dot + q2_dot) * ca.sin(q2) * q2_dot
                + tau1
            )
            - (I2 + l1 * lc2 * m2 * ca.cos(q2) + lc2**2 * m2)
            * (
                -g * l2 * m_payload * ca.cos(q1 + q2)
                - g * lc2 * m2 * ca.cos(q1 + q2)
                - l1 * lc2 * m2 * ca.sin(q2) * q1_dot**2
                + tau2
            )
        )
        / (
            I1 * I2
            + I1 * lc2**2 * m2
            + I2 * l1**2 * m2
            + I2 * lc1**2 * m1
            + l1**2 * lc2**2 * m2**2 * ca.sin(q2) ** 2
            + lc1**2 * lc2**2 * m1 * m2
        ),
        q2_dot: (
            -(I2 + l1 * lc2 * m2 * ca.cos(q2) + lc2**2 * m2)
            * (
                -g * l1 * m2 * ca.cos(q1)
                - g * l1 * m_payload * ca.cos(q1)
                - g * l2 * m_payload * ca.cos(q1 + q2)
                - g * lc1 * m1 * ca.cos(q1)
                - g * lc2 * m2 * ca.cos(q1 + q2)
                + l1 * lc2 * m2 * (2 * q1_dot + q2_dot) * ca.sin(q2) * q2_dot
                + tau1
            )
            + (
                -g * l2 * m_payload * ca.cos(q1 + q2)
                - g * lc2 * m2 * ca.cos(q1 + q2)
                - l1 * lc2 * m2 * ca.sin(q2) * q1_dot**2
                + tau2
            )
            * (I1 + I2 + l1**2 * m2 + 2 * l1 * lc2 * m2 * ca.cos(q2) + lc1**2 * m1 + lc2**2 * m2)
        )
        / (
            I1 * I2
            + I1 * lc2**2 * m2
            + I2 * l1**2 * m2
            + I2 * lc1**2 * m1
            + l1**2 * lc2**2 * m2**2 * ca.sin(q2) ** 2
            + lc1**2 * lc2**2 * m1 * m2
        ),
    }
)
"""
