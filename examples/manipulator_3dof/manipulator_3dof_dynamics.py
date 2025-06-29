import sympy as sm
import sympy.physics.mechanics as me

from maptor.mechanics import lagrangian_to_maptor_dynamics


# ============================================================================
# Physical Parameters
# ============================================================================

# Link masses (kg)
m1, m2, m3 = sm.symbols("m1 m2 m3")

# Weight mass (kg)
m_box = sm.symbols("m_box")

# Link lengths (m)
l1, l2, l3 = sm.symbols("l1 l2 l3")

# Center of mass distances (m)
lc1, lc2, lc3 = sm.symbols("lc1 lc2 lc3")

# Moments of inertia about COM (kg⋅m²)
I1, I2, I3 = sm.symbols("I1 I2 I3")

# Gravity
g = sm.symbols("g")

# Joint torques (control inputs)
tau1, tau2, tau3 = sm.symbols("tau1 tau2 tau3")

# Joint coordinates and velocities
q1, q2, q3 = me.dynamicsymbols("q1 q2 q3")
q1d, q2d, q3d = me.dynamicsymbols("q1 q2 q3", 1)


# ============================================================================
# Reference Frames
# ============================================================================

# N: Inertial frame
N = me.ReferenceFrame("N")

# A: First link frame (rotation about Z - base rotation)
A = N.orientnew("A", "Axis", (q1, N.z))

# B: Second link frame (rotation about Y - shoulder pitch)
B = A.orientnew("B", "Axis", (q2, A.y))

# C: Third link frame (rotation about Y - elbow pitch)
C = B.orientnew("C", "Axis", (q3, B.y))


# ============================================================================
# Points and Velocities
# ============================================================================

# Fixed base origin
O = me.Point("O")
O.set_vel(N, 0)

# Joint 1 to Joint 2 (end of link 1)
P1 = O.locatenew("P1", l1 * A.z)
P1.v2pt_theory(O, N, A)

# Joint 2 to Joint 3 (end of link 2)
P2 = P1.locatenew("P2", l2 * B.x)
P2.v2pt_theory(P1, N, B)

# End effector (end of link 3)
P3 = P2.locatenew("P3", l3 * C.x)
P3.v2pt_theory(P2, N, C)

# Centers of mass
G1 = O.locatenew("G1", lc1 * A.z)
G1.v2pt_theory(O, N, A)

G2 = P1.locatenew("G2", lc2 * B.x)
G2.v2pt_theory(P1, N, B)

G3 = P2.locatenew("G3", lc3 * C.x)
G3.v2pt_theory(P2, N, C)


# ============================================================================
# Rigid Bodies
# ============================================================================

# Link 1: Vertical link (rotation about Z)
I1_dyadic = I1 * me.inertia(A, 0, 0, 1)
link1_body = me.RigidBody("link1", G1, A, m1, (I1_dyadic, G1))

# Link 2: Horizontal link (rotation about Y)
I2_dyadic = I2 * me.inertia(B, 1, 0, 0)
link2_body = me.RigidBody("link2", G2, B, m2, (I2_dyadic, G2))

# Link 3: Horizontal link (rotation about Y)
I3_dyadic = I3 * me.inertia(C, 1, 0, 0)
link3_body = me.RigidBody("link3", G3, C, m3, (I3_dyadic, G3))


# ============================================================================
# Forces (Passive Only)
# ============================================================================

# Gravitational forces on each center of mass
loads = [
    (G1, -m1 * g * N.z),  # Gravity on link 1
    (G2, -m2 * g * N.z),  # Gravity on link 2
    (G3, -m3 * g * N.z),  # Gravity on link 3
    (P3, -m_box * g * N.z),  # Gravity on box at end effector
]


# ============================================================================
# Lagrangian Mechanics
# ============================================================================

L = me.Lagrangian(N, link1_body, link2_body, link3_body)
LM = me.LagrangesMethod(L, [q1, q2, q3], forcelist=loads, frame=N)


# ============================================================================
# Control Forces
# ============================================================================

# Joint torques acting on each generalized coordinate
control_forces = sm.Matrix([tau1, tau2, tau3])


# ============================================================================
# Convert to MAPTOR Format
# ============================================================================

lagrangian_to_maptor_dynamics(LM, [q1, q2, q3], control_forces, "3dof_dynamics.txt")
