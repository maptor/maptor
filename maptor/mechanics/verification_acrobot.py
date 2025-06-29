import sympy as sm
import sympy.physics.mechanics as me

from maptor.mechanics import lagrangian_to_maptor_dynamics


# === Define Symbols ===
# Link masses (kg)
m1, m2 = sm.symbols("m1 m2")

# Link lengths (m)
l1, l2 = sm.symbols("l1 l2")

# Center of mass distances (m)
lc1, lc2 = sm.symbols("lc1 lc2")

# Moments of inertia about pivots (kg⋅m²)
I1, I2 = sm.symbols("I1 I2")

# Gravity
g = sm.symbols("g")

# Control torque (only on second joint)
tau = sm.symbols("tau")

# Joint coordinates: theta1 (shoulder from vertical), theta2 (elbow relative)
theta1, theta2 = me.dynamicsymbols("theta1 theta2")
theta1d, theta2d = me.dynamicsymbols("theta1 theta2", 1)


# === Reference Frames ===
# N: Inertial frame (Y-axis points up, X-axis points right)
N = me.ReferenceFrame("N")

# A: Link 1 frame (rotated by theta1 from vertical)
# Positive theta1 rotates counterclockwise from downward vertical (-Y direction)
A = N.orientnew("A", "Axis", (theta1, N.z))

# B: Link 2 frame (rotated by theta2 relative to link 1)
# Positive theta2 rotates counterclockwise relative to link 1
B = A.orientnew("B", "Axis", (theta2, A.z))


# === Points and Velocities ===
# Fixed shoulder joint (origin)
O = me.Point("O")
O.set_vel(N, 0)

# Elbow joint (end of link 1)
# Link 1 extends in -A.y direction (downward in link 1 frame)
P1 = O.locatenew("P1", l1 * (-A.y))
P1.v2pt_theory(O, N, A)

# End effector (end of link 2)
# Link 2 extends in -B.y direction (downward in link 2 frame)
P2 = P1.locatenew("P2", l2 * (-B.y))
P2.v2pt_theory(P1, N, B)

# Centers of mass
# Link 1 COM: distance lc1 from shoulder along link 1
G1 = O.locatenew("G1", lc1 * (-A.y))
G1.v2pt_theory(O, N, A)

# Link 2 COM: distance lc2 from elbow along link 2
G2 = P1.locatenew("G2", lc2 * (-B.y))
G2.v2pt_theory(P1, N, B)


# === Rigid Bodies ===
# Link 1: Inertia about shoulder pivot
I1_dyadic = I1 * me.inertia(A, 0, 0, 1)
link1_body = me.RigidBody("link1", G1, A, m1, (I1_dyadic, O))  # Inertia about pivot O

# Link 2: Inertia about elbow pivot
I2_dyadic = I2 * me.inertia(B, 0, 0, 1)
link2_body = me.RigidBody("link2", G2, B, m2, (I2_dyadic, P1))  # Inertia about pivot P1


# === Forces (Passive Only) ===
# Gravitational forces on centers of mass (gravity acts in -N.y direction)
loads = [
    (G1, -m1 * g * N.y),  # Gravity on link 1 COM
    (G2, -m2 * g * N.y),  # Gravity on link 2 COM
]


# === Lagrangian Mechanics ===
L = me.Lagrangian(N, link1_body, link2_body)
LM = me.LagrangesMethod(L, [theta1, theta2], forcelist=loads, frame=N)


# === Control Forces ===
# Acrobot: only second joint (elbow) is actuated
# No torque on theta1 (shoulder), torque tau on theta2 (elbow)
control_forces = sm.Matrix([0, tau])


# === Term-by-Term Verification ===
print("=== ACROBOT TERM-BY-TERM VERIFICATION ===")
print("Literature reference: https://underactuated.csail.mit.edu/acrobot.html#section1")
print()

# Form the equations to access components
LM.form_lagranges_equations()

# Extract mass matrix M(q)
M = LM.mass_matrix
print("MASS MATRIX M(q):")
print("Literature equation (8):")
print("M11 = I1 + I2 + m2*l1² + 2*m2*l1*lc2*cos(θ2)")
print("M12 = I2 + m2*l1*lc2*cos(θ2)")
print("M21 = I2 + m2*l1*lc2*cos(θ2)")
print("M22 = I2")
print()
print("SymPy generated:")
print(f"M11 = {sm.simplify(M[0, 0])}")
print(f"M12 = {sm.simplify(M[0, 1])}")
print(f"M21 = {sm.simplify(M[1, 0])}")
print(f"M22 = {sm.simplify(M[1, 1])}")
print()

# Extract gravity vector by setting velocities to zero
gravity_forcing = LM.forcing.subs([(theta1d, 0), (theta2d, 0)])
print("GRAVITY VECTOR τg(q):")
print("Literature equation (10):")
print("τg1 = -m1*g*lc1*sin(θ1) - m2*g*(l1*sin(θ1) + lc2*sin(θ1+θ2))")
print("τg2 = -m2*g*lc2*sin(θ1+θ2)")
print()
print("SymPy generated:")
print(f"τg1 = {sm.simplify(gravity_forcing[0])}")
print(f"τg2 = {sm.simplify(gravity_forcing[1])}")
print()

# Extract Coriolis terms by subtracting gravity from total forcing
total_forcing = LM.forcing
coriolis_forcing = sm.simplify(total_forcing - gravity_forcing)
print("CORIOLIS TERMS (velocity-dependent):")
print("Literature: C(q,q̇)*q̇ where")
print("C11 = -2*m2*l1*lc2*sin(θ2)*θ̇2")
print("C12 = -m2*l1*lc2*sin(θ2)*θ̇2")
print("C21 = m2*l1*lc2*sin(θ2)*θ̇1")
print("C22 = 0")
print()
print("SymPy generated Coriolis forcing:")
print(f"Coriolis1 = {sm.simplify(coriolis_forcing[0])}")
print(f"Coriolis2 = {sm.simplify(coriolis_forcing[1])}")
print()

# Verify equation structure
print("COMPLETE EQUATION VERIFICATION:")
print("Literature form: M(q)*q̈ + C(q,q̇)*q̇ = τg(q) + B*u")
print("Where B = [0, 1]ᵀ and u = τ (elbow torque)")
print()

# === Convert to MAPTOR Format ===
print("=== MAPTOR DYNAMICS GENERATION ===")
lagrangian_to_maptor_dynamics(LM, [theta1, theta2], control_forces, "acrobot_dynamics.txt")

""" OUTPUT
=== ACROBOT TERM-BY-TERM VERIFICATION ===
Literature reference: https://underactuated.csail.mit.edu/acrobot.html#section1

MASS MATRIX M(q):
Literature equation (8):
M11 = I1 + I2 + m2*l1² + 2*m2*l1*lc2*cos(θ2)
M12 = I2 + m2*l1*lc2*cos(θ2)
M21 = I2 + m2*l1*lc2*cos(θ2)
M22 = I2

SymPy generated:
M11 = I1 + I2 + l1**2*m2 + 2*l1*lc2*m2*cos(theta2(t))
M12 = I2 + l1*lc2*m2*cos(theta2(t))
M21 = I2 + l1*lc2*m2*cos(theta2(t))
M22 = I2

GRAVITY VECTOR τg(q):
Literature equation (10):
τg1 = -m1*g*lc1*sin(θ1) - m2*g*(l1*sin(θ1) + lc2*sin(θ1+θ2))
τg2 = -m2*g*lc2*sin(θ1+θ2)

SymPy generated:
τg1 = -g*(l1*m2*sin(theta1(t)) + lc1*m1*sin(theta1(t)) + lc2*m2*sin(theta1(t) + theta2(t)))
τg2 = -g*lc2*m2*sin(theta1(t) + theta2(t))

CORIOLIS TERMS (velocity-dependent):
Literature: C(q,q̇)*q̇ where
C11 = -2*m2*l1*lc2*sin(θ2)*θ̇2
C12 = -m2*l1*lc2*sin(θ2)*θ̇2
C21 = m2*l1*lc2*sin(θ2)*θ̇1
C22 = 0

SymPy generated Coriolis forcing:
Coriolis1 = l1*lc2*m2*(2*Derivative(theta1(t), t) + Derivative(theta2(t), t))*sin(theta2(t))*Derivative(theta2(t), t)
Coriolis2 = -l1*lc2*m2*sin(theta2(t))*Derivative(theta1(t), t)**2

COMPLETE EQUATION VERIFICATION:
Literature form: M(q)*q̈ + C(q,q̇)*q̇ = τg(q) + B*u
Where B = [0, 1]ᵀ and u = τ (elbow torque)

=== MAPTOR DYNAMICS GENERATION ===
CasADi MAPTOR Dynamics:
============================================================

# State variables:
# theta1 = phase.state('theta1')
# theta2 = phase.state('theta2')
# theta1_dot = phase.state('theta1_dot')
# theta2_dot = phase.state('theta2_dot')

# Control variables:
# tau = phase.control('tau')

# MAPTOR dynamics dictionary:
phase.dynamics(
    {
        theta1: theta1_dot,
        theta2: theta2_dot,
        theta1_dot: (
            -I2
            * (
                g * l1 * m2 * ca.sin(theta1)
                + g * lc1 * m1 * ca.sin(theta1)
                + g * lc2 * m2 * ca.sin(theta1 + theta2)
                - l1 * lc2 * m2 * (2 * theta1_dot + theta2_dot) * ca.sin(theta2) * theta2_dot
            )
            + (I2 + l1 * lc2 * m2 * ca.cos(theta2))
            * (
                g * lc2 * m2 * ca.sin(theta1 + theta2)
                + l1 * lc2 * m2 * ca.sin(theta2) * theta1_dot**2
                - tau
            )
        )
        / (I1 * I2 + I2 * l1**2 * m2 - l1**2 * lc2**2 * m2**2 * ca.cos(theta2) ** 2),
        theta2_dot: (
            (I2 + l1 * lc2 * m2 * ca.cos(theta2))
            * (
                g * l1 * m2 * ca.sin(theta1)
                + g * lc1 * m1 * ca.sin(theta1)
                + g * lc2 * m2 * ca.sin(theta1 + theta2)
                - l1 * lc2 * m2 * (2 * theta1_dot + theta2_dot) * ca.sin(theta2) * theta2_dot
            )
            - (
                g * lc2 * m2 * ca.sin(theta1 + theta2)
                + l1 * lc2 * m2 * ca.sin(theta2) * theta1_dot**2
                - tau
            )
            * (I1 + I2 + l1**2 * m2 + 2 * l1 * lc2 * m2 * ca.cos(theta2))
        )
        / (I1 * I2 + I2 * l1**2 * m2 - l1**2 * lc2**2 * m2**2 * ca.cos(theta2) ** 2),
    }
)
"""
