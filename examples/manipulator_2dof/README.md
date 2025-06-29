# 2DOF Manipulator Design and Trajectory Optimization

## Mathematical Formulation

### Problem Statement

Find the optimal motor sizing and joint torques that minimize actuator investment, mission time, and energy consumption for a 2DOF planar manipulator with 5kg payload performing point-to-point motion:

$$J = t_f + 0.1 \cdot C_{\text{actuator}} + 0.01 \int_0^{t_f} (\tau_1^2 + \tau_2^2) dt$$

Where the actuator cost model is:
$$C_{\text{actuator}} = 0.08 \cdot \tau_{\max,1} + 0.05 \cdot \tau_{\max,2}$$

Subject to the coupled 2DOF manipulator dynamics and actuator capability constraints:

$$\frac{dq_1}{dt} = \dot{q}_1$$

$$\frac{dq_2}{dt} = \dot{q}_2$$

$$\frac{d\dot{q}_1}{dt} = \frac{(I_2 + l_{c2}^2 m_2)(-g l_1 m_2 \cos(q_1) - g l_1 m_{payload} \cos(q_1) - g l_2 m_{payload} \cos(q_1 + q_2) - g l_{c1} m_1 \cos(q_1) - g l_{c2} m_2 \cos(q_1 + q_2) + l_1 l_{c2} m_2 (2\dot{q}_1 + \dot{q}_2) \sin(q_2) \dot{q}_2 + \tau_1) - (I_2 + l_1 l_{c2} m_2 \cos(q_2) + l_{c2}^2 m_2)(-g l_2 m_{payload} \cos(q_1 + q_2) - g l_{c2} m_2 \cos(q_1 + q_2) - l_1 l_{c2} m_2 \sin(q_2) \dot{q}_1^2 + \tau_2)}{I_1 I_2 + I_1 l_{c2}^2 m_2 + I_2 l_1^2 m_2 + I_2 l_{c1}^2 m_1 + l_1^2 l_{c2}^2 m_2^2 \sin^2(q_2) + l_{c1}^2 l_{c2}^2 m_1 m_2}$$

$$\frac{d\dot{q}_2}{dt} = \frac{-(I_2 + l_1 l_{c2} m_2 \cos(q_2) + l_{c2}^2 m_2)(-g l_1 m_2 \cos(q_1) - g l_1 m_{payload} \cos(q_1) - g l_2 m_{payload} \cos(q_1 + q_2) - g l_{c1} m_1 \cos(q_1) - g l_{c2} m_2 \cos(q_1 + q_2) + l_1 l_{c2} m_2 (2\dot{q}_1 + \dot{q}_2) \sin(q_2) \dot{q}_2 + \tau_1) + (-g l_2 m_{payload} \cos(q_1 + q_2) - g l_{c2} m_2 \cos(q_1 + q_2) - l_1 l_{c2} m_2 \sin(q_2) \dot{q}_1^2 + \tau_2)(I_1 + I_2 + l_1^2 m_2 + 2 l_1 l_{c2} m_2 \cos(q_2) + l_{c1}^2 m_1 + l_{c2}^2 m_2)}{I_1 I_2 + I_1 l_{c2}^2 m_2 + I_2 l_1^2 m_2 + I_2 l_{c1}^2 m_1 + l_1^2 l_{c2}^2 m_2^2 \sin^2(q_2) + l_{c1}^2 l_{c2}^2 m_1 m_2}$$

### Robot Configuration

**Joint Configuration:**
- **Joint 1**: Base rotation (shoulder joint)
- **Joint 2**: Elbow rotation (relative to upper arm)

**Forward Kinematics:**
$$x_{ee} = l_1\cos(q_1) + l_2\cos(q_1+q_2)$$
$$y_{ee} = l_1\sin(q_1) + l_2\sin(q_1+q_2)$$

### Design Parameters

- **Shoulder motor torque rating**: $\tau_{\max,1} \in [5, 100]$ N⋅m
- **Elbow motor torque rating**: $\tau_{\max,2} \in [5, 100]$ N⋅m

### Boundary Conditions

- **Initial end-effector position**: $(0.3, 0.1)$ m
- **Final end-effector position**: $(-0.6, 0.1)$ m
- **Initial joint velocities**: $\dot{q}_1(0) = \dot{q}_2(0) = 0$ rad/s
- **Final joint velocities**: $\dot{q}_1(t_f) = \dot{q}_2(t_f) = 0$ rad/s
- **Joint angle bounds**: $q_1 \in [0, \pi]$ rad, $q_2 \in [-\pi, \pi]$ rad
- **Joint velocity bounds**: $\dot{q}_1, \dot{q}_2 \in [-2.0, 2.0]$ rad/s
- **Actuator capability constraints**: $-\tau_{\max,i} \leq \tau_i \leq \tau_{\max,i}$ for $i = 1,2$

### Physical Parameters

- **Link masses**: $m_1 = 2.0$ kg, $m_2 = 1.5$ kg
- **Payload mass**: $m_{payload} = 5.0$ kg
- **Link lengths**: $l_1 = 0.4$ m, $l_2 = 0.4$ m
- **Center of mass distances**: $l_{c1} = 0.20$ m, $l_{c2} = 0.20$ m
- **Moments of inertia**: $I_1 = 0.0267$ kg⋅m², $I_2 = 0.02$ kg⋅m²
- **Gravity**: $g = 9.81$ m/s²

### State Variables

- $q_1(t)$: Shoulder joint angle (rad)
- $q_2(t)$: Elbow joint angle (relative to upper arm) (rad)
- $\dot{q}_1(t)$: Shoulder joint angular velocity (rad/s)
- $\dot{q}_2(t)$: Elbow joint angular velocity (rad/s)

### Control Variables

- $\tau_1(t)$: Shoulder joint torque (N⋅m)
- $\tau_2(t)$: Elbow joint torque (N⋅m)

### Constraints

- **Ground clearance constraint**: End-effector height $y_{ee} \geq 0.05$ m
- **Actuator capability**: $|\tau_i(t)| \leq \tau_{\max,i}$ for all $t$ and $i = 1,2$
- **Inverse kinematics**: Joint angles computed to achieve desired end-effector positions
- **Reachability verification**: Target positions must satisfy $d_{min} \leq \|r_{target}\| \leq d_{max}$

### Notes

This problem demonstrates **simultaneous design and trajectory optimization** for robotics applications. Unlike pure trajectory planning, MAPTOR optimizes both the motor sizing (design parameters) and the motion trajectory simultaneously. The optimization determines the minimum motor torque ratings required while minimizing mission time and energy consumption for a planar 2DOF manipulator transporting a 5kg payload.

The planar configuration simplifies the 3D case while maintaining the essential coupling between joint dynamics, payload effects, and actuator sizing trade-offs.

## Dynamics Derivation

The 2DOF manipulator dynamics were derived using Lagrangian mechanics with SymPy, systematically constructing the equations of motion for the two-joint planar system with payload:

```{literalinclude} ../../../examples/manipulator_2dof/manipulator_2dof_dynamics.py
:language: python
:caption: examples/manipulator_2dof/manipulator_2dof_dynamics.py
:linenos:
```

This symbolic derivation produces the coupled dynamics equations accounting for gravitational forces from both links and payload, centrifugal forces, Coriolis coupling between joints, and inertial effects. The systematic approach ensures mathematical correctness for the planar multi-body system.

## Running This Example

```bash
cd examples/manipulator_2dof
python manipulator_2dof.py
python manipulator_2dof_animate.py
```