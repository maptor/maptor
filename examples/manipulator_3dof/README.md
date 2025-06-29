# 3DOF Manipulator Design and Trajectory Optimization

## Mathematical Formulation

### Problem Statement

Find the optimal motor sizing and joint torques that minimize actuator investment, mission time, and energy consumption for a 3DOF manipulator with 5kg payload performing point-to-point motion:

$$J = t_f + 0.1 \cdot C_{\text{actuator}} + 0.01 \int_0^{t_f} (\tau_1^2 + \tau_2^2 + \tau_3^2) dt$$

Where the actuator cost model is:
$$C_{\text{actuator}} = 0.05 \cdot \tau_{\max,1} + 0.08 \cdot \tau_{\max,2} + 0.03 \cdot \tau_{\max,3}$$

Subject to the coupled 3DOF manipulator dynamics and actuator capability constraints:

$$\frac{dq_1}{dt} = \dot{q}_1$$

$$\frac{dq_2}{dt} = \dot{q}_2$$

$$\frac{dq_3}{dt} = \dot{q}_3$$

$$\frac{d\dot{q}_1}{dt} = \frac{2I_2\sin(2q_2)\dot{q}_1\dot{q}_2 + 2I_3\sin(2q_2+2q_3)\dot{q}_1\dot{q}_2 + 2I_3\sin(2q_2+2q_3)\dot{q}_1\dot{q}_3 - 2l_2^2m_3\sin(2q_2)\dot{q}_1\dot{q}_2 - 4l_2l_{c3}m_3\sin(2q_2+q_3)\dot{q}_1\dot{q}_2 - 2l_2l_{c3}m_3\sin(2q_2+q_3)\dot{q}_1\dot{q}_3 - 2l_2l_{c3}m_3\sin(q_3)\dot{q}_1\dot{q}_3 - 2l_{c2}^2m_2\sin(2q_2)\dot{q}_1\dot{q}_2 - 2l_{c3}^2m_3\sin(2q_2+2q_3)\dot{q}_1\dot{q}_2 - 2l_{c3}^2m_3\sin(2q_2+2q_3)\dot{q}_1\dot{q}_3 - 2\tau_1}{-2I_1 + I_2\cos(2q_2) - I_2 + I_3\cos(2q_2+2q_3) - I_3 - l_2^2m_3\cos(2q_2) - l_2^2m_3 - 2l_2l_{c3}m_3\cos(2q_2+q_3) - 2l_2l_{c3}m_3\cos(q_3) - l_{c2}^2m_2\cos(2q_2) - l_{c2}^2m_2 - l_{c3}^2m_3\cos(2q_2+2q_3) - l_{c3}^2m_3}$$

$\frac{d\dot{q}_2}{dt} = f_2(q_1, q_2, q_3, \dot{q}_1, \dot{q}_2, \dot{q}_3, \tau_2, \tau_3)$

$\frac{d\dot{q}_3}{dt} = f_3(q_1, q_2, q_3, \dot{q}_1, \dot{q}_2, \dot{q}_3, \tau_2, \tau_3)$

The complete analytical expressions for $f_2$ and $f_3$ involving gravitational, centrifugal, Coriolis, and coupling terms are provided in the dynamics derivation output below.

### Robot Configuration

**Joint Configuration:**
- **Joint 1**: Base rotation about Z-axis (azimuth)
- **Joint 2**: Shoulder pitch about Y-axis (elevation)
- **Joint 3**: Elbow pitch about Y-axis (relative to upper arm)

**Forward Kinematics:**
$$x_{ee} = (l_2\cos(q_2) + l_3\cos(q_2+q_3))\cos(q_1)$$
$$y_{ee} = (l_2\cos(q_2) + l_3\cos(q_2+q_3))\sin(q_1)$$
$$z_{ee} = l_1 + l_2\sin(q_2) + l_3\sin(q_2+q_3)$$

### Design Parameters

- **Base motor torque rating**: $\tau_{\max,1} \in [5, 50]$ N⋅m
- **Shoulder motor torque rating**: $\tau_{\max,2} \in [5, 50]$ N⋅m
- **Elbow motor torque rating**: $\tau_{\max,3} \in [5, 50]$ N⋅m

### Boundary Conditions

- **Initial end-effector position**: $(0.0, 0.5, 0.1)$ m
- **Final end-effector position**: $(0.0, -0.5, 0.1)$ m
- **Initial joint velocities**: $\dot{q}_1(0) = \dot{q}_2(0) = \dot{q}_3(0) = 0$ rad/s
- **Final joint velocities**: $\dot{q}_1(t_f) = \dot{q}_2(t_f) = \dot{q}_3(t_f) = 0$ rad/s
- **Joint angle bounds**: $q_1 \in [-\pi, \pi]$ rad, $q_2 \in [-\pi/6, 5\pi/6]$ rad, $q_3 \in [-2.5, 2.5]$ rad
- **Joint velocity bounds**: $\dot{q}_1 \in [-1.5, 1.5]$ rad/s, $\dot{q}_2 \in [-1.2, 1.2]$ rad/s, $\dot{q}_3 \in [-2.0, 2.0]$ rad/s
- **Actuator capability constraints**: $-\tau_{\max,i} \leq \tau_i \leq \tau_{\max,i}$ for $i = 1,2,3$

### Physical Parameters

- **Link masses**: $m_1 = 3.0$ kg, $m_2 = 2.5$ kg, $m_3 = 1.5$ kg
- **Payload mass**: $m_{box} = 5.0$ kg (payload)
- **Link lengths**: $l_1 = 0.3$ m, $l_2 = 0.4$ m, $l_3 = 0.4$ m
- **Center of mass distances**: $l_{c1} = 0.15$ m, $l_{c2} = 0.20$ m, $l_{c3} = 0.20$ m
- **Moments of inertia**: $I_1 = 0.0225$ kg⋅m², $I_2 = 0.0333$ kg⋅m², $I_3 = 0.02$ kg⋅m²
- **Gravity**: $g = 9.81$ m/s²

### State Variables

- $q_1(t)$: Base joint angle (azimuth rotation) (rad)
- $q_2(t)$: Shoulder joint angle (elevation) (rad)
- $q_3(t)$: Elbow joint angle (relative to upper arm) (rad)
- $\dot{q}_1(t)$: Base joint angular velocity (rad/s)
- $\dot{q}_2(t)$: Shoulder joint angular velocity (rad/s)
- $\dot{q}_3(t)$: Elbow joint angular velocity (rad/s)

### Control Variables

- $\tau_1(t)$: Base joint torque (N⋅m)
- $\tau_2(t)$: Shoulder joint torque (N⋅m)
- $\tau_3(t)$: Elbow joint torque (N⋅m)

### Constraints

- **Workspace constraint**: End-effector height $z_{ee} \geq 0.05$ m (ground clearance)
- **Actuator capability**: $|\tau_i(t)| \leq \tau_{\max,i}$ for all $t$ and $i = 1,2,3$
- **Inverse kinematics**: Joint angles computed to achieve desired end-effector positions
- **Reachability verification**: Target positions must satisfy $d_{min} \leq \|r_{target}\| \leq d_{max}$

### Notes

This problem demonstrates **simultaneous design and trajectory optimization** for robotics applications. Unlike pure trajectory planning, MAPTOR optimizes both the motor sizing (design parameters) and the motion trajectory simultaneously. The optimization determines the minimum motor torque ratings required while minimizing mission time and energy consumption.

## Dynamics Derivation

The 3DOF manipulator dynamics were derived using Lagrangian mechanics with SymPy, systematically constructing the equations of motion for the three-joint system with payload:

```{literalinclude} ../../../examples/manipulator_3dof/manipulator_3dof_dynamics.py
:language: python
:caption: examples/manipulator_3dof/manipulator_3dof_dynamics.py
:linenos:
```

This symbolic derivation produces the complex coupled dynamics equations accounting for gravitational forces from all links and payload, centrifugal forces from base rotation, Coriolis coupling between joints, and inertial effects. The systematic approach ensures mathematical correctness for the multi-body system while handling the coupling between base rotation and the vertical plane motion of the upper arm and forearm.

## Running This Example

```bash
cd examples/manipulator_3dof
python manipulator_3dof.py
python manipulator_3dof_animate.py
```
