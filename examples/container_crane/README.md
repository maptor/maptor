# Container Crane Control

## Mathematical Formulation

### Problem Statement

Find the optimal control inputs $u_1(t)$ and $u_2(t)$ that minimize the performance index:

$$J = \int_0^{t_f} \frac{1}{2}\left( x_3^2 + x_6^2 + \rho (u_1^2 + u_2^2) \right) dt$$

Subject to the crane dynamic constraints:

$$\frac{dx_1}{dt} = x_4$$

$$\frac{dx_2}{dt} = x_5$$

$$\frac{dx_3}{dt} = x_6$$

$$\frac{dx_4}{dt} = u_1 + c_4 x_3$$

$$\frac{dx_5}{dt} = u_2$$

$$\frac{dx_6}{dt} = -\frac{u_1 + c_5 x_3 + 2 x_5 x_6}{x_2}$$

### Boundary Conditions

- **Initial conditions**: $x_1(0) = 0$, $x_2(0) = 22$, $x_3(0) = 0$, $x_4(0) = 0$, $x_5(0) = -1$, $x_6(0) = 0$
- **Final conditions**: $x_1(t_f) = 10$, $x_2(t_f) = 14$, $x_3(t_f) = 0$, $x_4(t_f) = 2.5$, $x_5(t_f) = 0$, $x_6(t_f) = 0$
- **Control bounds**: $-c_1 \leq u_1 \leq c_1$, $c_2 \leq u_2 \leq c_3$
- **State bounds**: $-2.5 \leq x_4 \leq 2.5$, $-1 \leq x_5 \leq 1$

### Physical Parameters

- Final time: $t_f = 9.0$ s
- Control penalty: $\rho = 0.01$
- System constants: $c_1 = 2.83374$, $c_2 = -0.80865$, $c_3 = 0.71265$, $c_4 = 17.2656$, $c_5 = 27.0756$

### State Variables

- $x_1(t)$
- $x_2(t)$
- $x_3(t)$
- $x_4(t)$
- $x_5(t)$
- $x_6(t)$

### Control Variables

- $u_1(t)$
- $u_2(t)$

### Notes

This problem involves controlling a container crane to transport a load while minimizing swing oscillations and control effort.

## Running This Example

```bash
cd examples/container_crane
python container_crane.py
```
