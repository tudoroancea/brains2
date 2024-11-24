# sim

## Interface with other modules

The simulation module is entirely implemented in the `brains2::sim::Sim` class, and used in the `brains2::sim::SimNode` node.
### Inputs

- car parameters:
    - mass $m$
    - inertia $I_z$
    - distance between COG and front/rear axle $l_\mathrm{F},l_\mathrm{R}$
    - drivetrain parameter $C_\mathrm{m0}$
    - rolling resistance $C_\mathrm{r0}$
    - linear drag coefficient $C_\mathrm{r1}$
    - quadratic drag coefficient $C_\mathrm{r2}$
    - time constant of the steering actuator and the drivetrain $t_\delta,t_\tau$
    - height of the COG $z_\mathrm{CG}$
    - axle track width $W$
    - downforce coefficient $C_\mathrm{downforce}$
    - Pacejka coefficients $B_\mathrm{a}, C_\mathrm{a}, D_\mathrm{a}, E_\mathrm{a}$
- target controls: target steering angle $u_\delta$ and target torques for each wheel $u_{\tau_\mathrm{FL}}, u_{\tau_\mathrm{FR}}, u_{\tau_\mathrm{RL}}, u_{\tau_\mathrm{RR}}$.

> [!NOTE]
> The `SimNode` actually accepts other inputs, such as a signal to reset the position of the car.

### Outputs

- pose $X,Y,\varphi$ (in global cartesian coordinates)
- longitudinal, lateral and angular velocity $v_x,v_y,\omega$ 
- longitudinal and lateral acceleration $a_x,a_y$ 
- current actuator state $\delta, \tau_\mathrm{FL}, \tau_\mathrm{FR}, \tau_\mathrm{RL}, \tau_\mathrm{RR}$

## Models

The simulation currently relies on a complex dynamic 4-wheel model used in the nominal case, and a much simpler kinematic bicycle model that is used at small velocities (where the dynamic model has a singularity).

### Kinematic bicycle model

The kinematic bicycle model is given by the following system of explicit ODEs:

```math
\begin{aligned}
\dot{X} & = v_x \cos(\varphi) - v_y \sin(\varphi) \\
\dot{Y} & = v_x \sin(\varphi) + v_y \cos(\varphi) \\
\dot{\varphi} & = \omega \\
\dot{v}_x &= \dot{v} \cos(\beta) - v_y \dot{\beta}, \\
\dot{v}_y &= \dot{v} \sin(\beta) + v_x \dot{\beta}, \\
\dot{\omega} &= \frac{1}{l_\mathrm{R}}(\dot{v} \sin(\beta) + v_x \dot{\beta}), \\
\dot{\delta} &= \frac{1}{t_\delta}(u_\delta - \delta), \\
\dot{\tau}_{ij} & = \frac{1}{t_\tau}(u_\tau - \tau_{ij}), \, ij \in \{\mathrm{FL},\mathrm{FR},\mathrm{RL},\mathrm{RR}\},
\end{aligned}
```

where $\beta = \frac{l_\mathrm{R}}{l_\mathrm{R} + l_\mathrm{F}}\delta$ is the slip angle,
$\dot{\beta} = \frac{l_\mathrm{R}}{l_\mathrm{R} + l_\mathrm{F}} \dot{\delta}$ its derivative,
$\dot{v}=\frac{1}{m}(\cos(\beta) F_{\mathrm{R},x} + \sin(\beta)F_{\mathrm{R},x})$ is the
derivative of the absolute velocity $v=\sqrt{v_x^2+v_y^2}$ composed of the longitudinal forces
applied on the rear and front axles $F_{\mathrm{R},x} = \frac{1}{2}C_\mathrm{m0}\tau - C_\mathrm{r0} - C_\mathrm{r1} v_x - C_\mathrm{r2} v_x^2$ and $F_{\mathrm{F},x} = \frac{1}{2}C_\mathrm{m0}\tau$ (themselves composed of the drivetrain and the drag forces).

In this case, the linear and lateral accelerations are given by
```math
\begin{aligned}
a_x &= \frac{1}{m} (F_{\mathrm{R},x} + \cos(\delta) F_{\mathrm{F},x}), \\
a_y &=  \frac{1}{m}  \sin(\delta) F_{\mathrm{F},y}. \\
\end{aligned}
```

### Dynamic 4-wheel model

<!-- The dynamic model is given by the following system of implicit ODEs: -->

In the dynamic model, the dynamics of the pose and actuator are the same. The difference
resides in the dynamics of the velocity. The accelerations can be written as
```math
\begin{aligned} 
a_x &= \dot{v}_x - v_y \omega, \\
a_y &= \dot{v}_y + v_x \omega. \\
\end{aligned}
```
Then the velocity dynamics read
```math
\begin{aligned}
m a_x & = (F_{\mathrm{FL}, x} + F_{\mathrm{FR}, x}) \cos(\delta) - (F_{\mathrm{FL}, y} + F_{\mathrm{FR}, y}) \sin(\delta) + F_{\mathrm{RL}, x} + F_{\mathrm{RR}, x} + F_\mathrm{drag} , \\
m a_y & =  (F_{\mathrm{FL}, x} + F_{\mathrm{FR}, x}) \sin(\delta) + (F_{\mathrm{FL}, y} + F_{\mathrm{FR}, y}) \cos(\delta) + F_{\mathrm{RL},y} + F_{\mathrm{RR},y}, \\ 
I_z \omega & =  \big(F_{\mathrm{FR}, x} \cos(\delta) - F_{\mathrm{FR}, y} \sin(\delta) \big) \frac{W}{2}  
              + \big(F_{\mathrm{FR}, x} \sin(\delta) + F_{\mathrm{FR}, y} \cos(\delta) \big) l_\mathrm{F} \\
           & \quad - \big(F_{\mathrm{FL}, x} \cos(\delta) - F_{\mathrm{FL}, y} \sin(\delta) \big) \frac{W}{2} 
              + \big(F_{\mathrm{FL}, x} \sin(\delta) + F_{\mathrm{FL}, y} \cos(\delta) \big) l_\mathrm{F} \\
           & \quad + F_{\mathrm{RR}, x} \frac{W}{2} - F_{\mathrm{RR}, y} l_\mathrm{R} \\
           & \quad - F_{\mathrm{RL}, x} \frac{W}{2} - F_{\mathrm{RL}, y} l_\mathrm{R},  
\end{aligned}
```
where the force produced by the drivetrain at each wheel is given by
```math
F_{ij, x}  = C_{m0} \tau_{ij}, \\
```
the vertical wheel loads are given by
```math
\begin{aligned}
F_{\mathrm{FL}, z} & = \frac{1}{2} \left(  \frac{l_\mathrm{F}}{L} (mg + F_\mathrm{down}) - m a_x \frac{z_\mathrm{CG}}{L} - m a_y \frac {z_\mathrm{CG}}{W} \right), \\
F_{\mathrm{FR}, z} & = \frac{1}{2} \left(  \frac{l_\mathrm{F}}{L} (mg + F_\mathrm{down}) - m a_x \frac{z_\mathrm{CG}}{L} + m a_y \frac {z_\mathrm{CG}}{W} \right), \\
F_{\mathrm{RL}, z} & = \frac{1}{2} \left(  \frac{l_\mathrm{R}}{L} (mg + F_\mathrm{down}) + m a_x \frac{z_\mathrm{CG}}{L} - m a_y \frac {z_\mathrm{CG}}{W} \right), \\
F_{\mathrm{RR}, z} & = \frac{1}{2} \left(  \frac{l_\mathrm{R}}{L} (mg + F_\mathrm{down}) + m a_x \frac{z_\mathrm{CG}}{L} + m a_y \frac {z_\mathrm{CG}}{W} \right), \\
\end{aligned}
```
the slip angles of each wheel are given by
```math
\begin{aligned}
\alpha_\mathrm{FL} & = \delta - \arctan\left(\frac{v_{\mathrm{FL},y}}{v_{\mathrm{FL},x}}\right), \\ 
\alpha_\mathrm{FR} & = \delta - \arctan\left(\frac{v_{\mathrm{FR},y}}{v_{\mathrm{FR},x}}\right), \\ 
\alpha_\mathrm{RL} & = - \arctan\left(\frac{v_{\mathrm{RL},y}}{v_{\mathrm{RL},x}}\right), \\ 
\alpha_\mathrm{RR} & = - \arctan\left(\frac{v_{\mathrm{RR},y}}{v_{\mathrm{RR},x}}\right), \\ 
\end{aligned}
```

## Implementation

The ODEs presented above are solved using integrators offered by `acados`, mainly due
to their speed and the ability to be generated from casadi expressions.
We chose the IRK4 integrator and the Gauss-Radau collocation method, which proved to be 
the most numerically stable for our system. Note that we use an implicit integrator even
for the explicit equations of the kinematic model because the very small time constant $t_\tau$
makes the system too stiff for an explicit RK4 integrator.

Both models are implemented in [gen.py](gen.py) using casadi and acados. Custom C code is 
generated for both the integrators
