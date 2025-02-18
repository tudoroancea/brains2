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
$\dot{v}=\frac{1}{m}(\cos(\beta) F_{x,\mathrm{R}} + \sin(\beta)F_{x,\mathrm{R}})$ is the
derivative of the absolute velocity $v=\sqrt{v_x^2+v_y^2}$ composed of the longitudinal forces
applied on the rear and front axles $F_{\mathrm{R},x} = \frac{1}{2}C_\mathrm{m0}\tau - C_\mathrm{r0} - C_\mathrm{r1} v_x - C_\mathrm{r2} v_x^2$ and $F_{\mathrm{F},x} = \frac{1}{2}C_\mathrm{m0}\tau$ (themselves composed of the drivetrain and the drag forces).

In this case, the linear and lateral accelerations are given by
```math
\begin{aligned}
a_x &= \frac{1}{m} (F_{x, \mathrm{R}} + \cos(\delta) F_{x, \mathrm{F}}), \\
a_y &=  \frac{1}{m}  \sin(\delta) F_{y, \mathrm{F}}. \\
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
m a_x & = (F_{x, \mathrm{FL}} + F_{x, \mathrm{FR}}) \cos(\delta) - (F_{y, \mathrm{FL}} + F_{y, \mathrm{FR}}) \sin(\delta) + F_{x, \mathrm{RL}} + F_{x, \mathrm{RR}} + F_\mathrm{drag} , \\
m a_y & =  (F_{x, \mathrm{FL}} + F_{x, \mathrm{FR}}) \sin(\delta) + (F_{y, \mathrm{FL}} + F_{y, \mathrm{FR}}) \cos(\delta) + F_{\mathrm{RL},y} + F_{\mathrm{RR},y}, \\ 
I_z \omega & =  \big(F_{x, \mathrm{FR}} \cos(\delta) - F_{y, \mathrm{FR}} \sin(\delta) \big) \frac{W}{2}  
              + \big(F_{x, \mathrm{FR}} \sin(\delta) + F_{y, \mathrm{FR}} \cos(\delta) \big) l_\mathrm{F} \\
           & \quad - \big(F_{x, \mathrm{FL}} \cos(\delta) - F_{y, \mathrm{FL}} \sin(\delta) \big) \frac{W}{2} 
              + \big(F_{x, \mathrm{FL}} \sin(\delta) + F_{y, \mathrm{FL}} \cos(\delta) \big) l_\mathrm{F} \\
           & \quad + F_{x, \mathrm{RR}} \frac{W}{2} - F_{y, \mathrm{RR}} l_\mathrm{R} \\
           & \quad - F_{x, \mathrm{RL}} \frac{W}{2} - F_{y, \mathrm{RL}} l_\mathrm{R},  
\end{aligned}
```
where the force produced by the drivetrain at each wheel is given by
```math
F_{x, ij}  = C_{m0} \tau_{ij}, \\
```
and the lateral forces produced by the tires are given by
```math
F_{y, ij} = F_{z, ij} \mu_{ij}.
```

The latter are computed based on the vertical wheel loads given by
```math
\begin{aligned}
F_{z, \mathrm{FL}} & = \frac{1}{2} \left(  \frac{l_\mathrm{F}}{L} (mg + F_\mathrm{down}) - m a_x \frac{z_\mathrm{CG}}{L} - m a_y \frac {z_\mathrm{CG}}{W} \right), \\
F_{z, \mathrm{FR}} & = \frac{1}{2} \left(  \frac{l_\mathrm{F}}{L} (mg + F_\mathrm{down}) - m a_x \frac{z_\mathrm{CG}}{L} + m a_y \frac {z_\mathrm{CG}}{W} \right), \\
F_{z, \mathrm{RL}} & = \frac{1}{2} \left(  \frac{l_\mathrm{R}}{L} (mg + F_\mathrm{down}) + m a_x \frac{z_\mathrm{CG}}{L} - m a_y \frac {z_\mathrm{CG}}{W} \right), \\
F_{z, \mathrm{RR}} & = \frac{1}{2} \left(  \frac{l_\mathrm{R}}{L} (mg + F_\mathrm{down}) + m a_x \frac{z_\mathrm{CG}}{L} + m a_y \frac {z_\mathrm{CG}}{W} \right), \\
\end{aligned}
```
and the variable friction coefficient computed using the Pacejka's magic formula
```math
\mu_{ij}  = D_a \sin(C_a \arctan(B_a \alpha_{ij} - E_a ( B_a \alpha_{ij} - \arctan(B_a \alpha_{ij})))),
```
which is based on the slip angles 
```math
\begin{aligned}
\alpha_\mathrm{FL} & = \delta - \arctan\left(\frac{v_{y, \mathrm{FL}}}{v_{x, \mathrm{FL}}}\right), \\ 
\alpha_\mathrm{FR} & = \delta - \arctan\left(\frac{v_{y, \mathrm{FR}}}{v_{x, \mathrm{FR}}}\right), \\ 
\alpha_\mathrm{RL} & = - \arctan\left(\frac{v_{y, \mathrm{RL}}}{v_{x, \mathrm{RL}}}\right), \\ 
\alpha_\mathrm{RR} & = - \arctan\left(\frac{v_{y, \mathrm{RR}}}{v_{x, \mathrm{RR}}}\right), \\ 
\end{aligned}
```
and the longitudinal and lateral velocities of each wheel
```math
\begin{aligned}
v_{x, \mathrm{FL}} & = v_{x, \mathrm{RL}} & = v_x - \frac{w}{2} \omega \\
v_{x, \mathrm{FR}} & = v_{x, \mathrm{RR}} & = v_x + \frac{w}{2} \omega \\
v_{y, \mathrm{FL}} & = v_{y, \mathrm{FR}} & = v_y + l_\mathrm{F} \omega \\
v_{y, \mathrm{RL}} & = v_{y, \mathrm{RR}} & = v_y - l_\mathrm{F} \omega \\
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
