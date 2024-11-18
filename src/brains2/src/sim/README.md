# sim

## Interface with other modules

### Inputs

- The only inputs of the actual simulation module (encapsulated in the `brains2::sim::Sim` class) are target controls: target steering angle $u_\delta$ and target torques for each wheel $\tau_\mathrm{FL}, \tau_\mathrm{FR}, \tau_\mathrm{RL}, \tau_\mathrm{RR}$.
- The actual `SimNode` accepts other inputs such as a signal to reset the position of the car.

### Outputs

- the car's pose $X,Y,\varphi$ in global cartesian coordinates
- the car's linear and angular velocity $v_x,v_y,\omega$ in car local
- car accelerations $a_x,a_y$ in global cartesian coordinates
- The current state of the actuators (current steering angle )

## Models

The simulation currently relies on a complex dynamic 4-wheel model used in the nominal case, and a much simpler kinematic bicycle model that is used at small velocities (where the dynamic model has a singularity).

### Kinematic biycle model

```math
\begin{aligned}
\dot{X} & = v_x \cos(\varphi) - v_y \sin(\varphi) \\
\dot{Y} & = v_x \sin(\varphi) + v_y \cos(\varphi) \\
\dot{\varphi} & = \omega \\
\dot{v}_x &= \dot{v} \cos(\beta) - v_y \dot{\beta}, \\
\dot{v}_y &= \dot{v} \sin(\beta) + v_x \dot{\beta}, \\
\dot{\omega} &= \frac{1}{l_\mathrm{R}}(\dot{v} \sin(\beta) + v_x \dot{\beta}), \\
\dot{\delta} &= \frac{1}{t_\delta}(u_\delta - \delta), \\
\dot{\tau}_{ij} & = \frac{1}{t_\tau}(u_\tau - \tau_{ij}), \, ij \in \{FL,FR,RL,RR\},
\end{aligned}
```

where $\beta = \frac{l_\mathrm{R}}{l_\mathrm{R} + l_\mathrm{F}}\delta$ is the slip angle, $\dot{\beta} = \frac{l_\mathrm{R}}{l_\mathrm{R} + l_\mathrm{F}} \dot{\delta}$ its derivative,
$\dot{v}=\frac{1}{m}(\cos(\beta) (\frac{1}{2}F_\mathrm{motor}+F_\mathrm{drag}) + \frac{1}{2}\sin(\beta)F_\mathrm{motor})$ is the
derivative of the absolute velocity $v=\sqrt{v_x^2+v_y^2}$, $F_\mathrm{motor}=C_\mathrm{m0}\tau$ is the force produce by the drivetrain,
and finally $F_\mathrm{drag}=-C_\mathrm{r0} - C_\mathrm{r1} v_x - C_\mathrm{r2} v_x^2$ is the drag force.

### Dynamic 4-wheel model

```math
\begin{aligned}
\dot{X} & = v_x \cos(\varphi) - v_y \sin(\varphi) \\
\dot{Y} & = v_x \sin(\varphi) + v_y \cos(\varphi) \\
\dot{\varphi} & = \omega 
\end{aligned}
```

```math
\begin{aligned}
m a_x & = (F_{\mathrm{FL}, x} + F_{\mathrm{FR}, x}) \cos(\delta) - (F_{\mathrm{FL}, y} + F_{\mathrm{FR}, y}) \sin(\delta) + F_{\mathrm{RL}, x} + F_{\mathrm{RR}, x} + F_\mathrm{drag}  \\
m a_y & =  (F_{\mathrm{FL}, x} + F_{\mathrm{FR}, x}) \sin(\delta) + (F_{\mathrm{FL}, y} + F_{\mathrm{FR}, y}) \cos(\delta) + F_{\mathrm{RL},y} + F_{\mathrm{RR},y} \\ 
I_z \omega & =  \big(F_{\mathrm{FR}, x} \cos(\delta) - F_{\mathrm{FR}, y} \sin(\delta) \big) \frac{W}{2} \\
           & \quad +\big(F_{\mathrm{FR}, x} \sin(\delta) + F_{\mathrm{FR}, y} \cos(\delta) \big) l_\mathrm{F} \\
           & \quad - \big(F_{\mathrm{FL}, x} \cos(\delta) - F_{\mathrm{FL}, y} \sin(\delta) \big) \frac{W}{2}  \\
           & \quad +\big(F_{\mathrm{FL}, x} \sin(\delta) + F_{\mathrm{FL}, y} \cos(\delta) \big) l_\mathrm{F} \\
           & \quad + F_{\mathrm{RR}, x} \frac{W}{2} - F_{\mathrm{RR}, y} l_\mathrm{R} \\
           & \quad - F_{\mathrm{RL}, x} \frac{W}{2} - F_{\mathrm{RL}, y} l_\mathrm{R} 
\end{aligned}
```

```math
F_{ij, x}  = C_{m0} \tau_{ij} \\
```

```math
\begin{aligned}
F_{\mathrm{FL}, z} & = \frac{1}{2} \left(  \frac{l_\mathrm{F}}{L} (mg + F_\mathrm{down}) - m a_x \frac{z_\mathrm{CG}}{L} - m a_y \frac {z_\mathrm{CG}}{W} \right) \\
F_{\mathrm{FR}, z} & = \frac{1}{2} \left(  \frac{l_\mathrm{F}}{L} (mg + F_\mathrm{down}) - m a_x \frac{z_\mathrm{CG}}{L} + m a_y \frac {z_\mathrm{CG}}{W} \right) \\
F_{\mathrm{RL}, z} & = \frac{1}{2} \left(  \frac{l_\mathrm{R}}{L} (mg + F_\mathrm{down}) + m a_x \frac{z_\mathrm{CG}}{L} - m a_y \frac {z_\mathrm{CG}}{W} \right) \\
F_{\mathrm{RR}, z} & = \frac{1}{2} \left(  \frac{l_\mathrm{R}}{L} (mg + F_\mathrm{down}) + m a_x \frac{z_\mathrm{CG}}{L} + m a_y \frac {z_\mathrm{CG}}{W} \right) \\
\end{aligned}
```

```math
\begin{aligned}
\alpha_\mathrm{FL} & = \delta - \arctan\left(\frac{v_{\mathrm{FL},y}}{v_{\mathrm{FL},x}}\right) \\ 
\alpha_\mathrm{FR} & = \delta - \arctan\left(\frac{v_{\mathrm{FR},y}}{v_{\mathrm{FR},x}}\right) \\ 
\alpha_\mathrm{RL} & = - \arctan\left(\frac{v_{\mathrm{RL},y}}{v_{\mathrm{RL},x}}\right) \\ 
\alpha_\mathrm{RR} & = - \arctan\left(\frac{v_{\mathrm{RR},y}}{v_{\mathrm{RR},x}}\right) \\ 
\end{aligned}
```

## Implementation