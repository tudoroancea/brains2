# control

## Input/outputs

We receive from the estimation module (or the simulation in certain scenarios):

- the car's pose $X,Y,\varphi$ in global cartesian coordinates
- the car's longitudinal, lateral and angular velocity $v_x,v_y,\omega$ (in local coordinates)
- an estimate of the track around us in the form of splines of degree 1 for the
  center line, its heading, its curvature, and the track width:

  ```math
  X^\mathrm{cen}(s),Y^\mathrm{cen}(s),\varphi^\mathrm{cen}(s),\kappa^\mathrm{cen}(s),w^\mathrm{cen}(s)\,.
  ```

  These splines are represented by a set of values for the track progress
  $\{s_i\}_{i=1,\dots,M}$ and corresponding values for each quantity. We always
  expect the same number of points for this representations to facilitate the
  usage of splines in the OCP implementation.

  The track progress values $s_i$ are also supposed to be consistent throughout
  the drive. This means that they should be extracted from the same global
  representation of the track, in such a way that a certain position would have
  the same projection $s$ on two consecutive local tracks.

## Concept

We formulate an OCP by taking inspiration from the paper [**_"NMPC for
Racing Using a Singularity-Free Path-Parametric Model with Obstacle
Avoidance"_**](https://www.sciencedirect.com/science/article/pii/S2405896320317845).
This formulation has the advantage of being computationally simpler than MPCC
because of its quadratic cost function and the curvilinear model that simplifies
the state constraints formulation.

In milestone v0, we chose not to focus on actual racing, but rather on simply 
tracking the center line. The quadratic cost function of the paper above allows
us to eventually transition from one goal to the other by simply changing the 
reference states and cost tuning.

In this milestone, we also use the state ground truth, which makes the optimal
state and control trajectories quite smooth with respect to the initial conditions,
thus allowing us to omit some details for now.

Concretely, our formulation:
- uses a 4DOF kinematic bicycle model in the Frenet frame (curvilinear
  coordinates) and no actuator dynamics.
- only follow the center line with a certain target velocity.
- choses evenly spaced points on the center line as state reference.
- includes hard constraints on the velocity, lateral deviation and controls.
- doesn't include any control rate constraints or penalties because the .
- 

This already constitutes a working first implementation that lets us test the
other modules.
Once we implement the estimation modules and build up a more comprehensive test
suite (both unit and integration tests) and are able to better figure out the
edge cases not covered by the current formulation, we will progressively
include the following improvements:
- soft constraints on velocity and track constraints
- control rate constraints and penalties
- local racing line optimization by choosing a state reference slightly out of
  reach.
- **global race line optimization**
  After the 1st exploration lap, we use the past trajectory as a raceline and change the costs for the exploitation phase. We can re-estimate it lap after lap, but
  we keep track of the lateral deviation $n$ and the heading deviation $\psi$ during the exploration phase, and then we recompute the actual taken poses based on the adjusted center line (after loop closure)

  > [!WARNING]: during the first lap the first and last points will not be the same !!!!!

  ⇒ we stitch the last prediction with the beginning of the lap

  we compute the curvature values outside of the OCP, based on the initial guess, constructed based on the last prediction extended by the same control

  we could use this not as reference but as terminal target

## OCP Description

### State and control variables

The state and control variables are:
```math
\begin{aligned}
x &= (\Delta s, n, \psi, v)^T, \\
u &= (\delta, \tau)^T,
\end{aligned}
```

where $\Delta s = s - s_0$ is the track progress since the first OCP stage
(i.e. the current state), $n$ is the lateral deviation from the center line,
$\psi$ is the heading deviation from the center line, and
$v = \sqrt{v_x^2+v_y^2}$ is the absolute velocity.

### Dynamics

```math
\begin{aligned}
\Delta \dot{s} &= \dot{s} = \frac{v \cos(\psi + beta)}{1 + n \kappa^\mathrm{cen}(s)}, \\
\dot{n} &= v \sin(\psi + \beta), \\
\dot{\psi} &= \frac{v \sin(\beta)}{l_R} + \kappa^\mathrm{cen}(s) \dot{s}, \\
\dot{v} &= \frac{1}{m}(C_\mathrm{m0}\tau - (C_\mathrm{r0} + C_\mathrm{r1} v + C_\mathrm{r2} v^2)), \\
\end{aligned}
```

where $\beta = \frac{l_R}{l_R + l_F}\delta$ is the slip angle.

### Objective function

We formulate a quadratic objective function as follows:

```math
J(\mathbf{x},\mathbf{u}) = \|x_N-x^\mathrm{ref}_N\|^2_{Q_f} + \sum_{k=0}^{N-1} \|x_k-x^\mathrm{ref}_k\|^2_Q + \|u_k-u^\mathrm{ref}_k\|^2_R
```

where the reference states and controls are chosen as follows:

- The reference states $x^\mathrm{ref}_k$ are chosen as

  ```math
  x^\mathrm{ref}_k = (k \Delta t v^\mathrm{ref}, 0, 0, v^\mathrm{ref})^T
  ```

  where $v^\mathrm{ref}$ is the reference velocity and $\Delta t$ is the sampling
  time.

  > [!NOTE]
  > Since we chose to represent the state as $\Delta s$ instead of $s$, we
  > can use the same reference and initial guess at every call to the controller.

- The reference controls $u^\mathrm{ref}_k$ are chosen as

  ```math
  u^\mathrm{ref}_k = (0, \tau^\mathrm{ref})^T
  ```

  where $\tau^\mathrm{ref}$ is the steady state torque to be applied for the
  velocity $v^\mathrm{ref}$.

### Constraints

For the moment, we only impose:

- **velocity constraints**: $0 \leq v \leq v_\mathrm{max}$ ,
- **track constraints**: $-w^{\mathrm{cen}}(s) \leq n \leq w^{\mathrm{cen}}(s)$, where $w^{\mathrm{cen}}$ is the track width,
- **control input constraints**: $-\delta_\mathrm{max} \leq \delta \leq \delta_\mathrm{max}$ and $-\tau_\mathrm{max} \leq \tau \leq \tau_\mathrm{max}$ ,

## Implementation

We implement the OCP formulation directly in C++ using the `Opti` class in
`casadi` and the `fatrop` solver.

The provided initial guess is exactly the state and control reference, which
should be sufficiently close to the optimal solution. This strategy has been 
observed to work well in practice, and has the advantage of being _stateless_,
in the sense that we don't need to store any state between calls to the
controller and there should not be erros that can be propagated from one call
to the next.

To simplify the optimization problem and the runtime, we evaluate all the
variables depending on `s` (like the curavture `kappa^\mathrm{cen}(s)`
and the track width `w^\mathrm{cen}(s)`) outside of the OCP, and fix the
values at each stage throughout the optimization.
The values of `s` are chosen based on the initial guess.
