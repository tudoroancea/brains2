# control

## Input/outputs

We receive from the estimation module (or the simulation in certain scenarios):

- the car's pose $X,Y,\varphi$ in global cartesian coordinates
- car accelerations $a_x,a_y$ in global cartesian coordinates
- the car's linear and angular velocity $v_x,v_y,\omega$ in car local
  coordinates
- an estimate of the track around us in the form of splines of degree 1 for the
  center line, its heading, its curvature, and the track width:

  ```math
  X^\mathrm{cen}(s),Y^\mathrm{cen}(s),\varphi^\mathrm{cen}(s),\kappa^\mathrm{cen}(s),w^\mathrm{cen}(s)\,.
  ```

  These splines are represented by a set of values for the track progress
  $\{s_i\}_{i=1,\dots,M}$ and corresponding values for each quantity. We always
  expect the same number of points for this representations to facilitate the
  usage of splines in the OCP implementation.

  > Roughly 200 points should be enough for up to 15m of local track.

  The track progress values $s_i$ are also supposed to be consistent throughout
  the drive. This means that they should be extracted from the same global
  representation of the track, in such a way that a certain position would have
  the same projection $s$ on two consecutive local tracks.

## Concept

#### V1

- We only follow the center line with a certain target velocity.
- We use a curvilinear temporal model.
- We formulate the OCP by taking inspiration from the paper [**_"NMPC for
  Racing Using a Singularity-Free Path-Parametric Model with Obstacle
  Avoidance"_**](https://www.sciencedirect.com/science/article/pii/S2405896320317845),
  where the 'racing objective' is implemented as a simple quadratic cost with
  respect to a target progress variable $s^\mathrm{ref}_f$ slightly out of reach
- The track constraints are simple bounds on the lateral deviation (we ignore
  heading for the moment).
- For the moment, we put high enough cost on lateral deviation such that we
  track the center line instead of actually racing it.

#### For later versions

- [ ] Incorporate heading into track contraints
- [ ] Retune to do racing instead of center-line tracking
- [ ] **global race line optimization**
      After the 1st exploration lap, we use the past trajectory as a raceline and change the costs for the exploitation phase. We can re-estimate it lap after lap, but
      we keep track of the lateral deviation $n$ and the heading deviation $\psi$ during the exploration phase, and then we recompute the actual taken poses based on the adjusted center line (after loop closure)

  > !NOTE: during the first lap the first and last points will not be the same !!!!!

  ⇒ we stitch the last prediction with the beginning of the lap

  we compute the curvature values outside of the OCP, based on the initial guess, constructed based on the last prediction extended by the same control

  we could use this not as reference but as terminal target

## OCP Description

### State and control variables

```math
\begin{aligned}
x &= (s, n, \psi, v_x, v_y, \omega, \delta, \tau)^T, \\
u &= (u_\delta, u_\tau)^T
\end{aligned}
```

where $s$ is the progress along the center line, $n$ is the lateral deviation from the center line, $\psi$ is the heading deviation from the center line.
These three variabels form the _Frenet pose_.

Based on the center line data $X^\mathrm{cen}(s), Y^\mathrm{cen}(s), \varphi^\mathrm{cen}(s)$ and the _cartesian pose_ $(X, Y, \varphi)$, we can compute the
Frenet pose with:

```math
\begin{aligned}
s &= \argmin_{\sigma} \sqrt{(X^\mathrm{cen}(\sigma) - X)^2 + (Y^\mathrm{cen}(\sigma) - Y)^2}, \\
n &= -(X-X^\mathrm{cen}(s)) \sin(\varphi^\mathrm{cen}(s)) + (Y-Y^\mathrm{cen}(s)) \cos(\varphi^\mathrm{cen}(s)), \\
\psi &= \varphi - \varphi^\mathrm{cen}(s)
\end{aligned}
```

### Dynamics

```math
\begin{aligned}
\dot{s} &= \frac{v_x \cos(\psi) - v_y \sin(\psi)}{1-n \kappa^\mathrm{cen}(s)}, \\
\dot{n} &= v_x \sin(\psi) + v_y \cos(\psi), \\
\dot{\psi} &= \omega - \kappa^\mathrm{cen}(s) \dot{s} \\
\dot{v}_x &= \dot{v} \cos(\beta) - v_y \dot{\beta}, \\
\dot{v}_y &= \dot{v} \sin(\beta) + v_x \dot{\beta}, \\
\dot{\omega} &= \frac{1}{l_R}(\dot{v} \sin(\beta) + v_x \dot{\beta}), \\
\dot{\delta} &= \frac{1}{t_\delta}(u_\delta - \delta), \\
\dot{\tau} &= \frac{1}{t_\tau}(u_\tau - \tau)
\end{aligned}
```

where $\beta = \frac{l_R}{l_R + l_F}\delta$ is the slip angle, $\dot{\beta} = \frac{l_R}{l_R + l_F} \dot{\delta}$ its derivative,
$\dot{v}=\frac{1}{m}(\cos(\beta) (\frac{1}{2}F_\mathrm{motor}+F_\mathrm{drag}) + \frac{1}{2}\sin(\beta)F_\mathrm{motor})$ is the
derivative of the absolute velocity $v=\sqrt{v_x^2+v_y^2}$, $F_\mathrm{motor}=C_\mathrm{m0}\tau$ is the force produce by the drivetrain,
and finally $F_\mathrm{drag}=-C_\mathrm{r0} - C_\mathrm{r1} v_x - C_\mathrm{r2} v_x^2$ is the drag force.

### Objective function

We formulate a quadratic objective function as follows:

```math
J(\mathbf{x},\mathbf{u}) = \|x_N-x^\mathrm{ref}_N\|^2_{Q_f} + \sum_{k=0}^{N-1} \|x_k-x^\mathrm{ref}_k\|^2_Q + \|u_k-u^\mathrm{ref}_k\|^2_{R_u} + \|\dot{u}_k\|^2_{R_{du}}
```

where we wrote $\dot{u}=(\dot{\delta}, \dot{\tau})^T$ the control rates, and the reference states and controls are chosen as follows:

- The reference states $x^\mathrm{ref}_k$ are chosen as

  ```math
  x^\mathrm{ref}_k = (s_0 + \frac{k}{N}s^\mathrm{ref}_f, 0, 0, v^\mathrm{ref}_x, 0,0, 0, \tau^\mathrm{ref}_k)^T
  ```

  where $v^\mathrm{ref}_x$ and $s^\mathrm{ref}_f$ are two tuning parameter that
  one should choose to be respectively _the expected velocity throughout the
  horizon_ and _a length slightly larger than the expected progress over the
  horizon_.

  > In practice we could choose $s^\mathrm{ref}_f \approx \frac{3}{2} T_f
  > v^\mathrm{ref}_x$.

- The reference controls $u^\mathrm{ref}_k$ are chosen as

  ```math
  u^\mathrm{ref}_k = (0, \tau^\mathrm{ref}_k)^T
  ```

  where $\tau^\mathrm{ref}_k$ is the steady state torque to be applied for the
  velocity $v^\mathrm{ref}_x$.

### Constraints

For the moment, we only impose:

- **velocity constraints**: $0 \leq v_x \leq v^\mathrm{max}_x$ ,
- **track constraints**: $-w^{\mathrm{cen}} \leq n \leq w^{\mathrm{cen}}$, where $w^{\mathrm{cen}}$ is the track width (at the current track progress $s$) ,
- **control input constraints**: $-\delta_\mathrm{max} \leq \delta \leq \delta_\mathrm{max}$ and $-\tau_\mathrm{max} \leq \tau \leq \tau_\mathrm{max}$ ,
- **control rate constrains**: $\dot{\delta}_\mathrm{max} \leq \dot{\delta} \leq \dot{\delta}_\mathrm{max}$ and $\dot{\tau}_\mathrm{max} \leq \dot{\tau} \leq \dot{\tau}_\mathrm{max}$ .

> In the future, we may also incoporate the heading in the track contraints.

## Implementation

TODO