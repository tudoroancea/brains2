# Copyright (c) 2024. Tudor Oancea, Matteo Berthet
import os
import shutil
from typing import Literal

import casadi as ca

sym_t = ca.SX | ca.MX

g = 9.81  # gravity acceleration

####################################################################################################
# modelling utilities
####################################################################################################


def smooth_sgn(x: sym_t) -> sym_t:
    """Smooth sign function"""
    return ca.tanh(1e1 * x)


####################################################################################################
# model
####################################################################################################

nx = 8
nu = 2


def generate_model():
    """Generate the discretized curvilinear kinematic bicycle model"""
    # state variables
    x = ca.SX.sym("x", nx)
    s, n, psi, v_x, v_y, omega, delta, tau = ca.vertsplit(x, 1)

    # control variables
    u = ca.SX.sym("u", nu)
    u_delta, u_tau = ca.vertsplit(u, 1)

    # parameters
    kappa_cen = ca.SX.sym("kappa_cen")
    p = ca.SX.sym("p", 10)
    dt, m, l_R, l_F, C_m0, C_r0, C_r1, C_r2, t_tau, t_delta = ca.vertsplit(p, 1)

    # actuator dynamics
    delta_dot = (u_delta - delta) / t_delta
    tau_dot = (u_tau - tau) / t_tau

    # drivetrain dynamics
    F_motor = C_m0 * tau  # the traction force
    F_drag = -(C_r0 + C_r1 * v_x + C_r2 * v_x**2) * smooth_sgn(v_x)  # total drag force
    F_Rx = 0.5 * F_motor + F_drag  # force applied at the rear wheels
    F_Fx = 0.5 * F_motor  # force applied at the front wheels
    beta = ca.arctan(l_R / (l_R + l_F) * ca.tan(delta))
    v_dot = (ca.cos(beta) * F_Rx + ca.cos(delta - beta) * F_Fx) / m
    beta_dot = l_R / (l_R + l_F) * delta_dot

    # assemble the continuous dynamics
    s_dot = (v_x * ca.cos(psi) - v_y * ca.sin(psi)) / (1 - n * kappa_cen)
    f_cont = ca.Function(
        "f_cont",
        [x, u, p, kappa_cen],
        [
            ca.cse(
                ca.vertcat(
                    s_dot,
                    v_x * ca.sin(psi) + v_y * ca.cos(psi),
                    omega - kappa_cen * s_dot,
                    v_dot * ca.cos(beta) - v_y * beta_dot,
                    v_dot * ca.sin(beta) + v_x * beta_dot,
                    (v_dot * ca.sin(beta) + v_x * beta_dot) / l_R,
                    delta_dot,
                    tau_dot,
                )
            )
        ],
    )

    # discretize the dynamics with RK4
    k1 = f_cont(x, u, p, kappa_cen)
    k2 = f_cont(x + dt / 2 * k1, u, p, kappa_cen)
    k3 = f_cont(x + dt / 2 * k2, u, p, kappa_cen)
    k4 = f_cont(x + dt * k3, u, p, kappa_cen)
    f_disc = ca.Function(
        "f_disc",
        [x, u, p, kappa_cen],
        [ca.cse(x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4))],
    )

    return f_disc


####################################################################################################
# controller
####################################################################################################


def generate_controller(Nf: int, solver: Literal["ipopt", "fatrop"] = "ipopt"):
    """Create the MPC solver and generate C code for it"""

    ####################################################################################
    # create optimization problem and variables
    ####################################################################################
    opti = ca.Opti()
    x = []
    u = []
    for _ in range(Nf):
        x.append(opti.variable(nx))
        u.append(opti.variable(nu))
    x.append(opti.variable(nx))

    ####################################################################################
    # create optimization problem parameters
    ####################################################################################
    all_params = []
    # current state
    all_params.append((x0 := opti.parameter(nx)))

    # model parameters
    kappa_cen = opti.parameter(Nf)  # stages 0 to Nf-1
    dt = opti.parameter()
    m = opti.parameter()
    l_R = opti.parameter()
    l_F = opti.parameter()
    C_m0 = opti.parameter()
    C_r0 = opti.parameter()
    C_r1 = opti.parameter()
    C_r2 = opti.parameter()
    t_delta = opti.parameter()
    t_tau = opti.parameter()
    model_params = ca.vertcat(dt, m, l_R, l_F, C_m0, C_r0, C_r1, C_r2, t_delta, t_tau)

    # constraints parameters
    w_cen = opti.parameter(Nf)  # stages 1 to Nf
    tau_max = opti.parameter()
    delta_max = opti.parameter()
    v_x_max = opti.parameter()

    # cost function parameters
    v_x_ref = opti.parameter()
    delta_s_ref = opti.parameter()
    Q_diag = opti.parameter(nx)
    Q = ca.diag(Q_diag)
    R_diag = opti.parameter(nu)
    R = ca.diag(R_diag)
    Q_f_diag = opti.parameter(nx)
    Q_f = ca.diag(Q_f_diag)

    f_disc = generate_model()

    ####################################################################################
    # construct cost function
    ####################################################################################
    cost_function = 0.0
    # initial track progress
    s0 = x0[0]
    # steady-state torque
    tau_ref = smooth_sgn(v_x_ref) * (C_r0 + C_r1 * v_x_ref + C_r2 * v_x_ref**2) / C_m0
    for i in range(Nf):
        # stage control costs
        u_ref = ca.vertcat(0.0, tau_ref)
        u_diff = u[i] - u_ref
        cost_function += u_diff.T @ R @ u_diff
        # stage state costs (since the initial state is fixed, we can't optimize
        # the cost at stage 0 and can ignore it in the cost function)
        if i > 0:
            x_ref = ca.vertcat(
                s0 + i / Nf * delta_s_ref, 0, 0, v_x_ref, 0, 0, 0, tau_ref
            )
            x_diff = x[i] - x_ref
            cost_function += x_diff.T @ Q @ x_diff

    # terminal state costs
    x_ref = ca.vertcat(s0 + delta_s_ref, 0, 0, v_x_ref, 0, 0, 0, tau_ref)
    x_diff = x[Nf] - x_ref
    cost_function += x_diff.T @ Q_f @ x_diff
    opti.minimize(ca.cse(cost_function))

    ####################################################################################
    # formulate OCP constraints
    ####################################################################################
    # NOTE: the constraints have to be declared stage by stage for fatrop
    # to properly auto-detect the problem structure.
    for i in range(Nf):
        # equality constraints coming from the dynamics
        opti.subject_to(x[i + 1] == f_disc(x[i], u[i], model_params, kappa_cen[i]))
        # initial state constraint
        if i == 0:
            opti.subject_to(x[i] == x0)
        # control input constraints
        opti.subject_to((-tau_max <= u[i][0]) <= tau_max)
        opti.subject_to((-delta_max <= u[i][1]) <= delta_max)
        if i > 0:
            # track constraints
            opti.subject_to((-w_cen[i - 1] <= x[i][1]) <= w_cen[i - 1])
            # velocity constraints
            opti.subject_to((0.0 <= x[i][3]) <= v_x_max)
            # TODO: add constraints on current control ?
    # terminal constraints
    opti.subject_to((-w_cen[Nf - 1] <= x[Nf][1]) <= w_cen[Nf - 1])
    opti.subject_to((0.0 <= x[Nf][3]) <= v_x_max)

    ####################################################################################
    # set solver and options
    ####################################################################################
    if solver == "ipopt":
        options = {
            "print_time": 0,
            "expand": True,
            "ipopt": {"sb": "yes", "print_level": 0, "max_resto_iter": 0},
        }
    elif solver == "fatrop":
        options = {
            "print_time": 0,
            "debug": True,
            "expand": True,
            "structure_detection": "auto",
            "fatrop": {"print_level": 0},
        }
    else:
        raise ValueError(f"Unknown solver: {solver}")

    opti.solver(solver, options)

    ####################################################################################
    # convert to casadi Function and codegen
    ####################################################################################
    states = ca.horzcat(*x)
    controls = ca.horzcat(*u)
    function_name = "nmpc_solver"
    # NOTE: we group and order the parameters differently than declared above
    # to improve the external solver interface.
    solver_function = opti.to_function(
        function_name,
        [
            states,
            controls,
            x0,
            kappa_cen,
            w_cen,
            model_params,
            ca.vertcat(v_x_max, delta_max, tau_max),
            ca.vertcat(v_x_ref, delta_s_ref, Q_diag, R_diag, Q_f_diag),
        ],
        [states, controls],
        [
            "x_guess",
            "u_guess",
            "x0",
            "kappa_cen",
            "w_cen",
            "model_params",
            "limits",
            "cost_params",
        ],
        ["x_opt", "u_opt"],
    )
    solver_function.generate(
        function_name,
        {
            "with_mem": True,
            "with_header": True,
            "verbose": True,
            "indent": 4,
            "main": True,
        },
    )
    os.makedirs("generated", exist_ok=True)
    shutil.move(f"{function_name}.c", f"generated/{function_name}.c")
    shutil.move(f"{function_name}.h", f"generated/{function_name}.h")
    # cleanup temporary files
    for file in os.listdir("."):
        if file.endswith(".mtx") or file.endswith(".casadi"):
            os.remove(file)


if __name__ == "__main__":
    # TODO: change 10 to a reasonable value
    generate_controller(Nf=10, solver="fatrop")
