# Copyright (c) 2024. Tudor Oancea, Matteo Berthet
import shutil
import os
import numpy as np
import casadi as ca
from acados_template import AcadosSimOptions, AcadosModel, AcadosSim, AcadosSimSolver

sym_t = ca.SX | ca.MX

g = 9.81  # gravity acceleration

####################################################################################################
# modelling utilities
####################################################################################################


def smooth_dev(x: sym_t) -> sym_t:
    return x + 1e-6 * ca.exp(-x * x)


def smooth_sgn(x: sym_t) -> sym_t:
    return ca.tanh(1e1 * x)


def smooth_abs(x: sym_t) -> sym_t:
    return smooth_sgn(x) * x


def smooth_abs_nonzero(x: sym_t, min_val: float = 1e-6) -> sym_t:
    return smooth_abs(x) + min_val * ca.exp(-x * x)


####################################################################################################
# models
####################################################################################################


def gen_kin6_model(rk_steps: int = 1):
    """
    create a function with inputs current state, parameters, and sampling time, and output the next state and a_x, a_y

    state: x = (X, Y, phi, v_x, v_y, omega, delta, tau_FL, tau_FR, tau_RL, tau_RR)
    control: u = (u_delta, u_tau_FL, u_tau_FR, u_tau_RL, u_tau_RR)
    parameters: p = (m, I_z, l_R, l_F, C_m0, C_r0, C_r1, C_r2, t_T, t_delta)
    """
    # state variables
    X = ca.SX.sym("X")
    Y = ca.SX.sym("Y")
    phi = ca.SX.sym("phi")
    v_x = ca.SX.sym("v_x")
    v_y = ca.SX.sym("v_y")
    omega = ca.SX.sym("omega")
    delta = ca.SX.sym("delta")
    tau_FL = ca.SX.sym("tau_FL")
    tau_FR = ca.SX.sym("tau_FR")
    tau_RL = ca.SX.sym("tau_RL")
    tau_RR = ca.SX.sym("tau_RR")
    x = ca.vertcat(X, Y, phi, v_x, v_y, omega, delta, tau_FL, tau_FR, tau_RL, tau_RR)

    # control variables
    u_delta = ca.SX.sym("u_delta")
    u_tau_FL = ca.SX.sym("u_tau_FL")
    u_tau_FR = ca.SX.sym("u_tau_FR")
    u_tau_RL = ca.SX.sym("u_tau_RL")
    u_tau_RR = ca.SX.sym("u_tau_RR")
    u = ca.vertcat(u_delta, u_tau_FL, u_tau_FR, u_tau_RL, u_tau_RR)

    # parameters
    m = ca.SX.sym("m")
    I_z = ca.SX.sym("I_z")
    l_R = ca.SX.sym("l_R")
    l_F = ca.SX.sym("l_F")
    C_m0 = ca.SX.sym("C_m0")
    C_r0 = ca.SX.sym("C_r0")
    C_r1 = ca.SX.sym("C_r1")
    C_r2 = ca.SX.sym("C_r2")
    t_tau = ca.SX.sym("t_tau")
    t_delta = ca.SX.sym("t_delta")
    p = ca.vertcat(m, I_z, l_R, l_F, C_m0, C_r0, C_r1, C_r2, t_tau, t_delta)

    # actuator dynamics
    delta_dot = (u_delta - delta) / t_delta
    tau_FL_dot = (u_tau_FL - tau_FL) / t_tau
    tau_FR_dot = (u_tau_FR - tau_FR) / t_tau
    tau_RL_dot = (u_tau_RL - tau_RL) / t_tau
    tau_RR_dot = (u_tau_RR - tau_RR) / t_tau

    # drivetrain
    T = tau_FL + tau_FR + tau_RL + tau_RR  # total torque
    F_motor = C_m0 * T  # the traction force
    F_drag = -(C_r0 + C_r1 * v_x + C_r2 * v_x**2) * smooth_sgn(v_x)  # total drag force
    F_Rx = 0.5 * F_motor + F_drag  # force applied at the rear wheels
    F_Fx = 0.5 * F_motor  # force applied at the front wheels
    beta = ca.arctan(l_R / (l_R + l_F) * ca.tan(delta))
    v_dot = (ca.cos(beta) * F_Rx + ca.cos(delta - beta) * F_Fx) / m
    beta_dot = l_R / (l_R + l_F) * delta_dot

    # accelerations
    a_x = (F_Rx + F_Fx * ca.cos(delta)) / m
    a_y = F_Fx * ca.sin(delta) / m
    accels = ca.Function(
        "kin6_accels", [x, p], [ca.vertcat(a_x, a_y)], ["x", "p"], ["a"]
    )

    # assemble the continuous dynamics
    xdot = ca.cse(
        ca.vertcat(
            v_x * ca.cos(phi) - v_y * ca.sin(phi),
            v_x * ca.sin(phi) + v_y * ca.cos(phi),
            omega,
            v_dot * ca.cos(beta) - v_y * beta_dot,
            v_dot * ca.sin(beta) + v_x * beta_dot,
            (v_dot * ca.sin(beta) + v_x * beta_dot) / l_R,
            delta_dot,
            tau_FL_dot,
            tau_FR_dot,
            tau_RL_dot,
            tau_RR_dot,
        )
    )

    model = AcadosModel()
    model.name = "kin6"
    model.x = x
    model.u = u
    model.p = p
    model.f_expl_expr = xdot
    model.xdot = ca.SX.sym("xdot", x.shape)
    model.f_impl_expr = model.xdot - xdot

    # Create simulation solver options
    sim_opts = AcadosSimOptions()
    sim_opts.T = 0.01  # will be overwritten by the simulation
    sim_opts.num_stages = 4  # number of collocation points
    sim_opts.num_steps = 10  # number of time steps
    sim_opts.integrator_type = "IRK"
    sim_opts.collocation_type = "GAUSS_RADAU_IIA"

    # Generate simulation solver code
    sim = AcadosSim()
    sim.model = model
    sim.solver_options = sim_opts
    sim.code_export_directory = "generated"
    sim.parameter_values = np.ones(
        model.p.shape
    )  # will be overwritten by the simulation
    AcadosSimSolver.generate(sim, json_file="generated/kin6_model.json")

    # Generate casadi function code for accelerations
    accels.generate(
        "kin6_accels",
        {
            "main": False,
            "mex": False,
            "with_mem": True,
            "verbose": True,
            "with_header": True,
            "indent": 4,
        },
    )
    shutil.move("kin6_accels.c", "generated/kin6_accels.c")
    shutil.move("kin6_accels.h", "generated/kin6_accels.h")

    # remove useless files
    os.remove("generated/main_sim_kin6.c")
    os.remove("generated/Makefile")
    os.remove("generated/acados_sim_solver.pxd")


def gen_dyn6_model() -> ca.SX:
    # state variables
    X = ca.SX.sym("X")
    Y = ca.SX.sym("Y")
    phi = ca.SX.sym("phi")
    v_x = ca.SX.sym("v_x")
    v_y = ca.SX.sym("v_y")
    omega = ca.SX.sym("omega")
    delta = ca.SX.sym("delta")
    tau_FL = ca.SX.sym("tau_FL")
    tau_FR = ca.SX.sym("tau_FR")
    tau_RL = ca.SX.sym("tau_RL")
    tau_RR = ca.SX.sym("tau_RR")
    x = ca.vertcat(X, Y, phi, v_x, v_y, omega, delta, tau_FL, tau_FR, tau_RL, tau_RR)

    # control variables
    u_delta = ca.SX.sym("u_delta")
    u_tau_FL = ca.SX.sym("u_tau_FL")
    u_tau_FR = ca.SX.sym("u_tau_FR")
    u_tau_RL = ca.SX.sym("u_tau_RL")
    u_tau_RR = ca.SX.sym("u_tau_RR")
    u = ca.vertcat(u_delta, u_tau_FL, u_tau_FR, u_tau_RL, u_tau_RR)

    # parameters
    m = ca.SX.sym("m")
    I_z = ca.SX.sym("I_z")
    l_R = ca.SX.sym("l_R")
    l_F = ca.SX.sym("l_F")
    C_m0 = ca.SX.sym("C_m0")
    C_r0 = ca.SX.sym("C_r0")
    C_r1 = ca.SX.sym("C_r1")
    C_r2 = ca.SX.sym("C_r2")
    t_tau = ca.SX.sym("t_tau")
    t_delta = ca.SX.sym("t_delta")
    z_CG = ca.SX.sym("z_CG")
    axle_track = ca.SX.sym("axle_track")
    C_downforce = ca.SX.sym("C_downforce")
    Ba = ca.SX.sym("Ba")
    Ca = ca.SX.sym("Ca")
    Da = ca.SX.sym("Da")
    Ea = ca.SX.sym("Ea")
    p = ca.vertcat(
        m,
        I_z,
        l_R,
        l_F,
        C_m0,
        C_r0,
        C_r1,
        C_r2,
        t_tau,
        t_delta,
        z_CG,
        axle_track,
        C_downforce,
        Ba,
        Ca,
        Da,
        Ea,
    )
    wheelbase = l_R + l_F
    # rear_weight_distribution = l_R / wheelbase

    # derivative of the states (used for the implicit dynamic formulation)
    X_dot = ca.SX.sym("X_dot")
    Y_dot = ca.SX.sym("Y_dot")
    phi_dot = ca.SX.sym("phi_dot")
    v_x_dot = ca.SX.sym("v_x_dot")
    v_y_dot = ca.SX.sym("v_y_dot")
    omega_dot = ca.SX.sym("omega_dot")
    delta_dot = ca.SX.sym("delta_dot")
    tau_FL_dot = ca.SX.sym("tau_FL_dot")
    tau_FR_dot = ca.SX.sym("tau_FR_dot")
    tau_RL_dot = ca.SX.sym("tau_RL_dot")
    tau_RR_dot = ca.SX.sym("tau_RR_dot")
    xdot = ca.vertcat(
        X_dot,
        Y_dot,
        phi_dot,
        v_x_dot,
        v_y_dot,
        omega_dot,
        delta_dot,
        tau_FL_dot,
        tau_FR_dot,
        tau_RL_dot,
        tau_RR_dot,
    )

    # accelerations
    a_x = v_x_dot - v_y * omega
    a_y = v_y_dot + v_x * omega

    # vertical tire forces
    front_weight_distribution = 0.5 * l_F / wheelbase
    rear_weight_distribution = 0.5 * l_R / wheelbase
    static_weight = m * g
    F_downforce = 0.5 * C_downforce * v_x * v_x
    longitudinal_weight_transfer = 0.5 * m * a_x * z_CG / wheelbase
    lateral_weight_transfer = 0.5 * m * a_y * z_CG / axle_track
    F_z_FL = -(
        front_weight_distribution * (static_weight + F_downforce)
        - longitudinal_weight_transfer
        + lateral_weight_transfer
    )
    F_z_FR = -(
        front_weight_distribution * (static_weight + F_downforce)
        - longitudinal_weight_transfer
        - lateral_weight_transfer
    )
    F_z_RL = -(
        rear_weight_distribution * (static_weight + F_downforce)
        + longitudinal_weight_transfer
        + lateral_weight_transfer
    )
    F_z_RR = -(
        rear_weight_distribution * (static_weight + F_downforce)
        + longitudinal_weight_transfer
        - lateral_weight_transfer
    )

    # longitudinal and lateral velocity of each wheel (in its own reference frame)
    v_x_FL = v_x - 0.5 * axle_track * omega
    v_x_FR = v_x + 0.5 * axle_track * omega
    v_x_RL = v_x - 0.5 * axle_track * omega
    v_x_RR = v_x + 0.5 * axle_track * omega
    v_y_FL = v_y + l_F * omega
    v_y_FR = v_y + l_F * omega
    v_y_RL = v_y - l_R * omega
    v_y_RR = v_y - l_R * omega

    # lateral dynamics
    alpha_FL = ca.atan2(v_y_FL, v_x_FL) - delta
    alpha_FR = ca.atan2(v_y_FR, v_x_FR) - delta
    alpha_RL = ca.atan2(v_y_RL, v_x_RL)
    alpha_RR = ca.atan2(v_y_RR, v_x_RR)
    mu_y_FL = Da * ca.sin(
        Ca * ca.arctan(Ba * alpha_FL - Ea * (Ba * alpha_FL - ca.arctan(Ba * alpha_FL)))
    )
    mu_y_FR = Da * ca.sin(
        Ca * ca.arctan(Ba * alpha_FR - Ea * (Ba * alpha_FR - ca.arctan(Ba * alpha_FR)))
    )
    mu_y_RL = Da * ca.sin(
        Ca * ca.arctan(Ba * alpha_RL - Ea * (Ba * alpha_RL - ca.arctan(Ba * alpha_RL)))
    )
    mu_y_RR = Da * ca.sin(
        Ca * ca.arctan(Ba * alpha_RR - Ea * (Ba * alpha_RR - ca.arctan(Ba * alpha_RR)))
    )
    F_y_FL = F_z_FL * mu_y_FL
    F_y_FR = F_z_FR * mu_y_FR
    F_y_RL = F_z_RL * mu_y_RL
    F_y_RR = F_z_RR * mu_y_RR

    # longitudinal dynamics
    F_drag = -(C_r0 + C_r1 * v_x + C_r2 * v_x * v_x) * ca.tanh(10 * v_x)
    F_x_FL = C_m0 * tau_FL
    F_x_FR = C_m0 * tau_FR
    F_x_RL = C_m0 * tau_RL
    F_x_RR = C_m0 * tau_RR

    # complete dynamics
    model = AcadosModel()
    model.name = "dyn6"
    model.x = x
    model.u = u
    model.p = p
    model.xdot = xdot
    model.f_impl_expr = ca.cse(
        ca.vertcat(
            X_dot - (v_x * ca.cos(phi) - v_y * ca.sin(phi)),
            Y_dot - (v_x * ca.sin(phi) + v_y * ca.cos(phi)),
            phi_dot - omega,
            m * a_x
            - (
                (F_x_FR + F_x_FL) * ca.cos(delta)
                - (F_y_FR + F_y_FL) * ca.sin(delta)
                + F_x_RR
                + F_x_RL
                + F_drag
            ),
            m * a_y
            - (
                (F_x_FR + F_x_FL) * ca.sin(delta)
                + (F_y_FR + F_y_FL) * ca.cos(delta)
                + F_y_RR
                + F_y_RL
            ),
            I_z * omega_dot
            - (
                (F_x_FR * ca.cos(delta) - F_y_FR * ca.sin(delta)) * axle_track / 2
                + (F_x_FR * ca.sin(delta) + F_y_FR * ca.cos(delta)) * l_F
                - (F_x_FL * ca.cos(delta) - F_y_FL * ca.sin(delta)) * axle_track / 2
                + (F_x_FL * ca.sin(delta) + F_y_FL * ca.cos(delta)) * l_F
                + F_x_RR * axle_track / 2
                - F_y_RR * l_R
                - F_x_RL * axle_track / 2
                - F_y_RL * l_R
            ),
            delta_dot - (u_delta - delta) / t_delta,
            tau_FL_dot - (u_tau_FL - tau_FL) / t_tau,
            tau_FR_dot - (u_tau_FR - tau_FR) / t_tau,
            tau_RL_dot - (u_tau_RL - tau_RL) / t_tau,
            tau_RR_dot - (u_tau_RR - tau_RR) / t_tau,
        )
    )

    # Create simulation solver options
    sim_opts = AcadosSimOptions()
    sim_opts.T = 0.01  # will be overwritten by the simulation
    sim_opts.num_stages = 4  # number of collocation points
    sim_opts.num_steps = 10  # number of time steps
    sim_opts.integrator_type = "IRK"
    sim_opts.collocation_type = "GAUSS_RADAU_IIA"

    # Generate simulation solver code
    sim = AcadosSim()
    sim.model = model
    sim.solver_options = sim_opts
    sim.code_export_directory = "generated"
    sim.parameter_values = np.ones(
        model.p.shape
    )  # will be overwritten by the simulation
    AcadosSimSolver.generate(sim, json_file="generated/dyn6_model.json")

    # remove useless files
    os.remove("generated/main_sim_dyn6.c")
    os.remove("generated/Makefile")
    os.remove("generated/acados_sim_solver.pxd")

    # accelerations
    alpha_F = ca.atan2(v_y_FL, v_x) - delta
    mu_F = Da * ca.sin(
        Ca * ca.arctan(Ba * alpha_F - Ea * (Ba * alpha_F - ca.arctan(Ba * alpha_F)))
    )
    tpr = (l_R * (static_weight + F_downforce) + z_CG * F_drag) / wheelbase
    a_x_estimate = (
        (F_x_RL + F_x_RR)
        + (F_x_FL + F_x_FR) * ca.cos(delta)
        + F_drag
        - mu_F * ca.sin(delta) * tpr
    )
    a_y_estimate = (
        (F_x_FL + F_x_FR) * ca.sin(delta)
        + (tpr - m * a_x_estimate * z_CG / wheelbase) * ca.cos(delta)
        + (tpr + m * a_x_estimate * z_CG / wheelbase)
    )
    accels = ca.Function(
        "dyn6_accels",
        [x, p],
        [ca.vertcat(a_x_estimate, a_y_estimate)],
        ["x", "p"],
        ["a"],
    )
    accels.generate(
        "dyn6_accels",
        {
            "main": False,
            "mex": False,
            "with_mem": True,
            "verbose": True,
            "with_header": True,
            "indent": 4,
        },
    )
    shutil.move("dyn6_accels.c", "generated/dyn6_accels.c")
    shutil.move("dyn6_accels.h", "generated/dyn6_accels.h")


def main():
    gen_kin6_model()
    gen_dyn6_model()


if __name__ == "__main__":
    main()
