# Copyright (c) 2024. Tudor Oancea, Matteo Berthet
import numpy as np
import casadi as ca
from icecream import ic

sym_t = ca.SX | ca.MX

g = 9.81  # gravity

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


def kin_model(rk_steps: int = 1) -> ca.Function:
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
    x_name = (
        "["
        + ", ".join(
            [
                X.name(),
                Y.name(),
                phi.name(),
                v_x.name(),
                v_y.name(),
                omega.name(),
                delta.name(),
                tau_FL.name(),
                tau_FR.name(),
                tau_RL.name(),
                tau_RR.name(),
            ]
        )
        + "]"
    )

    # control variables
    u_delta = ca.SX.sym("u_delta")
    u_tau_FL = ca.SX.sym("u_tau_FL")
    u_tau_FR = ca.SX.sym("u_tau_FR")
    u_tau_RL = ca.SX.sym("u_tau_RL")
    u_tau_RR = ca.SX.sym("u_tau_RR")
    u = ca.vertcat(u_delta, u_tau_FL, u_tau_FR, u_tau_RL, u_tau_RR)
    u_name = (
        "["
        + ", ".join(
            [
                u_delta.name(),
                u_tau_FL.name(),
                u_tau_FR.name(),
                u_tau_RL.name(),
                u_tau_RR.name(),
            ]
        )
        + "]"
    )

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
    p_name = (
        "["
        + ", ".join(
            [
                m.name(),
                I_z.name(),
                l_R.name(),
                l_F.name(),
                C_m0.name(),
                C_r0.name(),
                C_r1.name(),
                C_r2.name(),
                t_tau.name(),
                t_delta.name(),
            ]
        )
        + "]"
    )

    # actuator dynamics
    delta_dot = (u_delta - delta) / t_delta
    tau_FL_dot = (u_tau_FL - tau_FL) / t_tau
    tau_FR_dot = (u_tau_FR - tau_FR) / t_tau
    tau_RL_dot = (u_tau_RL - tau_RL) / t_tau
    tau_RR_dot = (u_tau_RR - tau_RR) / t_tau

    # longitudinal dynamics
    T = tau_FL + tau_FR + tau_RL + tau_RR  # total torque
    F_motor = C_m0 * T  # the traction force
    F_drag = -(C_r0 + C_r1 * v_x + C_r2 * v_x**2) * smooth_sgn(v_x)  # total drag force
    # F_drag = 0.0
    F_Rx = 0.5 * F_motor + F_drag  # force applied at the rear wheels
    F_Fx = 0.5 * F_motor  # force applied at the front wheels

    a_x = (F_Rx + F_Fx * ca.cos(delta)) / m
    a_y = F_Fx * ca.sin(delta) / m
    accels = ca.Function("accels", [x, p], [ca.vertcat(a_x, a_y)], ["x", "p"], ["a"])

    cont_dynamics = ca.Function(
        "kin6_model_cont",
        [x, u, p],
        [
            ca.vertcat(
                v_x * ca.cos(phi) - v_y * ca.sin(phi),
                v_x * ca.sin(phi) + v_y * ca.cos(phi),
                omega,
                a_x + v_y * omega,
                a_y - v_x * omega,
                l_F * F_Fx * ca.sin(delta) / I_z,
                delta_dot,
                tau_FL_dot,
                tau_FR_dot,
                tau_RL_dot,
                tau_RR_dot,
            )
        ],
        [x_name, u_name, p_name],
        ["x_dot"],
    )

    dt = ca.SX.sym("dt")
    scaled_dt = dt / rk_steps
    x_next = x
    for _ in range(rk_steps):
        k1 = cont_dynamics(x_next, u, p)
        k2 = cont_dynamics(x_next + scaled_dt * k1 / 2, u, p)
        k3 = cont_dynamics(x_next + scaled_dt * k2 / 2, u, p)
        k4 = cont_dynamics(x_next + scaled_dt * k3, u, p)
        x_next = x_next + scaled_dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    disc_dynamics = ca.Function(
        "kin6_model",
        [x, u, p, dt],
        [x_next, accels(x_next, p)],
        [x_name, u_name, p_name, "dt"],
        ["x_next", "[a_x, a_y]"],
    )

    # test with some data
    x0 = ca.DM([0.0, 0.0, np.pi / 2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    u0 = ca.DM([0.0, 50.0, 50.0, 50.0, 50.0])
    p0 = ca.DM(
        [230.0, 137.583, 0.7853, 0.7853, 4.950, 297.030, 16.665, 0.6784, 0.001, 0.02]
    )
    dt = 0.01
    x1, a1 = disc_dynamics(x0, u0, p0, dt)
    x2, a2 = disc_dynamics(x1, u0, p0, dt)
    x3, a3 = disc_dynamics(x2, u0, p0, dt)
    ic(x0, u0, p0, dt, x1, x2, x3, a1, a2, a3)
    return disc_dynamics


# def dyn6_model(x: ca.SX, u: ca.SX, _: ca.SX) -> ca.SX:
#     # extract state and control variables
#     phi = x[2]
#     v_x = x[3]
#     v_y = x[4]
#     omega = x[5]
#     T = x[6]
#     delta = x[7]
#     u_T = u[0]
#     u_delta = u[1]

#     # derivative of the states (used for the implicit dynamic formulation)
#     X_dot = ca.SX.sym("X_dot")
#     Y_dot = ca.SX.sym("Y_dot")
#     phi_dot = ca.SX.sym("phi_dot")
#     v_x_dot = ca.SX.sym("v_x_dot")
#     v_y_dot = ca.SX.sym("v_y_dot")
#     r_dot = ca.SX.sym("r_dot")
#     T_dot = ca.SX.sym("T_dot")
#     delta_dot = ca.SX.sym("delta_dot")

#     # accelerations
#     a_x = v_x_dot - v_y * omega
#     a_y = v_y_dot + v_x * omega

#     # vertical tire forces
#     F_downforce = 0.5 * C_downforce * v_x * v_x  # always positive
#     static_weight = 0.5 * m * g * l_F / wheelbase
#     longitudinal_weight_transfer = 0.5 * m * a_x * z_CG / wheelbase
#     lateral_weight_transfer = 0.5 * m * a_y * z_CG / axle_track
#     F_z_FL = -(
#         static_weight
#         - longitudinal_weight_transfer
#         + lateral_weight_transfer
#         + 0.25 * F_downforce
#     )
#     F_z_FR = -(
#         static_weight
#         - longitudinal_weight_transfer
#         - lateral_weight_transfer
#         + 0.25 * F_downforce
#     )
#     F_z_RL = -(
#         static_weight
#         + longitudinal_weight_transfer
#         + lateral_weight_transfer
#         + 0.25 * F_downforce
#     )
#     F_z_RR = -(
#         static_weight
#         + longitudinal_weight_transfer
#         - lateral_weight_transfer
#         + 0.25 * F_downforce
#     )

#     # longitudinal and lateral velocity of each wheel (in its own reference frame)
#     v_x_FL = v_x - 0.5 * axle_track * omega
#     v_x_FR = v_x + 0.5 * axle_track * omega
#     v_x_RL = v_x - 0.5 * axle_track * omega
#     v_x_RR = v_x + 0.5 * axle_track * omega
#     v_y_FL = v_y + l_F * omega
#     v_y_FR = v_y + l_F * omega
#     v_y_RL = v_y - l_R * omega
#     v_y_RR = v_y - l_R * omega

#     # lateral dynamics
#     alpha_FL = ca.atan2(v_y_FL, v_x_FL) - delta
#     alpha_FR = ca.atan2(v_y_FR, v_x_FR) - delta
#     alpha_RL = ca.atan2(v_y_RL, v_x_RL)
#     alpha_RR = ca.atan2(v_y_RR, v_x_RR)
#     mu_y_FL = Da * ca.sin(
#         Ca
#         * ca.aca.tan(Ba * alpha_FL - Ea * (Ba * alpha_FL - ca.aca.tan(Ba * alpha_FL)))
#     )
#     mu_y_FR = Da * ca.sin(
#         Ca
#         * ca.aca.tan(Ba * alpha_FR - Ea * (Ba * alpha_FR - ca.aca.tan(Ba * alpha_FR)))
#     )
#     mu_y_RL = Da * ca.sin(
#         Ca
#         * ca.aca.tan(Ba * alpha_RL - Ea * (Ba * alpha_RL - ca.aca.tan(Ba * alpha_RL)))
#     )
#     mu_y_RR = Da * ca.sin(
#         Ca
#         * ca.aca.tan(Ba * alpha_RR - Ea * (Ba * alpha_RR - ca.aca.tan(Ba * alpha_RR)))
#     )
#     F_y_FL = F_z_FL * mu_y_FL
#     F_y_FR = F_z_FR * mu_y_FR
#     F_y_RL = F_z_RL * mu_y_RL
#     F_y_RR = F_z_RR * mu_y_RR

#     # longitudinal dynamics
#     F_drag = -(C_r0 + C_r1 * v_x + C_r2 * v_x * v_x) * ca.tanh(1000 * v_x)
#     tandelta = ca.tan(delta)
#     ca.sinbeta = (
#         rear_weight_distribution
#         * tandelta
#         / ca.sqrt(
#             1
#             + rear_weight_distribution * rear_weight_distribution * tandelta * tandelta
#         )
#     )
#     r_kin = v_x * ca.sinbeta / l_R
#     delta_tau = K_tv * (r_kin - omega)  # torque vectoring gain
#     tau_FL = (T - delta_tau) * F_z_FL / (-m * g - 0.25 * F_downforce)
#     tau_FR = (T + delta_tau) * F_z_FR / (-m * g - 0.25 * F_downforce)
#     tau_RL = (T - delta_tau) * F_z_RL / (-m * g - 0.25 * F_downforce)
#     tau_RR = (T + delta_tau) * F_z_RR / (-m * g - 0.25 * F_downforce)
#     F_x_FL = C_m0 * tau_FL
#     F_x_FR = C_m0 * tau_FR
#     F_x_RL = C_m0 * tau_RL
#     F_x_RR = C_m0 * tau_RR

#     # complete dynamics
#     return ca.vertcat(
#         X_dot - (v_x * ca.cos(phi) - v_y * ca.sin(phi)),
#         Y_dot - (v_x * ca.sin(phi) + v_y * ca.cos(phi)),
#         phi_dot - omega,
#         m * a_x
#         - (
#             (F_x_FR + F_x_FL) * ca.cos(delta)
#             - (F_y_FR + F_y_FL) * ca.sin(delta)
#             + F_x_RR
#             + F_x_RL
#             + F_drag
#         ),
#         m * a_y
#         - (
#             (F_x_FR + F_x_FL) * ca.sin(delta)
#             + (F_y_FR + F_y_FL) * ca.cos(delta)
#             + F_y_RR
#             + F_y_RL
#         ),
#         I_z * r_dot
#         - (
#             (F_x_FR * ca.cos(delta) - F_y_FR * ca.sin(delta)) * axle_track / 2
#             + (F_x_FR * ca.sin(delta) + F_y_FR * ca.cos(delta)) * l_F
#             - (F_x_FL * ca.cos(delta) - F_y_FL * ca.sin(delta)) * axle_track / 2
#             + (F_x_FL * ca.sin(delta) + F_y_FL * ca.cos(delta)) * l_F
#             + F_x_RR * axle_track / 2
#             - F_y_RR * l_R
#             - F_x_RL * axle_track / 2
#             - F_y_RL * l_R
#         ),
#         T_dot - (u_T - T) / t_T,
#         delta_dot - (u_delta - delta) / t_delta,
# )


def main():
    kin_model_fun = kin_model(10)
    C = ca.CodeGenerator(
        "models",
        {
            "main": True,
            "mex": False,
            "with_mem": True,
            "verbose": True,
            "with_header": True,
            "indent": 4,
        },
    )
    C.add(kin_model_fun)
    C.generate()


if __name__ == "__main__":
    main()
