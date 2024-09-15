# Copyright (c) 2024. Tudor Oancea, Matteo Berthet
import casadi as ca

sym_t = ca.SX | ca.MX
# from casadi import (
#     MX,
#     ca.SX,
#     Function,
#     atan,
#     ca.atan2,
#     ca.cos,
#     hypot,
#     interpolant,
#     ca.sin,
#     sqrt,
#     tan,
#     tanh,
#     ca.vertcat,
# )

g = 9.81  # gravity

# # car inertia parameters
# m = 230.0  # mass
# I_z = 137.583  # yaw moment of inertia

# # geometric parameters
# z_CG = 0.295  # height of center of gravity
# front_axle_track = rear_axle_track = axle_track = 1.24  # wheelbase
# l_R = 0.7853  # distance from CoG to rear axle
# l_F = 0.7853  # distance from CoG to front axle
# wheelbase = 1.5706  # distance between the two axles
# rear_weight_distribution = l_R / wheelbase
# front_weight_distribution = l_F / wheelbase
# car_length = 3.19  # length of the car
# car_width = 1.55  # width of the car

# # drivetrain parameters (simplified)
# C_m0 = 4.950
# C_r0 = 297.030
# C_r1 = 16.665
# C_r2 = 0.6784

# # Pacejka base parameters
# b1s = -6.75e-6
# b2s = 1.35e-1
# b3s = 1.2e-3
# c1s = 1.86
# d1s = 1.12e-4
# d2s = 1.57
# e1s = -5.38e-6
# e2s = 1.11e-2
# e3s = -4.26
# b1a = 3.79e1
# b2a = 5.28e2
# c1a = 1.57
# d1a = -2.03e-4
# d2a = 1.77
# e1a = -2.24e-3
# e2a = 1.81

# # Pacejka parameters (constant version)
# static_weight = 0.5 * m * g * l_F / wheelbase
# BCDs = (b1s * static_weight**2 + b2s * static_weight) * np.exp(-b3s * static_weight)
# Cs = c1s
# Ds = d1s * static_weight + d2s
# Es = e1s * static_weight**2 + e2s * static_weight + e3s
# Bs = BCDs / (Cs * Ds)
# BCDa = b1a * np.ca.sin(2 * np.arcca.tan(static_weight / b2a))
# Ca = c1a
# Da = d1a * static_weight + d2a
# Ea = e1a * static_weight + e2a
# Ba = BCDa / (Ca * Da)

# # wheel parameters (ony used in dyn10 model)
# R_w = 0.20809  # wheel radius
# I_w = 0.3  # wheel inertia
# k_d = 0.17  # dynamic friction coefficient
# k_s = 15.0  # static friction coefficient

# # time constants of actuators
# t_T = 1e-3  # time constant for throttle actuator
# t_delta = 0.02  # time constant for steering actuator

# # aerodynamic parameters
# C_downforce = 3.96864

# # torque vectoring gains
# K_tv = 300.0

# marginal_constant = t_T * BCDs * (R_w * R_w / I_w + 1 / 0.3)
marginal_constant = 0.01

# alpha = ca.SX.sym("alpha")
# lat_pacejka = Function(
#     "lat_pacejka",
#     [alpha],
#     [Da * ca.sin(Ca * ca.aca.tan(Ba * alpha - Ea * (Ba * alpha - ca.aca.tan(Ba * alpha))))],
# )
# s = ca.SX.sym("s")
# lon_pacejka = Function(
#     "lon_pacejka",
#     [s],
#     [Ds * ca.sin(Cs * ca.aca.tan(Bs * s - Es * (Bs * s - ca.aca.tan(Bs * s))))],
# )
# del alpha, s

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


# def get_drag_force(v_x: sym_t) -> sym_t:
#     """the drag force (aerodynamic and rolling resistance)"""
#     return -(C_r0 + C_r1 * v_x + C_r2 * v_x * v_x) * smooth_sgn(v_x)
# def get_motor_force(T: sym_t) -> sym_t:
#     return C_m0 * T
# def get_pose_derivative_cartesian_6(phi: ca.SX, v_x: ca.SX, v_y: ca.SX, omega: ca.SX) -> ca.SX:
#     return ca.vertcat(
#         v_x * ca.cos(phi) - v_y * ca.sin(phi),
#         v_x * ca.sin(phi) + v_y * ca.cos(phi),
#         omega,
#     )


def kin_model(rk_steps: int = 10) -> ca.Function:
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
    t_T = ca.SX.sym("t_T")
    t_delta = ca.SX.sym("t_delta")
    p = ca.vertcat(m, I_z, l_R, l_F, C_m0, C_r0, C_r1, C_r2, t_T, t_delta)

    # actuator dynamics
    delta_dot = (u_delta - delta) / t_delta
    tau_FL_dot = (u_tau_FL - tau_FL) / t_T
    tau_FR_dot = (u_tau_FR - tau_FR) / t_T
    tau_RL_dot = (u_tau_RL - tau_RL) / t_T
    tau_RR_dot = (u_tau_RR - tau_RR) / t_T

    # longitudinal dynamics
    T = tau_FL + tau_FR + tau_RL + tau_RR  # total torque
    F_motor = C_m0 * T  # the traction force
    F_drag = -(C_r0 + C_r1 * v_x + C_r2 * v_x * v_x) * smooth_sgn(
        v_x
    )  # drag force (aerodynamic and rolling resistance)
    F_Rx = 0.5 * F_motor + F_drag  # force applied at the rear wheels
    F_Fx = 0.5 * F_motor  # force applied at the front wheels

    a_x = (F_Rx + F_Fx * ca.cos(delta) + F_drag) / m
    a_y = F_Fx * ca.sin(delta) / m
    accels = ca.Function("accels", [x, p], [a_x, a_y])

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
                tau_FL_dot,
                tau_FR_dot,
                tau_RL_dot,
                tau_RR_dot,
                delta_dot,
            )
        ],
    )

    dt = ca.SX.sym("dt")
    x_next = x
    new_dt = dt / rk_steps
    for i in range(rk_steps):
        k1 = cont_dynamics(x, u, new_dt)
        k2 = cont_dynamics(x + k1 / 2, u, new_dt)
        k3 = cont_dynamics(x + k2 / 2, u, new_dt)
        k4 = cont_dynamics(x + k3, u, new_dt)
        x_next = x + k1 / 6 + k2 / 3 + k3 / 3 + k4 / 6

    return ca.Function(
        "kin6_model",
        [x, u, p, dt],
        [x_next, accels(x_next, p)[0]],
        ["x", "u", "p", "dt"],
        ["x_next", "accelerations"],
    )


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
    kin_model_fun = kin_model()
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
