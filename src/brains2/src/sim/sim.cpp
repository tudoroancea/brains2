// Copyright (c) 2024, Tudor Oancea, Matteo Berthet
#include "brains2/sim/sim.hpp"
#include "models.h"

brains2::sim::Sim::Sim() : x{0}, u{0}, x_next{0}, p{0}, accels{0} {
    mem = casadi_alloc(kin6_model_functions());
    mem->arg[0] = x.data();
    mem->arg[1] = u.data();
    mem->arg[2] = p.data();
    mem->res[0] = x_next.data();
    mem->res[1] = accels.data();
}
brains2::sim::Sim::~Sim() {
    casadi_free(mem);
}

void brains2::sim::Sim::reset() {
    std::memset(x.data(), 0, sizeof(x));
    std::memset(x_next.data(), 0, sizeof(x_next));
    std::memset(u.data(), 0, sizeof(u));
    std::memset(accels.data(), 0, sizeof(accels));
}

void brains2::sim::Sim::step(const brains2::sim::Sim::Control &u, double dt) {
    this->u[0] = u.u_delta;
    this->u[1] = u.u_tau_FL;
    this->u[2] = u.u_tau_FR;
    this->u[3] = u.u_tau_RL;
    this->u[4] = u.u_tau_RR;
    mem->arg[3] = &dt;
    casadi_eval(mem);
    x = x_next;
}

brains2::sim::Sim::State brains2::sim::Sim::get_current_state() {
    return State{x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7], x[8], x[9], x[10]};
}

std::pair<double, double> brains2::sim::Sim::get_current_accels() {
    return {accels[0], accels[1]};
}
