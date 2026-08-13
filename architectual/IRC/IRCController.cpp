#include <iostream>
#include "IRCController.h"


// Temporary placeholders
double f_x = 0.0;
double g_x = 1.0;

IRCController::IRCController(const IRCParams &p)
    :
    admittance_(p.admittance),
    smc_(p.lambda, p.eta, p.fi, p.D),
    ndo_(p.observerGain, p.dt)

{}
double IRCController::update(const RobotState &state)
{
    double torque_hat =
        ndo_.ObserverPhase(state.degree, f_x, g_x, previous_tau_e);
    // double torque_hat = 5.0;   // Fixed test value

    outputAdmittance desired =
        admittance_.update(torque_hat);

    State x{
        state.degree,
        state.degree_dot
    };

    double tau_tracking =
        smc_.compute(x, desired.degree_d);

    previous_tau_e = tau_tracking;
    std::cout<< "IRC ==================\n"
    << "theta      : " << state.degree << '\n'
    << "theta_dot  : " << state.degree_dot << '\n'
    << "tau_hat    : " << torque_hat << '\n'
    << "theta_d    : " << desired.degree_d << '\n'
    << "theta_ddot : " << desired.degree_d_dot << '\n'
    << "tau_track  : " << tau_tracking << '\n';
    return tau_tracking;
}