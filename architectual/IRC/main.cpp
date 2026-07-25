#include <iostream>
#include "IRCController.h"

int main()
{
    IRCParams params;
    // ===== Admittance =====
    params.admittance.Jd = 1.0;
    params.admittance.Bd = 5.0;
    params.admittance.alpha = 1.0;
    params.admittance.torque_g = 10.0;
    params.admittance.dt = 0.001;

    // ===== Sliding Mode Controller =====
    params.lambda = 10.0;
    params.eta = 5.0;
    params.fi = 0.05;
    params.D = 1.0;

    // ===== Nonlinear Disturbance Observer =====
    params.observerGain = 20.0;
    params.dt = 0.001;
    IRCController controller(params);

    RobotState state;

    state.degree = 0.0;
    state.degree_dot = 0.0;

    for (int i = 0; i < 2; i++)
    {
        double tau = controller.update(state);

        std::cout
            << "******************************************\n"
            << "\n [step] = " << i
            << " torque = "
            << tau
            << std::endl;

        // Fake plant
        state.degree += 0.001 * tau;
        state.degree_dot = tau;
    }

    return 0;
}