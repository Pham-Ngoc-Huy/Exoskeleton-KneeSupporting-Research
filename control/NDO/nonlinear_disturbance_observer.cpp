#include "nonlinear_disturbance_observer.h"
#include <iostream>
#include <cmath>

NonLinearDisturbanceObserver::NonLinearDisturbanceObserver (double L, double z, double d_hat, double dt){
    this->L=L;
    this->z=z;
    this->d_hat=d_hat;
    this->dt=dt;
}

double NonLinearDisturbanceObserver::ObserverPhase(double q_dot, double f_x, double g_x, double u) {
    double p_x = L * q_dot;
    d_hat = z + p_x;
    double z_dot = -L * d_hat - L * (f_x + g_x * u);
    z += z_dot * dt;
    d_hat = z + p_x;
    return d_hat;
}

double NonLinearDisturbanceObserver::CompensationPhase(double u_smc) {
    return u_smc - d_hat;
}

double NonLinearDisturbanceObserver::disturbanceEstimate() const {
    return d_hat;
}