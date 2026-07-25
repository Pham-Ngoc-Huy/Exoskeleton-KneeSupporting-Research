#include "nonlinear_disturbance_observer.h"
#include <iostream>
#include <cmath>

NonLinearDisturbanceObserver::NonLinearDisturbanceObserver (double L, double dt, double z0, double d_hat0){
    this->L=L;
    this->dt=dt;
    this->z=z0;
    this->d_hat=d_hat0;
}

double NonLinearDisturbanceObserver::EstimationError(double d, double d_hat){
    double error = d_hat - d;
    std::cout<<"Esitmation error of NDOB \n" 
        <<error<<std::endl;
    return error;
}

double NonLinearDisturbanceObserver::EstimatorDisturbance(double d_hat_current, double x_dot, double f_x, double g_x, double u){
    double d_hat_dot = -L * d_hat_current + L * (x_dot - f_x - g_x * u);
    std::cout << "[EstimatorDisturbance] d_hat_current=" << d_hat_current
              << " x_dot=" << x_dot
              << " f_x=" << f_x
              << " g_x=" << g_x
              << " u=" << u
              << " -> d_hat_dot=" << d_hat_dot << std::endl;
    return d_hat_dot;
}

double NonLinearDisturbanceObserver::AuxiliaryVariable(double d_hat_val, double p_x){
    double z_val = d_hat_val - p_x;
    std::cout << "[AuxiliaryVariable] d_hat_val=" << d_hat_val
              << " p_x=" << p_x
              << " -> z=" << z_val << std::endl;
    return z_val;
}

double NonLinearDisturbanceObserver::ObserverPhase(double x, double f_x, double g_x, double u) {
    double p_x = L * x;
    d_hat = z + p_x;
    double z_dot = -L * z - L * (f_x + g_x * u);
    z += z_dot * dt;
    d_hat = z + p_x;
    std::cout << "[ObserverPhase] ===============\n" 
            << " x=" << x << "\n"
            << " f_x=" << f_x << "\n"
            << " g_x=" << g_x << "\n"
            << " u=" << u << "\n"
            << "---------------------\n"
            << " p_x=" << p_x << "\n"
            << " z_dot=" << z_dot << "\n"
            << " z(new)=" << z << "\n"
            << " -> d_hat=" << d_hat << std::endl;
    return d_hat;
}

double NonLinearDisturbanceObserver::CompensationPhase(double u_smc, double g_x) {
    double u_final = u_smc - d_hat / g_x;
    std::cout << "[CompensationPhase] u_smc=" << u_smc
              << " g_x=" << g_x
              << " d_hat=" << d_hat
              << " -> u_final=" << u_final << std::endl;
    return u_final;
}

double NonLinearDisturbanceObserver::DisturbanceEstimate() const{
    std::cout<<"Disturbance estimate d_hat: "<<d_hat<<std::endl;
    return d_hat;
}

void NonLinearDisturbanceObserver::reset(double z0, double d_hat0){
    z=z0;
    d_hat=d_hat0;
    std::cout << "[reset] z=" << z << " d_hat=" << d_hat << std::endl;
}