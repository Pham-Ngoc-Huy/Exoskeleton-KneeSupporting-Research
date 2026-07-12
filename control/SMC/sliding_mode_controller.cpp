#include <iostream>
#include "sliding_mode_controller.h"
#include <cmath>

SlidingModeController::SlidingModeController(double lambda, double eta, double fi, double D){
    this->lambda = lambda;
    this->eta = eta;
    this->fi = fi;
    this->D = D;
}

double SlidingModeController::compute(const State& state, double xd){
    // error and its derivative -> Switching Function
    double e = state.x1 - xd;
    double edot = state.x2;
    std::cout<<"Switching-Function ===== \n"
            <<"e = x1 - xd = " <<e<<"\n"
            <<"e_dot = x2 = " <<edot
            <<std::endl;

    // sliding surface
    double s = edot + lambda * e;
    std::cout<<"Sliding-Surface ===== \n"
            <<"s = e_dot + lambda * e = "
            <<s
            <<std::endl;

    // control: proportional to sliding variable plus boundary-layer saturation
    double u_eq = -eta * ((s > 0.0) ? 1.0 : ((s < 0.0) ? 1.0 : 0.0)); // equivalent/linear part
    std::cout<<"Reachability ===== \n"
            <<"s_dot = -eta * sign(s) = "
            <<u_eq
            <<std::endl;

    double u_sw = -fi * SlidingModeController::saturation(s, fi); // switching with boundary layer size fi

    // total control (optionally compensate known disturbance bound D)
    double u = u_eq + u_sw;
    return u;
}

double SlidingModeController::reachability() const {
    return D;
}

double SlidingModeController::saturation(double s, double phi){
    if (phi <= 0.0) {
        return (s > 0.0) ? 1.0 : ((s < 0.0) ? -1.0 : 0.0);
    }
    double ratio = s / phi;
    if (std::fabs(ratio) < phi) return ratio;
    return (ratio > 0.0) ? 1.0 : -1.0;
}
