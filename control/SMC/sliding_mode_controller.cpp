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
    double sat_val = saturation(s,fi);
    double u = (-1.0) * lambda * edot - eta*sat_val;
    std::cout << "Control-Law ===== \n"
              << "sat(s, phi) = " << sat_val << "\n"
              << "u = -lambda*e_dot - eta*sat(s, phi) = " << u
              << std::endl;
    return u;
}

double SlidingModeController::reachability() const {
    return D;
}

double SlidingModeController::saturation(double s, double fi){
    double ratio = s / fi;
    if (fabs(s) <= fi){
        std::cout<<"Boundaries_2: abs(s) < fi \n"
            <<"get-result: "<<ratio
            <<std::endl;
        return ratio;
    };
    if (s > fi) {
        std::cout<<"Boundaries_1: s > fi \n"
            <<"get-result: "<<1.0
            <<std::endl;
        return 1.0; 
    };
    std::cout<<"Boundaries_3: s < fi \n"
        <<"get-result: "<<-1.0
        <<std::endl;   
    return -1.0;
}
