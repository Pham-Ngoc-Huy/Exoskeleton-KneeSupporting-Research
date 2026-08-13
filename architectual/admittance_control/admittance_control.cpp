#include "admittance_control.h"
#include <iostream>


AdmittanceControl::AdmittanceControl(double mass, double damping, double stiffness, double time_step){
    M=mass;
    B=damping;
    K=stiffness;
    dt=time_step;
    target_position = 0.0;
};

State AdmittanceControl::update(double F_ext){
    double spring_force = K * (currentState.position - target_position);
    double damping_force = B * currentState.velocity;

    currentState.acceleration = (F_ext - spring_force - damping_force) / M;
    currentState.velocity += (currentState.acceleration * dt);
    currentState.position += (currentState.velocity * dt);

    return currentState;
};

void AdmittanceControl::setTargetPosition(double target) {
    target_position = target;
}

void AdmittanceControl::resetState(double initial_position) {
    currentState.position = initial_position;
    currentState.velocity = 0.0;
    currentState.acceleration = 0.0;
}

State AdmittanceControl::getState() const {
    return currentState;
}