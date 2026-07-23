#include <iostream>
#include "PID.h"
#include <iomanip>
PIDController::PIDController(
    double Kp, 
    double Ki, 
    double Kd, 
    double time_step,
    double u_max,
    double u_min
): Kp(Kp), Ki(Ki), Kd(Kd), u_max(u_max), u_min(u_min), time_step(time_step)
{
    // Initialize derivative with sample time
    derivative = new Derivative(time_step);
    
    // Initialize integral with anti-windup limits
    integral = new TrapezodialIntegral(u_max, u_min, time_step);
}

PIDController::~PIDController()
{
    delete derivative;
    delete integral;
}

double PIDController::update(double x_desired, double x_actual)
{
    // Calculate error

    double error = x_desired - x_actual;
    
    // Proportional term
    double u_p = Kp * error;
    
    // Integral term (using trapezoidal rule)
    double u_i = Ki * integral->update(error);
    
    // Derivative term (using backward difference)
    double u_d = Kd * derivative->update(error);
    
    // Compute total control output
    double u_control = u_p + u_i + u_d;
    
    // Apply output saturation
    u_control = clampOutput(u_control);
    std::cout << std::fixed << std::setprecision(4)
            << "\n[PID] ====================>\n"
            << "desired = " << x_desired << "\n"
            << "actual = " << x_actual << "\n"
            << "error = " << error << "\n"
            << "P = " << u_p << "\n"
            << "I = " << u_i << "\n"
            << "D = " << u_d << "\n"
            << "OUTPUT-CONTROL = " << u_control << "\n"
            << "clamped = " << u_control
            << std::endl;
    return u_control;
}

void PIDController::setAntiWindupLimits(double max_limit, double min_limit)
{
    if (integral != nullptr) {
        integral->setAntiWindupLimits(max_limit, min_limit);
    }
}

void PIDController::setAntiWindupEnabled(bool enable)
{
    if (integral != nullptr) {
        integral->setAntiWindupEnabled(enable);
    }
}

void PIDController::reset()
{
    if (derivative != nullptr) {
        // Recreate derivative to reset state
        delete derivative;
        derivative = new Derivative(0.01);
    }
    if (integral != nullptr) {
        integral->reset();
    }
}

double PIDController::clampOutput(double value) const
{
    if (value > u_max) {
        return u_max;
    }
    if (value < u_min) {
        return u_min;
    }
    return value;
}
