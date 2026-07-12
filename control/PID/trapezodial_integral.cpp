#include "trapezodial_integral.h"
#include <cmath>

TrapezodialIntegral::TrapezodialIntegral(double max_limit, double min_limit) 
    : accumulated_integral(0.0),
      previous_error(0.0),
      firstRun(true),
      max_integral(max_limit),
      min_integral(min_limit),
      anti_windup_enabled(true) {}

double TrapezodialIntegral::update(double error, double dt) {
    // Skip integration on first call to avoid spurious values
    // t = 0
    if (firstRun) {
        firstRun = false;
        previous_error = error;
        return accumulated_integral;
    }
    // t > 0
    // Trapezoidal integration: area of trapezoid = (f0 + f1) / 2 * dt
    // This is more accurate than rectangular integration
    double integration_step = ((error + previous_error) / 2.0) * dt;
    accumulated_integral += integration_step;

    // Apply anti-windup (clamping) if enabled
    if (anti_windup_enabled) {
        accumulated_integral = clampIntegral(accumulated_integral);
    }
    previous_error = error;
    return accumulated_integral;
}

double TrapezodialIntegral::getIntegral() const {
    return accumulated_integral;
}

void TrapezodialIntegral::reset() {
    accumulated_integral = 0.0;
    previous_error = 0.0;
    firstRun = true;
}

void TrapezodialIntegral::setAntiWindupLimits(double max_limit, double min_limit) {
    max_integral = max_limit;
    min_integral = min_limit;
    // Clamp current integral to new limits
    accumulated_integral = clampIntegral(accumulated_integral);
}

void TrapezodialIntegral::setAntiWindupEnabled(bool enable) {
    anti_windup_enabled = enable;
}

double TrapezodialIntegral::clampIntegral(double value) const {
    if (value > max_integral) {
        return max_integral;
    } else if (value < min_integral) {
        return min_integral;
    }
    return value;
}
