#ifndef INTEGRAL_ACCUMULATE_H
#define INTEGRAL_ACCUMULATE_H

/**
 * @class IntegralAccumulator
 * @brief Accumulates integral error over time for PID controller
 * 
 * Handles time-based integral calculation with anti-windup protection
 * to prevent integral saturation in control systems.
 */
class IntegralAccumulator {
private:
    double accumulated_integral;      // Current accumulated integral value
    double previous_error;             // Previous error for trapezoidal integration
    bool first_call;                   // Flag to skip integration on first call
    double max_integral;               // Maximum integral value for anti-windup
    double min_integral;               // Minimum integral value for anti-windup
    bool anti_windup_enabled;          // Enable/disable anti-windup

public:
    /**
     * @brief Constructor for IntegralAccumulator
     * @param max_limit Maximum allowed integral value (default: no limit)
     * @param min_limit Minimum allowed integral value (default: no limit)
     */
    IntegralAccumulator(double max_limit = 1e9, double min_limit = -1e9);

    /**
     * @brief Update the integral with a new error value and time step
     * Uses trapezoidal integration: integral += (error + prev_error) / 2 * dt
     * 
     * @param error Current error value
     * @param dt Time step (seconds)
     * @return Updated integral value
     */
    double update(double error, double dt);

    /**
     * @brief Get the current accumulated integral value
     * 
     * @return Current integral accumulation
     */
    double getIntegral() const;

    /**
     * @brief Reset the integral accumulation to zero
     */
    void reset();

    /**
     * @brief Set anti-windup limits
     * 
     * @param max_limit Maximum integral value
     * @param min_limit Minimum integral value
     */
    void setAntiWindupLimits(double max_limit, double min_limit);

    /**
     * @brief Enable or disable anti-windup protection
     * 
     * @param enable True to enable anti-windup, false to disable
     */
    void setAntiWindupEnabled(bool enable);

    /**
     * @brief Clamp the integral value within limits (anti-windup)
     * 
     * @param value Value to clamp
     * @return Clamped value
     */
    double clampIntegral(double value) const;
};

#endif // INTEGRAL_ACCUMULATE_H
