#ifndef TRAPEZODIAL_INTEGRAL_H
#define TRAPEZODIAL_INTEGRAL_H

class TrapezodialIntegral {
private:
    // Current accumulated integral value
    double accumulated_integral;
    // Previous error for `Trapezodial` integration
    double previous_error;
    // Maximum integral value for anti-windup
    double max_integral;
    // Minimum integral value for anti-windup
    double min_integral;
    // Enable/disable anti-windup
    bool anti_windup_enabled;
    // step-time
    double dt;
    // Flag to skip integration on first Run
    bool firstRun;

public:
    /**
     * @brief: Apply `Trapezodial Rule`: the equation is:  
     * @param max_limit: which is 1e9 - stands for +infinity
     * @param min_limit: which is -1e9 - stands for -infinity
     * @param step_time: time-window
     */
    TrapezodialIntegral(double max_limit, double min_limit, double step_time);
    /**
     * @brief: 
     * @param: error -> current error
     */
    double update(double error);
    double getIntegral() const;
    void reset();
    void setAntiWindupLimits(double max_limit, double min_limit);
    void setAntiWindupEnabled(bool enable);
    double clampIntegral(double value) const;
};

#endif
