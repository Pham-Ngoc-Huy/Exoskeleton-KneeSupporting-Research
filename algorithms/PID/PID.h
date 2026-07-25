#ifndef PID_H
#define PID_H

#include "backward_derivative.h"
#include "trapezodial_integral.h"

class PIDController {
private:
    // PID gains
    double Kp;
    double Ki;
    double Kd;
    
    // Helper objects for derivative and integral calculations
    // set pointers
    Derivative* derivative;
    TrapezodialIntegral* integral;
    
    // Control output limits (for saturation)
    double u_max;
    double u_min;
    
    double time_step;
    
public:
    /**
     * @brief Constructor for PID Controller
     * @param Kp Proportional gain
     * @param Ki Integral gain
     * @param Kd Derivative gain
     * @param u_max Maximum control output
     * @param u_min Minimum control output
     */
    PIDController(double Kp, double Ki, double Kd,  double time_step, double u_max = 1e9, double u_min = -1e9);
    /**
     * @brief Destructor
     */
    ~PIDController();
    
    /**
     * @brief Update PID controller with desired and actual values
     * @param x_desired Desired setpoint
     * @param x_actual Current measured value
     * @param dt Time step
     * @return Control signal (u)
     */
    double update(double x_desired, double x_actual);
    
    /**
     * @brief Set anti-windup limits for integral term
     * @param max_limit Maximum integral value
     * @param min_limit Minimum integral value
     */
    void setAntiWindupLimits(double max_limit, double min_limit);
    
    /**
     * @brief Enable/disable anti-windup
     * @param enable True to enable, false to disable
     */
    void setAntiWindupEnabled(bool enable);
    
    /**
     * @brief Reset the PID controller state
     */
    void reset();
    
    /**
     * @brief Clamp control output to saturation limits
     * @param value Control output value
     * @return Clamped output
     */
    double clampOutput(double value) const;
};

#endif