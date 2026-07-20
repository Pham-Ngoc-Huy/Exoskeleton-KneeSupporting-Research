#ifndef NDO_H
#define NDO_H

class NonLinearDisturbanceObserver {
private:   
    double L;
    double dt;
    double z;
    double d_hat;

public:
    /**
    * @brief: constructor of Nonlinear-Disturbance-Observer
    * @param u control input
    * @param L observer gain
    * @param p nonlinear function selected to shape the observer dynammics
    * @param z internal observer state
    * @param d unknown disturbance
    * @param d_hat estimate disturbance
    */
    NonLinearDisturbanceObserver(double L, double dt, double z = 0.0, double d_hat = 0.0);

    /**
     * @brief estimator error 
     * @param d actual disturbance 
     * @param d_hat desired disturbance
     */
    double EstimationError(double d, double d_hat);

    /**
     * @brief computes d_hat_dot directly: d_hat_dot = -L*d_hat_current + L*(x_dot - f_x - g_x*u)
     *        equivalent formulation to the z + p(x) decomposition used in ObserverPhase
     * @param d_hat_current current disturbance estimate
     * @param x_dot generalized velocity (rate of change of x)
     * @param f_x drift dynamics term
     * @param g_x input control matrix
     * @param u control input signal
     */
    double EstimatorDisturbance(double d_hat_current, double x_dot, double f_x, double g_x, double u);

    /**
     * @brief Auxiliary variable chosen -> Jie Chen introduces this as z -> which is for consume all the \dot{x}
     * @param d_hat estimated disturbance 
     * @param p_x non-linear function (self-chosen) -> this must relate with the \dot{p(x)} = L(x) * \dot{x(t)} (this was applied by Chain's Rule)
     */
    double AuxiliaryVariable(double d_hat, double p_x);

    /**
     * @brief finding \dot{z} -> this is observer state
     * @param q_dot generalized velocity
     * @param f_x drift dynamics term
     * @param g_x input control matrix
     * @param u control input signal
     */
    double ObserverPhase(double x, double f_x, double g_x, double u);

    /**
     * @brief compensates the SMC (Sliding Mode Controller) input using the current
     *          disturbance estimate, scaled throguh the input control matrix
     * @param u_smc control input of `sliding-mode controller`
     * @param d_hat current disturbance estimate (torque units)
     * @param g_x input control matrix used to scale `d_hat` back into the same channel as `u_smc`
     */
    double CompensationPhase(double u_smc, double g_x);

    double DisturbanceEstimate() const;
    
    void reset(double z0=0.0, double d_hat0=0.0);
};
#endif