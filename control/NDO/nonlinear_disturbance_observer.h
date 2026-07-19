#ifndef NDO_H
#define NDO_H

class NonLinearDisturbanceObserver {
private:   
    double u;
    double L;
    double p;
    double z;
    double d_hat;
    double dt;
public:
    /**
    @brief: constructor of Nonlinear-Disturbance-Observer
    @param u control input
    @param L observer gain
    @param p nonlinear function selected to shape the observer dynammics
    @param z internal observer state
    @param d_hat estimate disturbance
    */
    NonLinearDisturbanceObserver(double L, double z, double d_hat, double dt);
    /**
     *
     */
    double ObserverPhase(double q_dot, double f_x, double g_x, double u);
    /**
     * 
     */
    double CompensationPhase(double u_smc, double g_c);
     /**
     * @brief: Return d_hat
     */
    double disturbanceEstimate () const;
};

#endif