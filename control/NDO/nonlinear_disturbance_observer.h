#ifndef NDO_H
#define NDO_H

class NonLinearDisturbanceObserver {
private:   
    double L;
    double z;
    double m;
    double w_hat;
    double c;
    double k;
public:
    /**
    @brief: constructor of Nonlinear-Disturbance-Observer
    @param: L = gain
    @param: z = 
    @param: m = mass
    @param: c = damping
    @param: k = stiffness
    @param: w_hat = estimate disturbance observer
    */
    NonLinearDisturbanceObserver(double L, double z, double m, double c, double k, double w_hat);
    /**
     *
     */
    void update(double x, double v, double u, double dt); 
    /**
     * @brief: Return d_hat
     */
    double disturbanceEstimate () const;
};

#endif