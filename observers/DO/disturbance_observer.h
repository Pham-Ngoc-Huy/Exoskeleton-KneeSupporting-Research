#ifndef DISTURBANCE_OBSERVER_H
#define DISTURBANCE_OBSERVER_H

class DisturbanceObserver {
private:
    double dt;
    double omega;
    int filter_relative_degree;
    double estimate_disturbance;
public:
    /**
     * @brief: constructor
     * @param dt: sampling time (s)
     * @param omega: cutoff frequency (rad/s)
     * @param filter_relative_degree: relative degree of Q(s)
     */
    DisturbanceObserver(
        double dt,
        double omega,
        int filter_relative_degree
    );

    /**
     * @brief Q(s) = omega^n / (s + omega)^n
     */
    double qFilter(
        double input
    );

    /**
     * @brief: Disturbance estimation
     *      d_hat = Q(P^-1 * y - u)
     * @param inversePlantOutput P^-1(s)*y
     * @param controlInput u 
     */
    double update(
        double inversePlantOutput,
        double controlInput
    );
    double getEstimatedDisturbance() const;

    void reset();
};
#endif