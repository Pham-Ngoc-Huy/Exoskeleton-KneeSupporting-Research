#ifndef ADMITTANCE_MODEL
#define ADMITTANCE_MODEL

struct State {
    double position;
    double velocity;
    double acceleration;

    State() : position(0.0), velocity(0.0), acceleration(0.0) {}
};

class AdmittanceModel {
private:
    double K;
    double B;
    double M;
    double dt;
    State currentState;
    double target_position;

public:
    /**
     * @brief Admittance Model Constructor
     * @param stiffness: virtual stiffness
     * @param damping: virtual damping ratio
     * @param mass: virtual mass
     * @param time_step: dt
     */
    AdmittanceModel(double stiffness, double damping, double mass, double time_step);

    /**
     * @brief Update the state when the F_ext changed
     * @param F_ext: External Force was measured by sensors 
     */
    State update(double F_ext);

    void setTargetPosition(double target);
    void resetState(double initial_position);
    
    State getState() const;
};
#endif