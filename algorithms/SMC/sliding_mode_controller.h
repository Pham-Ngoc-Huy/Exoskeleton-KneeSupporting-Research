#ifndef SMC_H
#define SMC_H

// this can be more than 2 -> but just set 2 first
struct State {
    double x1;
    double x2;
};

class SlidingModeController {
private:
    double lambda;
    double eta;
    double fi;
    double D;
public:
    /**
     * @brief: Sliding Mode Controller: constructor
     * @param lambda: 
     * @param eta: gain -> control velocity move to sliding surface
     * @param fi: 
     * @param D
     */
    SlidingModeController(double lambda, double eta, double fi, double D);
    /** 
     * @brief: Compute control action given current state and desired position `xd`.
     * @param: `state.x1` is position
     * @param: `state.x2` is velocity 
     * */ 
    double compute(const State& state, double xd);

    /** 
     * @brief: Return the disturbance bound or reachability margin (configured `D`).
     * */     
    double reachability() const;

    /** 
     * @brief: Smooth saturation function: sat(s/phi) with boundary layer `phi`.
     * @param s: sliding surface
     * @param fi
     * */ 
    static double saturation(double s, double fi);
};
#endif