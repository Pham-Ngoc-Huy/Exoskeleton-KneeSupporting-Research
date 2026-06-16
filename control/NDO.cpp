// NDO means: Nonlinear-Disturbance-Observer
// from system to state-space
// This works with example: Mass-Spring-Damp (Impedance Control) with Nonlinear Friction
#include<iostream>
#include<cmath>

class NonLinearDisturbanceObserver {
// control-input
private:
    // observer gain
    double L;
    // internal observer state
    double z;
    // estimated disturbance
    double d_hat;

    double m;
    double c;
    double k;
public:
    NonLinearDisturbanceObserver(double gain, double mass, double damping, double stiffness): 
        L(gain), z(0.0), d_hat(0.0), m(mass), c(damping), k(stiffness){}

    void update(double x, double v, double u, double dt){
    // Known nonlinear dynamics
    double f = (u - c*v - k*x - 0.1*std::pow(v,3))/ m;

    // NDO
    d_hat = z + L*v;
    double dz = -L*d_hat + -L*f;
    z += dz + dt;
    }
    double disturbanceEstimate () const{
        return d_hat;
    }
};

int main(){
    double dt = 0.001;

    double x = 0.0;
    double v = 0.0;

    double m = 1.0;
    double c = 2.0;
    double k = 20.0;

    NonLinearDisturbanceObserver ndo(50.0, m, c, k);
    for(int i=0;i<10000;i++)
    {
        double t = i*dt;

        // Control input
        double u = 10.0 * std::sin(t);

        // Unknown disturbance -> random disturbance init
        double disturbance = 2.0*std::sin(0.5*t) + 0.5*std::cos(3*t);

        // Real plant
        double acc = (u - c*v - k*x - 0.1*std::pow(v,3) + disturbance) / m;

        x += v*dt;
        v += acc*dt;

        // Observer
        ndo.update(x, v, u, dt);

        if(i % 1000 == 0)
        {
            std::cout
                << "t = " << t
                << "  d = " << disturbance
                << "  d_hat = "
                << ndo.disturbanceEstimate()
                << std::endl;
        }
    }

    return 0;
}