// Example about Sliding Mode Controller
// Problem: Control mass `position`

// we have Dynamic Equation:
// m * \ddot{x} = u + d(t)
// where:
// m = 1kg
// x: position
// \dot{x}: velocity
// u: control input(N)
// d(t): external noise = 0.5*sin(t)
// x(t) -> x_d = 10m
// 

#include<iostream>
#include<cmath>
#include<fstream>

struct State{
    double x1; //position
    double x2; //velocity
};


class SlidingModeController
{
private:
    double lambda;
    double eta;
    double fi;
    double D;

public:
    SlidingModeController(
        double lambda,
        double eta,
        double fi,
        double D
    ):
    lambda(lambda),
    eta(eta),
    fi(fi),
    D(D)
{}

    // this is for resolving the chattering
    double saturation(double s, double fi){
        if (s > fi) return 1.0;
        if (abs(s) <= fi) return s/fi;
        if (s < fi) return -1.0;
        return 0.0;
    };

    double compute(
        State& state,
        double xd
    )
    {
        // error gets: by  position estimated - position desired
        double e = state.x1 - xd;
        
        // this can get after changing into state-space function
        double s = state.x2 + lambda * e;
        
        // control input by transforming from switching function
        double u = (-1) * lambda * state.x2 -  (eta * D)*saturation(s, fi);

        return u;
    };
};

class MassSystem{
private:
    double mass;
public:
    MassSystem(double mass) : mass(mass) {}
    // giving in problem statement
    double disturbance(double t){
        return 0.5 * sin(t);
    }; 

    // update that x1 and x2 following time -> also update the disturbance following time
    void update(
        State& state,
        double u,
        double t,
        double dt
    ){
        double d = disturbance(t);
        double dx = state.x2;
        double dv = (u+d)/mass;
        state.x1 += dx * dt;
        state.x2 += dv * dt;
    };
};

int main()
{
    State state{0.0, 0.0};

    double xd = 10.0;

    double dt = 0.001;
    double T  = 20.0;

    SlidingModeController smc(
        2.0,   // lambda
        5.0,   // eta
        0.1,   // phi
        0.5    // D
    );

    MassSystem plant(1.0);

    for(double t=0; t<T; t+=dt)
    {
        double u =
            smc.compute(
                state,
                xd);

        plant.update(
            state,
            u,
            t,
            dt);
        std::cout
        <<"T:"
        <<t
        <<"\t control-input"
        <<u
        <<std::endl;
    }

    std::cout
        << "Final position = "
        << state.x1
        << std::endl;
}




