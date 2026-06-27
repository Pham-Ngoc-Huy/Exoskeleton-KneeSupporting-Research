// heating, ventilaton, and air-conditioning system
// a model of heating, ventilation, and air-conditioning system (HVAC) was given for one thermal zone as follow:
//
// /dot{x1} = (Cp/C1)(Ts − x1 )u + (1/C1*R)(x2 − x1 ) + (1/C1*Ro)(To − x1 ) + w,
// /dot{x2} = (1/(C2*R)) (x1 − x2 ),
// y = x1 ,
// where 
// symbol  | value                  | physical description
//---------------------------------------------------------
// x1      | state variable         | Air temperature of the thermal zone
// x2      | state variable         | Temperature of floors, walls, furniture etc.
// u       | input variable         | Mass flow rate of supply air
// w       | disturbance variable   | Unmeasured thermal load
// cp      | 0.000281, kWh/kg*K     | Heat capacity of thermal zone air
// C1      | 0.00275, kWh/K         | Thermal capacitance of air
// C2      | 1.87733, kWh/K         | Thermal capacitance of floors, walls, furniture etc.
// T_s     | 17, Celcius Degree     | Temperature of supply air
// R       | 2.08, K/kW             | Thermal resistance between C1 and C2
// R_0     | 11.849, K/kW           | Thermal resistance between the thermal zone and outside air 
// T_0     | 27, Celcius Degree     | Outside air temperature


// Objective: estimate w
// Assuming: that /dot{w} = 0, we extend the equation (21) with x3 = w, which yeilds an extended system -> this was missing
// /dot{x} = Ax + g(y,u)
// y = x1,
#include<iostream>
#include<cmath>
#include<iomanip>
#include<random>

struct State{
    double x1;
    double x2;
    double x3;
};

class DisturbanceObserver
{
private:
    // Physical parameters
    const double cp=0.000281;
    const double C1=0.00275;
    const double C2=1.87733;
    
    const double Ts=17.0;
    const double R=2.08;
    const double Ro=11.849;
    const double To=27.0;

    // Observer gain
    double l1;
    double l2;
    double l3;

    State xhat;

public:
    DisturbanceObserver(){
        xhat = {27.0, 27.0 , 0.0};

        // Example observer poles:
        l1=0.15;
        l2=0.005;
        l3=0.0005;
    }
    void update(double y, double u, double dt){
        double a11 = -1.0/(C1*R) - 1.0/(C1*Ro);
        double a12 = 1.0/(C1*R);
        double a13 = 1.0;
        double a21 = 1.0/(C2*R);
        double a22 = (-1.0)/(C2*R);

        // error - where y=x1 => innovation = x1 - x_hat.x1
        double innovation = y - xhat.x1 ;
        // g(y,u)
        double g1 = cp/C1 * (Ts-y)*u + To/(C1*Ro);

        // observer dynamics
        double dx1 = a11*xhat.x1 + a12*xhat.x2 + a13*xhat.x3 + g1 + l1*innovation;
        double dx2 = a21*xhat.x1 + a22*xhat.x2 + l2*innovation;
        double dx3 = l3*innovation;

        xhat.x1 += dt*dx1;
        xhat.x2 += dt*dx2;
        xhat.x3 += dt*dx3;

    }
    double roomTempEstimate() const{
        return xhat.x1;
    };
    double wallTempEstimate() const{
        return xhat.x2;
    };
    double disturbanceEstimate() const{
        return xhat.x3;
    };
};

int main()
{
    DisturbanceObserver observer;

    double dt = 1.0/6000;       // seconds
    double u  = 0.10;      // airflow -> mass flow rate of air

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> x1_dist(20.0, 27.0);

    double x1 = x1_dist(gen); 
    double x2 = 27.0;
    double w  = 0;

    const double cp = 0.000281;
    const double C1 = 0.00275;
    const double C2 = 1.87733;

    const double Ts = 17.0;
    const double R  = 2.08;
    const double Ro = 11.849;
    const double To = 27.0;

    std::cout << "t | x1 | x2 | w_real |  w_hat \n";
    std::cout << "==============================\n";

    for(int k=0;k<1000;k++)
    {
        double dx1 =
            -(1.0/(C1*R)+1.0/(C1*Ro))*x1
            + (1.0/(C1*R))*x2
            + ((cp/C1)*(Ts-x1))*u
            + To/(C1*Ro)
            + w;

        double dx2 =
            (1.0/(C2*R))*x1
            - (1.0/(C2*R))*x2;

        x1 += dt*dx1;
        x2 += dt*dx2;

        observer.update(x1,u,dt);
        std::cout
            << k << " | "
            << x1 << " | "
            << observer.roomTempEstimate() << " | "
            << x2 << " | "
            << observer.wallTempEstimate() << " | "
            << w << " | "
            << observer.disturbanceEstimate()
            << "\n";
    }

    return 0;
}