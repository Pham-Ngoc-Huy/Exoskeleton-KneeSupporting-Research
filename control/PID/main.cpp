#include<iostream>
#include<cmath>



class PIDController {
private:
    // propotion parameter
    double Kp;
    // integral parameter
    double Ki;
    // derivative parameter
    double Kd;
    // error
    double x_desired;
    double x_actual;
public:
    PIDController(
        double propotional,
        double integral,
        double derivative
    ):
    Kp(propotional),
    Ki(integral),
    Kd(derivative),
    x_desired(0.0),
    x_actual(0.0)
    {}


}
