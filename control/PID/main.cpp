#include <iostream>
#include <cmath>
#include "PID.h"


int main(){
    double Kp = 2.0;
    double Ki = 0.5;
    double Kd = 0.2;

    double u_max = 100.0;
    double u_min = -100.0;

    double time_step = 0.1;  // 100ms for better response
    double step = 0.0;
    double time_simulation = 100;
    double x_desired = 30;
    PIDController pid(Kp, Ki, Kd, time_step, u_max, u_min);

    // Simple plant model state
    double x_actual = 0.0;
    int k = 0;
    while (step <= time_simulation)
    {
        std::cout << "========== STEP " << k
                << " | t = " << step
                << " ==========" << std::endl;

        double u_control = pid.update(x_desired, x_actual);
        double dx_dt = -x_actual + u_control;
        x_actual += dx_dt * time_step;
        std::cout << "=> x_actual = " << x_actual << std::endl;
        step += time_step;
        k++;
    }
    return 0;
};