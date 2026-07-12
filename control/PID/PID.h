#ifndef PID_H
#define PID_H

class PIDController
{
private:
    double Kp;
    double Ki;
    double Kd;
public:
    PIDController(double Kp, double Ki, double Kd);
};

#endif