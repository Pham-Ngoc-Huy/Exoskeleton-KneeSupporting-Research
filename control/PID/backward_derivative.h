#ifndef DERIVATIVE_H
#define DERIVATIVE_H

class Derivative
{
private:
    double previousError;
    double dt;
    bool firstRun;
public:
    Derivative(double sampleTime);
    double update(double currentError);
};
#endif