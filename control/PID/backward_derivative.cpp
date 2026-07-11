#include "backward_derivative.h"
#include <cmath>

Derivative::Derivative(double sampleTime)
{
    dt=sampleTime;
    previousError=0.0;
    firstRun=true;
}
double Derivative::update(double currentError)
{
    if (firstRun)
    {
        previousError=currentError;
        firstRun=false;
        return 0.0;
    }
    double derivative=(currentError-previousError) / dt;

    previousError=currentError;
    return derivative;
}
