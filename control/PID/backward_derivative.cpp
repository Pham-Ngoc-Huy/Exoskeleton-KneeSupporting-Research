#include "backward_derivative.h"
#include <cmath>
#include <iostream>
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
    };
    std::cout << "\nDERIVATIVE DEBUG ==========>\n";
    std::cout << "e(k-1) = " << previousError << '\n';
    std::cout << "e(k)   = " << currentError << '\n';
    double derivative = (currentError - previousError) / dt;
    std::cout << "D(k)   = ("
            << currentError
            << " - "
            << previousError
            << ") / "
            << dt
            << " = "
            << derivative
            << '\n';
    previousError = currentError;
    return derivative;
}
