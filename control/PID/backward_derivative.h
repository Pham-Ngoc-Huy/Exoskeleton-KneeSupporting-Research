#ifndef DERIVATIVE_H
#define DERIVATIVE_H

/**
 * @class Derivative
 * @brief BackWard Differences (technique to apply for derivative through time)
 */
class Derivative
{
private:
    double previousError;
    double dt;
    bool firstRun;
public:
    Derivative(double sampleTime);
    /**
     * @brief Update the new derivative value base on previous error
     * @param currentError update the new currentError and will use for the next 
     */
    double update(double currentError);
};
#endif