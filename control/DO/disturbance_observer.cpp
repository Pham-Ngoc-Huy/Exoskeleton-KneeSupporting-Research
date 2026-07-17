#include "disturbance_observer.h"

#include <iostream>

DisturbanceObserver::DisturbanceObserver(
    double dt,
    double omega,
    int filter_relative_degree
)
    : dt(dt),
      omega(omega),
      filter_relative_degree(filter_relative_degree),
      estimate_disturbance(0.0)
{
    std::cout
        << "[DOB] Initialized\n"
        << "dt                    = " << dt << '\n'
        << "omega                 = " << omega << '\n'
        << "filter_relative_degree = "
        << filter_relative_degree
        << "\n\n";
}

double DisturbanceObserver::qFilter(
    double input
)
{
    static double state = 0.0;

    double alpha = omega * dt;

    std::cout
        << "Q FILTER ==========\n";

    std::cout
        << "input  = " << input << '\n'
        << "alpha  = " << alpha << '\n'
        << "state(before) = " << state << '\n';

    for (int i = 0; i < filter_relative_degree; i++)
    {
        double previous_state = state;

        state += alpha * (input - state);

        std::cout
            << "stage " << i + 1 << '\n'
            << "    previous_state = "
            << previous_state << '\n'
            << "    new_state      = "
            << state << '\n';
    }

    std::cout
        << "state(after) = " << state << "\n\n";

    return state;
}

double DisturbanceObserver::update(
    double inversePlantOutput,
    double controlInput
)
{
    std::cout
        << "DOB UPDATE ==========\n";

    std::cout
        << "inversePlantOutput = "
        << inversePlantOutput
        << '\n';

    std::cout
        << "controlInput       = "
        << controlInput
        << '\n';

    double raw_disturbance =
        inversePlantOutput
        - controlInput;

    std::cout
        << "raw_disturbance    = "
        << raw_disturbance
        << '\n';

    estimate_disturbance =
        qFilter(raw_disturbance);

    std::cout
        << "estimated_disturbance = "
        << estimate_disturbance
        << "\n\n";

    return estimate_disturbance;
}

double DisturbanceObserver::getEstimatedDisturbance() const
{
    return estimate_disturbance;
}

void DisturbanceObserver::reset()
{
    estimate_disturbance = 0.0;

    std::cout
        << "[DOB] Reset observer.\n";
}