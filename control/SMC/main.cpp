// test_smc.cpp
// Test harness for SlidingModeController
//
// Assumes sliding_mode_controller.h declares:
//   struct State { double x1, x2; };
//   class SlidingModeController {
//     public:
//       SlidingModeController(double lambda, double eta, double fi, double D);
//       double compute(const State& state, double xd);
//       double reachability() const;
//       static double saturation(double s, double fi);
//   };
// If your actual header differs (e.g. saturation is non-static or private),
// adjust the calls in section 2 accordingly.

#include <iostream>
#include <sstream>
#include <fstream>
#include <cmath>
#include <cassert>
#include "sliding_mode_controller.h"

// Simple point-mass plant: m*xddot = u + d(t), d(t) = 0.5*sin(t)
static double disturbance(double t) {
    return 0.5 * std::sin(t);
}

static void plant_step(State& state, double u, double t, double dt, double mass) {
    double d  = disturbance(t);
    double dx = state.x2;
    double dv = (u + d) / mass;
    state.x1 += dx * dt;
    state.x2 += dv * dt;
}

int main() {
    // ---------------------------------------------------------------
    // 1) Single hand-checkable call
    //    state = {x1=0, x2=0}, xd=10, lambda=2, eta=5, fi=0.1, D=0.5
    //    e = 0 - 10 = -10
    //    edot = 0
    //    s = edot + lambda*e = 0 + 2*(-10) = -20
    //    saturation(-20, 0.1): |s| > fi -> -1.0
    //    u = -lambda*edot - eta*sat = -2*0 - 5*(-1.0) = 5.0
    // ---------------------------------------------------------------
    std::cout << "=== Test 1: single compute() call ===\n";
    {
        SlidingModeController smc(2.0, 5.0, 0.1, 0.5);
        State s0{0.0, 0.0};
        double u = smc.compute(s0, 10.0);
        std::cout << "Expected u = 5.0, got u = " << u << "\n\n";
        assert(std::fabs(u - 5.0) < 1e-9);
    }

    // ---------------------------------------------------------------
    // 2) saturation() boundary checks (calls it directly, no plant)
    // ---------------------------------------------------------------
    std::cout << "=== Test 2: saturation() boundaries ===\n";
    {
        double fi = 0.1;
        struct Case { double s; double expected; const char* label; };
        Case cases[] = {
            {0.05,  0.5,  "inside boundary layer (linear region)"},
            {-0.05, -0.5, "inside boundary layer, negative"},
            {0.1,   1.0,  "exactly at +fi"},
            {-0.1,  -1.0, "exactly at -fi"},
            {5.0,   1.0,  "well beyond +fi"},
            {-5.0,  -1.0, "well beyond -fi"},
        };
        for (auto& c : cases) {
            double got = SlidingModeController::saturation(c.s, fi);
            std::cout << c.label << " -> got " << got
                      << " (expected " << c.expected << ")\n";
        }
        std::cout << "\n";
    }

    // ---------------------------------------------------------------
    // 3) Full closed-loop simulation
    //    Debug prints from compute()/saturation() are suppressed after
    //    a short verbose window, since they fire every dt.
    // ---------------------------------------------------------------
    std::cout << "=== Test 3: full simulation (m=1kg, xd=10, T=20s) ===\n";
    {
        State state{0.0, 0.0};
        double xd   = 10.0;
        double dt   = 0.001;
        double T    = 20.0;
        double mass = 1.0;

        SlidingModeController smc(2.0, 5.0, 0.1, 0.5);

        std::ofstream log("sim_log.csv");
        log << "t,u,x1,x2\n";

        std::streambuf* real_cout = std::cout.rdbuf();
        std::ostringstream sink; // swallow the per-step debug prints

        int step = 0;
        const int verbose_steps = 3; // show the trace for the first few steps only

        for (double t = 0.0; t < T; t += dt, ++step) {
            if (step == verbose_steps) {
                std::cout.rdbuf(sink.rdbuf()); // start suppressing
            }

            double u = smc.compute(state, xd);
            plant_step(state, u, t, dt, mass);
            log << t << "," << u << "," << state.x1 << "," << state.x2 << "\n";
        }

        std::cout.rdbuf(real_cout); // restore real stdout
        log.close();

        std::cout << "\n--- Simulation finished ---\n";
        std::cout << "Final position x1 = " << state.x1 << " (target " << xd << ")\n";
        std::cout << "Final velocity x2 = " << state.x2 << "\n";
        std::cout << "Position error    = " << (state.x1 - xd) << "\n";
        std::cout << "Full trace logged to sim_log.csv\n";
    }

    std::cout << "\nAll tests completed.\n";
    return 0;
}