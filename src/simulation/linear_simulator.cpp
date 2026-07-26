#include "simulation/linear_simulator.hpp"

namespace controls {

SimulationResult simulateOpenLoop(const StateSpace &system,
                                   const Eigen::VectorXd &x0,
                                   const std::vector<Eigen::VectorXd> &controlSequence) {
    SimulationResult result;
    Eigen::VectorXd x = x0;

    for (const auto &u : controlSequence) {
        result.states.push_back(x);
        result.outputs.push_back(system.output(x, u));
        result.controls.push_back(u);

        x = system.step(x, u);
    }

    return result;
}

}  // namespace controls
