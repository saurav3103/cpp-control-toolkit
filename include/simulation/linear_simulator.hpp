#pragma once
#include <Eigen/Dense>
#include <vector>
#include "controls/core/state_space.hpp"

namespace controls {

struct SimulationResult {
    std::vector<Eigen::VectorXd> states;
    std::vector<Eigen::VectorXd> outputs;
    std::vector<Eigen::VectorXd> controls;
};

SimulationResult simulateOpenLoop(const StateSpace &system,
                                   const Eigen::VectorXd &x0,
                                   const std::vector<Eigen::VectorXd> &controlSequence);

}  // namespace controls
