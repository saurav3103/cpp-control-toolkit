#include "controls/core/state_space.hpp"
#include <stdexcept>

namespace controls {

StateSpace::StateSpace(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                        const Eigen::MatrixXd &C, const Eigen::MatrixXd &D)
    : A_(A), B_(B), C_(C), D_(D) {

    if (A_.rows() != A_.cols()) {
        throw std::invalid_argument("A must be square");
    }
    if (B_.rows() != A_.rows()) {
        throw std::invalid_argument("B row count must match A");
    }
    if (C_.cols() != A_.rows()) {
        throw std::invalid_argument("C column count must match A");
    }
}

Eigen::VectorXd StateSpace::step(const Eigen::VectorXd &x, const Eigen::VectorXd &u) const {
    return A_ * x + B_ * u;
}

Eigen::VectorXd StateSpace::output(const Eigen::VectorXd &x, const Eigen::VectorXd &u) const {
    return C_ * x + D_ * u;
}

int StateSpace::stateDim() const {
    return A_.rows();
}

int StateSpace::inputDim() const {
    return B_.cols();
}

int StateSpace::outputDim() const {
    return C_.rows();
}

}  // namespace controls
