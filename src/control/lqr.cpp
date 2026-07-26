#include "controls/control/lqr.hpp"
#include <stdexcept>

namespace controls {

LQR::LQR(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
         const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
         int iterations)
    : A_(A), B_(B), Q_(Q), R_(R) {

    if (A_.rows() != A_.cols()) {
        throw std::invalid_argument("A must be square");
    }
    if (B_.rows() != A_.rows()) {
        throw std::invalid_argument("B row count must match A");
    }

    P_ = solveDARE(iterations);
    K_ = computeGain();
}

Eigen::MatrixXd LQR::solveDARE(int iterations) const {
    Eigen::MatrixXd P = Q_;
    for (int i = 0; i < iterations; i++) {
        Eigen::MatrixXd denom = R_ + B_.transpose() * P * B_;
        Eigen::MatrixXd denomInv = denom.inverse();

        Eigen::MatrixXd P_next = Q_ + A_.transpose() * P * A_
            - (A_.transpose() * P * B_) * denomInv * (B_.transpose() * P * A_);

        P = P_next;
    }
    return P;
}

Eigen::MatrixXd LQR::computeGain() const {
    Eigen::MatrixXd denom = R_ + B_.transpose() * P_ * B_;
    Eigen::MatrixXd denomInv = denom.inverse();
    return denomInv * (B_.transpose() * P_ * A_);
}

Eigen::VectorXd LQR::computeControl(const Eigen::VectorXd &x) const {
    return -K_ * x;
}

}  // namespace controls
