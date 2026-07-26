#include "controls/estimation/luenberger_observer.hpp"
#include "controls/control/pole_placement.hpp"

namespace controls {

LuenbergerObserver::LuenbergerObserver(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                                        const Eigen::MatrixXd &C,
                                        const std::vector<std::complex<double>> &desiredPoles)
    : A_(A), B_(B), C_(C) {

    // Duality: placing poles for L on (A, C) is equivalent to placing poles
    // for K on (A^T, C^T), then transposing the result.
    Eigen::MatrixXd K_dual = acker(A_.transpose(), C_.transpose(), desiredPoles);
    L_ = K_dual.transpose();

    xHat_ = Eigen::VectorXd::Zero(A_.rows());
}

void LuenbergerObserver::setInitialEstimate(const Eigen::VectorXd &x0) {
    xHat_ = x0;
}

void LuenbergerObserver::update(const Eigen::VectorXd &u, const Eigen::VectorXd &y) {
    Eigen::VectorXd predicted = A_ * xHat_ + B_ * u;
    Eigen::VectorXd innovation = y - C_ * xHat_;
    xHat_ = predicted + L_ * innovation;
}

}  // namespace controls
