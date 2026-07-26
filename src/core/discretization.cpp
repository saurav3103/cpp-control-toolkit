#include "controls/core/discretization.hpp"
#include <unsupported/Eigen/MatrixFunctions>  // needed for .exp()

namespace controls {

Eigen::MatrixXd discretizeA(const Eigen::MatrixXd &A, double dt) {
    return (A * dt).exp();
}

Eigen::MatrixXd discretizeB(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, double dt) {
    Eigen::MatrixXd A_d = discretizeA(A, dt);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(A.rows(), A.rows());
    return A.inverse() * (A_d - I) * B;
}

}  // namespace controls
