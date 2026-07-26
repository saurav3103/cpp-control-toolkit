#pragma once
#include <Eigen/Dense>

namespace controls {

Eigen::MatrixXd discretizeA(const Eigen::MatrixXd &A, double dt);
Eigen::MatrixXd discretizeB(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, double dt);

}  // namespace controls
