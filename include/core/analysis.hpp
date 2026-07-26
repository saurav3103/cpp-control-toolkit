#pragma once
#include <Eigen/Dense>

namespace controls {

Eigen::MatrixXd controllabilityMatrix(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B);
Eigen::MatrixXd observabilityMatrix(const Eigen::MatrixXd &A, const Eigen::MatrixXd &C);

bool isControllable(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B);
bool isObservable(const Eigen::MatrixXd &A, const Eigen::MatrixXd &C);

bool isStableContinuous(const Eigen::MatrixXd &A);
bool isStableDiscrete(const Eigen::MatrixXd &A);

}  // namespace controls
