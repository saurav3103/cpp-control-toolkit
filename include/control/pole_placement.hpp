#pragma once
#include <Eigen/Dense>
#include <vector>
#include <complex>

namespace controls {

Eigen::MatrixXd acker(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                      const std::vector<std::complex<double>> &desiredPoles);

}  // namespace controls
