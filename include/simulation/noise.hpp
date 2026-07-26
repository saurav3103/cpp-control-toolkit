#pragma once
#include <Eigen/Dense>
#include <random>

namespace controls {

class GaussianNoise {
public:
    GaussianNoise(const Eigen::MatrixXd &covariance, unsigned int seed = 42);

    Eigen::VectorXd sample();

private:
    Eigen::MatrixXd L_;  // Cholesky factor of the covariance
    std::mt19937 generator_;
    std::normal_distribution<double> standardNormal_;
};

}  // namespace controls
