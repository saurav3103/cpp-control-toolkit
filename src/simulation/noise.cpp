#include "controls/simulation/noise.hpp"

namespace controls {

GaussianNoise::GaussianNoise(const Eigen::MatrixXd &covariance, unsigned int seed)
    : generator_(seed), standardNormal_(0.0, 1.0) {

    Eigen::LLT<Eigen::MatrixXd> llt(covariance);
    L_ = llt.matrixL();
}

Eigen::VectorXd GaussianNoise::sample() {
    int n = L_.rows();
    Eigen::VectorXd z(n);
    for (int i = 0; i < n; i++) {
        z(i) = standardNormal_(generator_);
    }
    return L_ * z;
}

}  // namespace controls
