#pragma once
#include <Eigen/Dense>
#include "controls/control/lqr.hpp"
#include "controls/estimation/kalman_filter.hpp"

namespace controls {

class LQG {
public:
    LQG(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &C,
        const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
        const Eigen::MatrixXd &processNoiseCov, const Eigen::MatrixXd &measurementNoiseCov);

    Eigen::VectorXd computeControl(const Eigen::VectorXd &y);

    const Eigen::VectorXd& estimatedState() const { return kf_.state(); }
    const Eigen::MatrixXd& gain() const { return lqr_.gain(); }

private:
    LQR lqr_;
    KalmanFilter kf_;
    Eigen::VectorXd lastControl_;
};

}  // namespace controls
