#include "control/lqg.hpp"

namespace controls {

LQG::LQG(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B, const Eigen::MatrixXd &C,
         const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
         const Eigen::MatrixXd &processNoiseCov, const Eigen::MatrixXd &measurementNoiseCov)
    : lqr_(A, B, Q, R),
      kf_(A, B, C, processNoiseCov, measurementNoiseCov),
      lastControl_(Eigen::VectorXd::Zero(B.cols())) {}

Eigen::VectorXd LQG::computeControl(const Eigen::VectorXd &y) {
    kf_.predict(lastControl_);
    kf_.update(y);

    Eigen::VectorXd u = lqr_.computeControl(kf_.state());
    lastControl_ = u;
    return u;
}

}  // namespace controls
