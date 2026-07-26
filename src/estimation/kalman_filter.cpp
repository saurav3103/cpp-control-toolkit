#include "controls/estimation/kalman_filter.hpp"

namespace controls {

KalmanFilter::KalmanFilter(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                            const Eigen::MatrixXd &C,
                            const Eigen::MatrixXd &processNoiseCov,
                            const Eigen::MatrixXd &measurementNoiseCov)
    : A_(A), B_(B), C_(C), Q_kf_(processNoiseCov), R_kf_(measurementNoiseCov) {

    xHat_ = Eigen::VectorXd::Zero(A_.rows());
    P_ = Eigen::MatrixXd::Identity(A_.rows(), A_.rows());
}

void KalmanFilter::setInitialState(const Eigen::VectorXd &x0, const Eigen::MatrixXd &P0) {
    xHat_ = x0;
    P_ = P0;
}

void KalmanFilter::predict(const Eigen::VectorXd &u) {
    xHat_ = A_ * xHat_ + B_ * u;
    P_ = A_ * P_ * A_.transpose() + Q_kf_;
}

void KalmanFilter::update(const Eigen::VectorXd &y) {
    Eigen::MatrixXd S = C_ * P_ * C_.transpose() + R_kf_;
    Eigen::MatrixXd K = P_ * C_.transpose() * S.inverse();

    xHat_ = xHat_ + K * (y - C_ * xHat_);

    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(A_.rows(), A_.rows());
    P_ = (I - K * C_) * P_;
}

}  // namespace controls
