#pragma once
#include <Eigen/Dense>

namespace controls {

class KalmanFilter {
public:
    KalmanFilter(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                 const Eigen::MatrixXd &C,
                 const Eigen::MatrixXd &processNoiseCov,
                 const Eigen::MatrixXd &measurementNoiseCov);

    void setInitialState(const Eigen::VectorXd &x0, const Eigen::MatrixXd &P0);

    void predict(const Eigen::VectorXd &u);
    void update(const Eigen::VectorXd &y);

    const Eigen::VectorXd& state() const { return xHat_; }
    const Eigen::MatrixXd& covariance() const { return P_; }

private:
    Eigen::MatrixXd A_, B_, C_;
    Eigen::MatrixXd Q_kf_, R_kf_;
    Eigen::VectorXd xHat_;
    Eigen::MatrixXd P_;
};

}  // namespace controls
