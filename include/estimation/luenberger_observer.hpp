#pragma once
#include <Eigen/Dense>
#include <vector>
#include <complex>

namespace controls {

class LuenbergerObserver {
public:
    LuenbergerObserver(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                        const Eigen::MatrixXd &C,
                        const std::vector<std::complex<double>> &desiredPoles);

    void setInitialEstimate(const Eigen::VectorXd &x0);

    void update(const Eigen::VectorXd &u, const Eigen::VectorXd &y);

    const Eigen::VectorXd& state() const { return xHat_; }
    const Eigen::MatrixXd& gain() const { return L_; }

private:
    Eigen::MatrixXd A_, B_, C_;
    Eigen::MatrixXd L_;
    Eigen::VectorXd xHat_;
};

}  // namespace controls
