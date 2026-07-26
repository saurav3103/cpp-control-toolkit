#pragma once
#include <Eigen/Dense>

namespace controls {

class LQR {
public:
    LQR(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
        const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
        int iterations = 500);

    Eigen::VectorXd computeControl(const Eigen::VectorXd &x) const;

    const Eigen::MatrixXd& gain() const { return K_; }
    const Eigen::MatrixXd& riccatiSolution() const { return P_; }

private:
    Eigen::MatrixXd A_, B_, Q_, R_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd K_;

    Eigen::MatrixXd solveDARE(int iterations) const;
    Eigen::MatrixXd computeGain() const;
};

}  // namespace controls
