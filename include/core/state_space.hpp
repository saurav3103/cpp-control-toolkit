#pragma once
#include <Eigen/Dense>

namespace controls {

class StateSpace {
public:
    StateSpace(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
               const Eigen::MatrixXd &C, const Eigen::MatrixXd &D);

    Eigen::VectorXd step(const Eigen::VectorXd &x, const Eigen::VectorXd &u) const;
    Eigen::VectorXd output(const Eigen::VectorXd &x, const Eigen::VectorXd &u) const;

    int stateDim() const;
    int inputDim() const;
    int outputDim() const;

    const Eigen::MatrixXd& A() const { return A_; }
    const Eigen::MatrixXd& B() const { return B_; }
    const Eigen::MatrixXd& C() const { return C_; }
    const Eigen::MatrixXd& D() const { return D_; }

private:
    Eigen::MatrixXd A_, B_, C_, D_;
};

}  // namespace controls
