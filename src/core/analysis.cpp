#include "controls/core/analysis.hpp"

namespace controls {

Eigen::MatrixXd controllabilityMatrix(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B) {
    int n = A.rows();
    int m = B.cols();
    Eigen::MatrixXd C(n, n * m);

    Eigen::MatrixXd A_power = Eigen::MatrixXd::Identity(n, n);
    for (int i = 0; i < n; i++) {
        C.block(0, i * m, n, m) = A_power * B;
        A_power = A_power * A;
    }
    return C;
}

Eigen::MatrixXd observabilityMatrix(const Eigen::MatrixXd &A, const Eigen::MatrixXd &C) {
    int n = A.rows();
    int p = C.rows();
    Eigen::MatrixXd O(n * p, n);

    Eigen::MatrixXd A_power = Eigen::MatrixXd::Identity(n, n);
    for (int i = 0; i < n; i++) {
        O.block(i * p, 0, p, n) = C * A_power;
        A_power = A_power * A;
    }
    return O;
}

bool isControllable(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B) {
    Eigen::MatrixXd C = controllabilityMatrix(A, B);
    Eigen::FullPivLU<Eigen::MatrixXd> lu(C);
    return lu.rank() == A.rows();
}

bool isObservable(const Eigen::MatrixXd &A, const Eigen::MatrixXd &C) {
    Eigen::MatrixXd O = observabilityMatrix(A, C);
    Eigen::FullPivLU<Eigen::MatrixXd> lu(O);
    return lu.rank() == A.rows();
}

bool isStableContinuous(const Eigen::MatrixXd &A) {
    Eigen::EigenSolver<Eigen::MatrixXd> solver(A);
    Eigen::VectorXcd eigenvalues = solver.eigenvalues();

    for (int i = 0; i < eigenvalues.size(); i++) {
        if (eigenvalues(i).real() >= 0) {
            return false;
        }
    }
    return true;
}

bool isStableDiscrete(const Eigen::MatrixXd &A) {
    Eigen::EigenSolver<Eigen::MatrixXd> solver(A);
    Eigen::VectorXcd eigenvalues = solver.eigenvalues();

    for (int i = 0; i < eigenvalues.size(); i++) {
        if (std::abs(eigenvalues(i)) >= 1.0) {
            return false;
        }
    }
    return true;
}

}  // namespace controls
