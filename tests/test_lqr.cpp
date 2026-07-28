#include <gtest/gtest.h>
#include "controls/control/lqr.hpp"

using namespace controls;

TEST(LQRTest, ComputesCorrectGainForKnownSystem) {
    Eigen::Matrix2d A;
    A << 1.0, 0.01,
        -0.05, 0.98;

    Eigen::Vector2d B;
    B << 0.0, 0.01;

    Eigen::Matrix2d Q;
    Q << 10.0, 0.0,
          0.0, 1.0;

    Eigen::MatrixXd R(1, 1);
    R << 0.1;

    LQR lqr(A, B, Q, R);

    Eigen::MatrixXd K = lqr.gain();

    // Known result from our earlier godbolt run: K ≈ [6.00, 3.13]
    EXPECT_NEAR(K(0, 0), 6.00373, 1e-3);
    EXPECT_NEAR(K(0, 1), 3.13323, 1e-3);
}

TEST(LQRTest, ClosedLoopSystemIsStable) {
    Eigen::Matrix2d A;
    A << 1.0, 0.01,
        -0.05, 0.98;

    Eigen::Vector2d B;
    B << 0.0, 0.01;

    Eigen::Matrix2d Q;
    Q << 10.0, 0.0,
          0.0, 1.0;

    Eigen::MatrixXd R(1, 1);
    R << 0.1;

    LQR lqr(A, B, Q, R);
    Eigen::MatrixXd K = lqr.gain();

    Eigen::Matrix2d closedLoop = A - B * K;

    Eigen::EigenSolver<Eigen::Matrix2d> solver(closedLoop);
    Eigen::VectorXcd eigenvalues = solver.eigenvalues();

    for (int i = 0; i < eigenvalues.size(); i++) {
        EXPECT_LT(std::abs(eigenvalues(i)), 1.0);
    }
}

TEST(LQRTest, ControlDrivesStateTowardZero) {
    Eigen::Matrix2d A;
    A << 1.0, 0.01,
        -0.05, 0.98;

    Eigen::Vector2d B;
    B << 0.0, 0.01;

    Eigen::Matrix2d Q;
    Q << 10.0, 0.0,
          0.0, 1.0;

    Eigen::MatrixXd R(1, 1);
    R << 0.1;

    LQR lqr(A, B, Q, R);

    Eigen::Vector2d x;
    x << 1.0, 0.0;

    for (int k = 0; k < 300; k++) {
        Eigen::VectorXd u = lqr.computeControl(x);
        x = A * x + B * u;
    }

    EXPECT_NEAR(x(0), 0.0, 1e-3);
    EXPECT_NEAR(x(1), 0.0, 1e-3);
}
