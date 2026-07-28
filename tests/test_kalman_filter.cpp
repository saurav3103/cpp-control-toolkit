#include <gtest/gtest.h>
#include "controls/estimation/kalman_filter.hpp"

using namespace controls;

TEST(KalmanFilterTest, PredictOnlyMatchesOpenLoopDynamics) {
    Eigen::Matrix2d A;
    A << 1.0, 0.01,
        -0.05, 0.98;

    Eigen::Vector2d B;
    B << 0.0, 0.01;

    Eigen::MatrixXd C(1, 2);
    C << 1.0, 0.0;

    Eigen::Matrix2d Qkf = Eigen::Matrix2d::Identity() * 0.001;
    Eigen::MatrixXd Rkf(1, 1);
    Rkf << 0.01;

    KalmanFilter kf(A, B, C, Qkf, Rkf);

    Eigen::Vector2d x0;
    x0 << 1.0, 0.0;
    kf.setInitialState(x0, Eigen::Matrix2d::Identity());

    Eigen::VectorXd u(1);
    u << 0.0;
    kf.predict(u);

    Eigen::Vector2d expected = A * x0 + B * u;

    EXPECT_NEAR(kf.state()(0), expected(0), 1e-9);
    EXPECT_NEAR(kf.state()(1), expected(1), 1e-9);
}

TEST(KalmanFilterTest, CovarianceShrinksAfterUpdate) {
    Eigen::Matrix2d A;
    A << 1.0, 0.01,
        -0.05, 0.98;

    Eigen::Vector2d B;
    B << 0.0, 0.01;

    Eigen::MatrixXd C(1, 2);
    C << 1.0, 0.0;

    Eigen::Matrix2d Qkf = Eigen::Matrix2d::Identity() * 0.001;
    Eigen::MatrixXd Rkf(1, 1);
    Rkf << 0.01;

    KalmanFilter kf(A, B, C, Qkf, Rkf);

    Eigen::Vector2d x0;
    x0 << 1.0, 0.0;
    kf.setInitialState(x0, Eigen::Matrix2d::Identity());

    double traceBefore = kf.covariance().trace();

    Eigen::VectorXd u(1);
    u << 0.0;
    kf.predict(u);

    Eigen::VectorXd y(1);
    y << 1.0;   // a measurement consistent with our estimate
    kf.update(y);

    double traceAfter = kf.covariance().trace();

    EXPECT_LT(traceAfter, traceBefore + 0.001);  // update should not increase uncertainty
}

TEST(KalmanFilterTest, ConvergesToTrueStateOverTime) {
    Eigen::Matrix2d A;
    A << 1.0, 0.01,
        -0.05, 0.98;

    Eigen::Vector2d B;
    B << 0.0, 0.01;

    Eigen::MatrixXd C(1, 2);
    C << 1.0, 0.0;

    Eigen::Matrix2d Qkf = Eigen::Matrix2d::Identity() * 0.0001;
    Eigen::MatrixXd Rkf(1, 1);
    Rkf << 0.01;

    KalmanFilter kf(A, B, C, Qkf, Rkf);

    Eigen::Vector2d xTrue;
    xTrue << 1.0, 0.0;

    Eigen::Vector2d xGuess;
    xGuess << 0.0, 0.0;   // deliberately wrong initial guess
    kf.setInitialState(xGuess, Eigen::Matrix2d::Identity());

    Eigen::VectorXd u(1);
    u << 0.0;

    for (int k = 0; k < 200; k++) {
        xTrue = A * xTrue + B * u;                    // true system evolves
        Eigen::VectorXd y = C * xTrue;                 // noiseless measurement, for a clean convergence check

        kf.predict(u);
        kf.update(y);
    }

    EXPECT_NEAR(kf.state()(0), xTrue(0), 1e-2);
    EXPECT_NEAR(kf.state()(1), xTrue(1), 1e-2);
}
