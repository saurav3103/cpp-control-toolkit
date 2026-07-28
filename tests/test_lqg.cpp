#include <gtest/gtest.h>
#include "controls/control/lqg.hpp"

using namespace controls;

TEST(LQGTest, DrivesTrueStateTowardZeroUsingOnlyMeasurements) {
    Eigen::Matrix2d A;
    A << 1.0, 0.01,
        -0.05, 0.98;

    Eigen::Vector2d B;
    B << 0.0, 0.01;

    Eigen::MatrixXd C(1, 2);
    C << 1.0, 0.0;

    Eigen::Matrix2d Q;
    Q << 10.0, 0.0,
          0.0, 1.0;

    Eigen::MatrixXd R(1, 1);
    R << 0.1;

    Eigen::Matrix2d Qkf = Eigen::Matrix2d::Identity() * 0.0001;
    Eigen::MatrixXd Rkf(1, 1);
    Rkf << 0.001;

    LQG controller(A, B, C, Q, R, Qkf, Rkf);

    Eigen::Vector2d xTrue;
    xTrue << 1.0, 0.0;   // true initial state, never directly visible to the controller

    for (int k = 0; k < 300; k++) {
        Eigen::VectorXd y(1);
        y << (C * xTrue)(0);   // only a noiseless measurement is available

        Eigen::VectorXd u = controller.computeControl(y);
        xTrue = A * xTrue + B * u;
    }

    EXPECT_NEAR(xTrue(0), 0.0, 1e-2);
    EXPECT_NEAR(xTrue(1), 0.0, 1e-2);
}

TEST(LQGTest, StateEstimateConvergesToTrueState) {
    Eigen::Matrix2d A;
    A << 1.0, 0.01,
        -0.05, 0.98;

    Eigen::Vector2d B;
    B << 0.0, 0.01;

    Eigen::MatrixXd C(1, 2);
    C << 1.0, 0.0;

    Eigen::Matrix2d Q;
    Q << 10.0, 0.0,
          0.0, 1.0;

    Eigen::MatrixXd R(1, 1);
    R << 0.1;

    Eigen::Matrix2d Qkf = Eigen::Matrix2d::Identity() * 0.0001;
    Eigen::MatrixXd Rkf(1, 1);
    Rkf << 0.001;

    LQG controller(A, B, C, Q, R, Qkf, Rkf);

    Eigen::Vector2d xTrue;
    xTrue << 1.0, 0.0;

    for (int k = 0; k < 300; k++) {
        Eigen::VectorXd y(1);
        y << (C * xTrue)(0);

        Eigen::VectorXd u = controller.computeControl(y);
        xTrue = A * xTrue + B * u;
    }

    Eigen::VectorXd estimate = controller.estimatedState();

    EXPECT_NEAR(estimate(0), xTrue(0), 1e-2);
    EXPECT_NEAR(estimate(1), xTrue(1), 1e-2);
}
