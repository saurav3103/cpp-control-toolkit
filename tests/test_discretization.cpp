#include <gtest/gtest.h>
#include "controls/core/discretization.hpp"

using namespace controls;

TEST(DiscretizationTest, DiscretizeAMatchesKnownExponential) {
    // A simple 1D decay system: xdot = -1 * x
    // Exact discretization: A_d = exp(-1 * dt)
    Eigen::MatrixXd A(1, 1);
    A << -1.0;

    double dt = 0.1;
    Eigen::MatrixXd A_d = discretizeA(A, dt);

    double expected = std::exp(-1.0 * dt);
    EXPECT_NEAR(A_d(0, 0), expected, 1e-9);
}

TEST(DiscretizationTest, DiscretizeAAtZeroDtGivesIdentity) {
    Eigen::Matrix2d A;
    A << -1.0, 0.5,
          0.0, -2.0;

    Eigen::MatrixXd A_d = discretizeA(A, 0.0);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(2, 2);

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            EXPECT_NEAR(A_d(i, j), I(i, j), 1e-9);
        }
    }
}

TEST(DiscretizationTest, DiscretizeBProducesFiniteResult) {
    Eigen::Matrix2d A;
    A << -1.0, 0.5,
          0.0, -2.0;

    Eigen::Vector2d B;
    B << 0.0, 1.0;

    Eigen::MatrixXd B_d = discretizeB(A, B, 0.01);

    EXPECT_TRUE(B_d.allFinite());
}
