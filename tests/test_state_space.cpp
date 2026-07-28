#include <gtest/gtest.h>
#include "controls/core/state_space.hpp"

using namespace controls;

TEST(StateSpaceTest, StepMatchesManualCalculation) {
    Eigen::Matrix2d A;
    A << 1.0, 0.01,
        -0.05, 0.98;

    Eigen::Vector2d B;
    B << 0.0, 0.01;

    Eigen::MatrixXd C(1, 2);
    C << 1.0, 0.0;

    Eigen::MatrixXd D(1, 1);
    D << 0.0;

    StateSpace sys(A, B, C, D);

    Eigen::Vector2d x;
    x << 1.0, 0.5;

    Eigen::VectorXd u(1);
    u << 2.0;

    Eigen::VectorXd result = sys.step(x, u);
    Eigen::VectorXd expected = A * x + B * u;

    EXPECT_NEAR(result(0), expected(0), 1e-9);
    EXPECT_NEAR(result(1), expected(1), 1e-9);
}

TEST(StateSpaceTest, OutputMatchesManualCalculation) {
    Eigen::Matrix2d A;
    A << 1.0, 0.01,
        -0.05, 0.98;

    Eigen::Vector2d B;
    B << 0.0, 0.01;

    Eigen::MatrixXd C(1, 2);
    C << 1.0, 0.0;

    Eigen::MatrixXd D(1, 1);
    D << 0.0;

    StateSpace sys(A, B, C, D);

    Eigen::Vector2d x;
    x << 1.0, 0.5;

    Eigen::VectorXd u(1);
    u << 2.0;

    Eigen::VectorXd y = sys.output(x, u);
    Eigen::VectorXd expected = C * x + D * u;

    EXPECT_NEAR(y(0), expected(0), 1e-9);
}

TEST(StateSpaceTest, ConstructorRejectsMismatchedDimensions) {
    Eigen::Matrix2d A;
    A << 1.0, 0.01,
        -0.05, 0.98;

    Eigen::MatrixXd B(3, 1);  // deliberately wrong row count
    B << 0.0, 0.01, 0.0;

    Eigen::MatrixXd C(1, 2);
    C << 1.0, 0.0;

    Eigen::MatrixXd D(1, 1);
    D << 0.0;

    EXPECT_THROW(StateSpace(A, B, C, D), std::invalid_argument);
}

TEST(StateSpaceTest, DimensionGettersReturnCorrectSizes) {
    Eigen::Matrix2d A;
    A << 1.0, 0.01,
        -0.05, 0.98;

    Eigen::Vector2d B;
    B << 0.0, 0.01;

    Eigen::MatrixXd C(1, 2);
    C << 1.0, 0.0;

    Eigen::MatrixXd D(1, 1);
    D << 0.0;

    StateSpace sys(A, B, C, D);

    EXPECT_EQ(sys.stateDim(), 2);
    EXPECT_EQ(sys.inputDim(), 1);
    EXPECT_EQ(sys.outputDim(), 1);
}
