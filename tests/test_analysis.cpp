#include <gtest/gtest.h>
#include "controls/core/analysis.hpp"

using namespace controls;

TEST(AnalysisTest, ControllableSystemIsDetectedCorrectly) {
    Eigen::Matrix2d A;
    A << 1.0, 0.01,
        -0.05, 0.98;

    Eigen::Vector2d B;
    B << 0.0, 0.01;

    EXPECT_TRUE(isControllable(A, B));
}

TEST(AnalysisTest, UncontrollableSystemIsDetectedCorrectly) {
    // B only affects the first state, and A has no coupling between states,
    // so the second state can never be influenced by u -- not controllable
    Eigen::Matrix2d A;
    A << 1.0, 0.0,
         0.0, 0.98;

    Eigen::Vector2d B;
    B << 1.0, 0.0;

    EXPECT_FALSE(isControllable(A, B));
}

TEST(AnalysisTest, ObservableSystemIsDetectedCorrectly) {
    Eigen::Matrix2d A;
    A << 1.0, 0.01,
        -0.05, 0.98;

    Eigen::MatrixXd C(1, 2);
    C << 1.0, 0.0;

    EXPECT_TRUE(isObservable(A, C));
}

TEST(AnalysisTest, StableDiscreteSystemIsDetectedCorrectly) {
    Eigen::Matrix2d A;
    A << 0.5, 0.0,
         0.0, 0.3;

    EXPECT_TRUE(isStableDiscrete(A));
}

TEST(AnalysisTest, UnstableDiscreteSystemIsDetectedCorrectly) {
    Eigen::Matrix2d A;
    A << 1.5, 0.0,
         0.0, 0.3;

    EXPECT_FALSE(isStableDiscrete(A));
}
