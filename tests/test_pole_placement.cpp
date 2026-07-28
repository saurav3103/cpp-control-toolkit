#include <gtest/gtest.h>
#include <vector>
#include <complex>
#include <algorithm>
#include "controls/control/pole_placement.hpp"

using namespace controls;

TEST(PolePlacementTest, PlacesPolesAtRequestedLocations) {
    Eigen::Matrix2d A;
    A << 1.0, 0.01,
        -0.05, 0.98;

    Eigen::Vector2d B;
    B << 0.0, 0.01;

    std::vector<std::complex<double>> desiredPoles = {
        std::complex<double>(0.6, 0.0),
        std::complex<double>(0.3, 0.0)
    };

    Eigen::MatrixXd K = acker(A, B, desiredPoles);
    Eigen::Matrix2d closedLoop = A - B * K;

    Eigen::EigenSolver<Eigen::Matrix2d> solver(closedLoop);
    Eigen::VectorXcd eigenvalues = solver.eigenvalues();

    std::vector<double> actual = {eigenvalues(0).real(), eigenvalues(1).real()};
    std::vector<double> expected = {0.6, 0.3};
    std::sort(actual.begin(), actual.end());
    std::sort(expected.begin(), expected.end());

    EXPECT_NEAR(actual[0], expected[0], 1e-3);
    EXPECT_NEAR(actual[1], expected[1], 1e-3);
}

TEST(PolePlacementTest, ThrowsOnUncontrollableSystem) {
    Eigen::Matrix2d A;
    A << 1.0, 0.0,
         0.0, 0.98;

    Eigen::Vector2d B;
    B << 1.0, 0.0;   // second state unreachable, same as our analysis test

    std::vector<std::complex<double>> desiredPoles = {
        std::complex<double>(0.5, 0.0),
        std::complex<double>(0.5, 0.0)
    };

    EXPECT_THROW(acker(A, B, desiredPoles), std::invalid_argument);
}

TEST(PolePlacementTest, ThrowsOnMismatchedPoleCount) {
    Eigen::Matrix2d A;
    A << 1.0, 0.01,
        -0.05, 0.98;

    Eigen::Vector2d B;
    B << 0.0, 0.01;

    std::vector<std::complex<double>> tooFewPoles = {
        std::complex<double>(0.5, 0.0)
    };

    EXPECT_THROW(acker(A, B, tooFewPoles), std::invalid_argument);
}
