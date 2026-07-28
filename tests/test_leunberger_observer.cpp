#include <gtest/gtest.h>
#include <complex>
#include <vector>
#include "controls/estimation/luenberger_observer.hpp"
#include "controls/core/analysis.hpp"

using namespace controls;

TEST(LuenbergerObserverTest, ErrorDynamicsMatchDesiredPoles) {
    Eigen::Matrix2d A;
    A << 1.0, 0.01,
        -0.05, 0.98;

    Eigen::Vector2d B;
    B << 0.0, 0.01;

    Eigen::MatrixXd C(1, 2);
    C << 1.0, 0.0;

    // desired estimation-error poles, chosen well inside the unit circle
    // (faster decay than the LQR closed-loop poles, since the estimator
    // should converge faster than the controller needs it to)
    std::vector<std::complex<double>> desiredPoles = {
        std::complex<double>(0.5, 0.0),
        std::complex<double>(0.4, 0.0)
    };

    LuenbergerObserver observer(A, B, C, desiredPoles);

    Eigen::MatrixXd L = observer.gain();
    Eigen::Matrix2d closedLoopError = A - L * C;

    Eigen::EigenSolver<Eigen::Matrix2d> solver(closedLoopError);
    Eigen::VectorXcd eigenvalues = solver.eigenvalues();

    // sort both sets by real part before comparing, since eigenvalue order
    // returned by the solver isn't guaranteed to match our input order
    std::vector<double> actual = {eigenvalues(0).real(), eigenvalues(1).real()};
    std::vector<double> expected = {0.5, 0.4};
    std::sort(actual.begin(), actual.end());
    std::sort(expected.begin(), expected.end());

    EXPECT_NEAR(actual[0], expected[0], 1e-3);
    EXPECT_NEAR(actual[1], expected[1], 1e-3);
}

TEST(LuenbergerObserverTest, ClosedLoopErrorIsStable) {
    Eigen::Matrix2d A;
    A << 1.0, 0.01,
        -0.05, 0.98;

    Eigen::Vector2d B;
    B << 0.0, 0.01;

    Eigen::MatrixXd C(1, 2);
    C << 1.0, 0.0;

    std::vector<std::complex<double>> desiredPoles = {
        std::complex<double>(0.5, 0.0),
        std::complex<double>(0.4, 0.0)
    };

    LuenbergerObserver observer(A, B, C, desiredPoles);
    Eigen::MatrixXd L = observer.gain();
    Eigen::Matrix2d closedLoopError = A - L * C;

    EXPECT_TRUE(isStableDiscrete(closedLoopError));
}

TEST(LuenbergerObserverTest, ConvergesToTrueStateOverTime) {
    Eigen::Matrix2d A;
    A << 1.0, 0.01,
        -0.05, 0.98;

    Eigen::Vector2d B;
    B << 0.0, 0.01;

    Eigen::MatrixXd C(1, 2);
    C << 1.0, 0.0;

    std::vector<std::complex<double>> desiredPoles = {
        std::complex<double>(0.5, 0.0),
        std::complex<double>(0.4, 0.0)
    };

    LuenbergerObserver observer(A, B, C, desiredPoles);

    Eigen::Vector2d xTrue;
    xTrue << 1.0, 0.0;

    Eigen::Vector2d xGuess;
    xGuess << 0.0, 0.0;   // deliberately wrong initial guess
    observer.setInitialEstimate(xGuess);

    Eigen::VectorXd u(1);
    u << 0.0;

    for (int k = 0; k < 50; k++) {
        xTrue = A * xTrue + B * u;
        Eigen::VectorXd y = C * xTrue;   // noiseless measurement

        observer.update(u, y);
    }

    EXPECT_NEAR(observer.state()(0), xTrue(0), 1e-3);
    EXPECT_NEAR(observer.state()(1), xTrue(1), 1e-3);
}
