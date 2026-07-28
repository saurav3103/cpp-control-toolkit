#include <iostream>
#include <vector>
#include <complex>
#include "controls/estimation/luenberger_observer.hpp"

using namespace controls;

int main() {
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

    std::cout << "Observer gain L =\n" << observer.gain() << std::endl;

    Eigen::Vector2d xTrue;
    xTrue << 1.0, 0.0;

    Eigen::Vector2d xGuess;
    xGuess << 0.0, 0.0;   // deliberately wrong initial guess
    observer.setInitialEstimate(xGuess);

    Eigen::VectorXd u(1);
    u << 0.0;

    std::cout << "\nTrue state vs. estimate over time:\n";
    for (int k = 0; k < 50; k++) {
        xTrue = A * xTrue + B * u;
        Eigen::VectorXd y = C * xTrue;

        observer.update(u, y);

        if (k % 5 == 0) {
            Eigen::VectorXd est = observer.state();
            std::cout << "step " << k
                      << ": true=[" << xTrue(0) << ", " << xTrue(1) << "]"
                      << "  estimate=[" << est(0) << ", " << est(1) << "]"
                      << std::endl;
        }
    }

    return 0;
}
