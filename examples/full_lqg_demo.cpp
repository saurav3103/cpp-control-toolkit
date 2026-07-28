#include <iostream>
#include "controls/control/lqg.hpp"

using namespace controls;

int main() {
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
    xTrue << 1.0, 0.0;   // the controller never sees this directly

    std::cout << "LQG closed-loop simulation (controller only sees noisy measurements):\n";
    for (int k = 0; k < 300; k++) {
        Eigen::VectorXd y(1);
        y << (C * xTrue)(0);

        Eigen::VectorXd u = controller.computeControl(y);
        xTrue = A * xTrue + B * u;

        if (k % 30 == 0) {
            Eigen::VectorXd est = controller.estimatedState();
            std::cout << "step " << k
                      << ": true pos=" << xTrue(0)
                      << "  estimated pos=" << est(0)
                      << std::endl;
        }
    }

    std::cout << "\nFinal true state: pos=" << xTrue(0) << " vel=" << xTrue(1) << std::endl;

    return 0;
}
