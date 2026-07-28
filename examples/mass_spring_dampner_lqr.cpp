#include <iostream>
#include "controls/control/lqr.hpp"

using namespace controls;

int main() {
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

    std::cout << "LQR gain K =\n" << lqr.gain() << std::endl;

    Eigen::Vector2d x;
    x << 1.0, 0.0;

    std::cout << "\nClosed-loop simulation:\n";
    for (int k = 0; k < 300; k++) {
        Eigen::VectorXd u = lqr.computeControl(x);
        x = A * x + B * u;

        if (k % 20 == 0) {
            std::cout << "step " << k << ": pos=" << x(0) << " vel=" << x(1) << std::endl;
        }
    }

    std::cout << "\nFinal state: pos=" << x(0) << " vel=" << x(1) << std::endl;

    return 0;
}
