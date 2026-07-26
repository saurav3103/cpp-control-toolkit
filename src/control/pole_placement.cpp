#include "controls/control/pole_placement.hpp"
#include "controls/core/analysis.hpp"
#include <stdexcept>

namespace controls {

Eigen::MatrixXd acker(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                      const std::vector<std::complex<double>> &desiredPoles) {
    int n = A.rows();

    if (B.cols() != 1) {
        throw std::invalid_argument("acker() only supports single-input systems");
    }
    if (static_cast<int>(desiredPoles.size()) != n) {
        throw std::invalid_argument("Number of desired poles must match state dimension");
    }

    if (!isControllable(A, B)) {
        throw std::invalid_argument("System is not controllable, cannot place poles arbitrarily");
    }

    // Build the desired characteristic polynomial coefficients:
    // phi(s) = (s - p1)(s - p2)...(s - pn), expanded into coefficients
    std::vector<std::complex<double>> coeffs = {1.0};  // start with "1" (leading coefficient)
    for (const auto &pole : desiredPoles) {
        std::vector<std::complex<double>> newCoeffs(coeffs.size() + 1, 0.0);
        for (size_t i = 0; i < coeffs.size(); i++) {
            newCoeffs[i]     += coeffs[i];
            newCoeffs[i + 1] -= coeffs[i] * pole;
        }
        coeffs = newCoeffs;
    }

    // Evaluate phi(A) = A^n + c1*A^(n-1) + ... + cn*I
    Eigen::MatrixXd phiA = Eigen::MatrixXd::Zero(n, n);
    Eigen::MatrixXd A_power = Eigen::MatrixXd::Identity(n, n);

    // coeffs[0] corresponds to A^n, coeffs[n] corresponds to A^0
    std::vector<Eigen::MatrixXd> powers(n + 1);
    powers[0] = Eigen::MatrixXd::Identity(n, n);
    for (int i = 1; i <= n; i++) {
        powers[i] = powers[i - 1] * A;
    }

    for (int i = 0; i <= n; i++) {
        double c = coeffs[i].real();  // imaginary parts should cancel for real A
        phiA += c * powers[n - i];
    }

    Eigen::MatrixXd C = controllabilityMatrix(A, B);
    Eigen::MatrixXd C_inv = C.inverse();

    Eigen::MatrixXd lastRow = Eigen::MatrixXd::Zero(1, n);
    lastRow(0, n - 1) = 1.0;

    Eigen::MatrixXd K = lastRow * C_inv * phiA;
    return K;
}

}  // namespace controls
