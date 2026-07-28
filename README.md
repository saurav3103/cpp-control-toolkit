# controls-cpp

A from-scratch C++/Eigen library for linear state-space control — LQR, Kalman filtering, and observer design, implemented and tested against analytical benchmarks.

Built as a companion to prior MATLAB-based control work (LQR/LQG design, Luenberger observer design, Kalman filter state estimation), this library re-implements the same core algorithms in C++, using [Eigen](https://eigen.tuxfamily.org/) for linear algebra, with a proper multi-file structure, CMake build, and a Google Test suite verifying correctness against known results.

## Scope

This library deliberately covers **state-space / modern control and estimation**, not classical control (no PID, root locus, or frequency-response tooling) — kept narrow so every module reflects the same depth of implementation and testing, rather than trading breadth for shallower coverage.

| Area | Contents |
|---|---|
| **Core** | `StateSpace` model class, continuous→discrete conversion (`c2d`), controllability/observability/stability analysis |
| **Control** | Ackermann's formula (pole placement), DARE-based LQR, combined LQG (LQR + Kalman filter) |
| **Estimation** | Discrete-time Kalman filter, Luenberger observer (via control/estimation duality) |
| **Simulation** | Generic linear system simulator, correlated Gaussian noise injection (Cholesky-based) |

## File structure

```
controls-cpp/
├── CMakeLists.txt
├── README.md
│
├── include/
│   ├── core/
│   │   ├── state_space.hpp
│   │   ├── discretization.hpp
│   │   └── analysis.hpp
│   ├── control/
│   │   ├── pole_placement.hpp
│   │   ├── lqr.hpp
│   │   └── lqg.hpp
│   ├── estimation/
│   │   ├── kalman_filter.hpp
│   │   └── luenberger_observer.hpp
│   └── simulation/
│       ├── linear_simulator.hpp
│       └── noise.hpp
│
├── src/
│   ├── core/
│   │   ├── state_space.cpp
│   │   ├── discretization.cpp
│   │   └── analysis.cpp
│   ├── control/
│   │   ├── pole_placement.cpp
│   │   ├── lqr.cpp
│   │   └── lqg.cpp
│   ├── estimation/
│   │   ├── kalman_filter.cpp
│   │   └── luenberger_observer.cpp
│   └── simulation/
│       ├── linear_simulator.cpp
│       └── noise.cpp
│
├── tests/
│   ├── CMakeLists.txt
│   ├── test_state_space.cpp
│   ├── test_discretization.cpp
│   ├── test_analysis.cpp
│   ├── test_pole_placement.cpp
│   ├── test_lqr.cpp
│   ├── test_lqg.cpp
│   ├── test_kalman_filter.cpp
│   └── test_luenberger_observer.cpp
│
└── examples/
    ├── mass_spring_damper_lqr.cpp
    ├── observer_state_estimation.cpp
    └── full_lqg_demo.cpp
```

**Note on `#include` paths:** headers are included by their path relative to `include/` — e.g. `#include "core/state_space.hpp"`, `#include "control/lqr.hpp"` — rather than being nested under a top-level `controls/` namespace folder. All library code itself still lives inside the `controls` C++ namespace (`controls::StateSpace`, `controls::LQR`, etc.), so there's no naming collision risk at the code level; only the header file paths are flat relative to `include/`.

## Design choices

- **Dynamic-size Eigen types (`MatrixXd`/`VectorXd`)** throughout, rather than templated fixed-size matrices. This keeps the API simple and lets a single compiled class handle any system dimension at runtime — a deliberate tradeoff favoring readability and flexibility for research/prototyping use over the compile-time-checked, allocation-free performance of a templated fixed-size design (a natural v2 direction for hard real-time use cases).
- **Runtime dimension validation** in constructors (`StateSpace`, `acker`, etc.), throwing `std::invalid_argument` on mismatch — since dynamic-size types can't be dimension-checked at compile time, this is the safety net that choice requires.
- **Separation principle made explicit in code**: `LQG` doesn't reimplement estimation or control — it owns an `LQR` and a `KalmanFilter` as members and composes them, mirroring how the underlying theory treats controller and estimator design as independent problems.
- **Control/estimation duality reused directly**: `LuenbergerObserver`'s gain is computed by calling the same `acker()` function used for LQR pole placement, on the transposed system — avoiding duplicate logic for what is mathematically the same problem.

## Building

Requires [Eigen3](https://eigen.tuxfamily.org/) and [Google Test](https://github.com/google/googletest) installed (e.g. via `apt install libeigen3-dev libgtest-dev` on Linux).

```bash
mkdir build && cd build
cmake ..
make
ctest        # run the full test suite
```

This produces three example executables (`mass_spring_damper_lqr`, `observer_state_estimation`, `full_lqg_demo`) alongside the test binary.

## Verification approach

Every module is checked against a known result, not just "does it run":

- **LQR**: gain matrix compared against an independently-verified analytical solution; closed-loop eigenvalues checked for discrete stability; full closed-loop simulation checked to converge to zero
- **Kalman filter**: predict-only step checked against the plain state-space equation; covariance checked to shrink (never grow) after a measurement update; full filter checked to converge to the true state from a deliberately wrong initial guess
- **Luenberger observer**: closed-loop error-dynamics eigenvalues checked to land exactly at the requested pole locations; convergence checked from a wrong initial guess
- **LQG**: full closed-loop simulation checked to drive the *true* state to zero using only noisy measurements — the controller never has access to the true state, matching a real deployment scenario

## Example: full LQG demo

```cpp
LQG controller(A, B, C, Q, R, processNoiseCov, measurementNoiseCov);

Eigen::VectorXd y = /* noisy measurement */;
Eigen::VectorXd u = controller.computeControl(y);   // internally: estimate state, then apply LQR gain
```

See `examples/full_lqg_demo.cpp` for the complete closed-loop simulation.

## Background

This library follows on from prior MATLAB-based work on LQR controller design, Luenberger observer design for a wind turbine drivetrain (pole placement, gain scheduling), and Kalman-filter-based state estimation in an adaptive optics closed-loop control simulation. See [`adaptive-optics-control`](#) for the related Python/optics-focused project.

## Possible extensions

- Templated, fixed-size variant for real-time/embedded use, avoiding heap allocation
- Extended Kalman filter for nonlinear systems
- MPC (constrained optimal control) as a separate, larger module
