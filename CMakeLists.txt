cmake_minimum_required(VERSION 3.14)
project(controls_cpp VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)

add_library(controls
    src/core/state_space.cpp
    src/core/discretization.cpp
    src/core/analysis.cpp

    src/control/pole_placement.cpp
    src/control/lqr.cpp
    src/control/lqg.cpp

    src/estimation/kalman_filter.cpp
    src/estimation/luenberger_observer.cpp

    src/simulation/linear_simulator.cpp
    src/simulation/noise.cpp
)

target_include_directories(controls PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(controls PUBLIC Eigen3::Eigen)

add_executable(mass_spring_damper_lqr examples/mass_spring_damper_lqr.cpp)
target_link_libraries(mass_spring_damper_lqr PRIVATE controls)

add_executable(observer_state_estimation examples/observer_state_estimation.cpp)
target_link_libraries(observer_state_estimation PRIVATE controls)

add_executable(full_lqg_demo examples/full_lqg_demo.cpp)
target_link_libraries(full_lqg_demo PRIVATE controls)

enable_testing()
add_subdirectory(tests)
