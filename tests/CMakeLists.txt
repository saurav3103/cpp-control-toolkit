find_package(GTest REQUIRED)

add_executable(run_tests
    test_state_space.cpp
    test_discretization.cpp
    test_analysis.cpp
    test_pole_placement.cpp
    test_lqr.cpp
    test_lqg.cpp
    test_kalman_filter.cpp
    test_luenberger_observer.cpp
)

target_link_libraries(run_tests PRIVATE controls GTest::gtest GTest::gtest_main)

add_test(NAME UnitTests COMMAND run_tests)
