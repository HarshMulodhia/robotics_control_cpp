# Find GoogleTest
find_package(GTest REQUIRED)

# Test sources
set(TEST_SOURCES
    test_pid.cpp
    test_lqr.cpp
    test_state_space.cpp
    test_main.cpp
)

# Create test executable
add_executable(runTests ${TEST_SOURCES})

# Link libraries
target_link_libraries(runTests
    robotics_control
    GTest::gtest
    GTest::gtest_main
    Eigen3::Eigen
)

# Include directories
target_include_directories(runTests
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

# Discover tests
include(GoogleTest)
gtest_discover_tests(runTests)
