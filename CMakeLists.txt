cmake_minimum_required(VERSION 3.14)
project(robotics_control_cpp VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -pedantic")

# Find Eigen
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Add include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(/usr/include/eigen3)  # For Eigen

# Add source subdirectory
add_subdirectory(src)

# Add tests
enable_testing()
add_subdirectory(tests)
