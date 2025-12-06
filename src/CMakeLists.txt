# Library sources
set(CONTROL_SOURCES
    pid_controller.cpp
    lqr_controller.cpp
    state_space.cpp
)

# Create library
add_library(robotics_control STATIC ${CONTROL_SOURCES})

# Link Eigen
target_link_libraries(robotics_control PUBLIC Eigen3::Eigen)

# Include directories
target_include_directories(robotics_control
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

# Executable for testing
add_executable(control_example main.cpp)
target_link_libraries(control_example robotics_control)
