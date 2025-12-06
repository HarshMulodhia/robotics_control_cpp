#include <gtest/gtest.h>
#include "control/pid_controller.hpp"
#include "eigen3/Eigen/Dense"

using namespace robotics_control;

TEST(PIDControllerTest, Initialization) {
    PIDController pid(1.0, 0.1, 0.05, 0.01);
    EXPECT_EQ(pid.get_error(), 0.0);
    EXPECT_EQ(pid.get_integral(), 0.0);
}

TEST(PIDControllerTest, SetpointTracking) {
    PIDController pid(1.0, 0.0, 0.0, 0.01);  // P only
    
    double setpoint = 1.0;
    double measurement = 0.0;
    double output = pid.update(setpoint, measurement);
    
    // With Kp=1, error=1, should get u=1
    EXPECT_NEAR(output, 1.0, 1e-6);
}

TEST(PIDControllerTest, IntegralAction) {
    PIDController pid(0.0, 1.0, 0.0, 0.1);  // I only
    
    double setpoint = 1.0;
    double measurement = 0.0;
    
    // Run several steps
    pid.update(setpoint, measurement);
    pid.update(setpoint, measurement);
    pid.update(setpoint, measurement);
    
    // Integral should accumulate
    EXPECT_GT(pid.get_integral(), 0.0);
}

TEST(PIDControllerTest, SaturationLimits) {
    PIDController pid(1.0, 0.1, 0.05, 0.01, -0.5, 0.5);
    
    double setpoint = 100.0;
    double measurement = 0.0;
    double output = pid.update(setpoint, measurement);
    
    // Output should be saturated to 0.5
    EXPECT_LE(output, 0.5);
}

TEST(PIDControllerTest, Reset) {
    PIDController pid(1.0, 0.1, 0.05, 0.01);
    pid.update(1.0, 0.0);
    
    pid.reset();
    
    EXPECT_EQ(pid.get_error(), 0.0);
    EXPECT_EQ(pid.get_integral(), 0.0);
}
