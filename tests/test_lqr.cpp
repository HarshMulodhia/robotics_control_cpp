#include<gtest/gtest.h>
#include "control/lqr_controller.hpp"
#include<eigen3/Eigen/Dense>

using namespace robotics_control;

TEST(LQRControllerTest, Initialization) {
    Eigen::MatrixXd K = Eigen::MatrixXd::Zero(1, 2);
    LQRController lqr(K);
    EXPECT_EQ(lqr.get_gain().rows(), 1);
    EXPECT_EQ(lqr.get_gain().cols(), 2);
}

TEST(LQRControllerTest, ComputeInput) {
    Eigen::MatrixXd K(1, 2);
    K << 1.0, 2.0;

    LQRController lqr(K);

    Eigen::VectorXd state(2);
    state << 1.0, 0.5;

    Eigen::VectorXd u = lqr.compute_input(state);

    //u=-K*x = -(1*1 + 2*0.5) = -2
    EXPECT_NEAR(u(0), -2.0, 1e-6);
}