#include <gtest/gtest.h>
#include "control/state_space.hpp"

using namespace robotics_control;

TEST(StateSpaceTest, Initialization) {
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(2, 1);
    Eigen::MatrixXd C = Eigen::MatrixXd::Identity(2, 2);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(2, 1);
    
    StateSpace sys(A, B, C, D);
    
    Eigen::VectorXd state = sys.get_state();
    EXPECT_EQ(state.size(), 2);
    EXPECT_EQ(state(0), 0.0);
    EXPECT_EQ(state(1), 0.0);
}

TEST(StateSpaceTest, Step) {
    // Simple system: x[k+1] = x[k], y = x
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(1, 1);
    Eigen::MatrixXd B = Eigen::MatrixXd::Identity(1, 1);
    Eigen::MatrixXd C = Eigen::MatrixXd::Identity(1, 1);
    Eigen::MatrixXd D = Eigen::MatrixXd::Zero(1, 1);
    
    StateSpace sys(A, B, C, D);
    
    Eigen::VectorXd u(1);
    u << 1.0;
    
    Eigen::VectorXd output = sys.step(u);
    
    EXPECT_NEAR(output(0), 0.0, 1e-6);
    EXPECT_NEAR(sys.get_state()(0), 1.0, 1e-6);
}
