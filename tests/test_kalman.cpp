#include<gtest/gtest.h>
#include "estimation/kalman_filter.hpp"

using namespace robotics_control;
using namespace Eigen;

TEST(KalmanFilterTest, Initialization) {
    MatrixXd A = MatrixXd::Identity(2, 2);
    MatrixXd B = MatrixXd::Zero(2, 1);
    MatrixXd C = MatrixXd::Identity(2, 2);
    MatrixXd Q = 0.01 * MatrixXd::Identity(2, 2);
    MatrixXd R = 0.1 * MatrixXd::Identity(2, 2);
    MatrixXd P0 = MatrixXd::Identity(2, 2);
    VectorXd x0 = VectorXd::Zero(2);

    KalmanFilter kf(A, B, C, Q, R, P0, x0);

    EXPECT_EQ(kf.get_state().size(), 2);
    EXPECT_EQ(kf.get_covariance().rows(), 2);
}

TEST(KalmanFilterTest, PredictionStep) {
    MatrixXd A = MatrixXd::Identity(1, 1);
    MatrixXd B(1, 1);
    B << 1.0;
    MatrixXd C = MatrixXd::Identity(1, 1);
    MatrixXd Q = 0.01 * MatrixXd::Identity(1, 1);
    MatrixXd R = 0.1 * MatrixXd::Identity(1, 1);
    MatrixXd P0 = MatrixXd::Identity(1, 1);
    VectorXd x0(1);
    x0 << 0.0;

    KalmanFilter kf(A, B, C, Q, R, P0, x0);

    VectorXd u(1);
    u << 1.0;

    kf.prediction_update(u);

    EXPECT_NEAR(kf.get_state()(0), 1.0, 1e-6);
}

TEST(KalmanFilterTest, MeasurementStep) {
    MatrixXd A = MatrixXd::Identity(1, 1);
    MatrixXd B = MatrixXd::Zero(1, 1);
    MatrixXd C = MatrixXd::Identity(1, 1);
    MatrixXd Q = 0.01 * MatrixXd::Identity(1, 1);
    MatrixXd R = 0.1 * MatrixXd::Identity(1, 1);
    MatrixXd P0 = MatrixXd::Identity(1, 1);
    VectorXd x0(1);
    x0 << 0.0;

    KalmanFilter kf(A, B, C, Q, R, P0, x0);

    VectorXd z(1);  //New Measurement
    z << 5.0;

    kf.measurement_update(z);

    // State should move towards measurement
    EXPECT_GT(kf.get_state()(0), 0.0);
    EXPECT_LT(kf.get_state()(0), 5.0);

}