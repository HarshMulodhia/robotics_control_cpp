#include<iostream>
#include "estimation/kalman_filter.hpp"
#include <eigen3/Eigen/Dense>

using namespace robotics_control;
using namespace Eigen;

int main() {

    std::cout << "--- Kalman Filter: Tracking with Noise ---" << std::endl;

    // System: simple integrator (position = velocity * dt)
    // State: [position, velocity]

    double dt = 0.1;
    MatrixXd A(2, 2); MatrixXd B(2, 1);
    A << 1, dt,
        0, 1;
    B << 0, 0; //No control input

    MatrixXd C = MatrixXd::Identity(2, 2);  // Both states are measured

    // Noise covariances
    MatrixXd Q = 0.001 * MatrixXd::Identity(2, 2);  // Process noise (low)
    MatrixXd R = 0.1 * MatrixXd::Identity(2, 2);    // Measurement noise (high)
    
    MatrixXd P0 = MatrixXd::Identity(2, 2);
    VectorXd x0 = VectorXd::Zero(2);
    
    KalmanFilter kf(A, B, C, Q, R, P0, x0);

    //Simulate: true state moves at constant velocity
    VectorXd true_state(2);
    true_state << 0, 1.0;   // velcoity=1.0m/s

    std::cout << "Time\tTrue Pos\tMeas Noise\tKF EST Pos\tKF STD Dev" << std::endl;

    for(int k=0; k<100; ++k){
        //update true state
        true_state(0) += true_state(1)*dt;

        //Noisy Measurement
        double meas_noise = 0.3 * (rand() / (double)RAND_MAX -0.5);
        VectorXd z(2);
        z << true_state(0) + meas_noise, true_state(1);

        //KF update
        kf.measurement_update(z);
        kf.prediction_update(VectorXd::Zero(1));

        if (k%20 == 0) {
            printf("%.1f\t%.2f\t\t%.2f\t\t%.2f\t\t%.4f\n", 
                   k*dt, true_state(0), meas_noise, 
                   kf.get_state()(0), sqrt(kf.get_covariance()(0,0)));
        }
    }
    return 0;
}