#pragma once

#include<eigen3/Eigen/Dense>
using namespace Eigen;

namespace robotics_control {

    /**
     * @class KalmanFilter
     * @brief Discrete-time Kalman Filter for linear systems
     * 
     * State estimate with covariance:
     * x_hat[k|k] := E[x[k]  | measurements up to k]
     * P[k|k] := covariance of estimation error
     * 
     * Prediction: 
     *  x_hat[k|k-1] = A*x_hat[k-1|k-1] + B*u[k-1]
     *  P[k|k-1] = A*P[k-1|k-1]*A' + Q
     * 
     * Measurement Update:
     *  K[k] = P[k|k-1]*C' / (C*P[k|k-1]*C' + R)
     *  x_hat[k|k] = x_hat[k|k-1] + K[k]*(z[k] - c*x[_hat[k|k-1]])
     *  P[k|k] = (I - K[k] *C) * P[k|k-1]
     */
class KalmanFilter {

private:
    MatrixXd A_, B_, C_;    //System Matrices
    MatrixXd Q_, R_;    //Noise covariances
    VectorXd x_hat_;    //State Estimate
    MatrixXd P_, K_;    // Estimate covariance and Kalman Gain
    MatrixXd I_;    //Identity Matrix
public:
    /**
     * @brief Constructor
     * @param A State transition matrix
     * @param B Input matrix
     * @param C Measurement matrix
     * @param Q Process noise covariance
     * @param R Measurement noise covariance
     * @param P0 Initial state covariance
     * @param x0 Initial state estimate
     */
    KalmanFilter(const MatrixXd& A, const MatrixXd& B, const MatrixXd& C,
    const MatrixXd& Q, const MatrixXd& R, const MatrixXd& P0, const VectorXd& x0);

    /**
     * @brief Prediction update
     * @param u Contol input
     */
    void prediction_update(const VectorXd& u);

    /**
     * @brief Measurement update
     * @param z Measurement vector
     */
    void measurement_update(const VectorXd& z);

    /**
     * @brief Get state estimate
     */
    VectorXd get_state() const {return x_hat_;}

    /**
     * @brief Get convariance, P
     */
    MatrixXd get_covariance() const {return P_;}

    /**
     * @brief Get Kalman gain
     */
    MatrixXd get_kalman_gain() const { return K_;}

    /**
     * @brief Reset filter
     */
    void reset(const VectorXd& x0, const MatrixXd& P0);
};

}