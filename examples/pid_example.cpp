#include <iostream>
#include <cmath>
#include <vector>
#include "control/pid_controller.hpp"
#include "control/state_space.hpp"
#include <eigen3/Eigen/Dense>

using namespace robotics_control;
using namespace Eigen;
using namespace std;

int main() {
    cout << "--- 2D Point Mass Control Example ---" << endl;
    cout << endl;

    // Setup: 2D point mass with position and velocity
    // State: [x, y, vx, vy]
    // Control: [ax, ay] (accelerations)
    // Discrete-time dynamics: 
    // x[k+1] = x[k] + vx[k]*dt
    // vx[k+1] = vx[k] + ax[k]*dt
    
    double dt = 0.01;  // 10 ms sampling time
    
    // State-space model for X-axis
    MatrixXd A = MatrixXd::Identity(2, 2);
    A(0, 1) = dt;  // x[k+1] = x[k] + vx[k]*dt
    
    MatrixXd B(2, 1);
    B << 0.5*dt*dt, dt;  // Acceleration affects position and velocity
    
    MatrixXd C = MatrixXd::Identity(2, 2);
    MatrixXd D = MatrixXd::Zero(2, 1);
    
    StateSpace plant(A, B, C, D);
    
    // PID controller for X-axis
    PIDController pid_x(5.0, 0.5, 1.0, dt, -50.0, 50.0);
    
    // Simulation parameters
    int num_steps = 500;  // 5 seconds at 100 Hz
    vector<double> time_vec, x_vec, u_vec;
    
    double target_x = 10.0;  // Target position
    VectorXd state(2);
    state << 0.0, 0.0;  // Initial state [x=0, vx=0]
    plant.set_state(state);
    
    cout << "Time(s)\t\tPosition(m)\tVelocity(m/s)\tControl(m/s^2)" << endl;
    cout << "---" << endl;
    
    for (int k = 0; k < num_steps; ++k) {
        double t = k * dt;
        state = plant.get_state();
        
        double x = state(0);
        double vx = state(1);
        
        // Compute control input
        double u = pid_x.update(target_x, x);
        
        // Apply control to plant
        VectorXd control(1);
        control << u;
        plant.step(control);
        
        // Log data
        time_vec.push_back(t);
        x_vec.push_back(x);
        u_vec.push_back(u);
        
        // Print every 50 steps
        if (k % 50 == 0) {
            printf("%.2f\t\t%.2f\t\t%.4f\t\t%.2f\n", t, x, vx, u);
        }
    }
    
    cout << endl;
    cout << "Final Position: " << x_vec.back() << " m" << endl;
    cout << "Target Position: " << target_x << " m" << endl;
    cout << "Steady-State Error: " << (target_x - x_vec.back()) << " m" << endl;
    
    return 0;
}
