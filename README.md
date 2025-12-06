# robotics_control_cpp

A modern C++ control library for robotics with PID and LQR controllers.

## Features

- **PID Controller**: Discrete-time with integral anti-windup and saturation
- **LQR Controller**: Linear Quadratic Regulator for state feedback
- **State-Space Models**: Discrete-time linear systems
- **Kalman-Filter**: Discrete-time Kalman Filter for linear systems
- **Unit Tests**: Comprehensive test coverage with GoogleTest
- **CMake Build**: Modern CMake-based build system

## Requirements

- C++17 or later
- CMake 3.14+
- Eigen3 (linear algebra library)
- GoogleTest (for running tests)

## Installation

### Ubuntu/WSL

```bash
sudo apt-get install libeigen3-dev libgtest-dev cmake
cd /usr/src/gtest && sudo mkdir build && cd build && \
sudo cmake .. && sudo make && sudo make install
```

### macOS
```bash
brew install eigen googletest cmake
```

## Building
```bash
mkdir build && cd build
cmake ..
make
```

## Running Tests
```bash
ctest --output-on-failure
# or
./tests/runTests
```

## Running Example
```bash
./src/control_example
```

## Project Structure
```text
robotics_control_cpp/
├── examples/              # Example Implementation
├── include/              # Header files
    ├── control/
    ├── estimation/
├── src/                  # Implementation files
├── tests/                # Unit tests
└── CMakeLists.txt       # Build configuration
```

## Classes
### PIDController
Discrete-time PID controller with features:

- Proportional, integral, and derivative terms
- Integral anti-windup
- Output saturation

```cpp
PIDController pid(Kp, Ki, Kd, dt, min, max);
double u = pid.update(setpoint, measurement);
```

### StateSpace
Discrete-time linear state-space model:

```text
x[k+1] = A*x[k] + B*u[k]
y[k]   = C*x[k] + D*u[k]
```
```cpp
StateSpace system(A, B, C, D);
VectorXd output = system.step(input);
```

### LQRController
Linear Quadratic Regulator for state feedback:

```text
u = -K*x
```
```cpp
LQRController lqr(K);
VectorXd u = lqr.compute_input(state);
```

### KalmanFilter
Discrete-time Kalman Filter for linear systems.

```text
State estimate with covariance:
    x_hat[k|k] := E[x[k]  | measurements up to k]
    P[k|k] := covariance of estimation error
Prediction: 
    x_hat[k|k-1] = A*x_hat[k-1|k-1] + B*u[k-1]
    P[k|k-1] = A*P[k-1|k-1]*A' + Q
Measurement Update:
    K[k] = P[k|k-1]*C' / (C*P[k|k-1]*C' + R)
    x_hat[k|k] = x_hat[k|k-1] + K[k]*(z[k] - c*x[_hat[k|k-1]])
    P[k|k] = (I - K[k] *C) * P[k|k-1]
```
```bash
KalmanFilter kf(A, B, C, Q, R, P0, x0);
kf.measurement_update(z);
kf.prediction_update(VectorXd::Zero(1));
```

## Example: 2D Point Mass Control
See src/main.cpp for a complete example of controlling a 2D point mass to a target position using a PID controller.

## Future Enhancements
- MPC (Model Predictive Control) solver
- LTI discrete-time optimal control
- Observer design for state estimation
- Documentation with Doxygen

## References
- Pieter P. "PID Controller Implementation in C++"
- "Discrete-Time Control Systems" (Franklin, Powell, Workman)

## License
- MIT License

## Author
Harsh Mulodhia