# robotics_control_cpp

A modern C++ control library for robotics with PID and LQR controllers.

## Features

- **PID Controller**: Discrete-time with integral anti-windup and saturation
- **LQR Controller**: Linear Quadratic Regulator for state feedback
- **State-Space Models**: Discrete-time linear systems
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
├── include/control/      # Header files
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

## Example: 2D Point Mass Control
See src/main.cpp for a complete example of controlling a 2D point mass to a target position using a PID controller.

## Future Enhancements
- Kalman filter implementation
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