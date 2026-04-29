# Observer-Based LQR/LQG Control of an Inverted Pendulum on a Cart

Author: Yonghao Huo

## Abstract

This report presents the design and nonlinear simulation of an observer-based servo controller for an inverted pendulum mounted on a cart. A linearized state-space model is used to design an LQR controller with integral action for cart-position tracking. A Kalman/LQE observer estimates unmeasured velocity states from noisy measurements of pendulum angle and cart position. The controller is validated on a nonlinear plant model with cart friction, actuator saturation, actuator rate limits, actuator lag, measurement noise, external disturbances, and safety constraints. The final simulation keeps the pendulum upright while tracking a smooth cart-position reference and recovering from disturbance forces.

Index Terms: Inverted pendulum, LQR, LQG, Kalman observer, state-space control, nonlinear simulation, servo control.

## I. Introduction

The inverted pendulum on a cart is a classical benchmark problem in control systems because it is nonlinear, unstable around the upright equilibrium, and underactuated in the sense that the controller applies force only to the cart while indirectly controlling the pendulum angle. This project uses the benchmark to demonstrate a complete controller-development workflow: modeling, linear control design, state estimation, nonlinear validation, practical actuator constraints, disturbance rejection, and performance logging.

The project objective is to stabilize the pendulum around the upright position while commanding the cart to follow a reference trajectory. Unlike an ideal textbook simulation, the practical model includes sensor noise, friction, limited actuator force, actuator dynamics, and disturbance pulses.

## II. System Model

The state vector is defined as

```text
x = [theta, theta_dot, x_c, x_c_dot]^T
```

where `theta` is the pendulum angle relative to the upright position and `x_c` is the cart position. The controller and observer are designed from a linearized state-space model around the upright equilibrium. The simulation then validates the design using nonlinear cart-pendulum dynamics.

The physical parameters used in the practical simulation are:

| Parameter | Value |
| --- | ---: |
| Cart mass | 2 kg |
| Pendulum mass | 1 kg |
| Pendulum length | 4 m |
| Gravity | 9.8 m/s^2 |
| Cart viscous friction | 0.25 N/(m/s) |

## III. Controller and Observer Design

The control law uses an observer-based servo structure:

```text
u = -Kx*x_hat - Ki*xi
```

where `x_hat` is the estimated state, `Kx` is the LQR feedback gain, `Ki` is the integral gain, and `xi` is the integral of cart-position tracking error. The integral state is added so the cart can track nonzero position references with reduced steady-state error.

Only pendulum angle and cart position are measured. Pendulum angular velocity and cart velocity are estimated by a Kalman/LQE observer. This gives the project an LQG-style structure: LQR provides the feedback gain and LQE provides the estimator gain.

## IV. Practical Simulation Features

The main script validates the controller under practical nonidealities:

- actuator force saturation at +/-20 N
- actuator slew-rate limit
- first-order actuator lag
- cart viscous friction
- noisy measurements
- external force disturbances at 6 s and 12 s
- cart travel safety limit
- pendulum angle safety limit
- anti-windup logic for the integral state

The nonlinear dynamics are integrated using a fourth-order Runge-Kutta method with a 0.005 s simulation step.

## V. Automatic LQR Weight Tuning

The practical script includes automatic tuning for the LQR state weights. The optimizer searches over positive state weights in log space and evaluates each candidate controller using multiple nonlinear simulation cases. These cases vary initial angle, cart offset, cart velocity, reference scale, disturbance scale, and noise seed.

Each candidate is scored using a fixed performance objective that penalizes pendulum angle, angular velocity, cart tracking error, cart velocity, actuator effort, final tracking error, saturation, settling time, and safety-limit violations. MATLAB `patternsearch` is used when available; otherwise the script uses a local coordinate pattern-search fallback.

The optimized state weights from the saved run were:

| State | Optimized Weight |
| --- | ---: |
| theta | 0.01 |
| theta_dot | 597.1608 |
| x_c | 260 |
| x_c_dot | 474.3416 |

## VI. Results

The final saved simulation log contains 6001 samples over 30 seconds. The controller stabilizes the pendulum from an initial 5 degree angle error, tracks the smooth cart-position reference, and handles the disturbance pulses without triggering a safety stop.

| Metric | Value |
| --- | ---: |
| Maximum pendulum angle | 8.06 deg |
| Maximum actual actuator force | 19.98 N |
| Tracking RMSE | 0.356 m |
| Final tracking error | 0.0082 m |
| RMS theta estimation error | 0.309 deg |
| RMS cart-position estimation error | 0.00744 m |
| Force saturation percentage | 1.60% |

The final cart-position error is small, showing that the integral action successfully removes most steady-state offset by the end of the simulation. The actuator reaches its practical force limit during transient periods, which confirms that actuator constraints are active and important in the design.

## VII. Conclusion

This project demonstrates a complete observer-based control workflow for an unstable nonlinear system. The controller is designed with linear LQR/LQE methods but validated on a more realistic nonlinear plant with noise, friction, disturbances, and actuator constraints. The final result is suitable as a portfolio project because it connects control theory with implementation details that appear in practical systems.

Future improvements could include comparing LQR against PID or MPC, adding hardware-in-the-loop testing, performing robustness analysis, and creating a Simulink version of the plant and controller.

## References

[1] K. J. Astrom and R. M. Murray, Feedback Systems: An Introduction for Scientists and Engineers, Princeton University Press, 2008.

[2] K. Ogata, Modern Control Engineering, 5th ed., Prentice Hall, 2010.

[3] B. D. O. Anderson and J. B. Moore, Optimal Control: Linear Quadratic Methods, Dover Publications, 2007.

