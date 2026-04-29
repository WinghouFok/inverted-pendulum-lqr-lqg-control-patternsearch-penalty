# Observer-Based LQR/LQG Control of an Inverted Pendulum

This project designs, simulates, and evaluates a practical controller for an inverted pendulum mounted on a moving cart. The goal is to keep the pendulum balanced upright while the cart tracks a changing position reference, even with sensor noise, actuator limits, friction, and external disturbance forces.

## Project Overview

The system is modeled as a cart-pendulum plant with the state vector:

```text
x = [theta, theta_dot, x_c, x_c_dot]
```

where `theta` is the pendulum angle from the upright position and `x_c` is the cart position. The controller is designed using a linearized state-space model, then validated using a nonlinear simulation.

The final implementation is in:

```text
src/cart_pendulum_H2_project_AutomaticPenalty.m
```

## What the Code Does

- Builds the linearized cart-pendulum state-space model.
- Checks controllability and observability.
- Designs a Kalman/LQE observer to estimate unmeasured states.
- Adds integral action for cart-position reference tracking.
- Designs an LQR servo controller.
- Automatically tunes the LQR state weights using a multi-case nonlinear objective.
- Simulates the nonlinear plant using RK4 integration.
- Includes practical effects such as:
  - cart friction
  - actuator saturation
  - actuator slew-rate limits
  - actuator first-order lag
  - measurement noise
  - external disturbance pushes
  - track and angle safety limits
- Logs simulation data to CSV.
- Generates dashboard plots.
- Exports an animation video of the cart-pendulum motion.

## Control Approach

The controller uses an observer-based servo structure:

```text
u = -Kx*x_hat - Ki*xi
```

where:

- `x_hat` is the state estimate from the LQE/Kalman observer.
- `Kx` is the LQR state-feedback gain.
- `Ki` is the integral gain.
- `xi` is the integral of cart-position tracking error.

The LQR weights are tuned automatically by simulating multiple nonlinear cases with different initial conditions, disturbance levels, reference amplitudes, and sensor noise. Candidate weights are scored using a penalty function based on tracking error, pendulum angle, velocity, actuator effort, saturation, settling behavior, and safety violations.

## Key Results

The saved run in `results/cart_pendulum_practical_log.csv` produced the following results:

| Metric | Value |
| --- | ---: |
| Maximum pendulum angle | 8.06 deg |
| Maximum actual actuator force | 19.98 N |
| Tracking RMSE | 0.356 m |
| Final cart-position tracking error | 0.0082 m |
| RMS pendulum-angle estimation error | 0.309 deg |
| RMS cart-position estimation error | 0.00744 m |
| Force saturation percentage | 1.60% |

The optimized LQR state weights are saved in:

```text
results/optimized_servo_state_weights.csv
```

## Main Files

| File | Description |
| --- | --- |
| `src/cart_pendulum_H2_project_AutomaticPenalty.m` | Main practical simulation with automatic LQR weight tuning |
| `src/cart_pendulum_H2_project.m` | Earlier project version with observer-based tracking and animation |
| `results/cart_pendulum_practical_log.csv` | Full simulation log from the practical controller run |
| `results/optimized_servo_state_weights.csv` | Optimized LQR state weights |
| `results/optimized_servo_state_weights.mat` | MATLAB binary version of optimized weights |
| `results/cart_pendulum_animation.mp4` | Animation video of the simulated cart-pendulum system |
| `docs/ieee_report.pdf` | IEEE-style project report |
| `docs/ieee_report.md` | Editable report content |
| `docs/generate_ieee_report.py` | Script used to generate the PDF report |

## How to Run

Open MATLAB in the project root folder and run:

```matlab
run("src/cart_pendulum_H2_project_AutomaticPenalty.m")
```

The script will design the observer and controller, tune the state weights if enabled, run the nonlinear simulation, save the log files, generate plots, and export the animation video. Generated files are saved relative to the current MATLAB working folder.

## Requirements

- MATLAB
- Control System Toolbox
- Global Optimization Toolbox is optional

If `patternsearch` is not available, the script uses a local pattern-search fallback included in the file.

## Portfolio Summary

This project demonstrates state-space control design, observer-based feedback, nonlinear validation, practical actuator modeling, disturbance rejection, automatic controller tuning, and simulation-based performance evaluation. It is a compact example of moving from control theory to a more realistic implementation workflow.
