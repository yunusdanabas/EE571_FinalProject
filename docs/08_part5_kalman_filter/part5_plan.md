# Part 5: Discrete-Time Steady-State Kalman Filter (LQE)

## Overview

Implement a discrete-time steady-state Kalman filter for the stochastic 6-mass spring system with process and measurement noise. The filter uses the Part 2 measurement configuration (measuring x1 and x6) and operates in open-loop with zero inputs.

## Objectives

1. Design steady-state Kalman filter using DARE (Discrete-time Algebraic Riccati Equation)
2. Simulate stochastic system with actuator noise (w) and sensor noise (v)
3. Validate estimator stability and compute RMS metrics
4. Generate plots and save artifacts for reproducibility

## Requirements

- Use Part 2 measurement matrix C_part2 (2x12 measuring x1 and x6)
- Use Part 2 initial conditions x0 and xhat0
- Stochastic model: x_{k+1} = Ad x_k + Bd u_k + Bd w_k, y_k = C_d x_k + v_k
- Noise: v ~ N(0, 0.1 I_p), w ~ N(0, 0.05 I_m)
- Random seed: 42 for reproducibility
- Frozen invariants: Ts = 0.01, N = 1000

## Implementation Steps

1. System setup: Load Ad, Bd, Cmeas, x0, xhat0
2. Define noise covariances: Qw = 0.05 * I_3, Rv = 0.1 * I_2, Qx = Bd @ Qw @ Bd.T
3. Design Kalman filter: Solve DARE, compute steady-state gain Lk
4. Run stochastic simulation with noise
5. Compute metrics: RMS errors, estimator stability
6. Generate plots and save artifacts
