# Part 5: Kalman Filter Design Report

## Objective

Design a steady-state Kalman filter for the stochastic 6-mass spring system with process and measurement noise. The filter estimates all system states from noisy measurements of x1 and x6, in the presence of actuator noise (process noise) and sensor noise (measurement noise).

## Approach

1. **Define noise covariances:**
   - Actuator noise: Qw = 0.05 × I₃ (3×3 identity)
   - Sensor noise: Rv = 0.1 × I₂ (2×2 identity)
   - Process noise: Qx = Bd @ Qw @ Bd.T (12×12, transformed to state space)

2. **Design Kalman filter:**
   - Solve discrete-time algebraic Riccati equation (DARE) for estimator: P = solve_discrete_are(Ad.T, Cmeas.T, Qx, Rv)
   - Compute steady-state Kalman gain: Lk = P @ Cmeas.T @ inv(Cmeas @ P @ Cmeas.T + Rv)
   - Verify estimator stability (spectral radius < 1.0)

3. **Simulate stochastic system:**
   - True system: x[k+1] = Ad @ x[k] + Bd @ w[k] (zero input, process noise only)
   - Measured output: y_meas[k] = Cmeas @ x[k] + v[k] (true output plus sensor noise)
   - Kalman filter: xhat[k+1] = Ad @ xhat[k] + Lk @ (y_meas[k] - yhat[k])
   - Simulation horizon: N = 1000 steps (10 seconds at Ts = 0.01s)

4. **Compute estimation metrics:**
   - RMS estimation errors (overall, per-state, steady-state)
   - Output tracking errors (true vs estimated)

## Key Results

- **Kalman gain Lk shape:** (12, 2)
- **Estimator spectral radius:** 0.999547 (estimator is stable)
- **Overall RMS estimation error (full window):** 9.607036×10⁻¹
- **Overall RMS estimation error (steady-state):** 5.312798×10⁻¹
- **RMS error e_x1 (full):** 1.816744×10⁻¹
- **RMS error e_x1 (steady-state):** 1.075163×10⁻¹
- **RMS error e_x6 (full):** 2.044338×10⁻¹
- **RMS error e_x6 (steady-state):** 5.154161×10⁻²
- **RMS output tracking y1 (full):** 1.816744×10⁻¹
- **RMS output tracking y1 (steady-state):** 1.075163×10⁻¹
- **RMS output tracking y6 (full):** 2.044338×10⁻¹
- **RMS output tracking y6 (steady-state):** 5.154161×10⁻²

### Verification: Comparison with Original Implementation

The simplified implementation has been verified against the original 780-line implementation (`python/part5/run_kalman_filter.py`):

- **Kalman gain Lk:** Matches exactly (max difference: 4.70×10⁻¹⁴, numerical precision)
- **Spectral radius:** 0.999547 (identical)
- **All RMS metrics:** Match exactly (all values identical within numerical precision)
- **Trajectories:** All trajectories (x, xhat, y_true, y_meas, yhat, innovations) match within numerical precision:
  - x: max difference 3.16×10⁻¹⁵
  - xhat: max difference 4.48×10⁻¹²
  - y_true: max difference 2.54×10⁻¹⁵
  - y_meas: max difference 2.55×10⁻¹⁵
  - yhat: max difference 3.06×10⁻¹²
  - innovations: max difference 3.06×10⁻¹²

The simplified code (213 lines) produces **numerically identical results** to the original implementation (780 lines), confirming that the simplifications preserved mathematical correctness while reducing code complexity by 73%.

### Results Summary

```
Part 5: Kalman Filter Design - Results
============================================================

Kalman Filter Design:
  Lk shape: (12, 2)
  Estimator spectral radius: 0.999547

RMS Metrics:
  Overall RMS estimation error (full): 9.607036e-01
  Overall RMS estimation error (steady-state): 5.312798e-01
  RMS error e_x1 (full): 1.816744e-01
  RMS error e_x1 (steady-state): 1.075163e-01
  RMS error e_x6 (full): 2.044338e-01
  RMS error e_x6 (steady-state): 5.154161e-02
  RMS output tracking y1 (full): 1.816744e-01
  RMS output tracking y1 (steady-state): 1.075163e-01
  RMS output tracking y6 (full): 2.044338e-01
  RMS output tracking y6 (steady-state): 5.154161e-02
```

## Figures

1. **True vs Estimated Outputs** (`outputs_y_vs_yhat.png`): Shows the true outputs (y_true, no noise), estimated outputs (yhat), and noisy measurements (y_meas) for both y1 (x1) and y6 (x6). The Kalman filter successfully tracks the true outputs despite measurement noise, with yhat closely following y_true.

2. **Estimation Error Norm** (`estimation_error_norm.png`): Shows the evolution of the estimation error norm ||x - xhat|| over time. The error decreases and stabilizes, demonstrating that the Kalman filter converges and maintains accurate state estimates in the presence of noise.

3. **Estimation Errors for x1 and x6** (`estimation_error_x1_x6.png`): Displays the estimation errors for the measured states x1 and x6. Both errors converge and remain bounded, showing the filter's effectiveness in estimating these directly measured states.

4. **Per-State RMS Bar Chart** (`per_state_rms_bar.png`): Shows the RMS estimation error for each of the 12 states. The errors are distributed across all states, with the directly measured states (x1, x6) having relatively lower errors than some unmeasured states.

## Detailed Explanation of Part 5

Part 5 implements a **steady-state Kalman filter** for state estimation in a stochastic system with both process noise (actuator noise) and measurement noise (sensor noise). This is a fundamental step toward optimal control under uncertainty.

### Execution Notes and Warnings

**No warnings were generated during execution.** The code ran successfully without any scipy.linalg warnings, numpy warnings, or other runtime warnings. All numerical operations completed successfully:
- DARE solver converged without warnings
- Matrix inversions were numerically stable
- Random number generation (with seed=42) produced valid results
- All array operations completed without overflow or underflow issues

The spectral radius of 0.999547 is close to 1.0 but still within the stable region (< 1.0), indicating a well-tuned but near-marginally-stable estimator. This is expected behavior for an optimal Kalman filter and does not indicate any numerical issues.

### What is a Kalman Filter?

A Kalman filter is an optimal state estimator for linear systems with Gaussian noise. It combines:
- **Process model**: How the system evolves (with process noise)
- **Measurement model**: What we observe (with measurement noise)
- **Optimal gain**: Computed to minimize estimation error covariance

The Kalman filter provides the best linear unbiased estimate (BLUE) of the system state given noisy measurements. Unlike the Part 2 observer (which assumes perfect measurements), the Kalman filter explicitly accounts for uncertainty in both the system dynamics (process noise) and measurements (sensor noise).

### Understanding the Results

**1. Kalman Gain Lk (12×2):**
- The gain matrix maps 2 measurements (x1 and x6) to corrections for all 12 states
- Each column corresponds to one measurement (column 1 for x1, column 2 for x6)
- The gain values determine how much the filter trusts measurements vs. the model prediction

**2. Spectral Radius (0.999547):**
- Close to 1.0 but strictly < 1.0, ensuring stability
- Indicates the filter converges slowly but reliably
- This near-unity value is typical for optimal estimators balancing noise rejection with responsiveness

**3. RMS Estimation Errors:**
- **Overall RMS (full):** 0.9607 - average error across all states over the full simulation
- **Overall RMS (steady-state):** 0.5313 - error in the last 20% of simulation, showing convergence
- **e_x1 and e_x6:** Errors for measured states are smaller (0.18-0.20) than unmeasured states
- The steady-state values are lower, confirming the filter converges and improves over time

**4. Output Tracking:**
- RMS tracking errors match state errors for x1 and x6 (since these are directly measured)
- The filter successfully tracks the true outputs despite measurement noise
- Steady-state tracking improves significantly (y6 error drops from 0.20 to 0.05)

### Key Differences from Part 2 Observer

1. **Part 2 Observer**: Designed via pole placement for a deterministic system (no noise)
2. **Part 5 Kalman Filter**: Designed via DARE for a stochastic system (with noise)

The Kalman filter is optimal for noisy systems, while the Part 2 observer is designed for deterministic systems.

### How the Kalman Filter Works

1. **Stochastic System Model:**
   - True system: x[k+1] = Ad @ x[k] + Bd @ w[k] (process noise w ~ N(0, Qw))
   - Measurement: y_meas[k] = Cmeas @ x[k] + v[k] (sensor noise v ~ N(0, Rv))

2. **Filter Design (DARE):**
   - Solve the discrete-time algebraic Riccati equation for the estimator
   - This finds the steady-state error covariance P
   - Compute Kalman gain: Lk = P @ Cmeas.T @ inv(Cmeas @ P @ Cmeas.T + Rv)

3. **Filter Update:**
   - Prediction: xhat[k+1|k] = Ad @ xhat[k] + Bd @ u[k]
   - Correction: xhat[k+1] = xhat[k+1|k] + Lk @ (y_meas[k] - Cmeas @ xhat[k])
   - For zero input: xhat[k+1] = Ad @ xhat[k] + Lk @ (y_meas[k] - yhat[k])

4. **Noise Handling:**
   - Process noise Qw: Affects system evolution (actuator uncertainty)
   - Measurement noise Rv: Affects sensor readings (sensor uncertainty)
   - The filter balances trust between the model and measurements based on noise levels

### Why This Matters

In real systems, noise is inevitable:
- **Actuator noise**: Imperfect control inputs, disturbances
- **Sensor noise**: Measurement uncertainty, quantization errors

The Kalman filter provides optimal state estimates by:
- Using the process model to predict state evolution
- Using measurements to correct predictions
- Balancing trust between model and measurements based on noise covariances

The steady-state design ensures the filter gain converges to a constant value, simplifying implementation while maintaining optimal performance.

### Key Insights from Results

1. **Noise Impact:** Despite process noise (Qw = 0.05) and measurement noise (Rv = 0.1), the filter maintains reasonable estimation accuracy (RMS ~0.96 overall, ~0.53 in steady-state).

2. **Convergence:** Steady-state errors are 45% lower than full-window errors, showing the filter successfully converges over the 10-second simulation.

3. **Measured vs Unmeasured States:** Directly measured states (x1, x6) have lower errors than unmeasured states, as expected. However, the filter successfully estimates all 12 states from just 2 measurements.

4. **Filter Performance:** The near-unity spectral radius (0.999547) indicates the filter is optimally tuned - aggressive enough to track the system but not so aggressive as to amplify noise.

### Mathematical Foundation

The Kalman filter design solves the **discrete-time algebraic Riccati equation (DARE)** for the estimator:

```
P = solve_discrete_are(Ad.T, Cmeas.T, Qx, Rv)
```

This yields the steady-state error covariance P, which is then used to compute the optimal gain:

```
Lk = P @ Cmeas.T @ inv(Cmeas @ P @ Cmeas.T + Rv)
```

The filter update combines prediction (using the model) and correction (using measurements):

```
xhat[k+1] = Ad @ xhat[k] + Lk @ (y_meas[k] - Cmeas @ xhat[k])
```

This is optimal in the sense that it minimizes the expected squared estimation error, given the noise statistics Qw and Rv.
