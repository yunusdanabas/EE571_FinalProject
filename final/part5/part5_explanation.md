# Part 5: Kalman Filter Design - Detailed Explanation

## Overview

Part 5 designs a **steady-state Kalman filter** for state estimation in a stochastic system with both process noise (actuator noise) and measurement noise (sensor noise). This is a fundamental step toward optimal control under uncertainty.

## What is a Kalman Filter?

A **Kalman filter** is an optimal state estimator for linear systems with Gaussian noise. Unlike the Part 2 observer (which assumes perfect measurements), the Kalman filter explicitly accounts for uncertainty in both:
- **System dynamics** (process noise)
- **Measurements** (sensor noise)

**Key properties:**
- **Optimal**: Minimizes estimation error covariance
- **Best Linear Unbiased Estimate (BLUE)**: No other linear estimator can do better
- **Steady-state**: Gain converges to a constant value

## Code Structure

### Main Components

1. **`design_kalman_filter()`**
   - Solves DARE for the estimator
   - Computes steady-state Kalman gain: Lk
   - Verifies estimator stability

2. **`simulate_kalman_noisy()`** (from `final/utils/simulation.py`)
   - Simulates stochastic system with process and measurement noise
   - Implements Kalman filter update equations

3. **`compute_rms_metrics()`**
   - Computes RMS estimation errors (overall, per-state, steady-state)
   - Evaluates filter performance

## Step-by-Step Execution

### Step 1: Define Noise Covariances

```python
Qw = 0.05 * np.eye(3)  # Actuator noise (process noise)
Rv = 0.1 * np.eye(2)   # Sensor noise (measurement noise)
Qx = Bd @ Qw @ Bd.T    # Process noise in state space
```

**Noise models:**
- **Qw (3×3)**: Actuator noise covariance
  - Models uncertainty in control inputs
  - w[k] ~ N(0, Qw) - Gaussian noise
  - Affects system: x[k+1] = Ad @ x[k] + Bd @ (u[k] + w[k])

- **Rv (2×2)**: Sensor noise covariance
  - Models uncertainty in measurements
  - v[k] ~ N(0, Rv) - Gaussian noise
  - Affects measurements: y_meas[k] = Cmeas @ x[k] + v[k]

- **Qx (12×12)**: Process noise in state space
  - Transformed from actuator space to state space
  - Qx = Bd @ Qw @ Bd^T
  - Represents how actuator noise affects each state

### Step 2: Design Kalman Filter

```python
Lk, P, rho = design_kalman_filter(Ad, Cmeas, Qx, Rv)
```

**DARE for estimator:**
```
P = solve_discrete_are(Ad.T, Cmeas.T, Qx, Rv)
```

This solves the **dual** of the controller DARE:
- Controller DARE: P = Q + Ad^T P Ad - Ad^T P Bd (R + Bd^T P Bd)^(-1) Bd^T P Ad
- Estimator DARE: P = Qx + Ad P Ad^T - Ad P Cmeas^T (Rv + Cmeas P Cmeas^T)^(-1) Cmeas P Ad^T

**Kalman gain computation:**
```
S = Cmeas @ P @ Cmeas.T + Rv  # Innovation covariance
Lk = P @ Cmeas.T @ inv(S)
```

**What does Lk do?**
- Lk is a (12×2) matrix
- Maps 2 measurements to corrections for all 12 states
- Balances trust between model prediction and measurements

**Estimator stability:**
```
Aest = Ad - Lk @ Cmeas
spectral_radius = max(|eigenvalues(Aest)|)
```

- Spectral radius < 1.0 → estimator is stable
- Result: 0.999547 (stable, but close to unity)

### Step 3: Simulate Stochastic System

```python
results = simulate_kalman_noisy(Ad, Bd, Cmeas, Lk, x0, xhat0, N, Ts, Qw, Rv, seed=42)
```

**System dynamics (zero input, noise only):**
1. **True system**: x[k+1] = Ad @ x[k] + Bd @ w[k]
   - Process noise w[k] ~ N(0, Qw) affects evolution

2. **Measurements**: y_meas[k] = Cmeas @ x[k] + v[k]
   - Sensor noise v[k] ~ N(0, Rv) corrupts measurements

3. **Kalman filter**: xhat[k+1] = Ad @ xhat[k] + Lk @ (y_meas[k] - yhat[k])
   - Predicts: xhat[k+1|k] = Ad @ xhat[k]
   - Corrects: xhat[k+1] = xhat[k+1|k] + Lk @ innovation[k]
   - Innovation: y_meas[k] - yhat[k] = y_meas[k] - Cmeas @ xhat[k]

**Key insight**: The filter combines:
- **Model prediction**: Uses system dynamics to predict state
- **Measurement correction**: Uses noisy measurements to correct prediction
- **Optimal balance**: Lk balances trust based on noise levels

### Step 4: Compute Metrics

```python
metrics = compute_rms_metrics(x, xhat, y_true, y_meas, yhat)
```

**RMS estimation errors:**
- **Overall RMS (full)**: 0.9607 - average error over entire simulation
- **Overall RMS (steady-state)**: 0.5313 - error in last 20% (shows convergence)
- **Per-state RMS**: Errors for each of the 12 states
- **Output tracking**: How well yhat tracks y_true

## Understanding the Results

### Kalman Gain Lk (12×2)

**Structure:**
- Each column corresponds to one measurement (column 1 for x1, column 2 for x6)
- Each row corresponds to one state
- Lk[i, j] tells us how much measurement j affects the estimate of state i

**Interpretation:**
- Large values → filter trusts measurements more
- Small values → filter trusts model prediction more
- Optimal balance minimizes estimation error

### Spectral Radius (0.999547)

**Near-unity value:**
- Close to 1.0 but strictly < 1.0 (stable)
- Indicates slow but reliable convergence
- Typical for optimal estimators balancing noise rejection with responsiveness

**Why near unity?**
- System is undamped (eigenvalues near unit circle)
- Filter must be careful not to amplify noise
- Optimal tuning balances tracking vs noise rejection

### RMS Estimation Errors

**Overall RMS (full): 0.9607**
- Average error across all states over 10 seconds
- Includes initial transient (larger errors)

**Overall RMS (steady-state): 0.5313**
- Error in last 20% of simulation
- 45% lower than full-window error
- Shows filter converges and improves over time

**Per-state errors:**
- Measured states (x1, x6): Lower errors (~0.1-0.2)
- Unmeasured states: Higher errors (up to ~1.0)
- Filter successfully estimates all 12 states from 2 measurements

### Output Tracking

**RMS tracking errors:**
- y1 (x1): 0.1817 (full), 0.1075 (steady-state)
- y6 (x6): 0.2044 (full), 0.0515 (steady-state)

**Key observation:**
- Steady-state tracking is much better than full-window
- y6 tracking improves significantly (0.20 → 0.05)
- Filter successfully tracks true outputs despite noise

## Key Differences from Part 2 Observer

| Feature | Part 2 Observer | Part 5 Kalman Filter |
|---------|----------------|---------------------|
| **Design method** | Pole placement | DARE (optimal) |
| **Noise** | None (deterministic) | Process + measurement noise |
| **Optimality** | Suboptimal | Optimal (minimizes error covariance) |
| **Gain computation** | L (pole placement) | Lk (DARE solution) |
| **Use case** | Perfect measurements | Noisy measurements |

**Why Kalman filter is better for noisy systems:**
- Explicitly accounts for noise statistics (Qw, Rv)
- Optimally balances model vs measurements
- Minimizes estimation error covariance
- Adapts to noise levels automatically

## How the Kalman Filter Works

### Prediction Step

```
xhat[k+1|k] = Ad @ xhat[k] + Bd @ u[k]
```

- Predicts next state using system model
- For zero input: xhat[k+1|k] = Ad @ xhat[k]

### Correction Step

```
innovation[k] = y_meas[k] - Cmeas @ xhat[k]
xhat[k+1] = xhat[k+1|k] + Lk @ innovation[k]
```

- Computes innovation (measurement residual)
- Corrects prediction using measurement
- Lk determines how much to trust the measurement

### Optimal Gain

The Kalman gain Lk is computed to minimize estimation error covariance:
- **High noise in measurements** (large Rv) → Lk is small (trust model more)
- **Low noise in measurements** (small Rv) → Lk is large (trust measurements more)
- **High process noise** (large Qx) → Lk is large (trust measurements more)

## Why This Matters

In real systems, noise is inevitable:
- **Actuator noise**: Imperfect control inputs, disturbances, modeling errors
- **Sensor noise**: Measurement uncertainty, quantization, environmental interference

The Kalman filter provides:
- **Optimal estimation**: Best possible state estimates given noise
- **Robustness**: Handles uncertainty gracefully
- **Foundation for LQG**: Enables optimal control under uncertainty (Part 6)

## Warnings

**No warnings were generated during execution.**
- DARE solver converged successfully
- All matrix operations were numerically stable
- Random number generation (seed=42) produced valid results

## What Happens Next?

Part 5 provides the optimal estimator for noisy systems. In Part 6:
- Combine LQR controller (Part 3) with Kalman filter (Part 5)
- Create LQG (Linear Quadratic Gaussian) controller
- Achieve optimal control under uncertainty

## Summary

Part 5 successfully:
1. ✓ Defined noise covariances (Qw, Rv)
2. ✓ Designed Kalman filter using DARE
3. ✓ Computed optimal gain Lk (12×2)
4. ✓ Verified estimator stability (spectral radius = 0.999547)
5. ✓ Simulated stochastic system with noise
6. ✓ Achieved reasonable estimation accuracy (RMS ~0.53 in steady-state)
7. ✓ Demonstrated convergence (steady-state errors 45% lower than full-window)

**Key achievement**: Designed an optimal state estimator that handles both process noise and measurement noise, providing accurate state estimates for control under uncertainty.
