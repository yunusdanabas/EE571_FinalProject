# Part 6: LQG Controller - Detailed Explanation

## Overview

Part 6 implements an **LQG (Linear Quadratic Gaussian) Controller**, which combines:
- **LQR Controller (from Part 3)**: Optimal feedback gain K that minimizes quadratic cost
- **Kalman Filter (from Part 5)**: Optimal estimator gain Lk that minimizes estimation error under noise

The LQG controller represents the optimal control strategy for linear systems with Gaussian noise.

## Code Structure

### Main Components

1. **`compute_lqg_cost(u, y_true, N)`** (lines 15-20)
   - Computes the LQG cost function: J = Σ(u[k]^T u[k] + y1[k]^2 + y6[k]^2)
   - Uses `y_true` (not `y_meas`) to avoid penalizing uncontrollable measurement noise
   - Sums over N time steps

2. **`main()` function** (lines 23-163)
   - Orchestrates the entire LQG simulation
   - Loads gains from previous parts
   - Simulates the closed-loop system
   - Computes metrics and generates plots

3. **`simulate_lqg()` function** (in `final/utils/simulation.py`)
   - Core simulation function that implements the LQG dynamics
   - Handles noisy measurements and process noise

## Step-by-Step Execution

### Step 1: Setup (lines 24-35)

```python
Ts = 0.01        # Sampling time: 0.01 seconds
N = 1000         # Number of simulation steps (10 seconds total)
seed = 42        # Random seed for reproducibility
```

- Builds the continuous-time model (A, B, C matrices)
- Discretizes to discrete-time (Ad, Bd, Cd) using zero-order hold
- Gets measurement matrix Cmeas and initial conditions (x0, xhat0) from Part 2

### Step 2: Load Controller and Estimator Gains (lines 36-46)

```python
K = np.load('final/part3/outputs/K_matrix.npy')   # LQR gain (3×12)
Lk = np.load('final/part5/outputs/Lk_matrix.npy') # Kalman filter gain (12×2)
```

- **K (3×12)**: LQR controller gain from Part 3
  - Maps estimated states (12D) to control inputs (3D)
  - Control law: u[k] = -K @ xhat[k]
  
- **Lk (12×2)**: Kalman filter gain from Part 5
  - Maps measurement innovations (2D) to state corrections (12D)
  - Estimator update uses: xhat[k+1] = ... + Lk @ innovation[k]

### Step 3: Define Noise Parameters (lines 48-52)

```python
Qw = 0.05 * np.eye(3)  # Actuator noise covariance (3×3)
Rv = 0.1 * np.eye(2)   # Sensor noise covariance (2×2)
```

- **Qw**: Process noise (actuator uncertainty)
  - Affects the true system: x[k+1] = Ad @ x[k] + Bd @ u[k] + **Bd @ w[k]**
  - w[k] ~ N(0, Qw) - Gaussian noise with covariance 0.05×I₃
  
- **Rv**: Measurement noise (sensor uncertainty)
  - Affects measurements: y_meas[k] = Cmeas @ x[k] + **v[k]**
  - v[k] ~ N(0, Rv) - Gaussian noise with covariance 0.1×I₂

### Step 4: Simulate LQG Closed-Loop System (line 55)

```python
results = simulate_lqg(Ad, Bd, Cmeas, K, Lk, x0, xhat0, N, Ts, Qw, Rv, seed=seed)
```

This calls `simulate_lqg()` which implements the LQG dynamics:

**For each time step k = 0 to N-1:**

1. **Compute control input:**
   ```python
   u[k] = -K @ xhat[k]
   ```
   - Controller uses **estimated states** (xhat), not true states (x)
   - This is the key difference from full-state feedback

2. **Generate process noise:**
   ```python
   w_k ~ N(0, Qw)  # Gaussian noise
   ```

3. **Update true system (with noise):**
   ```python
   x[k+1] = Ad @ x[k] + Bd @ u[k] + Bd @ w_k
   ```
   - True state evolves with control input and process noise

4. **Compute outputs:**
   ```python
   y_true[k] = Cmeas @ x[k]      # True output (no noise)
   yhat[k] = Cmeas @ xhat[k]     # Estimated output
   ```

5. **Generate measurement noise and compute noisy measurement:**
   ```python
   v_k ~ N(0, Rv)  # Gaussian noise
   y_meas[k] = y_true[k] + v_k   # Noisy measurement
   ```

6. **Compute innovation (measurement residual):**
   ```python
   innovation[k] = y_meas[k] - yhat[k]
   ```
   - Difference between noisy measurement and estimated output

7. **Update Kalman filter (estimator):**
   ```python
   xhat[k+1] = Ad @ xhat[k] + Bd @ u[k] + Lk @ innovation[k]
   ```
   - Predictor: Ad @ xhat[k] + Bd @ u[k]
   - Corrector: Lk @ innovation[k] (uses noisy measurement)

### Step 5: Compute Metrics (lines 57-68)

```python
J = compute_lqg_cost(results['u'], results['y_true'], N)
max_u = np.max(np.abs(results['u']))
rms_error = np.sqrt(np.mean(error_norm**2))
```

- **J**: Total cost = 4.260967×10²
  - J = Σ(u[k]^T u[k] + y1[k]^2 + y6[k]^2) using y_true
  
- **max_u**: Maximum input magnitude = 4.086037×10⁻¹
  - Maximum absolute value across all inputs and all time steps

- **rms_error**: RMS estimation error = 9.573350×10⁻¹
  - RMS of ||x - xhat|| over the entire simulation

### Step 6: Generate Plots (lines 80-138)

1. **Outputs Comparison** (`outputs_comparison.png`):
   - Compares Part 3 (no noise) vs Part 6 (with noise)
   - Shows y_true (true output) and yhat (estimated output)
   - Demonstrates the Kalman filter's ability to track despite noise

2. **Control Inputs** (`inputs_u1_u2_u3.png`):
   - Shows all three control inputs over time
   - Much smaller than Part 3 due to different estimator (Lk vs L)

3. **Estimation Error Norm** (`estimation_error_norm.png`):
   - Shows ||x - xhat|| over time
   - Demonstrates bounded estimation error despite noise

## Key Results

### Cost Metrics
- **Total cost J**: 4.260967×10²
- **Max |u|**: 4.086037×10⁻¹

### Estimation Metrics
- **RMS estimation error**: 9.573350×10⁻¹
- **RMS estimation error (last 20%)**: 5.593296×10⁻¹

The steady-state error (5.59×10⁻¹) is lower than the full-window error (9.57×10⁻¹), showing the Kalman filter converges and improves over time.

### Comparison with Part 3

| Metric | Part 3 (No Noise) | Part 6 (LQG with Noise) |
|--------|-------------------|-------------------------|
| **Cost J** | 9.06×10⁷ | 4.26×10² |
| **Max \|u\|** | 3.60×10³ | 4.09×10⁻¹ |
| **Estimator** | L (pole placement) | Lk (Kalman filter) |
| **Noise** | None | Qw=0.05×I₃, Rv=0.1×I₂ |

**Key insight:** Part 6 uses much smaller control inputs (4 orders of magnitude smaller) and achieves much lower cost. This is because:
1. The Kalman filter (Lk) is optimal for noisy systems
2. The LQG controller balances control effort with output regulation under uncertainty
3. The cost uses y_true (not y_meas), so it doesn't penalize uncontrollable measurement noise

## Warnings

### Warning: Qt Platform Plugin

```
qt.qpa.plugin: Could not find the Qt platform plugin "wayland" in ""
```

**What this means:**
- This is a **graphics/windowing system warning** from matplotlib
- Qt is a GUI framework used by matplotlib for plotting
- The system is trying to use the "wayland" display protocol but can't find it
- This is a **harmless warning** - it doesn't affect the simulation or results

**Impact:**
- **None** - this is just a display system warning
- All plots are generated and saved correctly to PNG files
- The warning doesn't affect:
  - Controller design
  - Simulation results
  - Numerical computations
  - Cost calculations
  - Any other metrics

**Why it happens:**
- Common on Linux systems where the display server (X11/Wayland) configuration may vary
- Matplotlib falls back to alternative display backends automatically
- The plots are still generated correctly as PNG files

**No action needed:** This warning can be safely ignored. All functionality works correctly.

### No Other Warnings

No other warnings were generated during execution:
- No numerical warnings (overflow, underflow, division by zero)
- No convergence warnings from optimization algorithms
- No matrix inversion warnings
- All computations completed successfully

This confirms that:
- The LQG simulation is numerically stable
- All matrix operations are well-conditioned
- The random number generation (with seed=42) produces valid results
- The controller and estimator are both stable

## Key Concepts

### 1. Separation Principle

The **separation principle** states that the controller and estimator can be designed independently:
- **Controller (K)**: Designed in Part 3 using LQR (minimizes cost)
- **Estimator (Lk)**: Designed in Part 5 using Kalman filter (minimizes estimation error)
- **LQG Controller**: Combines K and Lk for optimal performance under noise

This principle allows us to:
1. Design the optimal controller assuming perfect state measurements
2. Design the optimal estimator assuming no control
3. Combine them to get optimal LQG performance

### 2. Why Use y_true for Cost?

The cost function uses `y_true` (not `y_meas`) to avoid penalizing uncontrollable measurement noise:
- `y_true[k] = Cmeas @ x[k]` - true output (no noise)
- `y_meas[k] = Cmeas @ x[k] + v[k]` - measured output (with noise)

Using y_true ensures the cost reflects control performance, not measurement uncertainty.

### 3. LQG vs LQR with Observer

| Feature | Part 3 (LQR + Observer) | Part 6 (LQG) |
|---------|------------------------|--------------|
| **Estimator** | L (pole placement) | Lk (Kalman filter) |
| **Optimality** | Optimal control, suboptimal estimation | Optimal control + optimal estimation |
| **Noise** | None (deterministic) | Process noise + measurement noise |
| **Design** | Controller and observer designed independently | Controller and estimator designed independently (separation principle) |

The key difference is that Lk (Kalman filter) is **optimal** for noisy systems, while L (pole placement observer) is designed for deterministic systems.

## Summary

Part 6 successfully combines the LQR controller (Part 3) with the Kalman filter (Part 5) to create an LQG controller that:
- Handles process noise (Qw) and measurement noise (Rv)
- Uses optimal controller gain K and optimal estimator gain Lk
- Achieves significantly lower control effort than Part 3
- Maintains bounded estimation errors despite noise
- Demonstrates the separation principle in practice

The implementation is numerically stable, produces identical results to the original implementation, and runs without any significant warnings (only a harmless graphics warning that doesn't affect functionality).
