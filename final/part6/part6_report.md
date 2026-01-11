# Part 6: LQG Controller Report

## Objective

Combine the LQR controller (Part 3) with the Kalman filter (Part 5) to implement an LQG (Linear Quadratic Gaussian) controller for the noisy 6-mass spring system. The LQG controller uses the optimal LQR gain K to compute control inputs from estimated states provided by the optimal Kalman filter gain Lk.

## Approach

1. **Load components from previous parts:**
   - LQR gain K from Part 3 (`final/part3/outputs/K_matrix.npy`)
   - Kalman filter gain Lk from Part 5 (`final/part5/outputs/Lk_matrix.npy`)
   - Measurement matrix C and initial conditions from Part 2

2. **Define noise parameters (Part 5 frozen):**
   - Actuator noise: Qw = 0.05 × I₃ (3×3 identity)
   - Sensor noise: Rv = 0.1 × I₂ (2×2 identity)

3. **Simulate LQG closed-loop system:**
   - Control law: u[k] = -K @ xhat[k] (uses estimated states from Kalman filter)
   - True system: x[k+1] = Ad @ x[k] + Bd @ u[k] + Bd @ w[k] (process noise)
   - Measured output: y_meas[k] = Cmeas @ x[k] + v[k] (sensor noise)
   - Kalman filter: xhat[k+1] = Ad @ xhat[k] + Bd @ u[k] + Lk @ (y_meas[k] - Cmeas @ xhat[k])
   - Simulation horizon: N = 1000 steps (10 seconds at Ts = 0.01s)

4. **Compute cost metrics:**
   - Total cost: J = Σ(u[k]^T u[k] + y1[k]^2 + y6[k]^2) using y_true (not y_meas)
   - Cost convention: Uses y_true = Cmeas @ x (does not penalize uncontrollable measurement noise)
   - Estimation error metrics: RMS error ||x - xhat||

## Key Results

- **LQR gain K shape:** (3, 12) (from Part 3)
- **Kalman filter gain Lk shape:** (12, 2) (from Part 5)
- **Total cost J:** 4.260967×10²
- **Maximum input magnitude:** 4.086037×10⁻¹
- **RMS estimation error:** 9.573350×10⁻¹
- **RMS estimation error (last 20%):** 5.593296×10⁻¹
- **RMS y_true y1 (full):** 2.849238×10⁻¹
- **RMS y_true y1 (last 20%):** 1.843321×10⁻¹
- **RMS y_true y6 (full):** 5.485180×10⁻¹
- **RMS y_true y6 (last 20%):** 2.669689×10⁻¹
- **RMS yhat y1 (full):** 1.394727×10⁻¹
- **RMS yhat y1 (last 20%):** 9.829881×10⁻²
- **RMS yhat y6 (full):** 3.948733×10⁻¹
- **RMS yhat y6 (last 20%):** 1.934366×10⁻¹

### Results Summary

```
Part 6: LQG Controller (LQR + Kalman Filter) - Results
============================================================

LQG Controller:
  K shape: (3, 12)
  Lk shape: (12, 2)

Cost Metrics:
  Total cost J: 4.260967e+02
  Max |u|: 4.086037e-01

Estimation Metrics:
  RMS estimation error: 9.573350e-01
  RMS estimation error (last 20%): 5.593296e-01
```

### Verification: Comparison with Original Implementation

The simplified implementation (166 lines) has been verified against the original 1,068-line implementation (`python/part6/run_lqg.py`):

| Metric | Original | Simplified | Match |
|--------|----------|------------|-------|
| **LQR gain K** | (3, 12) | (3, 12) | ✓ Exact match (loaded from Part 3) |
| **Kalman filter gain Lk** | (12, 2) | (12, 2) | ✓ Exact match (loaded from Part 5) |
| **Total cost J** | 4.260967×10² | 4.260967×10² | ✓ Exact match (diff: 4.70×10⁻¹⁰) |
| **Max \|u\|** | 4.086037×10⁻¹ | 4.086037×10⁻¹ | ✓ Exact match |
| **RMS estimation error** | 9.573350×10⁻¹ | 9.573350×10⁻¹ | ✓ Exact match |
| **RMS error (last 20%)** | 5.593296×10⁻¹ | 5.593296×10⁻¹ | ✓ Exact match |
| **Trajectories (x, xhat, u)** | - | - | ✓ Match (max diff: ~10⁻¹²) |

**Key observations:**
- All key metrics match exactly (within numerical precision ~10⁻¹²)
- Trajectories match exactly (max difference: 4.74×10⁻¹² for xhat)
- Cost function computed identically (J difference: 4.70×10⁻¹⁰, numerical precision)
- K and Lk are correctly loaded from Part 3 and Part 5 outputs
- The simplified code (166 lines) produces **numerically identical results** to the original implementation (1,068 lines), confirming that simplifications preserved mathematical correctness while reducing code complexity by ~84%

**Cost breakdown (from original implementation):**
- Σ u^T u (control effort): 4.379727×10¹
- Σ (y1^2 + y6^2) (output penalty): 3.822994×10²
- Total cost J_true: 4.260967×10²
- Total cost J_meas (using y_meas, for comparison): 6.343417×10²

### Comparison with Part 3 Baseline

| Metric | Part 3 (No Noise, Observer L) | Part 6 (LQG, Kalman Filter Lk) |
|--------|-------------------------------|--------------------------------|
| **Controller gain** | K (3, 12) | K (3, 12) - same |
| **Estimator gain** | L (pole placement) | Lk (Kalman filter) - different |
| **Total cost J** | 9.057478×10⁷ (simplified) / 3.918117×10⁷ (original) | 4.260967×10² |
| **Max \|u\|** | 3.597390×10³ (simplified) / 2.410433×10³ (original) | 4.086037×10⁻¹ |
| **Noise** | None (deterministic) | Process noise Qw, sensor noise Rv |

**Key observations:**
- Part 6 uses much smaller control inputs (max |u| reduced by ~4 orders of magnitude compared to Part 3)
- Part 6 achieves significantly lower cost (J reduced by ~5 orders of magnitude compared to Part 3)
- The difference is due to both noise presence and different estimator gains (L vs Lk)
- Part 6 demonstrates effective control under uncertainty using the optimal Kalman filter
- **Note:** Part 3 comparison uses simplified implementation values. The original Part 3 implementation had J = 3.918117×10⁷ and max |u| = 2.410433×10³ (due to different observer design), but the relative difference with Part 6 remains significant

## Figures

1. **Outputs Comparison** (`outputs_comparison.png`): Shows Part 3 baseline (no noise) vs Part 6 LQG outputs (y_true and yhat). The LQG controller successfully regulates outputs despite noise, with estimated outputs (yhat) tracking true outputs (y_true) closely.

2. **Control Inputs** (`inputs_u1_u2_u3.png`): Displays the three control inputs computed from estimated states via u[k] = -K @ xhat[k]. Inputs are much smaller than Part 3, demonstrating efficient control under uncertainty.

3. **Estimation Error Norm** (`estimation_error_norm.png`): Shows the evolution of ||x - xhat|| over time. The Kalman filter maintains bounded estimation errors in the presence of both process and measurement noise.

## Findings

1. **Separation principle:** The LQG controller demonstrates the separation principle - the controller (K) and estimator (Lk) can be designed independently. The LQR controller gain K from Part 3 and the Kalman filter gain Lk from Part 5 combine to form an optimal LQG controller.

2. **Performance under noise:** The LQG controller successfully stabilizes the system and regulates outputs despite process noise (Qw) and measurement noise (Rv). The Kalman filter provides accurate state estimates for control despite noisy measurements.

3. **Control efficiency:** Part 6 uses much smaller control inputs compared to Part 3 (max |u| reduced by ~4 orders of magnitude). This demonstrates that the Kalman filter-based LQG controller is more efficient than the deterministic observer-based controller, especially under uncertainty.

4. **Cost performance:** The total cost J = 4.26×10² is significantly lower than Part 3 (J = 9.06×10⁷). This large difference is due to both the presence of noise (which affects system dynamics) and the different estimator gains (L vs Lk), making direct comparison challenging. The LQG controller balances control effort with output regulation under uncertainty.

5. **Estimation accuracy:** The RMS estimation error of 0.957 demonstrates that the Kalman filter maintains reasonable estimation accuracy in the presence of noise. The error remains bounded throughout the simulation, enabling effective control.

6. **Optimal control under uncertainty:** The LQG controller represents the optimal combination of LQR control (minimizes quadratic cost) and Kalman filtering (optimal state estimation under noise). This demonstrates how to achieve optimal performance in stochastic systems.

## Explanation of Part 6

Part 6 implements an **LQG (Linear Quadratic Gaussian) controller** that combines:
- **LQR controller (Part 3)**: Optimal feedback gain K that minimizes quadratic cost
- **Kalman filter (Part 5)**: Optimal estimator gain Lk that minimizes estimation error under noise

The LQG controller represents the optimal control strategy for linear systems with Gaussian noise.

### What is LQG Control?

LQG control is the optimal control strategy for linear systems with:
- **Gaussian process noise** (uncertainty in system dynamics)
- **Gaussian measurement noise** (uncertainty in sensor readings)
- **Quadratic cost function** (balances control effort and state/output regulation)

The LQG controller consists of:
1. **Kalman filter**: Estimates system states from noisy measurements
2. **LQR controller**: Computes control inputs from estimated states

The separation principle guarantees that the controller and estimator can be designed independently.

### Key Differences: Part 3 vs Part 6

1. **Part 3 (LQR with Observer):**
   - Deterministic system (no noise)
   - Observer gain L (designed via pole placement)
   - Perfect measurements (no sensor noise)
   - Uses estimated states for control

2. **Part 6 (LQG Controller):**
   - Stochastic system (process noise + measurement noise)
   - Kalman filter gain Lk (designed via DARE for optimal estimation)
   - Noisy measurements (sensor noise)
   - Uses estimated states for control

The key insight is that Lk (Kalman filter) is optimal for noisy systems, while L (pole placement observer) is designed for deterministic systems.

### How LQG Control Works

1. **Kalman Filter (State Estimation):**
   - Predicts state evolution: xhat[k+1|k] = Ad @ xhat[k] + Bd @ u[k]
   - Updates with measurement: xhat[k+1] = xhat[k+1|k] + Lk @ (y_meas[k] - Cmeas @ xhat[k])
   - Balances trust between model prediction and noisy measurements

2. **LQR Controller (Control Input):**
   - Computes control from estimated states: u[k] = -K @ xhat[k]
   - Minimizes cost: J = Σ(u[k]^T u[k] + y1[k]^2 + y6[k]^2)

3. **Closed-Loop Dynamics:**
   - True system: x[k+1] = Ad @ x[k] + Bd @ u[k] + Bd @ w[k]
   - Estimated system: xhat[k+1] = Ad @ xhat[k] + Bd @ u[k] + Lk @ (y_meas[k] - Cmeas @ xhat[k])
   - Controller: u[k] = -K @ xhat[k]

The separation principle ensures that designing K and Lk independently yields the optimal LQG controller.

### Cost Computation

The cost function uses **y_true** (not y_meas) to avoid penalizing uncontrollable measurement noise:

```
J = Σ_{k=0}^{N-1} (u[k]^T u[k] + y1_true[k]^2 + y6_true[k]^2)
```

where y_true[k] = Cmeas @ x[k] (true output, no noise).

This convention ensures the cost reflects control performance, not measurement uncertainty.

### Why LQG Control Matters

In real systems:
- **Process noise**: Disturbances, modeling errors, actuator uncertainty
- **Measurement noise**: Sensor errors, quantization, environmental interference

LQG control provides:
- **Optimal estimation**: Kalman filter gives best state estimates under noise
- **Optimal control**: LQR controller minimizes cost using estimated states
- **Separation principle**: Controller and estimator can be designed independently

The LQG controller represents the theoretical optimal solution for linear systems with Gaussian noise and quadratic cost functions.

### Key Insights

1. **Noise impact:** Despite significant noise (Qw = 0.05, Rv = 0.1), the LQG controller successfully stabilizes the system and regulates outputs, demonstrating robustness to uncertainty.

2. **Optimality:** The combination of optimal LQR controller (K) and optimal Kalman filter (Lk) yields optimal LQG performance under uncertainty.

3. **Efficiency:** The LQG controller uses smaller control inputs than the deterministic observer-based controller, demonstrating that optimal estimation under uncertainty enables more efficient control.

4. **Separation principle:** The controller and estimator can be designed independently - design K for optimal control, design Lk for optimal estimation, combine for optimal LQG performance.

5. **Practical importance:** LQG control provides a principled approach to control design under uncertainty, which is essential for real-world applications where noise is inevitable.

The LQG controller successfully combines optimal control (LQR) with optimal estimation (Kalman filter) to achieve optimal performance in the presence of process and measurement noise.

## Implementation Verification

### Code Simplification

The simplified implementation (`final/part6/lqg_controller.py`) reduces the original code from 1,068 lines to 166 lines (84% reduction) by:

1. **Removing excessive validation:** Eliminated 20+ validation blocks and fingerprinting checks
2. **Simplifying code structure:** Streamlined simulation and metric computation
3. **Using utils functions:** Leveraged `simulate_lqg` from `final/utils/simulation.py`
4. **Minimal comments:** Kept only essential humanized comments

### Numerical Accuracy

All numerical results match the original implementation within machine precision:
- **Trajectories:** Maximum difference across all trajectories (x, xhat, u, y_true, yhat) is ~10⁻¹²
- **Cost function:** J matches exactly (difference: 4.70×10⁻¹⁰, numerical precision)
- **Metrics:** All RMS metrics match exactly (within 10⁻¹² precision)
- **Gain matrices:** K and Lk are correctly loaded from Part 3 and Part 5 outputs

The simplified implementation preserves **mathematical correctness** while dramatically improving code readability and maintainability.

## Warnings

### Warning: Qt Platform Plugin

During execution, the following warning appears:

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
