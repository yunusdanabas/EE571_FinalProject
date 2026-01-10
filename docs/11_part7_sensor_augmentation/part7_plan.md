# Part 7: Sensor Augmentation Analysis - Plan

## 1. Objective and Scope

### Primary Objectives

1. **Investigate Sensor Augmentation Effects**
   - Determine whether adding more sensors helps with estimation and/or regulation
   - Compare two augmented sensor configurations against Part 6 baseline (2 sensors)
   - Case 1: 4 sensors (x1, x2, x5, x6)
   - Case 2: 6 sensors (x1, x2, x3, x4, x5, x6)

2. **Key Design Decisions**
   - **Controller K remains UNCHANGED** from Part 3 - LQR depends on cost function, not sensors
   - **Kalman gain Lk must be REDESIGNED** for each sensor configuration
   - **Rv dimension changes** - 0.1 * I_p where p = number of sensors
   - **Qw remains UNCHANGED** - 0.05 * I_3 (process noise via Bd)
   - **Cost function stays the SAME** - J = sum(u^T u + y1^2 + y6^2)

3. **Simulation and Comparison**
   - Run LQG simulation for both sensor configurations
   - Compare against Part 6 baseline metrics
   - Generate comparison plots and tables
   - Answer the exam question: "Do more sensors help?"

### Scope Boundaries

- **No new controller design**: Reuse K from Part 3 (loaded from `python/part3/outputs/K_matrix.npy`)
- **Estimator redesign required**: New Lk for each sensor configuration
- **Initial conditions**: Same as Part 2 (x0, xhat0)
- **Simulation parameters**: Ts = 0.01, N = 1000 (frozen invariants)
- **Random seed**: 42 (reproducibility)

## 2. Exam Mapping

Source: `docs/sources/final_exam_extract.md` Section 9 (Part 7 Requirement)

> Closed-loop system from Part 6, add more sensors. Additional sensors have the same 
> uncertainty level as others. Determine whether more sensors help estimation and or regulation.

### Case 1 Measurement Matrix (4x12)
```
C_case1 = [[1 0 0 0 0 0 0 0 0 0 0 0]   # x1
           [0 1 0 0 0 0 0 0 0 0 0 0]   # x2
           [0 0 0 0 1 0 0 0 0 0 0 0]   # x5
           [0 0 0 0 0 1 0 0 0 0 0 0]]  # x6
```

### Case 2 Measurement Matrix (6x12)
```
C_case2 = [[1 0 0 0 0 0 0 0 0 0 0 0]   # x1
           [0 1 0 0 0 0 0 0 0 0 0 0]   # x2
           [0 0 1 0 0 0 0 0 0 0 0 0]   # x3
           [0 0 0 1 0 0 0 0 0 0 0 0]   # x4
           [0 0 0 0 1 0 0 0 0 0 0 0]   # x5
           [0 0 0 0 0 1 0 0 0 0 0 0]]  # x6
```

## 3. Conventions

### Array Indexing Convention (FROZEN)

- **State trajectory x:** `(n, N+1)` stores `x[0]` through `x[N]` (N+1 samples)
- **Input trajectory u:** `(m, N)` stores `u[0]` through `u[N-1]` (N samples)
- **Output trajectory y:** `(p, N+1)` stores `y[0]` through `y[N]` (N+1 samples)
- **Cost range:** k = 0 to N-1 (inclusive)

### Cost Function (FROZEN)

```
J = sum_{k=0}^{N-1} (u[k]^T u[k] + y1[k]^2 + y6[k]^2)
```

Where y1 and y6 are extracted from x using cost output selector Cy (2x12):
```
Cy = [[1 0 0 0 0 0 0 0 0 0 0 0]
      [0 0 0 0 0 1 0 0 0 0 0 0]]
```

**CRITICAL**: Cost uses y1 and y6 regardless of sensor configuration. The additional sensors
improve estimation but do not change the cost function.

### Noise Model (FROZEN from Part 5)

- **Process noise**: w ~ N(0, Qw) where Qw = 0.05 * I_3
- **Measurement noise**: v ~ N(0, Rv) where Rv = 0.1 * I_p (p = number of sensors)
- **Process noise entry**: x[k+1] = Ad @ x[k] + Bd @ u[k] + Bd @ w[k]
- **Random seed**: 42

### Controller Implementation

- Controller uses estimated state: u[k] = -K @ xhat[k]
- K is loaded from Part 3 (UNCHANGED)
- Kalman filter: xhat[k+1] = Ad @ xhat[k] + Bd @ u[k] + Lk @ (y_meas[k] - Cmeas @ xhat[k])

## 4. Validation Gates

### P7-G1: Case 1 Measurement Matrix
- **Criterion**: C_case1 shape = (4, 12), measures x1, x2, x5, x6
- **Threshold**: Exact match
- **Evidence**: results.txt, matrix printout

### P7-G2: Case 2 Measurement Matrix
- **Criterion**: C_case2 shape = (6, 12), measures x1..x6
- **Threshold**: Exact match
- **Evidence**: results.txt, matrix printout

### P7-G3: K Matrix Unchanged
- **Criterion**: K loaded from Part 3, max_abs_diff = 0
- **Threshold**: Exact equality (no modification)
- **Evidence**: K fingerprint (||K||_F, max|K|, hash)

### P7-G4: Lk_case1 Redesigned
- **Criterion**: Lk_case1 shape = (12, 4)
- **Threshold**: Correct dimension for 4 sensors
- **Evidence**: results.txt, saved matrix

### P7-G5: Lk_case2 Redesigned
- **Criterion**: Lk_case2 shape = (12, 6)
- **Threshold**: Correct dimension for 6 sensors
- **Evidence**: results.txt, saved matrix

### P7-G6: Rv_case1 Dimension
- **Criterion**: Rv_case1 = 0.1 * I_4, shape (4, 4)
- **Threshold**: Exact
- **Evidence**: results.txt

### P7-G7: Rv_case2 Dimension
- **Criterion**: Rv_case2 = 0.1 * I_6, shape (6, 6)
- **Threshold**: Exact
- **Evidence**: results.txt

### P7-G8: Qw Unchanged
- **Criterion**: Qw = 0.05 * I_3
- **Threshold**: Exact (frozen from Part 5)
- **Evidence**: results.txt

### P7-G9: Reproducibility Seed
- **Criterion**: seed = 42
- **Threshold**: Exact
- **Evidence**: results.txt

### P7-G10: Estimator Stability
- **Criterion**: Spectral radius < 1.0 for both cases
- **Threshold**: rho_est < 1.0
- **Evidence**: results.txt, eigenvalue analysis

### P7-G11: Cost Reported
- **Criterion**: J_true reported for both cases
- **Threshold**: Numeric value logged
- **Evidence**: results.txt

### P7-G12: Estimation Improvement
- **Criterion**: RMS estimation error should decrease vs Part 6
- **Threshold**: Lower RMS with more sensors
- **Evidence**: Comparison table

### P7-G13: Comparison Table
- **Criterion**: Quantitative comparison table present
- **Threshold**: Part 6 vs Case 1 vs Case 2 metrics
- **Evidence**: results.txt

### P7-G14: Conclusion Justified
- **Criterion**: "Do more sensors help?" answered with justification
- **Threshold**: Clear conclusion based on metrics
- **Evidence**: Analysis section in results.txt

## 5. Implementation Steps

### Step 1: System Setup
1. Load discrete-time matrices (Ad, Bd) from Part 0 utilities
2. Load Part 2 initial conditions (x0, xhat0)
3. Define cost output selector Cy (2x12, extracts y1 and y6)

### Step 2: Load Part 3 Controller
1. Load K from `python/part3/outputs/K_matrix.npy`
2. Verify K fingerprint (||K||_F, max|K|)
3. Check controller stability: rho(Ad - Bd @ K) < 1.0

### Step 3: Define Sensor Configurations
1. Define C_case1 (4x12) measuring x1, x2, x5, x6
2. Define C_case2 (6x12) measuring x1..x6
3. Define corresponding Rv: 0.1 * I_4 and 0.1 * I_6

### Step 4: Kalman Filter Design (Both Cases)
For each case:
1. Compute Qx = Bd @ Qw @ Bd.T (process noise covariance)
2. Solve DARE: P = solve_discrete_are(Ad.T, Cmeas.T, Qx, Rv)
3. Compute innovation covariance: S = Cmeas @ P @ Cmeas.T + Rv
4. Compute Kalman gain: Lk = P @ Cmeas.T @ inv(S)
5. Verify estimator stability: rho(Ad - Lk @ Cmeas) < 1.0

### Step 5: LQG Simulation (Both Cases)
For each case:
1. Set random seed to 42
2. Generate noise samples: w ~ N(0, Qw), v ~ N(0, Rv)
3. Simulate closed-loop:
   - y_cost[k] = Cy @ x[k] (for cost computation)
   - y_true[k] = Cmeas @ x[k] (true sensor outputs)
   - y_meas[k] = y_true[k] + v[k] (noisy measurements)
   - u[k] = -K @ xhat[k]
   - x[k+1] = Ad @ x[k] + Bd @ u[k] + Bd @ w[k]
   - xhat[k+1] = Ad @ xhat[k] + Bd @ u[k] + Lk @ (y_meas[k] - Cmeas @ xhat[k])

### Step 6: Compute Metrics
For each case:
1. Cost: J_true = sum(u^T u + y1^2 + y6^2) using y_cost
2. Input metrics: max|u| overall
3. RMS estimation error: full window and last 20%
4. Per-state RMS errors

### Step 7: Comparison and Analysis
1. Load Part 6 baseline metrics
2. Create comparison table: Part 6 vs Case 1 vs Case 2
3. Calculate percentage improvements
4. Answer: "Do more sensors help with estimation and/or regulation?"

### Step 8: Generate Outputs
1. Save results.txt with all metrics and analysis
2. Save Kalman gain matrices (Lk_case1_matrix.npy, Lk_case2_matrix.npy)
3. Save trajectory files (traj_case1.npz, traj_case2.npz)
4. Generate comparison plots

## 6. Run Command

```bash
cd /home/yunusdanabas/EE571_FinalProject
mamba activate main
python python/part7/run_part7.py
```

## 7. Expected Artifacts

### Code Files
1. `python/part7/__init__.py` - Package initialization
2. `python/part7/run_part7.py` - Main implementation script

### Output Files (in `python/part7/outputs/`)
1. `results.txt` - Comprehensive results with metrics and analysis
2. `Lk_case1_matrix.npy` - Kalman gain for Case 1 (12x4)
3. `Lk_case2_matrix.npy` - Kalman gain for Case 2 (12x6)
4. `traj_case1.npz` - Trajectory data for Case 1
5. `traj_case2.npz` - Trajectory data for Case 2

### Plots
1. `estimation_error_comparison.png` - Part 6 vs Case 1 vs Case 2 error norms
2. `outputs_comparison.png` - Output trajectories (y1, y6)
3. `inputs_comparison.png` - Input trajectories (u1, u2, u3)
4. `per_state_rms_comparison.png` - Per-state RMS bar chart

## 8. Expected Outcomes

Based on control theory:

1. **Estimation should IMPROVE** with more sensors
   - More measurements provide more information
   - Kalman filter can better estimate unobserved states
   - RMS estimation error should decrease

2. **Regulation should IMPROVE or stay similar**
   - Better estimation leads to better control
   - J_true should decrease or stay similar
   - Effect may be modest since controller K is unchanged

3. **Convergence should be FASTER**
   - Estimator spectral radius should decrease
   - More sensors provide faster error correction

## 9. Source References

- Part 7 requirement: `docs/sources/final_exam_extract.md` Section 9
- Part 3 LQR controller: `python/part3/outputs/K_matrix.npy`
- Part 6 baseline: `python/part6/outputs/results.txt`
- Frozen invariants: `docs/07_comprehensive_review_part0_to_part4/cross_part_invariants.md`
- Part 2 initial conditions: `docs/sources/final_exam_extract.md` Section 4
