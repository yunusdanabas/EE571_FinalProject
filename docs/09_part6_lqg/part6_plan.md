# Part 6: LQG Controller (LQR + Kalman Filter) - Plan

## 1. Objective and Scope

### Primary Objectives

1. **Combine Part 3 LQR Controller with Part 5 Kalman Filter**
   - Load K matrix from Part 3 (`python/part3/outputs/K_matrix.npy`) or recompute if missing
   - Load Lk matrix from Part 5 (`python/part5/outputs/Lk_matrix.npy`) or recompute if missing
   - Use Part 2 sensor setup (Cmeas) and initial conditions (x0, xhat0)
   - Use Part 5 noise settings: Qw = 0.05 * I3, Rv = 0.1 * I2, seed = 42

2. **Simulate Noisy Closed-Loop System with LQG**
   - Plant: x[k+1] = Ad @ x[k] + Bd @ u[k] + Bd @ w[k]
   - Measurement: y_meas[k] = Cmeas @ x[k] + v[k]
   - Kalman filter: xhat[k+1] = Ad @ xhat[k] + Bd @ u[k] + Lk @ (y_meas[k] - Cmeas @ xhat[k])
   - Controller: u[k] = -K @ xhat[k] (uses estimated state)
   - Standard convention: x has length N+1, u has length N

3. **Compute Cost and Metrics**
   - Primary cost: J = sum_{k=0}^{N-1} (u[k]^T u[k] + y_true1[k]^2 + y_true6[k]^2)
   - Input metrics: max_abs_u_overall, max_u_inf
   - RMS metrics: y_true, yhat, estimation error (full window and last 20%)
   - Innovation statistics (optional)

4. **Compare Against Part 3 Baseline**
   - Recreate Part 3 baseline (no noise, Part 2 observer L) inside run_lqg.py
   - Verify reproduced Part 3 J matches Part 3 results.txt within tolerance
   - Generate comparison plots: Part 3 baseline vs Part 6 y_true, plus y_meas and yhat
   - Comparison table in results.txt: Part 3 vs Part 6 metrics

5. **Validation Gates**
   - Array shapes: x (12,N+1), xhat (12,N+1), u (3,N), y_meas (2,N+1)
   - Spectral radius checks: rho(Ad - Bd@K) < 1, rho(Ad - Lk@Cmeas) < 1
   - Controller uses estimated state: log ||u - (-K@x)|| max over first 100 samples
   - Numerical sanity: finite values, no NaNs, J >= 0

### Scope Boundaries

- **No new controller design**: Reuse K from Part 3
- **No new estimator design**: Reuse Lk from Part 5 (do not change Qw, Rv unless exam says so)
- **Sensor setup**: Part 2 Cmeas (measures x1 and x6)
- **Initial conditions**: Part 2 x0, xhat0
- **Simulation parameters**: Ts = 0.01, N = 1000 (frozen invariants)

## 2. Exam Mapping

Source: `docs/sources/final_exam_extract.md` Section 8 (Part 6 Requirement)

- Use the LQR from Part 3 on the uncertain system from Part 5
- Combines LQR with estimated states from the Kalman filter
- Simulate and compare outputs against those obtained in Part 3

## 3. Conventions

### Array Indexing Convention

- **Standard discrete-time convention**:
  - x has length N+1 (stores x[0] through x[N])
  - u has length N (stores u[0] through u[N-1])
  - y has length N+1 (stores y[0] through y[N])
  - w has length N (stores w[0] through w[N-1])
  - v has length N+1 (stores v[0] through v[N])

### Cost Pairing Convention

- Cost pairs u[k] with y_true[k] (k=0..N-1)
- Stage cost: stage_cost[k] = u[k]^T @ u[k] + y_true[0,k]^2 + y_true[1,k]^2
- Total cost: J = sum_{k=0}^{N-1} stage_cost[k]

### Controller Implementation

- Controller uses estimated state: u[k] = -K @ xhat[k] (CRITICAL: uses xhat, not x)
- This is validated by checking that ||u - (-K @ xhat)|| is near zero
- And that ||u - (-K @ x)|| is nonzero (especially in early samples)

## 4. Validation Gates

### Hard Fail Conditions

1. **Array shapes**:
   - x: (12, N+1) - FAIL if mismatch
   - xhat: (12, N+1) - FAIL if mismatch
   - u: (3, N) - FAIL if mismatch
   - y_meas: (2, N+1) - FAIL if mismatch

2. **Spectral radius checks**:
   - rho(Ad - Bd @ K) < 1.0 - FAIL if >= 1.0 (controller unstable)
   - rho(Ad - Lk @ Cmeas) < 1.0 - FAIL if >= 1.0 (estimator unstable)

3. **Controller uses estimated state**:
   - max ||u - (-K @ xhat)|| should be ~0 (within numerical tolerance)
   - max ||u - (-K @ x)|| over first 100 samples should be nonzero
   - WARNING if u appears to use x instead of xhat

4. **Numerical sanity**:
   - All values finite (no inf, no NaN) - FAIL if any non-finite
   - J >= 0 - FAIL if negative
   - Arrays consistent (no shape mismatches) - FAIL if inconsistent

## 5. Run Command

```bash
python python/part6/run_lqg.py
```

## 6. Expected Artifacts

### Files

1. `python/part6/__init__.py` - Package initialization
2. `python/part6/run_lqg.py` - Main implementation script
3. `python/part6/outputs/results.txt` - Results and metrics
4. `python/part6/outputs/traj.npz` - Trajectory data

### Plots

1. `outputs_y1_y6_comparison.png` - Part 3 baseline vs Part 6 y_true plus yhat
2. `outputs_y_meas_vs_yhat.png` - Noisy measurement vs estimate
3. `inputs_u1_u2_u3.png` - Input trajectories
4. `estimation_error_norm.png` - Estimation error norm over time
5. Optional: `cumulative_cost.png` - Cumulative cost over time
