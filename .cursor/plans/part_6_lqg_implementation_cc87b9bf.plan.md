---
name: Part 6 LQG Implementation
overview: "Implement Part 6 (LQG): combine Part 3 LQR controller (K) with Part 5 steady-state Kalman filter (Lk) on the noisy system. Simulate closed-loop with noise and compare outputs against Part 3 baseline."
todos:
  - id: create_doc_structure
    content: "Create documentation structure: docs/09_part6_lqg/plan.md and closeout.md"
    status: completed
  - id: create_part6_structure
    content: Create python/part6/ directory structure with __init__.py
    status: completed
  - id: implement_load_components
    content: Implement loading/recomputing of K (Part 3) and Lk (Part 5) matrices
    status: completed
    dependencies:
      - create_part6_structure
  - id: implement_part3_baseline
    content: Implement recreate_part3_baseline() function to generate Part 3 baseline for comparison
    status: completed
    dependencies:
      - implement_load_components
  - id: implement_lqg_simulation
    content: Implement simulate_lqg_closed_loop() function for noisy closed-loop with Kalman filter
    status: completed
    dependencies:
      - implement_load_components
  - id: implement_metrics
    content: Implement compute_lqg_metrics() function for cost and RMS metrics
    status: completed
    dependencies:
      - implement_lqg_simulation
  - id: implement_validation
    content: "Implement validation gates: array shapes, spectral radii, controller uses xhat, numerical sanity"
    status: completed
    dependencies:
      - implement_lqg_simulation
  - id: implement_comparison
    content: "Implement Part 3 vs Part 6 comparison: plots and results.txt table"
    status: completed
    dependencies:
      - implement_part3_baseline
      - implement_metrics
  - id: implement_plots
    content: "Generate all required plots: outputs comparison, y_meas vs yhat, inputs, estimation error"
    status: completed
    dependencies:
      - implement_comparison
  - id: implement_results_output
    content: Write results.txt with all metrics, comparison table, and validation gate results
    status: completed
    dependencies:
      - implement_validation
      - implement_comparison
  - id: save_artifacts
    content: Save traj.npz with all trajectories and noise samples
    status: completed
    dependencies:
      - implement_lqg_simulation
---

# Part 6: LQG Controller (LQR + Kalman Filter) - Implementation Plan

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

## 3. Implementation Details

### A. Load Frozen Components

**File**: `python/part6/run_lqg.py`

1. **Model matrices** (from utils):
   ```python
   from build_model import build_continuous_model, discretize_zoh
   A, B, C = build_continuous_model()
   Ad, Bd, Cd, Dd = discretize_zoh(A, B, C, Ts=0.01)
   ```

2. **Part 2 components**:
   ```python
   from observer_design import get_part2_C_matrix
   from run_observer_sim import get_part2_initial_conditions
   Cmeas = get_part2_C_matrix()  # (2, 12) measures x1 and x6
   x0, xhat0 = get_part2_initial_conditions()
   ```

3. **Part 3 LQR gain K**:

   - Try loading: `python/part3/outputs/K_matrix.npy`
   - If missing, recompute K exactly as Part 3 did:
     - Q = Cmeas.T @ Cmeas, R = I3
     - Solve DARE: P = solve_discrete_are(Ad, Bd, Q, R)
     - K = (R + Bd.T @ P @ Bd)^(-1) @ (Bd.T @ P @ Ad)
   - Log whether K was loaded or recomputed

4. **Part 5 Kalman gain Lk**:

   - Try loading: `python/part5/outputs/Lk_matrix.npy`
   - If missing, recompute Lk exactly as Part 5 did:
     - Qw = 0.05 * I3, Rv = 0.1 * I2
     - Qx = Bd @ Qw @ Bd.T
     - Solve DARE: P = solve_discrete_are(Ad.T, Cmeas.T, Qx, Rv)
     - Lk = P @ Cmeas.T @ inv(Cmeas @ P @ Cmeas.T + Rv)
   - Log whether Lk was loaded or recomputed

5. **Part 5 noise settings** (frozen):

   - Qw = 0.05 * np.eye(3)
   - Rv = 0.1 * np.eye(2)
   - seed = 42

### B. Recreate Part 3 Baseline (for comparison)

**Function**: `recreate_part3_baseline(Ad, Bd, Cmeas, K, x0, xhat0, N, Ts)`

1. Load Part 2 observer gain L:

   - Try loading: `python/part3/outputs/L_matrix.npy` (Part 3 saves it)
   - If missing, recompute using Part 2 observer design:
     ```python
     from observer_design import design_observer
     L, _ = design_observer(Ad, Cmeas, method='pole_placement', 
                           pole_range=(0.4, 0.8), fallback_to_lqr=True)
     ```


2. Run Part 3 closed-loop (no noise):

   - Use `simulate_closed_loop_with_observer` function (same as Part 3)
   - Controller: u[k] = -K @ xhat[k]
   - Observer: xhat[k+1] = Ad @ xhat[k] + Bd @ u[k] + L @ (y[k] - Cmeas @ xhat[k])
   - Plant: x[k+1] = Ad @ x[k] + Bd @ u[k] (no noise)

3. Compute Part 3 cost:

   - J_part3 = sum_{k=0}^{N-1} (u[k]^T u[k] + y1[k]^2 + y6[k]^2)
   - Verify against Part 3 results.txt (if exists) within tolerance (e.g., 1e-6)

### C. Simulate Noisy Closed-Loop with Kalman Filter

**Function**: `simulate_lqg_closed_loop(Ad, Bd, Cmeas, K, Lk, x0, xhat0, N, Ts, Qw, Rv, seed)`

**Dynamics**:

- y_true[k] = Cmeas @ x[k]
- y_meas[k] = y_true[k] + v[k]
- yhat[k] = Cmeas @ xhat[k]
- u[k] = -K @ xhat[k]
- x[k+1] = Ad @ x[k] + Bd @ u[k] + Bd @ w[k]
- xhat[k+1] = Ad @ xhat[k] + Bd @ u[k] + Lk @ (y_meas[k] - Cmeas @ xhat[k])

**Indexing convention**:

- w: length N (k=0..N-1)
- v: length N+1 (k=0..N) - also store y_meas[N] for plotting
- y_meas[0..N-1] used in updates, y_meas[N] computed for completeness

**Returns**:

- x: (12, N+1) true state trajectory
- xhat: (12, N+1) estimated state trajectory
- u: (3, N) input trajectory
- y_true: (2, N+1) true output (Cmeas @ x, no noise)
- y_meas: (2, N+1) measured output (Cmeas @ x + v, with noise)
- yhat: (2, N+1) estimated output (Cmeas @ xhat)
- w: (3, N) process noise samples
- v: (2, N+1) measurement noise samples
- innovations: (2, N) innovation sequence (y_meas - yhat)
- t: (N+1,) time vector

### D. Cost and Metrics Computation

**Function**: `compute_lqg_metrics(x, xhat, u, y_true, yhat, Cmeas, N)`

1. **Primary cost**:
   ```python
   J = sum_{k=0}^{N-1} (u[k]^T @ u[k] + y_true[0,k]^2 + y_true[1,k]^2)
   ```

2. **Input metrics**:

   - max_abs_u_overall = max(abs(u))
   - max_u_inf = max_k ||u[k]||_inf

3. **RMS metrics**:

   - RMS of y_true (full window and last 20%)
   - RMS of yhat (full window and last 20%)
   - RMS of estimation error ||x - xhat|| (full window and last 20%)

4. **Innovation stats** (optional):

   - r[k] = y_meas[k] - Cmeas @ xhat[k] (before update)
   - Mean innovation, innovation covariance

### E. Validation Gates

**Hard fail if violated**:

1. **Array shapes**:

   - x: (12, N+1)
   - xhat: (12, N+1)
   - u: (3, N)
   - y_meas: (2, N+1)

2. **Spectral radius checks**:

   - rho(Ad - Bd @ K) < 1.0 (controller stability)
   - rho(Ad - Lk @ Cmeas) < 1.0 (estimator stability)

3. **Controller uses estimated state**:

   - Compute u_from_x = -K @ x (for first 100 samples)
   - Compute u_from_xhat = -K @ xhat
   - Log max ||u - u_from_xhat|| (should be ~0)
   - Log max ||u - u_from_x|| over first 100 samples (should be nonzero)

4. **Numerical sanity**:

   - All values finite (no inf, no NaN)
   - J >= 0
   - Arrays consistent (no shape mismatches)

### F. Comparison Against Part 3

**Comparison outputs**:

1. **Plot**: `outputs_y1_y6_comparison.png`

   - Part 3 baseline: y1 and y6 (no noise)
   - Part 6 LQG: y_true (true outputs, with noise in plant)
   - Part 6 LQG: yhat (estimated outputs)
   - Overlay all on same axes

2. **Plot**: `outputs_y_meas_vs_yhat.png`

   - y_meas (noisy measurements) vs yhat (estimates)

3. **Table in results.txt**:
   ```
   Part 3 (baseline, no noise):
     J = ...
     max_abs_u_overall = ...
   
   Part 6 (LQG, with noise):
     J = ...
     max_abs_u_overall = ...
   
   Change:
     ΔJ = ...
     Δmax_abs_u_overall = ...
   ```


### G. Artifacts

**Files to create**:

1. `python/part6/__init__.py` (empty file)

2. `python/part6/run_lqg.py` (main script)

3. `python/part6/outputs/results.txt`:

   - N, Ts, seed, Qw, Rv
   - K and Lk shapes and how obtained (loaded vs recomputed)
   - Spectral radii and stability margins
   - J and max input metrics
   - Baseline vs LQG comparison table
   - Validation gate results

4. `python/part6/outputs/traj.npz`:

   - Keys: x, xhat, u, w, v, y_true, y_meas, yhat, t

5. **Plots**:

   - `outputs_y1_y6_comparison.png` (Part 3 baseline vs Part 6 y_true plus yhat)
   - `outputs_y_meas_vs_yhat.png` (noisy measurement vs estimate)
   - `inputs_u1_u2_u3.png`
   - `estimation_error_norm.png`
   - Optional: `cumulative_cost.png`

## 4. File Structure

```
python/part6/
├── __init__.py
├── run_lqg.py
└── outputs/
    ├── results.txt
    ├── traj.npz
    ├── outputs_y1_y6_comparison.png
    ├── outputs_y_meas_vs_yhat.png
    ├── inputs_u1_u2_u3.png
    └── estimation_error_norm.png
```

## 5. Run Command

```bash
python python/part6/run_lqg.py
```

## 6. Key Implementation Notes

1. **Standard convention**: x has length N+1 (x[0]..x[N]), u has length N (u[0]..u[N-1])
2. **Cost pairs**: u[k] pairs with y_true[k] (k=0..N-1)
3. **Noise indexing**: w has length N, v has length N+1 (also store v[N] for y_meas[N])
4. **Controller uses xhat**: Critical validation - u[k] = -K @ xhat[k], not -K @ x[k]
5. **Part 3 baseline recreation**: Verify J matches Part 3 results.txt within tolerance before using for comparison

## 7. Documentation

**Files to create**:

1. `docs/09_part6_lqg/plan.md`:

   - Exam mapping
   - Conventions (indexing, cost pairing)
   - Validation gates
   - Exact run command

2. `docs/09_part6_lqg/closeout.md`:

   - Checklist of completed items
   - Key metrics table (Part 3 vs Part 6)
   - Artifacts checklist
   - Validation gate results summary