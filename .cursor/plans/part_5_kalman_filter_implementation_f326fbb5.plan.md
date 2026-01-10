---
name: Part 5 Kalman Filter Implementation
overview: "Implement Part 5: Design and validate a discrete-time steady-state Kalman filter (LQE) for the stochastic system with actuator and sensor noise. Use Part 2 measurement matrix and initial conditions, implement DARE-based Kalman gain computation, simulate with noise (seed=42), and generate required metrics and plots."
todos:
  - id: setup_files
    content: "Create directory structure and empty files: docs/08_part5_kalman_filter/plan.md, docs/08_part5_kalman_filter/closeout.md, python/part5/__init__.py, python/part5/outputs/ directory"
    status: completed
  - id: implement_main_script
    content: Implement python/part5/run_kalman_filter.py with system setup, noise covariances, Kalman filter design (DARE), stochastic simulation, metrics computation, plots, and artifact generation
    status: completed
    dependencies:
      - setup_files
  - id: validate_implementation
    content: "Verify all validation gates: array shapes, covariance properties, DARE solution, estimator stability, results.txt completeness, and plot existence"
    status: completed
    dependencies:
      - implement_main_script
---

# Part 5: Discrete-Time Steady-State Kalman Filter (LQE)

## Overview

Implement a discrete-time steady-state Kalman filter for the stochastic 6-mass spring system with process and measurement noise. The filter uses the Part 2 measurement configuration (measuring x1 and x6) and operates in open-loop with zero inputs.

## Files to Create

1. **docs/08_part5_kalman_filter/plan.md** - Implementation plan document
2. **docs/08_part5_kalman_filter/closeout.md** - Summary of completed work
3. **python/part5/init.py** - Package initialization
4. **python/part5/run_kalman_filter.py** - Main implementation script

## Implementation Details

### A) System Setup

- Load `(Ad, Bd)` from `utils/build_model.py` using `build_continuous_model()` and `discretize_zoh()` (Ts=0.01)
- Get `Cmeas = C_part2` (2×12) from `part2/observer_design.py::get_part2_C_matrix()`
- Get initial conditions `x0, xhat0` from `part2/run_observer_sim.py::get_part2_initial_conditions()`
- Use frozen invariants: `Ts = 0.01`, `N = 1000`

### B) Noise Covariances

- `Qw = 0.05 * I_m` where `m = 3` (actuator noise)
- `Rv = 0.1 * I_p` where `p = 2` (sensor noise)
- Process noise covariance: `Qx = Bd @ Qw @ Bd.T` (12×12)
- Validate: Qw, Rv are PSD and have correct shapes (3×3, 2×2 respectively)

### C) Kalman Filter Design

- Solve estimator DARE: `P = solve_discrete_are(Ad.T, Cmeas.T, Qx, Rv)`
- Compute innovation covariance: `S = Cmeas @ P @ Cmeas.T + Rv` (2×2)
- Validate: S is invertible (check condition number)
- Compute steady-state Kalman gain: `Lk = P @ Cmeas.T @ inv(S)` (12×2)
- Validate: `spectral_radius(Ad - Lk @ Cmeas) < 1.0` (hard gate)

### D) Simulation with Noise

- Random seed: `np.random.seed(42)` for reproducibility
- Preallocate arrays: `x (12, N+1)`, `xhat (12, N+1)`, `y (2, N+1)`, `yhat (2, N+1)`, `u (3, N)`, `w (3, N)`, `v (2, N+1)`
- Initialize: `x[:,0] = x0`, `xhat[:,0] = xhat0`
- True system simulation (k=0 to N-1):
  ```python
  for k in range(N):
      w_k = np.random.multivariate_normal(np.zeros(3), Qw)
      v_k = np.random.multivariate_normal(np.zeros(2), Rv)
      y[:,k] = Cmeas @ x[:,k] + v_k
      if k < N:
          x[:,k+1] = Ad @ x[:,k] + Bd @ u[:,k] + Bd @ w_k
  ```


Generate `v_N` for final output: `y[:,N] = Cmeas @ x[:,N] + v_N`

- Kalman filter simulation (k=0 to N-1):
  ```python
  for k in range(N):
      yhat[:,k] = Cmeas @ xhat[:,k]
      if k < N:
          xhat[:,k+1] = Ad @ xhat[:,k] + Bd @ u[:,k] + Lk @ (y[:,k] - yhat[:,k])
  ```


Final output: `yhat[:,N] = Cmeas @ xhat[:,N]`

### E) Metrics Computation

- Estimator stability: `spectral_radius(Ad - Lk @ Cmeas) < 1.0` (hard gate)
- RMS estimation error (overall): `RMS(||x - xhat||_2)` over k=0..N
- Per-state RMS: 12 values (one per state)
- Position RMS: Focus on x1..x6 (displacement states)
- Output tracking RMS: `RMS(y - yhat)` for y1 and y6 separately
- Reproducibility logging:
  - Seed=42
  - Qw, Rv matrices
  - First 3 samples of `w_k` and `v_k` (as numeric vectors)

### F) Plots Generation

Save to `python/part5/outputs/`:

1. **outputs_y_vs_yhat.png**: y1, y6 vs yhat1, yhat6 over time (2 subplots)
2. **estimation_error_norm.png**: `||x - xhat||_2` over time
3. **estimation_error_x1_x6.png**: e_x1 and e_x6 time series
4. **per_state_rms_bar.png** (optional): Bar chart of per-state RMS errors

### G) Artifacts

Save to `python/part5/outputs/`:

1. **results.txt** with sections:

   - Simulation parameters (N, Ts, array dimensions)
   - Cmeas matrix (full or explicit rows), x0, xhat0
   - Noise covariances (Qw, Rv, Qx) - shapes and summary
   - Kalman gain Lk (full matrix or key rows, prefer full)
   - Estimator stability: spectral radius of (Ad - Lk @ Cmeas)
   - RMS metrics (overall, per-state, positions x1..x6, output tracking)
   - Reproducibility info: seed, first 3 noise samples

2. **Lk_matrix.npy**: Kalman gain matrix (12×2)

3. **traj.npz** with keys:

   - `x`: (12, N+1) true state trajectory
   - `xhat`: (12, N+1) estimated state trajectory
   - `y`: (2, N+1) noisy output trajectory
   - `yhat`: (2, N+1) estimated output trajectory
   - `u`: (3, N) input trajectory (all zeros)
   - `w`: (3, N) process noise samples
   - `v`: (2, N+1) measurement noise samples
   - Document shapes in results.txt

### H) Validation Gates (Hard Fails)

1. Array shapes: `x (12, N+1)`, `xhat (12, N+1)`, `u (3, N)`, `y (2, N+1)`
2. Covariances: Qw, Rv are PSD with correct shapes
3. DARE solves: P is finite, S = C P C^T + R is invertible
4. Estimator stability: `spectral_radius(Ad - Lk @ Cmeas) < 1.0`
5. results.txt exists with all required fields (no placeholders)
6. All plots exist and are saved correctly

## Code Structure

The main script `run_kalman_filter.py` will:

1. Import utilities from `utils/build_model.py`, `part2/observer_design.py`, `part2/run_observer_sim.py`
2. Set up system matrices and initial conditions
3. Define noise covariances
4. Design Kalman filter (solve DARE, compute Lk)
5. Run stochastic simulation with noise (seed=42)
6. Compute RMS metrics
7. Generate plots
8. Save results.txt, Lk_matrix.npy, and traj.npz

## Key Design Decisions

- **Input**: Use `u[k] = 0` for all k (open-loop, deterministic)
- **Random seed**: Fixed at 42 for reproducibility
- **Measurement matrix**: Reuse Part 2's C_part2 (measuring x1 and x6)
- **Initial conditions**: Use Part 2's x0 and xhat0 (frozen invariant)
- **Process noise**: Enters through Bd (so Qx = Bd @ Qw @ Bd.T)
- **DARE solver**: Use `scipy.linalg.solve_discrete_are` for steady-state solution

## References

- System model: `docs/sources/final_exam_extract.md` Section 7 (Part 5 Requirement)
- Invariants: `docs/07_comprehensive_review_part0_to_part4/cross_part_invariants.md`
- Part 2 implementation: `python/part2/run_observer_sim.py` (structure pattern)
- Part 3 implementation: `python/part3/run_lqr_with_observer.py` (metrics pattern)