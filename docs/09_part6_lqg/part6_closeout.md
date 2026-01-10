# Part 6: LQG Controller (LQR + Kalman Filter) - Closeout

## Summary

Part 6 successfully combined the Part 3 LQR controller with the Part 5 steady-state Kalman filter to implement an LQG controller on the noisy system. The implementation simulates the closed-loop system with process and measurement noise, and compares the results against the Part 3 baseline (no noise).

## Environment and Commit Hash

### Environment Recording

1. **Operating System**:
   ```bash
   uname -a
   ```
   Record output: `[RECORD HERE]`

2. **Python Version**:
   ```bash
   python --version
   ```
   Record output: `[RECORD HERE]`

3. **Environment Type**:
   - [ ] Virtual environment (venv)
   - [ ] Conda environment
   - [ ] System Python
   - [ ] Other: `[SPECIFY]`

4. **Environment Name/Path**:
   Record: `[RECORD HERE]`

5. **Git Commit Hash**:
   ```bash
   git rev-parse HEAD
   ```
   Record: `[RECORD HERE]`

## Validation Checklist

### Component Loading

- [ ] K matrix loaded from Part 3 or recomputed successfully
- [ ] Lk matrix loaded from Part 5 or recomputed successfully
- [ ] Part 2 Cmeas and initial conditions loaded successfully
- [ ] Noise settings match Part 5 (Qw = 0.05*I3, Rv = 0.1*I2, seed=42)

### Array Dimensions

- [ ] x shape: (12, N+1)
- [ ] xhat shape: (12, N+1)
- [ ] u shape: (3, N)
- [ ] y_meas shape: (2, N+1)
- [ ] w shape: (3, N)
- [ ] v shape: (2, N+1)

### Stability Checks

- [ ] Controller stability: rho(Ad - Bd @ K) < 1.0
- [ ] Estimator stability: rho(Ad - Lk @ Cmeas) < 1.0
- [ ] Both spectral radii logged in results.txt

### Controller Validation

- [ ] Controller uses xhat: max ||u - (-K @ xhat)|| is near zero
- [ ] Controller does not use x: max ||u - (-K @ x)|| is nonzero (first 100 samples)
- [ ] Validation logged in results.txt

### Numerical Sanity

- [ ] All values finite (no inf, no NaN)
- [ ] J >= 0
- [ ] Arrays consistent (no shape mismatches)

### Part 3 Baseline Recreation

- [ ] Part 3 baseline recreated successfully
- [ ] Part 3 J matches Part 3 results.txt within tolerance (if available)
- [ ] Part 3 baseline trajectories used for comparison

### Metrics Computation

- [ ] Primary cost J computed correctly
- [ ] Input metrics computed (max_abs_u_overall, max_u_inf)
- [ ] RMS metrics computed (full window and last 20%)
- [ ] Innovation statistics computed (optional)

### Comparison

- [ ] Part 3 vs Part 6 comparison table in results.txt
- [ ] Comparison plots generated (outputs_y1_y6_comparison.png)
- [ ] y_meas vs yhat plot generated

## Key Metrics Table

### Part 3 Baseline (No Noise)

| Metric | Value |
|--------|-------|
| Total Cost J | [RECORD HERE] |
| max_abs_u_overall | [RECORD HERE] |
| max_u_inf | [RECORD HERE] |

### Part 6 LQG (With Noise)

| Metric | Value |
|--------|-------|
| Total Cost J | [RECORD HERE] |
| max_abs_u_overall | [RECORD HERE] |
| max_u_inf | [RECORD HERE] |
| RMS estimation error (full) | [RECORD HERE] |
| RMS estimation error (last 20%) | [RECORD HERE] |

### Comparison

| Metric | Change |
|--------|--------|
| ΔJ | [RECORD HERE] |
| Δmax_abs_u_overall | [RECORD HERE] |

## Artifacts Checklist

### Code Files

- [ ] `python/part6/__init__.py`
- [ ] `python/part6/run_lqg.py`

### Output Files

- [ ] `python/part6/outputs/results.txt`
- [ ] `python/part6/outputs/traj.npz`

### Plots

- [ ] `python/part6/outputs/outputs_y1_y6_comparison.png`
- [ ] `python/part6/outputs/outputs_y_meas_vs_yhat.png`
- [ ] `python/part6/outputs/inputs_u1_u2_u3.png`
- [ ] `python/part6/outputs/estimation_error_norm.png`
- [ ] Optional: `python/part6/outputs/cumulative_cost.png`

## Notes and Observations

[RECORD ANY OBSERVATIONS, ISSUES, OR DEVIATIONS FROM PLAN HERE]
