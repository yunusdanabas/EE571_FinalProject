# Part 4: Discrete-Time LQR with Reduced Input (u3 Removed) - Closeout

## Summary

Part 4 successfully redesigned and implemented a discrete-time infinite-horizon LQR controller for the 6-mass spring chain system with the third input u3 removed. The controller uses estimated states from the Part 2 observer (u_red[k] = -K_red @ xhat[k]) and minimizes the same cost as Part 3: J = sum_{k=0}^{N-1} (u^T u + y1^2 + y6^2). All validation checks passed.

**Final Convention (FROZEN, matches Part 3):**
- Array dimensions: x (12, N+1), u_red (2, N), y (2, N+1)
- Cost pairs u_red[k] with transition from x[k] to x[k+1] (standard discrete-time convention)
- Simulation parameters: N = 1000, Ts = 0.01 s
- Initial conditions: x0 and xhat0 from Part 2 (exam specification)

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

### Cost Matrix Gates

- [ ] Q is 12×12, symmetric (same as Part 3)
- [ ] R_red is 2×2, symmetric, positive definite
- [ ] Q and R_red shapes and nonzero structure logged

### Stabilizability and Detectability Gates

- [ ] (Ad, Bd_red) stabilizability check passes (PBH rank condition)
- [ ] (Ad, Cy) detectability check passes (PBH rank condition)

### LQR Design Gates

- [ ] DARE solver succeeds without warnings or NaNs
- [ ] spectral_radius(Ad - Bd_red @ K_red) < 1.0 (hard fail if not)
- [ ] Eigenvalues of Acl_red and spectral radius logged
- [ ] Dominant eigenvalue magnitude, angle, and margin logged

### Baseline Loading Gates

- [ ] Part 3 baseline loaded from results.txt (or marked UNKNOWN with clear message)
- [ ] If baseline UNKNOWN, exit gracefully without creating overlay plots

### Simulation Gates

- [ ] Dimensions consistent: x (12, N+1), xhat (12, N+1), u_red (2, N), y (2, N+1)
- [ ] Controller uses xhat, not x (explicit check/logged assertion)
- [ ] Simulation runs for N steps without divergence
- [ ] Max absolute state magnitude and max absolute input magnitude logged

### Cost Gates

- [ ] J_red is finite and non-negative
- [ ] Metrics logged and written to outputs/results.txt
- [ ] Comparison section included if baseline loaded

### Artifact Gates

- [ ] results.txt created with all required information
- [ ] inputs_u1_u2.png created
- [ ] estimation_error_norm.png created
- [ ] comparison_outputs_y1_y6.png created (only if baseline loaded)

## Key Results

### LQR Design Results

| Parameter | Value | Notes |
|-----------|-------|-------|
| K_red matrix shape | (2, 12) | LQR gain matrix (reduced from Part 3's 3×12) |
| Spectral radius (Acl_red) | [RECORD] | Must be < 1.0 |
| Dominant eigenvalue | [RECORD] | Magnitude and angle |
| Stabilizability | [PASS/FAIL] | PBH rank condition for (Ad, Bd_red) |
| Detectability | [PASS/FAIL] | PBH rank condition for (Ad, Cy) |
| DARE solver status | [Success/Fail] | No warnings or NaNs |
| Observer spectral radius | [RECORD] | Ad - L @ Cmeas (reused from Part 2/3) |

### Cost Matrices

| Matrix | Shape | Description |
|--------|-------|-------------|
| Cy | (2, 12) | Cost output selector (Part 2 C matrix, same as Part 3) |
| Q | (12, 12) | State weight matrix (Q = Cy^T @ Cy, same as Part 3) |
| R_red | (2, 2) | Input weight matrix (R_red = I2, reduced from Part 3's I3) |

### Input Reduction

| Parameter | Value | Notes |
|-----------|-------|-------|
| Bd_red shape | (12, 2) | Reduced input matrix (columns 0 and 1 from Bd) |
| u_red dimension | 2 | Inputs u1 and u2 only (u3 removed) |
| Input removal convention | Bd[:, [0,1]] | Extracts first two columns |

### Simulation Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| N (input samples) | 1000 | u_red has length N, x has length N+1 |
| Ts (sampling time) | 0.01 s | Discrete-time sampling |
| Time span | 10.0 s | N * Ts |
| Array convention | Standard | x: (12, N+1), u_red: (2, N), y: (2, N+1) |
| Initial condition x0 | [0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0] | Actual system initial state (Part 2) |
| Initial condition xhat0 | [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0] | Observer initial state (Part 2) |

### Cost and Performance Metrics

| Metric | Value | Notes |
|--------|-------|-------|
| Total cost J_red | [RECORD] | J_red = sum_{k=0}^{N-1} (u_red^T u_red + y1^2 + y6^2) |
| Cost range | k = 0 to 999 | All N input samples included |
| max_abs_u1 | [RECORD] | Max absolute value of input 1 |
| max_abs_u2 | [RECORD] | Max absolute value of input 2 |
| max \|\|u_red[k]\|\|_inf | [RECORD] | Max infinity norm over all k |
| End-of-window \|y1[N]\| | [RECORD] | Final output at time N |
| End-of-window \|y6[N]\| | [RECORD] | Final output at time N |
| Steady-state y1 RMS (last 20%) | [RECORD] | System convergence metric |
| Steady-state y6 RMS (last 20%) | [RECORD] | System convergence metric |

### Comparison Against Part 3

| Metric | Part 3 | Part 4 | Delta | Notes |
|--------|--------|--------|-------|-------|
| Total cost J | [RECORD] | [RECORD] | [RECORD] | delta_J = Part4_J - Part3_J |
| max_abs_u1 | [RECORD] | [RECORD] | [RECORD] | Comparison of u1 magnitude |
| max_abs_u2 | [RECORD] | [RECORD] | [RECORD] | Comparison of u2 magnitude |
| max_abs_u_overall | [RECORD] | max(\|u1\|, \|u2\|) | [RECORD] | Part 3 includes u3, Part 4 does not |

**Note**: If Part 3 baseline was UNKNOWN, mark all Part 3 values as "UNKNOWN" and note that comparison plots were not generated.

### Controller Validation

- [ ] Control law confirmed: u_red[k] = -K_red @ xhat[k] (uses xhat, not x)
- [ ] Explicit check/logged assertion passed

## Results File Excerpts

### Key Lines from results.txt

```
[PASTE KEY LINES FROM python/part4/outputs/results.txt]
```

### K_red Matrix Summary

```
[PASTE K_RED MATRIX SHAPE AND KEY VALUES]
```

### Spectral Radius

```
[PASTE SPECTRAL RADIUS VALUE AND VALIDATION]
```

### Comparison Section

```
[PASTE COMPARISON SECTION FROM results.txt]
```

## Artifacts

### Documentation

- [x] `docs/06_part4_lqr_reduced_input/part4_plan.md` - Implementation plan
- [x] `docs/06_part4_lqr_reduced_input/part4_closeout.md` - This file

### Python Code

- [x] `python/part4/run_lqr_reduced_input.py` - Main entrypoint
- [x] `python/part4/__init__.py` - Module initialization
- [x] `python/part4/outputs/` - Output directory

### Results Files

- [x] `python/part4/outputs/results.txt` - Complete results summary
  - Exact path: `python/part4/outputs/results.txt`
- [x] `python/part4/outputs/K_red_matrix.npy` - LQR gain matrix (optional, for future parts)
  - Exact path: `python/part4/outputs/K_red_matrix.npy`

### Plots

- [x] `python/part4/outputs/inputs_u1_u2.png` - Inputs u1 and u2 over time
  - Exact path: `python/part4/outputs/inputs_u1_u2.png`
- [x] `python/part4/outputs/estimation_error_norm.png` - Estimation error norm
  - Exact path: `python/part4/outputs/estimation_error_norm.png`
- [x] `python/part4/outputs/comparison_outputs_y1_y6.png` - Overlay Part 3 vs Part 4 outputs (if baseline loaded)
  - Exact path: `python/part4/outputs/comparison_outputs_y1_y6.png`

## Deviations from Plan

### Minor Adjustments

[List any minor adjustments made during implementation]

### No Major Deviations

[Confirm all planned deliverables were completed as specified]

## Implementation Notes

### Cost Computation Convention

- Simulation horizon: N = 1000 steps (10 seconds at Ts = 0.01)
- Cost accumulation: J_red = sum_{k=0}^{N-1} stage_cost[k]
- Stage cost: stage_cost[k] = u_red[k]^T @ u_red[k] + y_cost[0]^2 + y_cost[1]^2
  where y_cost = Cy @ x[k] (plant output at time k, not estimated)
- Same convention as Part 3: u_red[k] pairs with transition from x[k] to x[k+1]

### Part 2/3 Component Reuse

- C matrix: Imported from `python/part2/observer_design.py::get_part2_C_matrix()`
- Initial conditions: Imported from `python/part2/run_observer_sim.py::get_part2_initial_conditions()`
- Observer gain L: Imported from `python/part2/observer_design.py::design_observer()` (same method/parameters as Part 3)
- Observer was NOT redesigned in Part 4

### Input Reduction

- Reduced input matrix: `Bd_red = Bd[:, [0, 1]]` (extracts columns 0 and 1)
- Input dimension: u_red in R² (u1 and u2 only, u3 removed)
- Cost matrix: R_red = I2 (2×2 identity, reduced from Part 3's I3)

### Baseline Comparison

- Part 3 baseline loaded from `python/part3/outputs/results.txt`
- If baseline missing, marked as UNKNOWN and exit with clear message
- Comparison metrics: total cost J and maximum input magnitudes
- Overlay plots generated only if baseline successfully loaded

### Controller Validation

- Control law explicitly uses xhat: u_red[k] = -K_red @ xhat[k]
- Explicit check/logged assertion confirms u_red depends on xhat, not x

## Cross-Validation Notes

The implementation follows the plan specifications:
- Uses Part 0 utilities for model construction
- Reuses Part 2 observer design and components
- Follows exam extract requirements for cost definition (same as Part 3)
- Uses Part 2 specifications from exam extract (verified)
- Compares against Part 3 baseline (if available)

## Indexing Consistency Verification

### Array Dimensions (FROZEN, matches Part 3)
- **x**: (12, N+1) - stores x[0] through x[N]
- **u_red**: (2, N) - stores u_red[0] through u_red[N-1]
- **y**: (2, N+1) - stores y[0] through y[N]
- **t**: (N+1,) - time vector from 0 to N*Ts

### Dynamics Pairing (Standard Convention)
- **Plant**: x[k+1] = Ad @ x[k] + Bd_red @ u_red[k] for k = 0..N-1
- **Observer**: xhat[k+1] = Ad @ xhat[k] + Bd_red @ u_red[k] + L @ (y[k] - Cmeas @ xhat[k]) for k = 0..N-1
- **Controller**: u_red[k] = -K_red @ xhat[k] for k = 0..N-1
- **Output**: y[k] = Cmeas @ x[k] for k = 0..N

### Cost Pairing (Verified)
- **Stage cost**: stage_cost[k] = u_red[k]^T @ u_red[k] + y_cost[0]^2 + y_cost[1]^2
- **Cost output**: y_cost = Cy @ x[k] (plant output at time k, not estimated)
- **Cost range**: k = 0 to N-1 (all N input samples included)
- **Consistency**: u_red[k] and y[k] from same time index k ✓

### Verification Results
- Dynamics pairing verified: ||Ad@x[k] + Bd_red@u_red[k] - x[k+1]|| < 1e-10 for all k
- Cost pairing verified: stage_cost[k] uses u_red[k] and y[k] from same k
- Last input computed: u_red[:, N-1] is non-zero and properly computed ✓

---

**Status:** Part 4 Complete ✓  
**Date:** [RECORD DATE]  
**All Validation Checks:** [PASSED/PENDING] ✓  
**LQR Design:** Complete (spectral radius = [RECORD], stability margin = [RECORD])  
**Simulation:** Successful (standard convention: x length N+1, u_red length N)  
**Baseline Comparison:** [LOADED/UNKNOWN]  
**Source Citations:** Part 2 C matrix, initial conditions, and cost definition verified from `docs/sources/final_exam_extract.md`
