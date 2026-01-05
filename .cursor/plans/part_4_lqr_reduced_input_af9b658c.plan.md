---
name: Part 4 LQR Reduced Input
overview: "Implement Part 4: redesign discrete-time infinite-horizon LQR with u3 removed (Bd_red = Bd[:, [0,1]]), using same cost as Part 3. Compare total cost J and maximum input magnitude against Part 3 baseline."
todos:
  - id: create_docs
    content: "Create documentation files: docs/06_part4_lqr_reduced_input/plan.md and closeout.md with objective, exam mapping, input removal convention, commands, gates, and comparison protocol"
    status: completed
  - id: create_part4_structure
    content: Create python/part4/ directory structure with __init__.py and outputs/ directory
    status: completed
  - id: implement_model_loading
    content: "Implement model loading: build/discretize model, get Cmeas, get x0/xhat0, design observer (reuse Part 2/3 approach)"
    status: completed
    dependencies:
      - create_part4_structure
  - id: implement_input_reduction
    content: "Implement input reduction: Bd_red = Bd[:, [0,1]], validate shape, define R_red = I2"
    status: completed
    dependencies:
      - implement_model_loading
  - id: implement_cost_matrices
    content: "Define cost matrices: Cy = Cmeas, Q = Cy.T @ Cy, R_red = I2, validate symmetry and positive definiteness"
    status: completed
    dependencies:
      - implement_input_reduction
  - id: implement_stabilizability_checks
    content: Implement PBH stabilizability check for (Ad, Bd_red) and detectability check for (Ad, Cy) with proper gates
    status: completed
    dependencies:
      - implement_cost_matrices
  - id: implement_lqr_design
    content: Solve DARE for reduced system, compute K_red, validate spectral radius < 1.0, log eigenvalues
    status: completed
    dependencies:
      - implement_stabilizability_checks
  - id: implement_baseline_loading
    content: Load Part 3 baseline metrics from python/part3/outputs/results.txt, handle missing baseline gracefully with UNKNOWN markers
    status: completed
    dependencies:
      - implement_lqr_design
  - id: implement_simulation
    content: "Implement closed-loop simulation with observer: standard convention (x length N+1, u_red length N), controller uses xhat, validate dimensions"
    status: completed
    dependencies:
      - implement_baseline_loading
  - id: implement_cost_computation
    content: "Compute cost metrics: stage_cost, J_red, max inputs (u1, u2), validate finite and non-negative"
    status: completed
    dependencies:
      - implement_simulation
  - id: implement_comparison
    content: Create comparison section in results.txt (Part3 vs Part4), generate overlay plot for y1/y6 if baseline loaded
    status: completed
    dependencies:
      - implement_cost_computation
  - id: implement_plotting
    content: "Generate all plots: inputs_u1_u2.png, estimation_error_norm.png, comparison_outputs_y1_y6.png (if baseline available)"
    status: completed
    dependencies:
      - implement_comparison
  - id: implement_results_logging
    content: Write comprehensive results.txt with all conventions, K_red, rho(Acl_red), J_red, max inputs, end-of-window metrics, comparison section
    status: completed
    dependencies:
      - implement_plotting
---

# Part 4: Discrete-Time LQR with Reduced Input (u3 Removed) - Implementation Plan

## Overview

Redesign discrete-time infinite-horizon LQR when the third input u3 is removed. Use the same nominal cost as Part 3 and compare total cost J and maximum input magnitude required against Part 3.

## Key Changes from Part 3

1. **Input Reduction**: `Bd_red = Bd[:, [0,1]] `(12×2), `u_red` in R²
2. **Cost Matrix**: `R_red = I2` (2×2) instead of `R = I3`
3. **LQR Gain**: `K_red` shape (2, 12) instead of (3, 12)
4. **Baseline Comparison**: Load Part 3 metrics from `python/part3/outputs/results.txt` and compare

## Implementation Structure

### A. Documentation (`docs/06_part4_lqr_reduced_input/`)

1. **plan.md**: Objective, exam mapping, input removal convention, commands, gates, comparison protocol
2. **closeout.md**: Environment, gate checklist, results tables, comparison table, artifact checklist

### B. Code (`python/part4/`)

1. **run_lqr_reduced_input.py**: Single entrypoint with all logic
2. **outputs/**: Directory for artifacts (results.txt, plots, matrices)

## Detailed Implementation Steps

### Step 1: Load Model and Part 2 Components

- Reuse from Part 3:
  - `build_continuous_model()`, `discretize_zoh()` from `python/utils/build_model.py`
  - `get_part2_C_matrix()` from `python/part2/observer_design.py`
  - `get_part2_initial_conditions()` from `python/part2/run_observer_sim.py`
  - `design_observer()` from `python/part2/observer_design.py` (same method/parameters as Part 3)

### Step 2: Create Reduced Input Matrix

```python
Bd_red = Bd[:, [0, 1]]  # Extract columns 0 and 1 (u1 and u2)
```

**Gate**: `Bd_red.shape == (12, 2)`

### Step 3: Define Cost Matrices (Same as Part 3)

- `Cy = Cmeas` (2×12, outputs [x1, x6])
- `Q = Cy.T @ Cy` (12×12)
- `R_red = np.eye(2)` (2×2)

**Gates**:

- Q symmetric within tolerance, shape (12, 12)
- R_red symmetric within tolerance, positive definite (log min eigenvalue)
- Log nonzero structure of Q

### Step 4: Check Stabilizability and Detectability

- **Stabilizability**: PBH for `(Ad, Bd_red)` - check `rank([λI - Ad, Bd_red]) == n `for `|λ| >= 1 - tol`
- **Detectability**: PBH for `(Ad, Cy)` - check `rank([λI - Ad.T, Cy.T]) == n `for `|λ| >= 1 - tol`

**Gates**:

- Stabilizability PASS for `Bd_red` (hard fail with logged offending eigenvalues if not)
- Detectability PASS (hard fail if not)

### Step 5: Solve DARE and Compute LQR Gain

```python
P_red = solve_discrete_are(Ad, Bd_red, Q, R_red)
K_red = np.linalg.solve(R_red + Bd_red.T @ P_red @ Bd_red, Bd_red.T @ P_red @ Ad)
Acl_red = Ad - Bd_red @ K_red
```

**Gates**:

- DARE solver succeeds, `P_red` finite
- `spectral_radius(Acl_red) < 1.0` (hard fail otherwise)
- Log dominant eigenvalue magnitude, angle, and margin `1 - rho`

### Step 6: Load Part 3 Baseline (if available)

```python
# Attempt to load from python/part3/outputs/results.txt
# Parse: Part3_J, Part3_max_inputs
# If missing: mark baseline as UNKNOWN and exit with clear message
```

**Gate**: If baseline missing, print/log "baseline UNKNOWN" and exit with instruction to run Part 3 first

### Step 7: Closed-Loop Simulation with Observer

**Standard convention** (same as Part 3):

- `x`, `xhat`, `y` have length `N+1`
- `u_red` has length `N`

**Dynamics**:

- Controller: `u_red[k] = -K_red @ xhat[k]`
- Plant: `x[k+1] = Ad @ x[k] + Bd_red @ u_red[k]`, `y[k] = Cmeas @ x[k]`
- Observer: `xhat[k+1] = Ad @ xhat[k] + Bd_red @ u_red[k] + L @ (y[k] - Cmeas @ xhat[k])`
- Compute `y[N]` at the end

**Gates**:

- Dimension checks at runtime
- Controller uses `xhat` (include diagnostic similar to Part 3)
- No divergence (log max `|x|`, max `||x - xhat||`)

### Step 8: Compute Cost Metrics

**Stage cost** (k = 0..N-1):

```python
stage_cost[k] = u_red[k]^T @ u_red[k] + (Cy @ x[k])[0]^2 + (Cy @ x[k])[1]^2
```

**Total**: `J_red = sum_k stage_cost[k]`

**Max inputs**:

- `max_abs_u1`, `max_abs_u2`
- `max ||u_red[k]||_inf`

**Gates**:

- `J_red` finite, non-negative
- Metrics logged to `outputs/results.txt`

### Step 9: Comparison Against Part 3

**If baseline loaded**:

- Create comparison section in Part 4 `results.txt`:
  - `Part3_J`, `Part4_J`, `delta_J`
  - `Part3_max_inputs` vs `Part4_max_inputs`
- Create overlay plot for y1 and y6: Part 3 vs Part 4

**If baseline UNKNOWN**:

- Do not create overlay plots
- Log and exit

### Step 10: Generate Artifacts

**In `python/part4/outputs/`**:

1. **results.txt**: Conventions, `K_red`, `rho(Acl_red)`, `J_red`, max inputs, end-of-window metrics, comparison section
2. **comparison_outputs_y1_y6.png**: Overlay Part 3 vs Part 4 (only if baseline loaded)
3. **inputs_u1_u2.png**: Part 4 inputs (u1, u2 only)
4. **estimation_error_norm.png**: Part 4 estimation error

## Key Files to Create/Modify

1. `docs/06_part4_lqr_reduced_input/plan.md` - Implementation plan
2. `docs/06_part4_lqr_reduced_input/closeout.md` - Closeout template
3. `python/part4/run_lqr_reduced_input.py` - Main implementation
4. `python/part4/__init__.py` - Module initialization
5. `python/part4/outputs/` - Output directory (created automatically)

## Critical Requirements

1. **Input Removal Convention**: `Bd_red = Bd[:, [0,1]]`, `u_red` in R²
2. **Same Cost Definition**: `Q = Cy.T @ Cy`, `R_red = I2` (same structure as Part 3)
3. **Baseline Loading**: Must handle missing Part 3 baseline gracefully
4. **Observer Reuse**: Same observer structure and gain policy as Part 3 (no redesign)
5. **Standard Convention**: Same N, Ts, x0, xhat0, cost indexing as Part 3

## Validation Gates Summary

- Cost matrices: Q symmetric, R_red positive definite
- Stabilizability: PASS for `Bd_red`
- Detectability: PASS for `(Ad, Cy)`
- LQR design: DARE succeeds, `rho(Acl_red) < 1.0`
- Simulation: Dimensions correct, controller uses `xhat`, no divergence
- Cost: `J_red` finite, non-negative
- Baseline: Loaded or marked UNKNOWN with clear message

## Commands

```bash
python python/part4/run_lqr_reduced_input.py
```

## Expected Artifacts

- `python/part4/outputs/results.txt`
- `python/part4/outputs/comparison_outputs_y1_y6.png` (if baseline loaded)
- `python/part4/outputs/inputs_u1_u2.png`
- `python/part4/outputs/estimation_error_norm.png`
- `python/part4/outputs/K_red_matrix.npy` (optional, for future parts)