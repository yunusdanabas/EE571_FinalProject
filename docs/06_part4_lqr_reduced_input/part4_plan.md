# Part 4: Discrete-Time LQR with Reduced Input (u3 Removed) - Plan

## 1. Objective and Scope

### Primary Objectives

1. **Redesign Discrete-Time Infinite-Horizon LQR Controller with Reduced Input**
   - Remove third input u3: `Bd_red = Bd[:, [0,1]]` (12×2), `u_red` in R²
   - Use same nominal cost as Part 3: J = sum_{k=0}^{N-1} (u[k]^T u[k] + y1[k]^2 + y6[k]^2)
   - Cost output selector: Cy = Part 2 C matrix (2×12, outputs [x1, x6])
   - Cost matrices: Q = Cy^T @ Cy (same as Part 3), R_red = I2 (2×2 instead of I3)
   - Solve discrete-time algebraic Riccati equation (DARE) for reduced system

2. **Compare Against Part 3 Baseline**
   - Load Part 3 metrics from `python/part3/outputs/results.txt`
   - Compare total cost J: Part3_J vs Part4_J, compute delta_J
   - Compare maximum input magnitudes: Part3_max_inputs vs Part4_max_inputs
   - Generate overlay plots for y1 and y6: Part 3 vs Part 4

3. **Reuse Part 2/3 Components**
   - Reuse Part 2 observer gain L (same design method/parameters as Part 3)
   - Reuse Part 2 measurement matrix Cmeas (2×12, measures x1 and x6)
   - Reuse Part 2 initial conditions (x0, xhat0)
   - Use same simulation parameters: N = 1000, Ts = 0.01

4. **Run Closed-Loop Simulation with Observer**
   - Plant: x[k+1] = Ad @ x[k] + Bd_red @ u_red[k], y[k] = Cmeas @ x[k]
   - Observer: xhat[k+1] = Ad @ xhat[k] + Bd_red @ u_red[k] + L @ (y[k] - Cmeas @ xhat[k])
   - Controller: u_red[k] = -K_red @ xhat[k] (uses xhat, not x)
   - Standard convention: x, xhat, y have length N+1; u_red has length N

5. **Compute Cost and Performance Metrics**
   - Stage cost: stage_cost[k] = u_red[k]^T @ u_red[k] + y_cost[0]^2 + y_cost[1]^2
     where y_cost = Cy @ x[k] (plant output at time k)
   - Total cost: J_red = sum_{k=0}^{N-1} stage_cost[k]
   - Max input magnitudes: max_abs_u1, max_abs_u2, max ||u_red[k]||_inf
   - Log all metrics to results.txt with comparison section

### Scope Boundaries

- **System Configuration**: Discrete-time system (Ad, Bd_red, Cmeas) with reduced input (u1, u2 only)
- **Control Design**: LQR only (same method as Part 3, but with reduced input)
- **Observer**: Reuse Part 2 observer design (do not redesign)
- **Simulation Type**: Nominal simulation (deterministic, no process or measurement noise)
- **Cost Definition**: Same as Part 3: J = sum (u^T u + y1^2 + y6^2)
- **Baseline Dependency**: Requires Part 3 results.txt for comparison (gracefully handles missing baseline)

## 2. Exam Mapping

Source: `docs/sources/final_exam_extract.md` Section 6 (Part 4 Requirement)

**Verified Requirements:**
- Remove third input: u3 is removed, system uses only u1 and u2
- Same nominal cost: J = sum (u^T u + y1^2 + y6^2) as Part 3
- Compare against Part 3: total cost J and maximum input magnitude
- Part 2 C matrix: C_part2 measures x1 and x6 (verified from exam screenshots, Question 2)
- Part 2 initial conditions: x0 and xhat0 (verified from exam screenshots, Question 2)
- Sampling time: Ts = 0.01 sec (verified from exam statement)

**Traceability:**
- Part 4 requirement: verified from exam screenshots, Question 4
- Part 3 cost definition: verified from exam screenshots, Question 3
- Part 2 C and initial conditions: verified from exam screenshots, Question 2

## 3. Required Inputs

### From Part 0 Utilities

- **Source**: `python/utils/build_model.py`
  - `A, B, C = build_continuous_model()`: Continuous-time matrices from `matlab/prep_final.m`
  - `Ad, Bd, Cd, Dd = discretize_zoh(A, B, C, Ts=0.01)`: Discrete-time matrices via zero-order hold
  - `Ad` (12×12), `Bd` (12×3), `Ts = 0.01` seconds

### From Part 2 (Reused Components)

- **Source**: `python/part2/observer_design.py`
  - `get_part2_C_matrix()`: Returns Part 2 measurement matrix Cmeas (2×12, measures x1 and x6)
  - `design_observer(Ad, Cmeas, ...)`: Returns observer gain L (12×2) and design info
  - **Note**: Observer is redesigned using the same method and parameters as Part 3 for consistency

- **Source**: `python/part2/run_observer_sim.py`
  - `get_part2_initial_conditions()`: Returns initial conditions (x0, xhat0) as (12,) arrays
  - **Note**: Initial conditions are carried over from Part 2 (exam specification)

### From Part 3 (Baseline Metrics)

- **Source**: `python/part3/outputs/results.txt`
  - Parse Part3_J (total cost)
  - Parse Part3_max_inputs (max_abs_u1, max_abs_u2, max_abs_u3, max_abs_u_overall)
  - **Note**: If baseline missing, mark as UNKNOWN and exit with clear message

### Cost Definition

- **Cost output selector**: Cy = Part 2 C matrix (2×12, outputs [x1, x6])
- **State weight matrix**: Q = Cy^T @ Cy (12×12, symmetric, positive semidefinite) - same as Part 3
- **Input weight matrix**: R_red = I2 (2×2 identity matrix) - reduced from Part 3's I3

## 4. Input Removal Convention

**Reduced Input Matrix:**
```python
Bd_red = Bd[:, [0, 1]]  # Extract columns 0 and 1 (u1 and u2)
```

- `Bd_red` shape: (12, 2)
- `u_red` shape: (2, N) for simulation
- Optional full input representation: `u_full = [u1, u2, 0]` for comparison plots (not used in dynamics)

**Gate**: `Bd_red.shape == (12, 2)`

## 5. Implementation Approach

### Step A: Load Model and Part 2 Components

1. Load continuous model and discretize:
   ```python
   from utils.build_model import build_continuous_model, discretize_zoh
   A, B, C = build_continuous_model()
   Ad, Bd, Cd, Dd = discretize_zoh(A, B, C, Ts=0.01)
   ```

2. Import Part 2 components:
   ```python
   from part2.observer_design import get_part2_C_matrix, design_observer
   from part2.run_observer_sim import get_part2_initial_conditions
   
   Cmeas = get_part2_C_matrix()  # 2×12, measures x1 and x6
   x0, xhat0 = get_part2_initial_conditions()  # Initial conditions
   L, design_info = design_observer(Ad, Cmeas, method='pole_placement', ...)  # Observer gain
   ```

### Step B: Create Reduced Input Matrix

1. Extract reduced input matrix:
   ```python
   Bd_red = Bd[:, [0, 1]]  # Extract columns 0 and 1 (u1 and u2)
   ```

2. Validate shape:
   - Gate: `Bd_red.shape == (12, 2)`

### Step C: Define Cost Matrices

1. Define cost output selector:
   ```python
   Cy = Cmeas  # 2×12, outputs [x1, x6]
   ```

2. Define state weight matrix (same as Part 3):
   ```python
   Q = Cy.T @ Cy  # 12×12, symmetric
   ```

3. Define input weight matrix (reduced from Part 3):
   ```python
   R_red = np.eye(2)  # 2×2 identity
   ```

4. Validate cost matrices:
   - Gate: Q is 12×12, symmetric
   - Gate: R_red is 2×2, symmetric, positive definite
   - Log Q and R_red shapes and nonzero structure

### Step D: Check Stabilizability and Detectability

1. Check stabilizability of (Ad, Bd_red):
   - Implement PBH rank condition: for eigenvalues with |λ| >= 1 - tol, check rank([λI - Ad, Bd_red]) == n
   - Gate: Stabilizability PASS for Bd_red (hard fail with logged offending eigenvalues if not)

2. Check detectability of (Ad, Cy):
   - Implement PBH rank condition: check stabilizability of (Ad.T, Cy.T) equivalently
   - Gate: Detectability PASS (hard fail if not)

### Step E: Solve DARE and Compute LQR Gain

1. Solve discrete-time algebraic Riccati equation:
   ```python
   from scipy.linalg import solve_discrete_are
   P_red = solve_discrete_are(Ad, Bd_red, Q, R_red)
   ```

2. Compute LQR gain:
   ```python
   K_red = np.linalg.solve(R_red + Bd_red.T @ P_red @ Bd_red, Bd_red.T @ P_red @ Ad)
   ```

3. Compute closed-loop matrix:
   ```python
   Acl_red = Ad - Bd_red @ K_red
   ```

4. Validate LQR design:
   - Gate: DARE solver succeeds without warnings or NaNs
   - Gate: spectral_radius(Acl_red) < 1.0 (hard fail if not)
   - Log eigenvalues of Acl_red, spectral radius, dominant eigenvalue magnitude and angle, margin 1 - rho

### Step F: Load Part 3 Baseline

1. Attempt to load baseline metrics:
   ```python
   # Parse python/part3/outputs/results.txt
   # Extract: Part3_J, Part3_max_inputs
   ```

2. Handle missing baseline:
   - If baseline missing: print/log "baseline UNKNOWN" and exit with clear message instructing to run Part 3 first
   - Gate: If baseline missing, do not create overlay plots, log and exit

### Step G: Closed-Loop Simulation

1. Initialize simulation:
   ```python
   N = 1000  # Simulation steps
   Ts = 0.01  # Sampling time
   x = np.zeros((12, N + 1))  # Standard convention: length N+1
   xhat = np.zeros((12, N + 1))
   u_red = np.zeros((2, N))  # Standard convention: length N
   y = np.zeros((2, N + 1))
   ```

2. Set initial conditions:
   ```python
   x[:, 0] = x0
   xhat[:, 0] = xhat0
   y[:, 0] = Cmeas @ x[:, 0]
   ```

3. Simulate forward (k = 0 to N-1):
   ```python
   for k in range(N):
       # Controller: u_red[k] = -K_red @ xhat[k] (CRITICAL: uses xhat, not x)
       u_red[:, k] = -K_red @ xhat[:, k]
       
       # Plant output at time k
       y[:, k] = Cmeas @ x[:, k]
       
       # Plant update
       x[:, k + 1] = Ad @ x[:, k] + Bd_red @ u_red[:, k]
       
       # Observer update
       yhat_k = Cmeas @ xhat[:, k]
       y_error = y[:, k] - yhat_k
       xhat[:, k + 1] = Ad @ xhat[:, k] + Bd_red @ u_red[:, k] + L @ y_error
       
       # Estimation error
       e[:, k + 1] = x[:, k + 1] - xhat[:, k + 1]
   ```

4. Final output:
   ```python
   y[:, N] = Cmeas @ x[:, N]
   ```

5. Validate simulation:
   - Gate: Dimensions consistent (x: 12×N+1, xhat: 12×N+1, u_red: 2×N, y: 2×N+1)
   - Gate: Controller uses xhat, not x (explicit check/logged assertion)
   - Gate: Simulation runs for N steps without divergence
   - Log max absolute state magnitude, max absolute input magnitude, max estimation error norm

### Step H: Compute Cost Metrics

1. Compute stage cost for each time step:
   ```python
   stage_cost = np.zeros(N)
   for k in range(N):
       y_cost = Cy @ x[:, k]  # Plant output at time k
       stage_cost[k] = u_red[:, k].T @ u_red[:, k] + y_cost[0]**2 + y_cost[1]**2
   ```

2. Compute total cost:
   ```python
   J_red = np.sum(stage_cost)
   ```

3. Compute max input magnitudes:
   ```python
   max_abs_u1 = np.max(np.abs(u_red[0, :]))
   max_abs_u2 = np.max(np.abs(u_red[1, :]))
   max_u_inf = np.max(np.max(np.abs(u_red), axis=0))  # max ||u_red[k]||_inf
   ```

4. Validate cost:
   - Gate: J_red is finite and non-negative
   - Gate: Metrics logged and written to outputs/results.txt

### Step I: Comparison Against Part 3

1. If baseline loaded:
   - Create comparison section in Part 4 results.txt:
     - Part3_J, Part4_J, delta_J
     - Part3_max_inputs vs Part4_max_inputs
   - Create overlay plot for y1 and y6: Part 3 vs Part 4
     - Load Part 3 trajectories from saved data or recompute (prefer loading if available)

2. If baseline UNKNOWN:
   - Do not create overlay plots
   - Log "baseline UNKNOWN" in results.txt
   - Exit with clear message

### Step J: Generate Plots and Save Results

1. Plot inputs u1 and u2:
   - Plot u_red[0, :] and u_red[1, :] over time
   - Save to inputs_u1_u2.png

2. Plot estimation error norm:
   - Compute e = x - xhat
   - Plot ||e||_2 over time
   - Save to estimation_error_norm.png

3. Plot comparison outputs (if baseline loaded):
   - Overlay plot: Part 3 y1 and y6 vs Part 4 y1 and y6
   - Save to comparison_outputs_y1_y6.png

4. Save results to results.txt:
   - N, Ts, initial conditions
   - Cy and R_red definitions summary
   - K_red matrix (shape and values)
   - spectral_radius(Ad - Bd_red @ K_red)
   - Total cost J_red and max input magnitudes
   - Comparison section (Part3 vs Part4) if baseline loaded
   - Statement that control uses xhat

## 6. Commands to Run

```bash
python python/part4/run_lqr_reduced_input.py
```

## 7. Validation Checklist

### 7.1 Cost Matrix Gates

- [ ] Q is 12×12, symmetric (same as Part 3)
- [ ] R_red is 2×2, symmetric, positive definite
- [ ] Q and R_red shapes and nonzero structure logged

### 7.2 Stabilizability and Detectability Gates

- [ ] (Ad, Bd_red) stabilizability check passes (PBH rank condition)
- [ ] (Ad, Cy) detectability check passes (PBH rank condition)

### 7.3 LQR Design Gates

- [ ] DARE solver succeeds without warnings or NaNs
- [ ] spectral_radius(Ad - Bd_red @ K_red) < 1.0 (hard fail if not)
- [ ] Eigenvalues of Acl_red and spectral radius logged
- [ ] Dominant eigenvalue magnitude, angle, and margin logged

### 7.4 Baseline Loading Gates

- [ ] Part 3 baseline loaded from results.txt (or marked UNKNOWN with clear message)
- [ ] If baseline UNKNOWN, exit gracefully without creating overlay plots

### 7.5 Simulation Gates

- [ ] Dimensions consistent: x (12, N+1), xhat (12, N+1), u_red (2, N), y (2, N+1)
- [ ] Controller uses xhat, not x (explicit check/logged assertion)
- [ ] Simulation runs for N steps without divergence
- [ ] Max absolute state magnitude and max absolute input magnitude logged

### 7.6 Cost Gates

- [ ] J_red is finite and non-negative
- [ ] Metrics logged and written to outputs/results.txt
- [ ] Comparison section included if baseline loaded

### 7.7 Artifact Gates

- [ ] results.txt created with all required information
- [ ] inputs_u1_u2.png created
- [ ] estimation_error_norm.png created
- [ ] comparison_outputs_y1_y6.png created (only if baseline loaded)

## 8. Expected Artifacts

### Results File (`python/part4/outputs/results.txt`)

Must include:
- N, Ts, initial conditions used
- Cy and R_red definitions summary
- K_red matrix (shape and values)
- spectral_radius(Ad - Bd_red @ K_red)
- Total cost J_red and max input magnitudes
- Comparison section: Part3_J, Part4_J, delta_J, Part3_max_inputs vs Part4_max_inputs (if baseline loaded)
- Statement that control uses xhat

### Plots

- `inputs_u1_u2.png`: Inputs u1 and u2 over time
- `estimation_error_norm.png`: ||x - xhat|| over time
- `comparison_outputs_y1_y6.png` (if baseline loaded): Overlay Part 3 vs Part 4 outputs

## 9. Comparison Protocol

**Same Parameters as Part 3:**
- N = 1000 (number of input samples)
- Ts = 0.01 s (sampling time)
- x0, xhat0 from Part 2 (exam specification)
- Cost indexing convention: J = sum from k=0 to N-1

**Baseline Loading:**
- Load Part 3 metrics from `python/part3/outputs/results.txt`
- If missing, mark baseline as UNKNOWN and require running Part 3 first
- Comparison metrics:
  - Total cost: Part3_J vs Part4_J, delta_J = Part4_J - Part3_J
  - Max inputs: Part3_max_inputs vs Part4_max_inputs (u1, u2 only for Part 4)

## 10. Non-Goals

Part 4 explicitly does NOT include:

- **Observer Redesign**: Reuse Part 2 observer (do not redesign)
- **Noise Models**: No process noise, no measurement noise (Part 5)
- **Kalman Filter**: No stochastic estimation (Part 5)
- **Additional Inputs**: Only u1 and u2 (u3 removed)
- **Alternative Control Methods**: LQR only (same method as Part 3)

## 11. Critical Requirements

1. **Input Removal Convention**: `Bd_red = Bd[:, [0,1]]`, `u_red` in R²
2. **Same Cost Definition**: Q = Cy^T @ Cy (same as Part 3), R_red = I2 (reduced from I3)
3. **Baseline Loading**: Must handle missing Part 3 baseline gracefully
4. **Observer Reuse**: Same observer structure and gain policy as Part 3 (no redesign)
5. **Standard Convention**: Same N, Ts, x0, xhat0, cost indexing as Part 3
