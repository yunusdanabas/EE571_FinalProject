# Part 3: Discrete-Time LQR Controller Design with Observer - Plan

## 1. Objective and Scope

### Primary Objectives

1. **Design Discrete-Time Infinite-Horizon LQR Controller**
   - Design LQR controller for the discrete-time system (Ad, Bd) with Ts = 0.01
   - Cost function: J = sum_{k=0}^{N-1} (u[k]^T u[k] + y1[k]^2 + y6[k]^2)
   - Cost output selector: Cy = Part 2 C matrix (2×12, outputs [x1, x6])
   - Cost matrices: Q = Cy^T @ Cy, R = I3
   - Solve discrete-time algebraic Riccati equation (DARE) to compute LQR gain K

2. **Implement Control Law Using Estimated States**
   - Control law: u[k] = -K @ xhat[k] (CRITICAL: uses xhat, not x)
   - Reuse Part 2 observer gain L (do not redesign observer)
   - Reuse Part 2 measurement matrix Cmeas (2×12, measures x1 and x6)
   - Reuse Part 2 initial conditions (x0, xhat0)

3. **Run Closed-Loop Simulation with Observer in the Loop**
   - Plant: x[k+1] = Ad @ x[k] + Bd @ u[k], y[k] = Cmeas @ x[k]
   - Observer: xhat[k+1] = Ad @ xhat[k] + Bd @ u[k] + L @ (y[k] - Cmeas @ xhat[k])
   - Controller: u[k] = -K @ xhat[k]
   - Simulation horizon: N = 1000 steps (10 seconds at Ts = 0.01)
   - No noise (nominal simulation)

4. **Compute Cost and Performance Metrics**
   - Stage cost: stage_cost[k] = u[k]^T @ u[k] + y_cost[0]^2 + y_cost[1]^2
     where y_cost = Cy @ x[k] (or equivalently y_cost = y[k] since Cy = Cmeas)
   - Total cost: J = sum_{k=0}^{N-1} stage_cost[k]
   - Max input magnitudes: max_abs_u1, max_abs_u2, max_abs_u3, max_abs_u_overall
   - Log all metrics to results.txt

5. **Generate Plots and Results**
   - Plot outputs y1 and y6 (displacements x1 and x6) over time
   - Plot all three inputs u1, u2, u3 over time
   - Optional: Plot estimation error norm ||x - xhat|| over time
   - Save all results to python/part3/outputs/

### Scope Boundaries

- **System Configuration**: Discrete-time system (Ad, Bd, Cmeas) with Part 2 sensor matrix
- **Control Design**: LQR only (no other control methods)
- **Observer**: Reuse Part 2 observer design (do not redesign)
- **Simulation Type**: Nominal simulation (deterministic, no process or measurement noise)
- **Cost Definition**: J = sum (u^T u + y1^2 + y6^2) as specified in exam

## 2. Exam Mapping

Source: `docs/sources/final_exam_extract.md` Section 5 (Part 3 Requirement)

**Verified Requirements:**
- Control uses estimated states: u[k] = -K xhat[k] (verified from exam screenshots)
- Cost definition: J = sum (u^T u + y1^2 + y6^2) (verified from exam screenshots)
- Part 2 C matrix: C_part2 measures x1 and x6 (verified from exam screenshots, Question 2)
- Part 2 initial conditions: x0 and xhat0 (verified from exam screenshots, Question 2)
- Sampling time: Ts = 0.01 sec (verified from exam statement)

**Traceability:**
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
  - **Note**: Observer is redesigned using the same method and parameters as Part 2 for consistency

- **Source**: `python/part2/run_observer_sim.py`
  - `get_part2_initial_conditions()`: Returns initial conditions (x0, xhat0) as (12,) arrays
  - **Note**: Initial conditions are carried over from Part 2 (exam specification for Part 2, used in Part 3)

### Cost Definition

- **Cost output selector**: Cy = Part 2 C matrix (2×12, outputs [x1, x6])
- **State weight matrix**: Q = Cy^T @ Cy (12×12, symmetric, positive semidefinite)
- **Input weight matrix**: R = I3 (3×3 identity matrix)

## 4. Implementation Approach

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

### Step B: Define Cost Matrices

1. Define cost output selector:
   ```python
   Cy = Cmeas  # 2×12, outputs [x1, x6]
   ```

2. Define state weight matrix:
   ```python
   Q = Cy.T @ Cy  # 12×12, symmetric
   ```

3. Define input weight matrix:
   ```python
   R = np.eye(3)  # 3×3 identity
   ```

4. Validate cost matrices:
   - Gate: Q is 12×12, symmetric
   - Gate: R is 3×3, symmetric, positive definite
   - Log Q and R shapes and nonzero structure

### Step C: Check Stabilizability

1. Check controllability/stabilizability of (Ad, Bd):
   - Construct controllability matrix or check rank
   - If rank-based check utility exists, use it
   - If not, add local check and log method
   - Gate: Stabilizability check passes

### Step D: Solve DARE and Compute LQR Gain

1. Solve discrete-time algebraic Riccati equation:
   ```python
   from scipy.linalg import solve_discrete_are
   P = solve_discrete_are(Ad, Bd, Q, R)
   ```

2. Compute LQR gain:
   ```python
   K = np.linalg.solve(R + Bd.T @ P @ Bd, Bd.T @ P @ Ad)
   ```

3. Compute closed-loop matrix:
   ```python
   Acl = Ad - Bd @ K
   ```

4. Validate LQR design:
   - Gate: DARE solver succeeds without warnings or NaNs
   - Gate: spectral_radius(Acl) < 1.0 (hard fail if not)
   - Log eigenvalues of Acl and spectral radius

### Step E: Closed-Loop Simulation

1. Initialize simulation:
   ```python
   N = 1000  # Simulation steps
   Ts = 0.01  # Sampling time
   x = np.zeros((12, N))
   xhat = np.zeros((12, N))
   u = np.zeros((3, N))
   y = np.zeros((2, N))
   ```

2. Set initial conditions:
   ```python
   x[:, 0] = x0
   xhat[:, 0] = xhat0
   y[:, 0] = Cmeas @ x[:, 0]
   ```

3. Simulate forward (k = 0 to N-2):
   ```python
   for k in range(N - 1):
       # Controller: u[k] = -K @ xhat[k] (CRITICAL: uses xhat, not x)
       u[:, k] = -K @ xhat[:, k]
       
       # Plant output
       y[:, k] = Cmeas @ x[:, k]
       
       # Plant update
       x[:, k + 1] = Ad @ x[:, k] + Bd @ u[:, k]
       
       # Observer update
       yhat_k = Cmeas @ xhat[:, k]
       xhat[:, k + 1] = Ad @ xhat[:, k] + Bd @ u[:, k] + L @ (y[:, k] - yhat_k)
   ```

4. Final output:
   ```python
   y[:, N - 1] = Cmeas @ x[:, N - 1]
   ```

5. Validate simulation:
   - Gate: Dimensions consistent (x: 12×N, xhat: 12×N, u: 3×N, y: 2×N)
   - Gate: Controller uses xhat, not x (explicit check/logged assertion)
   - Gate: Simulation runs for N steps without divergence
   - Log max absolute state magnitude and max absolute input magnitude

### Step F: Compute Cost Metrics

1. Compute stage cost for each time step:
   ```python
   stage_cost = np.zeros(N)
   for k in range(N):
       y_cost = Cy @ x[:, k]  # or equivalently y_cost = y[:, k]
       stage_cost[k] = u[:, k].T @ u[:, k] + y_cost[0]**2 + y_cost[1]**2
   ```

2. Compute total cost:
   ```python
   J = np.sum(stage_cost)
   ```

3. Compute max input magnitudes:
   ```python
   max_abs_u1 = np.max(np.abs(u[0, :]))
   max_abs_u2 = np.max(np.abs(u[1, :]))
   max_abs_u3 = np.max(np.abs(u[2, :]))
   max_abs_u_overall = np.max(np.abs(u))
   ```

4. Validate cost:
   - Gate: J is finite and non-negative
   - Gate: Metrics logged and written to outputs/results.txt

### Step G: Generate Plots and Save Results

1. Plot outputs y1 and y6:
   - Plot y[0, :] and y[1, :] over time
   - Save to outputs_y1_y6.png

2. Plot inputs u1, u2, u3:
   - Plot u[0, :], u[1, :], u[2, :] over time
   - Save to inputs_u1_u2_u3.png

3. Optional: Plot estimation error norm:
   - Compute e = x - xhat
   - Plot ||e||_2 over time
   - Save to estimation_error_norm.png

4. Save results to results.txt:
   - N, Ts, initial conditions
   - Cy and R definitions summary
   - K matrix (shape and values)
   - spectral_radius(Ad - Bd @ K)
   - Total cost J and max input magnitudes
   - Statement that control uses xhat

## 5. Commands to Run

```bash
python python/part3/run_lqr_with_observer.py
```

## 6. Validation Checklist

### 6.1 Cost Matrix Gates

- [ ] Q is 12×12, symmetric
- [ ] R is 3×3, symmetric, positive definite
- [ ] Q and R shapes and nonzero structure logged

### 6.2 LQR Design Gates

- [ ] (Ad, Bd) stabilizability check passes (rank check)
- [ ] DARE solver succeeds without warnings or NaNs
- [ ] spectral_radius(Ad - Bd @ K) < 1.0 (hard fail if not)
- [ ] Eigenvalues of Acl and spectral radius logged

### 6.3 Simulation Gates

- [ ] Dimensions consistent: x (12,), xhat (12,), u (3,), y (2,)
- [ ] Controller uses xhat, not x (explicit check/logged assertion)
- [ ] Simulation runs for N steps without divergence
- [ ] Max absolute state magnitude and max absolute input magnitude logged

### 6.4 Cost Gates

- [ ] J is finite and non-negative
- [ ] Metrics logged and written to outputs/results.txt

### 6.5 Artifact Gates

- [ ] results.txt created with all required information
- [ ] outputs_y1_y6.png created
- [ ] inputs_u1_u2_u3.png created
- [ ] Optional: estimation_error_norm.png created

## 7. Expected Artifacts

### Results File (`python/part3/outputs/results.txt`)

Must include:
- N, Ts, initial conditions used
- Cy and R definitions summary
- K matrix (shape and values)
- spectral_radius(Ad - Bd @ K)
- Total cost J and max input magnitudes
- Statement that control uses xhat

### Plots

- `outputs_y1_y6.png`: x1 and x6 over time (or y[0], y[1])
- `inputs_u1_u2_u3.png`: All three inputs over time
- `estimation_error_norm.png` (optional): ||x - xhat|| over time

## 8. Non-Goals

Part 3 explicitly does NOT include:

- **Observer Redesign**: Reuse Part 2 observer (do not redesign)
- **Noise Models**: No process noise, no measurement noise (Part 5)
- **Kalman Filter**: No stochastic estimation (Part 5)
- **Reduced Input Analysis**: No analysis of systems with fewer than 3 inputs (Part 4)
- **Alternative Control Methods**: LQR only (no other control designs)

## 9. Simulation Convention

- **Simulation horizon**: N = 1000 steps (10 seconds at Ts = 0.01)
- **Cost accumulation**: J = sum_{k=0}^{N-1} stage_cost[k]
- **Time indexing**: k = 0, 1, ..., N-1
- **Document convention**: Log in results.txt

## 10. Critical Requirements

1. **Control Law Must Use xhat**: u[k] = -K @ xhat[k] (not x[k])
   - Implement explicit check/logged assertion
   - Fail gate if u is computed from x

2. **Reuse Part 2 Components**: Do not redesign observer or duplicate Part 2 code
   - Import from existing Part 2 modules
   - Use Part 2 C matrix, initial conditions, and observer gain

3. **Cost Definition**: J = sum (u^T u + y1^2 + y6^2)
   - Use Cy = Part 2 C matrix (outputs [x1, x6])
   - Use actual plant outputs y[k] = Cmeas @ x[k] for cost accumulation
