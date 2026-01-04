---
name: Part 2 Observer Design Plan
overview: Design and implement a discrete-time Luenberger observer for the 6-mass spring chain system with augmented sensor matrix (measuring x1 and x6). Plan includes observability verification, observer design via pole placement or dual LQR, and nominal simulation comparing true vs estimated states.
todos:
  - id: verify_pdf_specs
    content: "Verify Part 2 specifications from final_exam.pdf: Cd_new matrix (2×12), initial conditions (x0, xhat0), simulation parameters (N, u[k]), and observer design requirements. Document page/section citations in plan."
    status: completed
  - id: implement_observability_check
    content: Implement observability rank computation for (Ad, Cd_new) in observer_design.py, reusing Part 1 SVD policy. Verify rank = 12 and save results.
    status: completed
    dependencies:
      - verify_pdf_specs
  - id: implement_observer_design
    content: Implement observer gain computation in observer_design.py using chosen method (pole placement or dual LQR). Document design parameters and compute L (12×2).
    status: completed
    dependencies:
      - verify_pdf_specs
      - implement_observability_check
  - id: implement_coupled_simulation
    content: Implement coupled plant-observer simulation in run_observer_sim.py. Use Part 2 initial conditions, shared input u[k], and compute trajectories x, xhat, y, yhat.
    status: completed
    dependencies:
      - implement_observer_design
  - id: implement_plotting_metrics
    content: Generate plots (outputs comparison, estimation errors) and compute RMS error metrics in run_observer_sim.py. Save all outputs to python/part2/outputs/.
    status: completed
    dependencies:
      - implement_coupled_simulation
  - id: validate_results
    content: "Run validation checklist: dimension checks, observability rank, observer stability, simulation correctness, error convergence, and plot accuracy."
    status: completed
    dependencies:
      - implement_plotting_metrics
  - id: create_closeout
    content: Create docs/03_part2_observer/closeout.md with results summary, key findings, plots, metrics, and any deviations from plan.
    status: completed
    dependencies:
      - validate_results
  - id: update_anchor_if_needed
    content: Update docs/00_anchor.md only if mismatches discovered in Part 2 Cd_new or initial conditions relative to current anchor document.
    status: completed
    dependencies:
      - verify_pdf_specs
---

# Pa

rt 2: Observer Design with Augmented Sensor Matrix - Plan

## 1. Objective and Scope

### Primary Objectives

1. **Replace Baseline Sensor Configuration**

- Replace baseline `Cd` from Part 1 (measuring only `x1`) with Part 2 sensor matrix from `docs/sources/final_exam.pdf`
- Part 2 `Cd_new` should measure `x1` and `x6` (displacements of masses 1 and 6)
- Verify exact `Cd_new` matrix from the PDF (expected 2×12):
     ```javascript
                    Cd_new = [1 0 0 0 0 0 0 0 0 0 0 0;
                              0 0 0 0 0 1 0 0 0 0 0 0]
     ```




- **Source verification required**: Confirm exact `Cd_new` from `docs/sources/final_exam.pdf` (cite page/section in plan)

2. **Verify Observability of Augmented System**

- Compute observability rank of `(Ad, Cd_new)` using the same SVD policy as Part 1
- Verify rank equals 12 (full observability) or document rank deficiency if present
- Document observability analysis results in output files

3. **Design Discrete-Time Luenberger Observer**

- Observer structure:
     ```javascript
                    xhat[k+1] = Ad @ xhat[k] + Bd @ u[k] + L @ (y[k] - yhat[k])
                    yhat[k] = Cd_new @ xhat[k]
     ```




- Design observer gain `L` (12×2) using one of the following methods:
    - **Option 1**: Pole placement via dual system `(Ad^T, Cd_new^T)`, then transpose result
    - **Option 2**: Dual LQR approach (if allowed by course conventions)
- Record chosen design method and rationale in `observer_design.py`
- **Pole selection policy**: Document intended pole placement strategy (conservative relative to plant poles) but do NOT choose specific numeric poles unless explicitly specified in `final_exam.pdf`

4. **Run Nominal (No-Noise) Simulation**

- Simulate coupled system: true plant and observer running in parallel
- Use Part 2 initial conditions from `docs/sources/final_exam.pdf`:
    - Actual state: `x0` (to be verified from PDF, preliminary: `[0; 0; 0; 1; 1; 1; 0; 0; 0; 0; 0; 0]`)
    - Observer initial state: `xhat0` (to be verified from PDF, preliminary: `[0; 0; 0; 0; 0; 1; 0; 0; 0; 0; 0; 0]`)
- **Input signal**: If not specified in exam, default to `u[k] = 0 `for all `k` and document this assumption
- **Simulation horizon**: Use `N = 1000` steps (10 seconds at `Ts = 0.01`) unless exam specifies otherwise
- Compare true states `x[k]` vs estimated states `xhat[k]` over simulation horizon

5. **Generate Plots and Metrics**

- Plot measured outputs `y1` and `y6` and their estimates `yhat1` and `yhat6`
- Plot state estimation errors for at least displacements `x1..x6` (errors: `x_true - xhat`)
- Compute RMS estimation error metrics for displacement states (positions 1-6)
- Save all plots to `python/part2/outputs/`

### Scope Boundaries

- **System Configuration**: Discrete-time system `(Ad, Bd, Cd_new)` with `Cd_new` measuring `x1` and `x6`
- **Design Focus**: Observer design only (no LQR controller, no noise models, no Kalman filter)
- **Simulation Type**: Nominal simulation (deterministic, no process or measurement noise)
- **Tuning**: Document observer design method and intended pole policy, but defer specific pole selection unless exam specifies

## 2. Required Inputs

### From Part 0 Utilities

- **Source**: `python/utils/build_model.py`
- `A, B, C = build_continuous_model()`: Continuous-time matrices from `matlab/prep_final.m`
- `Ad, Bd, Cd, Dd = discretize_zoh(A, B, C, Ts=0.01)`: Discrete-time matrices via zero-order hold
- `Ad` (12×12), `Bd` (12×3), `Ts = 0.01` seconds

### From final_exam.pdf (Authoritative Source)

**All values below must be verified and cited from `docs/sources/final_exam.pdf`:**

1. **Part 2 Sensor Matrix `Cd_new`**

- Exact 2×12 matrix definition
- Expected: measures `x1` (row 1) and `x6` (row 2)
- **Citation required**: Page number and section from PDF

2. **Part 2 Initial Conditions**

- Actual initial state `x0` (12×1 vector)
- Observer initial state `xhat0` (12×1 vector)
- **Citation required**: Page number and section from PDF
- **Preliminary values from anchor doc** (to be verified):
    - `x0 = [0; 0; 0; 1; 1; 1; 0; 0; 0; 0; 0; 0]`
    - `xhat0 = [0; 0; 0; 0; 0; 1; 0; 0; 0; 0; 0; 0]`

3. **Simulation Parameters**

- Simulation horizon `N` (number of time steps) if specified
- If not specified, default to `N = 1000` (10 seconds) and document this default
- Input signal `u[k]` specification if provided
- If not specified, default to `u[k] = 0` (zero input) and document this assumption

4. **Observer Design Requirements** (if any)

- Specific pole locations if provided
- Design method preference (pole placement vs dual LQR) if specified
- Observer performance requirements if any

### Reused from Part 1

- Observability analysis utilities (if applicable) from `python/part1/observability_rank.py`
- SVD tolerance policy: Same as Part 1 (`tol = max(1e-10, machine_epsilon × max(sigma))`)

## 3. What You Need to Know

### Theoretical Concepts

1. **Discrete-Time Observer Structure**

- Luenberger observer equations:
     ```javascript
                    xhat[k+1] = Ad @ xhat[k] + Bd @ u[k] + L @ (y[k] - yhat[k])
                    yhat[k] = Cd @ xhat[k]
     ```




- Observer gain `L` has shape (n, p) where n=12 states, p=2 outputs → L is (12×2)

2. **Estimation Error Dynamics**

- Error: `e[k] = x[k] - xhat[k]`
- Error dynamics: `e[k+1] = (Ad - L @ Cd) @ e[k]`
- Observer is stable if eigenvalues of `(Ad - L @ Cd)` are inside unit circle (|λ| < 1)
- Convergence rate depends on eigenvalue locations (faster for smaller magnitudes)

3. **Pole Placement Concept in Discrete Time**

- Place observer poles (eigenvalues of `Ad - L @ Cd`) at desired locations
- Common rule: Observer poles should be faster than plant poles (smaller magnitude, closer to origin)
- Conservative policy: Place poles with magnitude 0.5-0.8 times plant pole magnitudes (or use specific factor if course specifies)
- **Note**: Do not choose specific numeric poles unless exam provides them

4. **Dual System Approach (Option 1: Pole Placement)**

- Observer design for `(Ad, Cd)` is dual to controller design for `(Ad^T, Cd_new^T)`
- Solve pole placement for dual system: `(Ad^T - Cd_new^T @ L^T)` to place eigenvalues
- Transpose result to get observer gain: `L = (L_dual)^T`
- Python function: `scipy.signal.place_poles()` or manual Ackermann's formula

5. **Dual LQR Approach (Option 2: Alternative Method)**

- Formulate as LQR problem for dual system
- Solve Riccati equation for dual system
- Extract observer gain from solution
- **Note**: Only use if course conventions allow or exam specifies

### Implementation Prerequisites

- Understanding of NumPy/SciPy linear algebra: `np.linalg.svd()`, `scipy.signal.place_poles()`
- Understanding of coupled simulation: Plant and observer sharing input `u[k]`
- Familiarity with estimation error computation and RMS metrics

## 4. Design Approach Overview

### Step A: Compute Observability Rank for `(Ad, Cd_new)`

**Script/Module**: `python/part2/observer_design.py` or reuse `python/part1/observability_rank.py`**High-level approach**:

1. Construct observability matrix `O` for `(Ad, Cd_new)`:

- `O = [Cd_new; Cd_new @ Ad; Cd_new @ Ad^2; ...; Cd_new @ Ad^(n-1)]`
- Shape: (n*p, n) = (24, 12) since p=2 outputs, n=12 states

2. Compute rank using SVD with same tolerance policy as Part 1:

- `tol = max(1e-10, machine_epsilon × max(sigma))`
- `rank = count(sigma_i > tol × max(sigma))`

3. Verify rank equals 12 (full observability) or document rank deficiency
4. Save observability results to output directory

### Step B: Choose Observer Design Method

**Script/Module**: `python/part2/observer_design.py`**Decision point**: Select design method and document rationale**Option 1: Pole Placement via Dual System** (recommended default)

1. Form dual system: `(Ad^T, Cd_new^T)`
2. Choose desired observer pole locations (if not specified in exam):

- Policy: Place poles with magnitude 0.5-0.8 times fastest plant pole magnitude
- Or use conservative factor (e.g., 0.6×) if course specifies
- If exam specifies poles, use those exactly

3. Solve pole placement: Use `scipy.signal.place_poles()` for dual system
4. Transpose result: `L = (L_dual)^T` to get observer gain
5. Verify eigenvalues of `(Ad - L @ Cd_new)` match desired poles (within tolerance)

**Option 2: Dual LQR** (only if course/exam specifies)

1. Formulate dual LQR problem
2. Solve Riccati equation for dual system
3. Extract observer gain from solution
4. Document design parameters (Q, R matrices used)

**Implementation note**: Record chosen method, design parameters, and pole locations (if selected) in code comments and results file. Do NOT choose specific numeric poles unless exam provides them.

### Step C: Implement Coupled Simulation

**Script/Module**: `python/part2/run_observer_sim.py`**High-level approach**:

1. Define input signal `u[k]`:

- If exam specifies: Use provided `u[k]`
- If not specified: Default to `u[k] = 0 `for all `k` and document this assumption

2. Initialize states:

- True plant: `x[0] = x0` (from PDF)
- Observer: `xhat[0] = xhat0` (from PDF)

3. Simulate forward for `k = 0` to `N-1`:
   ```javascript
            # Plant simulation
            y[k] = Cd_new @ x[k]
            x[k+1] = Ad @ x[k] + Bd @ u[k]
            
            # Observer simulation
            yhat[k] = Cd_new @ xhat[k]
            xhat[k+1] = Ad @ xhat[k] + Bd @ u[k] + L @ (y[k] - yhat[k])
   ```




4. Store trajectories: `x` (12×N), `xhat` (12×N), `y` (2×N), `yhat` (2×N)
5. Compute estimation errors: `e[k] = x[k] - xhat[k] `for all `k`

**Reuse utilities**: Use `python/utils/sim.py` as reference, but extend for coupled plant-observer simulation

### Step D: Plotting and Metrics

**Script/Module**: `python/part2/run_observer_sim.py`**Plotting requirements**:

1. **Measured outputs and estimates**:

- Plot `y1` and `yhat1` (measured and estimated displacement of mass 1)
- Plot `y6` and `yhat6` (measured and estimated displacement of mass 6)
- Use `python/utils/plots.py` utilities with appropriate labels
- Save to `python/part2/outputs/outputs_comparison.png`

2. **State estimation errors** (at minimum, displacements):

- Plot `e1, e2, e3, e4, e5, e6` (estimation errors for positions x1..x6)
- Optionally plot all 12 state errors
- Use `python/utils/plots.py` or custom plotting
- Save to `python/part2/outputs/estimation_errors.png`

**Metrics computation**:

1. **RMS estimation error for displacements**:

- Compute RMS error per state: `RMS_i = sqrt(mean(e_i[k]^2))` for i = 1..6
- Compute overall RMS: `RMS_overall = sqrt(mean(sum(e_i[k]^2)))`
- Save metrics to `python/part2/outputs/results.txt`

2. **Convergence analysis** (optional):

- Check that estimation error decreases over time (if observer is stable)
- Document final error magnitude vs initial error

**Reuse utilities**: Use `python/utils/metrics.py` for RMS computation (extend if needed)

## 5. Deliverables

### Documentation

- **`docs/03_part2_observer/plan.md`**: This plan document
- **`docs/03_part2_observer/closeout.md`**: Results summary (to be created after implementation)
- **Update `docs/00_anchor.md`**: Only if mismatches discovered in Part 2 `C` matrix or initial conditions relative to current anchor document

### Python Code Structure

**Directory**: `python/part2/`

1. **`observer_design.py`**

- Function: `compute_observability_rank(Ad, Cd_new, tol=None)` → returns rank, observability matrix, results dict
    - Reuse logic from Part 1 or import from `python/part1/observability_rank.py`
- Function: `design_observer_pole_placement(Ad, Cd_new, desired_poles)` → returns `L` (12×2)
    - Implements Option 1: Pole placement via dual system
    - Documents design method and parameters
- Function: `design_observer_dual_lqr(Ad, Cd_new, Q, R)` → returns `L` (12×2)
    - Implements Option 2: Dual LQR (if chosen)
- Function: `design_observer(Ad, Cd_new, method='pole_placement', **kwargs)` → returns `L`, design_info
    - Main function that calls appropriate design method
    - Records design settings (method, poles, parameters) in `design_info` dict
- Main section: Demonstrates observability check and observer design
- Saves design results to `python/part2/outputs/observer_design_results.txt`

2. **`run_observer_sim.py`**

- Main runner script that:
    - Loads `Ad, Bd, Cd_new` from Part 0 utilities and Part 2 configuration
    - Loads initial conditions `x0, xhat0` from exam specification
    - Calls `observer_design.py` to compute `L`
    - Runs coupled plant-observer simulation
    - Generates plots (outputs comparison, estimation errors)
    - Computes RMS error metrics
    - Saves all results to `python/part2/outputs/`
- Defines input signal `u[k]` (default: zero input unless exam specifies)

3. **`__init__.py`** (if needed for module imports)
4. **`python/part2/outputs/`** (directory)

- `observer_design_results.txt`: Observability rank, design method, observer gain `L`, eigenvalues of `(Ad - L @ Cd_new)`
- `results.txt`: Simulation summary, RMS errors, convergence notes
- `outputs_comparison.png`: Plot of `y1, yhat1, y6, yhat6`
- `estimation_errors.png`: Plot of estimation errors (at least `e1..e6`)
- Optional: `all_state_errors.png` (all 12 state errors)

### Results File Format

**`observer_design_results.txt`** should include:

- Part 2 `Cd_new` matrix (2×12)
- Observability rank for `(Ad, Cd_new)`
- Design method used (pole placement or dual LQR)
- Observer gain `L` (12×2) or summary statistics
- Eigenvalues of `(Ad - L @ Cd_new)` (observer poles)
- Design parameters (pole locations if used, Q/R if dual LQR)

**`results.txt`** should include:

- Initial conditions used (`x0, xhat0`)
- Input signal description (zero input or other)
- Simulation horizon `N` and time span
- RMS estimation errors per displacement state (x1..x6)
- Overall RMS error
- Convergence notes (if applicable)

## 6. Validation Checklist

### 6.1 Dimension Checks

- [ ] `Cd_new` is 2×12
- [ ] `y` is 2×N (2 outputs over N time steps)
- [ ] `yhat` is 2×N
- [ ] `L` is 12×2 (observer gain)
- [ ] `x` is 12×N (true states)
- [ ] `xhat` is 12×N (estimated states)
- [ ] `e = x - xhat` is 12×N (estimation errors)

### 6.2 Observability Verification

- [ ] Observability rank for `(Ad, Cd_new)` equals 12 (full observability)
- [ ] Rank computation uses documented tolerance policy (same as Part 1)
- [ ] Observability results are saved to output file

### 6.3 Observer Design Validation

- [ ] Observer gain `L` computed successfully
- [ ] Design method documented (pole placement or dual LQR)
- [ ] Eigenvalues of `(Ad - L @ Cd_new)` are inside unit circle (|λ| < 1) for stability
- [ ] If poles were specified, eigenvalues match desired poles within tolerance
- [ ] Design parameters (poles, Q/R) recorded in results file

### 6.4 Simulation Validation

- [ ] Coupled simulation runs without errors
- [ ] True plant uses correct initial condition `x0`
- [ ] Observer uses correct initial condition `xhat0`
- [ ] Both plant and observer use same input signal `u[k]`
- [ ] Output reconstruction: `yhat[k] = Cd_new @ xhat[k] `for all `k`

### 6.5 Estimation Error Analysis

- [ ] Estimation error `e[k] = x[k] - xhat[k]` computed correctly
- [ ] Estimation error decreases over time (if observer is stable and `x0 ≠ xhat0`)
- [ ] RMS errors computed for at least displacement states (x1..x6)
- [ ] RMS error metrics saved to results file

### 6.6 Plot Validation

- [ ] Output comparison plot shows `y1, yhat1, y6, yhat6` with correct labels
- [ ] Estimation error plot shows at least `e1..e6` (displacement errors)
- [ ] Plot labels match actual outputs (no "d1..d6" unless explicitly plotting states)
- [ ] All plots saved to output directory

### 6.7 Input Signal Documentation

- [ ] Input signal `u[k]` is documented (zero input or exam-specified)
- [ ] Default assumption (if used) is explicitly stated in results

## 7. Non-Goals

Part 2 explicitly does NOT include:

- **LQR Controller Design**: No feedback control, no LQR gain computation (Part 3)
- **Noise Models**: No process noise `w[k]`, no measurement noise `v[k]` (Part 5)
- **Kalman Filter**: No stochastic estimation, no covariance propagation (Part 5)
- **Pole Tuning Beyond Policy**: Do not choose specific numeric pole locations unless exam provides them
- **Reduced Input Analysis**: No analysis of systems with fewer than 3 inputs (Part 4)
- **Sensor Augmentation Analysis**: No comparison of different `C` matrix configurations (Part 7)

## 8. References and Source Citations

### Authoritative Sources

1. **Model Definition**: `matlab/prep_final.m`

- Continuous-time matrices `A, B, C`
- Discretization method (ZOH) and sampling time `Ts = 0.01`
- Baseline `C` matrix (1×12 measuring x1)

2. **Part 2 Specifications**: `docs/sources/final_exam.pdf`

- **Requires verification**: Exact `Cd_new` matrix definition (page/section TBD)
- **Requires verification**: Initial conditions `x0, xhat0` (page/section TBD)
- **Requires verification**: Simulation parameters `N, u[k]` if specified (page/section TBD)
- **Requires verification**: Observer design requirements if any (page/section TBD)

3. **Part 0 Utilities**: `python/utils/`

- `build_model.py`: Model construction and discretization
- `sim.py`: Simulation utilities (reference for coupled simulation)
- `plots.py`: Plotting utilities with dimension-aware legends
- `metrics.py`: Metrics computation (extend for RMS errors)

4. **Part 1 Results**: Observability analysis approach

- `python/part1/observability_rank.py`: SVD-based rank computation
- Tolerance policy: `tol = max(1e-10, machine_epsilon × max(sigma))`

5. **Anchor Document**: `docs/00_anchor.md`

- System conventions (state ordering, discretization method)
- Part 2 preliminary values (to be verified against PDF)
- Plot policy (never label outputs as d1..d6 unless C selects them)

## 9. Implementation Notes

### Observer Design Method Selection

**Default choice**: Pole placement via dual system (Option 1)

- Standard approach for discrete-time observer design
- Flexible for pole selection (can use conservative policy or exam-specified poles)
- Well-supported by SciPy (`scipy.signal.place_poles`)

**Alternative**: Dual LQR (Option 2)

- Use only if exam specifies or course conventions require
- Requires Q and R weight matrices (document source)

### Pole Selection Policy

**If exam specifies poles**: Use those exactly and document source.**If exam does not specify poles**: Use conservative policy:

- Compute eigenvalues of plant `Ad`
- Choose observer poles with magnitude 0.5-0.8 times fastest plant pole magnitude
- Or use factor specified by course (e.g., 0.6×)
- Document policy in code and results file
- **Do not choose specific numeric values in plan** - defer to implementation

### Input Signal Assumption

**If exam specifies `u[k]`**: Use provided signal and document source.**If exam does not specify**: Default to `u[k] = 0` (zero input, open-loop) and document this assumption in results file. This is reasonable for nominal observer testing.

### Reusability Considerations

- Observability analysis can reuse Part 1 utilities (import or copy logic)
- Simulation utilities can extend Part 0 `sim.py` for coupled plant-observer case
- Plotting utilities from Part 0 should handle 2 outputs correctly
- Metrics utilities may need extension for RMS error computation

### Code Organization

- Keep design and simulation separate: `observer_design.py` for gain computation, `run_observer_sim.py` for simulation and plotting