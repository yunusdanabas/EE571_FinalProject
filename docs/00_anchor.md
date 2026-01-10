# EE571 Final Project - Anchor Document

## Project Summary

This project implements control system design for a **6-mass spring chain system** with:
- **12 states**: `[x1, x2, x3, x4, x5, x6, x1dot, x2dot, x3dot, x4dot, x5dot, x6dot]`
- **3 inputs**: Abstract channels defined by columns of `B` matrix
  - Effects inferred from nonzero rows: `u1` affects rows 7,8; `u2` affects rows 11,12; `u3` affects row 8
  - *Note: Physical interpretation (tension/force) from exam statement - verify against `docs/sources/final_exam.pdf`*
- **Task sequence**: Observability analysis → Observer design → LQR control → Reduced input LQR → Kalman filter → LQG (LQR + Kalman) → Sensor augmentation analysis

## Exam-Verified Definitions

The following definitions are explicitly stated in the final exam (`docs/sources/final_exam.pdf`) and are used as authoritative sources:

### Baseline Measurement (Parts 0 and 1)

The exam states that `prep_final.m` assumes **only a single output is measured**, the displacement of the first body. This aligns with:

\[
C_{\text{base}} = \begin{bmatrix} 1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \end{bmatrix}
\]

**Source**: Final exam statement about prep code. See also `docs/sources/final_exam_extract.md`. PDF page number: *[To be recorded from final_exam.pdf]*

### Part 2 Measurement Matrix and Initial Conditions

**Measurement Matrix (Cd_new)**:
The exam explicitly provides the augmented sensor matrix measuring x1 and x6:

\[
C_{\text{part2}} = \begin{bmatrix}
1 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0
\end{bmatrix}
\]

**Initial Conditions**:
- Actual system initial state:
  \[
  x_0 = \begin{bmatrix} 0 & 0 & 0 & 1 & 1 & 1 & 0 & 0 & 0 & 0 & 0 & 0 \end{bmatrix}^T
  \]
- Observer initial state:
  \[
  \hat{x}_0 = \begin{bmatrix} 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 \end{bmatrix}^T
  \]

**Source**: Final exam Question 2. See also `docs/sources/final_exam_extract.md`. PDF page number: *[To be recorded from final_exam.pdf]*

**Note**: These definitions are verified from exam screenshots. Page numbers should be recorded by opening `docs/sources/final_exam.pdf` and locating Question 2. For comprehensive reference, see `docs/sources/final_exam_extract.md`.

## File Inventory and Role

### Reference File: `matlab/prep_final.m`

**Authoritative source for:**
- Continuous-time system matrices: `A` (12×12), `B` (12×3), `C` (1×12)
- Sampling time: `Ts = 0.01` seconds
- Discretization method: Zero-order hold via MATLAB `c2d(ss(A,B,C,0), Ts)`
- Initial condition: `x0 = [0; 0; 0; 0; 0; 1; 0; 0; 0; 0; 0; 0]`
- Simulation horizon: `N = 1000` steps (10 seconds total)

**Known Issue:**
- The `C` matrix is 1×12 (only measures displacement of mass 1: `x1`)
- The plotting code (lines 77-78) uses legend `'d1', 'd2', 'd3', 'd4', 'd5', 'd6'` which suggests 6 outputs
- **All plots must be consistent with actual outputs** - if `C` is 1×12, only one output should be plotted

### Plot Policy

- **Never label outputs as d1..d6 unless `C` selects `[x1..x6]`**
- **Plot `y` as defined by the current `C` for that part** - number of plot traces must match `size(C, 1)`
- **For displacement plots**: If you need to plot all 6 displacements regardless of `C`, define a separate `C_disp = [I6, 0]` (6×12) and plot `C_disp * x`, not `y`

## Model Conventions

### State Ordering
States are ordered as:
```
[x1, x2, x3, x4, x5, x6, x1dot, x2dot, x3dot, x4dot, x5dot, x6dot]
```
- Positions: indices 1-6
- Velocities: indices 7-12

### Input Matrix B (12×3)

From `prep_final.m`:
- Row 7 (x1dot): `[1, 0, 0]` → `u1` affects acceleration of mass 1
- Row 8 (x2dot): `[-1, 0, -1]` → `u1` and `u3` affect acceleration of mass 2
- Row 11 (x5dot): `[0, 1, 0]` → `u2` affects acceleration of mass 5
- Row 12 (x6dot): `[0, -1, 0]` → `u2` affects acceleration of mass 6

**Interpretation:**
- Inputs are abstract channels defined by columns of `B`
- Effects: `u1` affects acceleration of masses 1 and 2 (rows 7, 8); `u2` affects acceleration of masses 5 and 6 (rows 11, 12); `u3` affects acceleration of mass 2 (row 8)
- *Physical meaning (tension/force) should be verified against exam statement in `docs/sources/final_exam.pdf`*

### Discretization
- Method: Zero-order hold (ZOH)
- Sampling time: `Ts = 0.01` seconds
- MATLAB command: `c2d(ss(A, B, C, 0), Ts)`
- All implementations must match this discretization exactly

### Validation Policy

**Discretization Tolerance:**
- Python implementations must match MATLAB `c2d` output within tolerance
- Use: `max(abs(Ad_python - Ad_matlab)) < 1e-10` or `np.allclose(Ad_python, Ad_matlab, rtol=1e-10, atol=1e-12)`
- Apply same tolerance to `Bd` and `Cd` matrices

**Random Number Generator Policy:**
- **Python**: Use `np.random.seed(42)` with default generator
- **MATLAB**: Use `rng(42)` with default generator
- Ensures reproducible noise sequences across implementations

## Deliverables

### Code Structure
- **Python implementations**: `python/part1/`, `python/part2/`, etc. (each part in its own directory)
  - Main scripts: `python/part1/main.py`, `python/part2/main.py`, etc.
  - Shared utilities: `python/utils/` (common functions for discretization, plotting, etc.)
- **MATLAB implementations**: `matlab/part1.m` through `matlab/part7.m`
- **Optional Part 8**: Python→MATLAB conversion and validation

### Documentation Structure
Each part has its own documentation directory:
- `docs/02_part1_observability/part1_plan.md` - Implementation plan for Part 1
- `docs/02_part1_observability/part1_closeout.md` - Results summary for Part 1
- `docs/03_part2_observer/part2_plan.md`, `docs/03_part2_observer/part2_closeout.md` - Part 2
- ... (similar for Parts 3-7)

## Global Decisions

### Simulation Horizon
- **Default**: `N = 1000` steps (10 seconds at `Ts = 0.01`)
- Parts may override this if specified in the problem statement
- **Time vector:** `t = np.arange(N+1) * Ts` (length N+1, matches state/output arrays: t[0] to t[N])

### Array Indexing Convention (FROZEN)
- **State trajectory x:** `(n, N+1)` stores `x[0]` through `x[N]` (N+1 samples)
- **Input trajectory u:** `(m, N)` stores `u[0]` through `u[N-1]` (N samples)
- **Output trajectory y:** `(p, N+1)` stores `y[0]` through `y[N]` (N+1 samples)
- **Time vector t:** `(N+1,)` stores `t[0]` through `t[N]` where `t[k] = k * Ts`
- **Cost indexing:** `J = sum from k=0 to N-1 of stage_cost[k]` (u[k] pairs with transition from x[k] to x[k+1])

### Cost Metrics

**Part 3 LQR Cost Function:**
```
J = Σ(uᵀu + y₁² + y₆²)
```
where:
- `u = [u₁, u₂, u₃]ᵀ` is the input vector
- `y₁` is the displacement of mass 1 (state `x1`)
- `y₆` is the displacement of mass 6 (state `x6`)

**LQR State Weighting Construction:**
- Part 3 uses the 2-output `C` from Part 2 (measures `x1` and `x6`)
- If cost uses `y₁` and `y₆` regardless of sensor set, compute them via `C_disp = [I6, 0]` and extract rows 1 and 6
- Map output-weighted cost to LQR: `Q = C^T W C` where `W = I` unless the statement specifies otherwise
- For `y₁² + y₆²`, use `W` with ones at positions corresponding to outputs 1 and 6 (or construct `Q` directly to weight states 1 and 6)

**Finite-horizon cost computation:**
- Sum over all time steps: `J_total = Σ_{k=0}^{N-1} (u_kᵀu_k + y₁_k² + y₆_k²)`

### Input Metrics
- **Max input magnitude**: `max(|u_i|)` for each channel `i = 1, 2, 3`
- **Total cost**: Finite-horizon cost `J_total` as defined above

### Estimation Error Metrics
- **RMS error**: `√(Σ(x_true - x_est)² / N)` per state or overall
- **Per-state error**: Time series of `x_true - x_est` for each state
- **Selected state tracking**: Focus on states of interest (e.g., positions x1-x6)

### Random Seed Policy
- **Fixed seed**: `seed = 42` for all noisy simulations
- **Python**: `np.random.seed(42)` with default generator
- **MATLAB**: `rng(42)` with default generator
- Ensures reproducibility across runs
- Apply to: process noise `w`, measurement noise `v`

## Per-Part Dependencies

| Part | C Matrix | Initial Conditions | Noise Covariances | Source |
|------|----------|---------------------|-------------------|--------|
| 0 | `[1 0 0 0 0 0 0 0 0 0 0 0]` (1×12) | `[0; 0; 0; 0; 0; 1; 0; 0; 0; 0; 0; 0]` | N/A | `matlab/prep_final.m` |
| 1 | `[1 0 0 0 0 0 0 0 0 0 0 0]` (1×12) | `[0; 0; 0; 0; 0; 1; 0; 0; 0; 0; 0; 0]` | N/A | `matlab/prep_final.m` |
| 2 | `[1 0 0 0 0 0 0 0 0 0 0 0; 0 0 0 0 0 1 0 0 0 0 0 0]` (2×12) | Actual: `[0; 0; 0; 1; 1; 1; 0; 0; 0; 0; 0; 0]`<br>Estimated: `[0; 0; 0; 0; 0; 1; 0; 0; 0; 0; 0; 0]` | N/A | Verified from final exam Question 2 |
| 3 | Same as Part 2 (2×12) | Same as Part 2 | N/A | From exam statement |
| 4 | Same as Part 2 (2×12) | Same as Part 2 | N/A | From exam statement |
| 5 | Same as Part 2 (2×12) | Same as Part 2 | `v ~ N(0, 0.1 I_p)`, `w ~ N(0, 0.05 I_m)` | From exam statement |
| 6 | Same as Part 2 (2×12) | Same as Part 2 | Same as Part 5 | From exam statement |
| 7 Case 1 | 4×12 (see Part 7 section) | Same as Part 2 | Same as Part 5 | From exam statement |
| 7 Case 2 | 6×12 (see Part 7 section) | Same as Part 2 | Same as Part 5 | From exam statement |

**Note**: Part 2 definitions (C matrix and initial conditions) are verified from final exam Question 2. See "Exam-Verified Definitions" section above for details. PDF page numbers should be recorded by opening `docs/sources/final_exam.pdf`.

## Breakdown into Subprojects

### Part 0: Workspace Setup + Baseline Verification
- Verify `prep_final.m` runs correctly
- Confirm discrete-time matrices match expectations
- Establish baseline simulation output
- Fix plotting inconsistency (C matrix vs legend)

### Part 1: Observability Analysis + Kalman Decomposition
- Analyze observability of discrete system with original `C` (1×12, measures x1 only)
- Perform Kalman decomposition to identify observable and unobservable modes
- Document observable/unobservable subspaces

### Part 2: Observer Design with Added Sensor
- **Modified C matrix**: Add sensor at mass 6
  ```
  C = [1 0 0 0 0 0 0 0 0 0 0 0;
       0 0 0 0 0 1 0 0 0 0 0 0]
  ```
- Design observer (pole placement or other method)
- **Initial conditions**:
  - Actual: `x0 = [0; 0; 0; 1; 1; 1; 0; 0; 0; 0; 0; 0]`
  - Estimated: `x̂0 = [0; 0; 0; 0; 0; 1; 0; 0; 0; 0; 0; 0]`
- Simulate and compare real vs estimated states

### Part 3: LQR Design
- Use observable system from Part 2
- **Cost function**: `J = Σ(uᵀu + y₁² + y₆²)`
- Design LQR controller (uses estimated states from observer)
- Simulate closed-loop performance
- Compute total cost and max input

### Part 4: Reduced Input LQR
- Remove `u3` (only `u1` and `u2` available)
- Redesign LQR with same cost function (adapted for 2 inputs)
- Simulate performance
- **Comparison metrics**:
  - Total cost vs Part 3
  - Max input magnitude vs Part 3

### Part 5: Kalman Filter Design
- System with process and measurement noise:
  - `x_{k+1} = A_d x_k + B_d u_k + B_d w_k` (verified from exam statement: noise enters through `B_d` per `docs/sources/final_exam_extract.md` Section 7)
  - `y_k = C_d x_k + v_k`
- **Noise characteristics** (frozen for Parts 5-7):
  - `v ~ N(0, 0.1 I_p)` where `p = size(C, 1) = 2` (sensor noise covariance Rv = 0.1 × I₂)
  - `w ~ N(0, 0.05 I_m)` where `m = size(B, 2) = 3` (actuator noise covariance Qw = 0.05 × I₃)
  - Random seed: `seed = 42` (for reproducibility)
- **Measurement matrix**: Uses Part 2 C matrix (Cmeas, 2×12, measures x₁ and x₆)
- **Output definitions**:
  - `y_true[k] = Cmeas @ x[k]` (true output, no noise)
  - `y_meas[k] = Cmeas @ x[k] + v[k]` (measured output, with noise)
  - `yhat[k] = Cmeas @ xhat[k]` (estimated output from Kalman filter)
- Design steady-state Kalman filter (LQE) using DARE
- Use initial conditions from Part 2
- Compare real vs estimated states
- **Metrics**: Report both tracking RMS (y_true vs yhat) and innovation RMS (y_meas vs yhat)

### Part 6: LQG (LQR + Kalman Filter)
- Combine LQR controller (K) from Part 3 with Kalman filter (Lk) from Part 5
- **Noise settings**: Same as Part 5 (Qw = 0.05 × I₃, Rv = 0.1 × I₂, seed = 42)
- **Measurement matrix**: Uses Part 2 C matrix (Cmeas, 2×12, measures x₁ and x₆)
- **Initial conditions**: Same as Part 2 (x₀, x̂₀)
- Simulate closed-loop system with noise:
  - Plant: `x[k+1] = Ad @ x[k] + Bd @ u[k] + Bd @ w[k]`
  - Measurement: `y_meas[k] = Cmeas @ x[k] + v[k]`
  - Kalman filter: `xhat[k+1] = Ad @ xhat[k] + Bd @ u[k] + Lk @ (y_meas[k] - Cmeas @ xhat[k])`
  - Controller: `u[k] = -K @ xhat[k]` (uses estimated state)
- **Cost computation**: 
  - `J_true = Σ(u^T u + y_true1² + y_true6²)` using `y_true = Cmeas @ x` (does not penalize measurement noise)
  - `J_meas = Σ(u^T u + y_meas1² + y_meas6²)` using `y_meas = Cmeas @ x + v` (penalizes noise)
  - Official metric: `J_true` (justified: does not penalize uncontrollable measurement noise)
- **Comparison**: Compare outputs against Part 3 (LQR with deterministic observer, no noise)
- **Validation**: Verify K matrix matches Part 3 exactly, confirm initial conditions, log early time control magnitudes

### Part 7: Sensor Augmentation Analysis
- **Case 1**: C matrix (4×12)
  ```
  [1 0 0 0 0 0 0 0 0 0 0 0]
  [0 1 0 0 0 0 0 0 0 0 0 0]
  [0 0 0 0 1 0 0 0 0 0 0 0]
  [0 0 0 0 0 1 0 0 0 0 0 0]
  ```
- **Case 2**: C matrix (6×12)
  ```
  [1 0 0 0 0 0 0 0 0 0 0 0]
  [0 1 0 0 0 0 0 0 0 0 0 0]
  [0 0 1 0 0 0 0 0 0 0 0 0]
  [0 0 0 1 0 0 0 0 0 0 0 0]
  [0 0 0 0 1 0 0 0 0 0 0 0]
  [0 0 0 0 0 1 0 0 0 0 0 0]
  ```
- Same noise levels as Part 5
- Simulate estimation and regulation performance
- **Question**: Does having more sensors help with estimation and/or regulation?

### Part 8 (Optional): Python→MATLAB Conversion
- Convert Python implementations to MATLAB
- Validate numerical equivalence
- Ensure plots match between implementations

## How to Use This Repo

### Workflow Rules

1. **One Part Per Chat Session**
   - Each chat session works on exactly ONE part
   - Do not mix parts within a single session

2. **Before Coding**
   - Ensure `docs/XX_partYY/partYY_plan.md` exists for the part
   - Review the plan and anchor document for conventions
   - Verify you understand the problem statement

3. **During Implementation**
   - Follow model conventions (state ordering, discretization method)
   - Use reference values from `prep_final.m` where applicable
   - Apply global decisions (simulation horizon, random seed, metrics)

4. **After Completion**
   - Create `docs/XX_partYY/partYY_closeout.md` with:
     - Results summary
     - Key findings
     - Plots and metrics
     - Any deviations from plan

5. **Cross-References**
   - Reference this anchor document for conventions
   - Reference `matlab/prep_final.m` for matrix definitions
   - Reference `docs/sources/final_exam.pdf` for exam statement requirements
   - Reference part-specific partX_plan.md for part-specific requirements

### File Naming Convention
- Python: `python/part1/main.py`, `python/part2/main.py`, etc. (each part in its own directory)
- Python utilities: `python/utils/` (shared functions)
- MATLAB: `matlab/part1.m`, `matlab/part2.m`, etc.
- Documentation: `docs/01_part1/`, `docs/02_part2/`, etc.

### Implementation Guidelines

- **High-level guidance only**: This anchor document provides conventions, not step-by-step code
- **No numeric specifications**: Do not specify pole locations or controller weights beyond what the exam statement requires
- **Consistency**: All parts must use the same model, discretization, and simulation parameters unless explicitly overridden

## Checklist for Each Part

- [ ] Read part-specific partX_plan.md
- [ ] Verify model matrices match `prep_final.m` conventions
- [ ] Use correct initial conditions (check problem statement)
- [ ] Apply appropriate noise models (if applicable)
- [ ] Compute required metrics (cost, max input, estimation error)
- [ ] Generate plots consistent with actual outputs
- [ ] Create partX_closeout.md with results summary

