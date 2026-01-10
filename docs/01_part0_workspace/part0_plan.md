# Part 0: Workspace Setup and Baseline Verification - Plan

## 1. Objectives

The primary goal of Part 0 is to establish a solid foundation for the EE571 Final Project by creating reproducible model definitions, verification tools, and baseline simulations. This part focuses on infrastructure setup rather than control design.

### Core Objectives

1. **Establish Repository Structure**
   - Create directory structure for Python-first and MATLAB-second workflows
   - Organize shared utilities, part-specific code, and documentation according to conventions in `docs/00_anchor.md`

2. **Model Definition and Discretization**
   - Extract exact continuous-time matrices (`A`, `B`, `C`) from `matlab/prep_final.m`
   - Implement Python utilities to reproduce the discrete-time model using zero-order hold (ZOH) discretization
   - Verify Python discretization matches MATLAB `c2d(ss(A,B,C,0),Ts)` output within numerical tolerance

3. **Simulation Infrastructure**
   - Create reusable discrete-time simulation core utility
   - Implement baseline open-loop simulation matching `prep_final.m` parameters
   - Establish time vector convention: `t = (0:N-1) * Ts` where `N = 1000`, `Ts = 0.01`

4. **Validation Framework**
   - Define and document global validation tolerances for matrix comparisons
   - Establish metrics conventions (cost computation, input magnitudes, estimation errors)
   - Define plotting policy that ensures plot labels/legends match actual output dimensions from `C` matrix

5. **Documentation and Consistency**
   - Document the known legend mismatch in `prep_final.m` (line 77: legend shows 6 outputs but `C` is 1×12)
   - Establish conventions for future parts to avoid similar inconsistencies

## 2. Inputs Required

### Required Files

- **`matlab/prep_final.m`** (authoritative source)
  - Continuous-time system matrices: `A` (12×12), `B` (12×3), `C` (1×12)
  - Sampling time: `Ts = 0.01` seconds
  - Initial condition: `x0 = [0; 0; 0; 0; 0; 1; 0; 0; 0; 0; 0; 0]`
  - Simulation horizon: `N = 1000` steps
  - Discretization method: `c2d(ss(A,B,C,0),Ts)` using zero-order hold

### Recommended Files

- **`final_exam.pdf`** (found at repo root)
  - Authoritative source for part-specific requirements
  - Needed for validation of Part 2+ parameters (C matrices, initial conditions, noise covariances)
  - Should be moved to `docs/sources/final_exam.pdf` for organization (note as TODO)

### Other Files to Commit

- Any starter scripts or handouts provided by instructor (if applicable)
- Reference plots or expected outputs (if provided)

## 3. Planned Repository Layout

The repository structure must align with conventions specified in `docs/00_anchor.md`:

```
/home/yunusdanabas/EE571_FinalProject/
├── docs/
│   ├── 00_anchor.md                    # Global conventions and reference
│   ├── 01_part0_workspace/
│   │   ├── part0_plan.md               # This file
│   │   └── part0_closeout.md           # Results summary (to be created)
│   └── sources/                        # TODO: Create and move final_exam.pdf here
│       └── final_exam.pdf
├── python/
│   ├── utils/                          # Shared utilities for all parts
│   │   ├── build_model.py              # Model construction and discretization
│   │   ├── sim.py                      # Discrete-time simulation loop
│   │   ├── metrics.py                  # Cost computation, input metrics, RMS errors
│   │   └── plots.py                    # Plotting utilities (dimension-aware)
│   └── part0/
│       └── baseline_check.py           # Baseline verification script
└── matlab/
    ├── prep_final.m                    # Reference implementation
    └── [future parts: part1.m, part2.m, ...]
```

### Key Conventions Reference

- **State ordering**: `[x1, x2, x3, x4, x5, x6, x1dot, x2dot, x3dot, x4dot, x5dot, x6dot]`
- **Input matrix B**: Columns define input channels (u1, u2, u3)
  - u1 affects rows 7 (x1dot) and 8 (x2dot)
  - u2 affects rows 11 (x5dot) and 12 (x6dot)
  - u3 affects row 8 (x2dot)
- **Discretization**: Zero-order hold (ZOH) at `Ts = 0.01` seconds
- **Random seed**: `seed = 42` for all noisy simulations (Python: `np.random.seed(42)`, MATLAB: `rng(42)`)

## 4. Python Baseline Tasks

### 4.1 Model Construction (`python/utils/build_model.py`)

**Function: `build_continuous_model()`**
- Hardcode exact `A`, `B`, `C` matrices from `prep_final.m` (lines 4-32)
- Return structured object or tuple `(A, B, C)` with metadata (dimensions, Ts)

**Function: `discretize_zoh(A, B, C, Ts)`**
- Use `scipy.signal.cont2discrete()` with method `'zoh'`
- Match MATLAB `c2d` convention:
  - `sys_discrete = c2d(ss(A, B, C, zeros(size(C,1), size(B,2))), Ts)`
  - Extract `Ad = sys_discrete.A`, `Bd = sys_discrete.B`, `Cd = sys_discrete.C`
- Return `(Ad, Bd, Cd)`

**Function: `load_reference_matrices()`**
- Optional: Load exported MATLAB matrices for comparison (if exported)
- For Part 0, focus on numerical verification via tolerance checks

### 4.2 Simulation Core (`python/utils/sim.py`)

**Function: `simulate_discrete(Ad, Bd, Cd, x0, u, N)`**
- Implement discrete-time simulation loop:
  - Initialize: `x[:, 0] = x0`
  - For `k = 0` to `N-2`: `x[:, k+1] = Ad @ x[:, k] + Bd @ u[:, k]`
  - Output: `y[:, k] = Cd @ x[:, k]` for all `k`
  - Final output: `y[:, N-1] = Cd @ x[:, N-1]`
- Return `(x, y, t)` where `t = np.arange(N) * Ts`
- Input `u` shape: `(m, N)` where `m = size(B, 2) = 3`
- State `x` shape: `(n, N)` where `n = 12`
- Output `y` shape: `(p, N)` where `p = size(C, 1)`

### 4.3 Baseline Open-Loop Simulation (`python/part0/baseline_check.py`)

**Main script tasks:**
1. Load continuous model: `A, B, C = build_continuous_model()`
2. Discretize: `Ad, Bd, Cd = discretize_zoh(A, B, C, Ts=0.01)`
3. Set parameters:
   - `x0 = np.array([0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0])`
   - `N = 1000`
   - `Ts = 0.01`
   - `u = np.zeros((3, N))` (zero input)
4. Simulate: `x, y, t = simulate_discrete(Ad, Bd, Cd, x0, u, N)`
5. Validate dimensions and perform sanity checks
6. Generate plots using `plots.py` utilities

### 4.4 Plotting Utilities (`python/utils/plots.py`)

**Function: `plot_outputs(t, y, labels=None, title='Outputs')`**
- Accept `y` with shape `(p, N)` where `p` is number of outputs
- Auto-generate labels if not provided: `['y1', 'y2', ..., 'yp']`
- **Critical**: Legend must have exactly `p` entries matching `size(C, 1)`
- Never assume 6 outputs unless `C` explicitly selects `[x1, x2, x3, x4, x5, x6]`

**Function: `plot_states(t, x, state_indices=None, title='States')`**
- For plotting all 6 displacements regardless of `C`:
  - Extract positions: `x_disp = x[0:6, :]`
  - Use labels: `['x1', 'x2', 'x3', 'x4', 'x5', 'x6']`
- Document: This is for visualization only, not the actual measured outputs

**Note on `prep_final.m` legend mismatch:**
- Lines 77-78: Legend shows `'d1', 'd2', 'd3', 'd4', 'd5', 'd6'` (6 outputs)
- Actual `C` matrix (line 32): `[1 0 0 0 0 0 0 0 0 0 0 0]` (1 output)
- **Repository policy**: Plot legends must match `size(C, 1)`. If displacement visualization is needed, use `plot_states()` with position indices.

### 4.5 Metrics Utilities (`python/utils/metrics.py`)

**Initial functions for Part 0:**
- `check_dimensions(Ad, Bd, Cd, expected_n=12, expected_m=3, expected_p=1)`
- Placeholder functions for future parts:
  - `compute_cost(u, y, Q=None, R=None)` (for Part 3+)
  - `max_input_magnitude(u)` (for Part 3+)
  - `rms_error(x_true, x_est)` (for Part 5+)

## 5. MATLAB Baseline Tasks

### 5.1 Reference Execution

**Task: Run `matlab/prep_final.m`**
- Execute in MATLAB and verify it runs without errors
- Confirm discrete matrices `Ad`, `Bd`, `Cd` are computed
- Verify simulation completes with `N = 1000` steps

### 5.2 Matrix Export (Optional for Part 0)

**Approach 1: Direct comparison via tolerance**
- Python discretization output compared against MATLAB `c2d` using numerical tolerance
- No export needed if tolerance check passes

**Approach 2: Export for detailed analysis (optional)**
- Add to `prep_final.m` (or create `matlab/export_matrices.m`):
  ```matlab
  save('matlab/discrete_matrices.mat', 'Ad', 'Bd', 'Cd');
  ```
- Load in Python via `scipy.io.loadmat()` for comparison
- **Decision**: Prefer Approach 1 for Part 0; use Approach 2 only if tolerance checks fail

### 5.3 Comparison Method

**Python validation script:**
1. Run MATLAB script (or load exported matrices)
2. Compute Python discretization
3. Compare element-wise:
   - `np.allclose(Ad_python, Ad_matlab, rtol=1e-10, atol=1e-12)`
   - Same for `Bd` and `Cd`
4. Report maximum absolute difference: `max(abs(Ad_python - Ad_matlab))`
5. Report if tolerance check passes or fails

## 6. Validation Checklist

### 6.1 Dimension Checks

- [ ] `A` is 12×12
- [ ] `B` is 12×3
- [ ] `C` is 1×12 (for Part 0 baseline)
- [ ] `Ad` is 12×12
- [ ] `Bd` is 12×3
- [ ] `Cd` is 1×12
- [ ] `x0` has length 12
- [ ] `u` has shape (3, N)
- [ ] `x` has shape (12, N)
- [ ] `y` has shape (1, N) for Part 0

### 6.2 Numerical Equivalence Check

**Tolerance Policy:**
- **Relative tolerance**: `rtol = 1e-10`
- **Absolute tolerance**: `atol = 1e-12`
- Use `np.allclose(Ad_python, Ad_matlab, rtol=1e-10, atol=1e-12)`
- Apply to all three matrices: `Ad`, `Bd`, `Cd`

**Reporting:**
- Maximum absolute difference: `max(abs(Ad_python - Ad_matlab))`
- Maximum relative difference: `max(abs((Ad_python - Ad_matlab) / (Ad_matlab + 1e-15)))`
- Pass/fail status for each matrix

### 6.3 Simulation Sanity Checks

- [ ] States remain bounded (no NaN, no Inf)
- [ ] Output magnitude is reasonable (no explosion)
- [ ] With zero input and initial condition `x0 = [0, 0, 0, 0, 0, 1, ...]`, expect oscillatory behavior (spring-mass system)
- [ ] Qualitative behavior matches MATLAB output (visual inspection of plots)

### 6.4 Plot Sanity Checks

- [ ] Number of plot traces equals `size(C, 1) = 1` for Part 0
- [ ] Legend has exactly 1 entry (e.g., `'y1'` or `'x1'`)
- [ ] Axis labels are correct: `'Time (s)'` for x-axis, appropriate label for y-axis
- [ ] Plot title reflects actual outputs, not assumed outputs
- [ ] **Documentation note**: If plotting all 6 displacements for visualization, use separate function with clear labeling

## 7. Deliverables for Part 0

### Documentation

- [x] `docs/01_part0_workspace/part0_plan.md` (this file)
- [ ] `docs/01_part0_workspace/part0_closeout.md` (results summary after completion)

### Python Code

- [ ] `python/utils/build_model.py`
  - `build_continuous_model()`: Extract A, B, C from prep_final.m
  - `discretize_zoh(A, B, C, Ts)`: ZOH discretization using scipy
- [ ] `python/utils/sim.py`
  - `simulate_discrete(Ad, Bd, Cd, x0, u, N)`: Discrete simulation loop
- [ ] `python/utils/metrics.py` (initial version)
  - `check_dimensions()`: Dimension validation
  - Placeholder functions for future parts
- [ ] `python/utils/plots.py` (initial version)
  - `plot_outputs(t, y, labels=None)`: Output plotting (dimension-aware)
  - `plot_states(t, x, state_indices=None)`: State plotting (for visualization)
- [ ] `python/part0/baseline_check.py`
  - Main script: Load model, discretize, simulate, validate, plot

### MATLAB Code

- [x] `matlab/prep_final.m` (existing reference)
- [ ] Optional: `matlab/export_matrices.m` (if needed for detailed comparison)

### Utility Reuse for Later Parts

All utilities created in Part 0 will be reused across Parts 1-7:

- **`build_model.py`**: Parts 1-7 will use `build_continuous_model()` and `discretize_zoh()` for consistent model definitions
- **`sim.py`**: Core simulation loop will be extended for closed-loop systems (LQR, observer, Kalman filter)
- **`metrics.py`**: Extended with cost computation (Part 3+), input metrics (Part 3+), and estimation errors (Part 5+)
- **`plots.py`**: Extended with control-specific plots (inputs, cost evolution, estimation errors) while maintaining dimension-aware output plotting

**Key principle**: Utilities are designed to be flexible—they accept `C` matrices as parameters, so different parts can use different sensor configurations without code duplication.

## 8. Non-Goals

Part 0 explicitly does NOT include:

- **Control design**: No LQR, observer, or Kalman filter implementation
- **Observability analysis**: Part 1 will handle Kalman decomposition
- **Noise modeling**: Part 5 will introduce process and measurement noise
- **Tuning decisions**: No pole placement, weight selection, or other design choices beyond what `prep_final.m` specifies
- **Multiple C matrices**: Only the baseline `C = [1 0 0 0 0 0 0 0 0 0 0 0]` is used in Part 0
- **Different initial conditions**: Only `x0` from `prep_final.m` is used
- **Cost computation**: Metrics utilities only have placeholders; actual cost computation is Part 3+

## 9. Risks and TODOs

### Risks

1. **Numerical Differences in Discretization**
   - **Risk**: Python `scipy.signal.cont2discrete()` may use slightly different algorithms than MATLAB `c2d`
   - **Mitigation**: Use tolerance-based comparison (`rtol=1e-10, atol=1e-12`). If differences exceed tolerance, investigate algorithm differences and potentially adjust Python implementation or tolerance policy
   - **Escalation**: If tolerance cannot be met, document the difference and verify it does not affect qualitative behavior

2. **Legend Mismatch Precedent**
   - **Risk**: Existing `prep_final.m` has legend showing 6 outputs but `C` defines 1 output
   - **Mitigation**: Document this inconsistency clearly. All repository code must match legends to actual `C` dimensions. Use `plot_states()` for displacement visualization when needed

3. **Missing Exam PDF Reference**
   - **Status**: `final_exam.pdf` exists at repo root
   - **Action**: Move to `docs/sources/final_exam.pdf` during Part 0 setup
   - **Impact**: Needed for Part 2+ to verify C matrices, initial conditions, and noise covariances

### TODOs

#### High Priority

- [ ] Move `final_exam.pdf` from repo root to `docs/sources/final_exam.pdf`
- [ ] Verify Part 2+ requirements from `final_exam.pdf`:
  - [ ] Part 2: C matrix (2×12), initial conditions (actual and estimated)
  - [ ] Part 5: Noise covariances (`v ~ N(0, 0.1 I_p)`, `w ~ N(0, 0.05 I_m)`)
  - [ ] Part 7: Sensor augmentation C matrices (4×12 and 6×12)
- [ ] Create `docs/sources/` directory if it doesn't exist

#### Medium Priority

- [ ] If discretization tolerance check fails, create `matlab/export_matrices.m` for detailed comparison
- [ ] Document any numerical differences found and their impact (if any)

#### Low Priority

- [ ] Consider adding unit tests for utilities (optional, not required for Part 0)
- [ ] Consider creating MATLAB wrapper scripts for Python comparisons (optional)

### Known Dependencies from Anchor Document

The following items are documented in `docs/00_anchor.md` but require verification from `final_exam.pdf`:

- **Part 2**: Modified C matrix (2×12), initial conditions
- **Part 5**: Noise characteristics (process noise `w`, measurement noise `v`)
- **Part 7**: Sensor augmentation C matrices

**Action**: During Part 0, verify these against `final_exam.pdf` and update `docs/00_anchor.md` if discrepancies are found.

---

## Next Steps

After completing Part 0:

1. Verify all deliverables are in place
2. Run validation checklist and document results in `part0_closeout.md`
3. Proceed to Part 1 (Observability Analysis) using established utilities

---

**Plan Status**: Ready for implementation
**Last Updated**: [Date when plan is finalized]

