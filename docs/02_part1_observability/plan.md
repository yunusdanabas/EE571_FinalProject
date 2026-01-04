# Part 1: Observability Analysis and Kalman Decomposition - Plan

## 1. Objective and Scope

### Primary Objectives

1. **Verify Observability of Discrete-Time System**
   - Use baseline sensor matrix `C` from `matlab/prep_final.m`: `Cd` is 1×12 measuring only `x1` (displacement of mass 1)
   - Construct the observability matrix `O` for the discrete-time system `(Ad, Cd)`
   - Compute the rank of `O` using SVD with a documented tolerance policy
   - Document that the system is **not observable** (rank deficiency expected)

2. **Quantify Rank Deficiency**
   - Report the observed rank of `O` and compare to system dimension `n = 12`
   - Compute the dimension of the unobservable subspace: `dim(unobservable) = n - rank(O)`
   - Document the dimension of the observable subspace: `dim(observable) = rank(O)`

3. **Perform Observability-Based Kalman Decomposition**
   - Construct a similarity transform `T = [To, Tuo]` that partitions the system into observable and unobservable components
   - Form transformed matrices: `Abar = inv(T) * Ad * T` and `Cbar = Cd * T`
   - Verify the partitioned structure where:
     - Observable block: `Abar[0:r, 0:r]` where `r = rank(O)`
     - Unobservable block: `Abar[r:n, r:n]`
     - `Cbar` has structure `[Cbar_o, 0]` where `Cbar_o` corresponds to observable states

4. **Identify Eigenvalues of Observable vs Unobservable Dynamics**
   - Extract eigenvalues of the observable block `Abar[0:r, 0:r]`
   - Extract eigenvalues of the unobservable block `Abar[r:n, r:n]`
   - Report and document these eigenvalue lists for analysis

### Scope Boundaries

- **System Configuration**: Discrete-time system `(Ad, Bd, Cd)` with `Cd = [1 0 0 0 0 0 0 0 0 0 0 0]` from `prep_final.m`
- **Analysis Focus**: Observability analysis only (no controllability, no observer design, no control design)
- **Output Format**: Rank report, decomposition structure, eigenvalue lists, and verification results

## 2. Knowledge Prerequisites

### Theoretical Concepts

1. **Discrete-Time Observability**
   - Definition: System `(Ad, Cd)` is observable if the observability matrix `O` has full rank `n`
   - Observability matrix construction: `O = [Cd; Cd*Ad; Cd*Ad^2; ...; Cd*Ad^(n-1)]`
   - Dimensions: For `Cd` with shape `(p, n)`, `O` has shape `(n*p, n)` where `p = 1` for Part 1

2. **Rank Computation with Numerical Tolerance**
   - Numerical rank vs. theoretical rank: SVD-based rank computation requires tolerance threshold
   - Policy: Use SVD singular values and set threshold based on maximum singular value and machine precision
   - Common approach: `rank = count(sigma_i > tol * max(sigma))` where `tol` is relative tolerance (e.g., `1e-10`)
   - Documentation requirement: State the tolerance policy explicitly in results

3. **Similarity Transforms and Subspace Decomposition**
   - Similarity transform: `Abar = inv(T) * Ad * T` preserves eigenvalues
   - Observability decomposition: Transform coordinates to separate observable and unobservable subspaces
   - Construction: Build basis for observable subspace (columns of `To`) and unobservable subspace (columns of `Tuo`)
   - Result: `T = [To, Tuo]` is invertible transformation matrix

4. **Interpretation of Transformed Structure**
   - Partitioned `Abar` structure:
     ```
     Abar = [Aoo  Aou]
            [0    Auu]
     ```
     where `Aoo` is observable block, `Auu` is unobservable block, `Aou` is coupling term (may be nonzero)
   - Partitioned `Cbar` structure: `Cbar = [Cbar_o, 0]` where only observable states appear in output
   - Eigenvalue interpretation: Eigenvalues of `Aoo` belong to observable modes, eigenvalues of `Auu` belong to unobservable modes

### Implementation Prerequisites

- Understanding of NumPy linear algebra functions: `np.linalg.svd()`, `np.linalg.qr()`, `np.linalg.inv()`
- Understanding of SciPy signal processing: `cont2discrete()` (already available from Part 0)
- Familiarity with matrix dimensions and indexing for partitioned matrices

## 3. Python Approach Overview

### 3.1 Model Construction (Reuse Part 0)

- Use `python/utils/build_model.py`:
  - `A, B, C = build_continuous_model()` to get continuous-time matrices
  - `Ad, Bd, Cd, Dd = discretize_zoh(A, B, C, Ts=0.01)` to get discrete-time matrices
- Verify dimensions: `Ad` is 12×12, `Cd` is 1×12 (measures `x1` only)

### 3.2 Observability Matrix Construction

**Script/Module**: `python/part1/observability_rank.py`

**High-level approach**:
1. For discrete-time system `(Ad, Cd)`, construct observability matrix `O`:
   - `O = [Cd; Cd*Ad; Cd*Ad^2; ...; Cd*Ad^(n-1)]`
   - Since `Cd` is 1×12, `O` will have shape `(12, 12)` (n*p = 12*1 = 12 rows)
2. Stack rows: Build `O` row-by-row using matrix multiplication
3. Verify dimensions: `O` should be `(12, 12)` for Part 1

### 3.3 Rank Computation

**Script/Module**: `python/part1/observability_rank.py` (same as above)

**High-level approach**:
1. Compute SVD: `U, sigma, Vh = np.linalg.svd(O)`
2. Apply tolerance policy:
   - Define tolerance threshold: `tol = max(1e-10, machine_epsilon * max(sigma))`
   - Count singular values above threshold: `rank = count(sigma_i > tol * max(sigma))`
   - Document the tolerance used in output
3. Report results:
   - Rank of `O`
   - Dimension of observable subspace: `r = rank`
   - Dimension of unobservable subspace: `n - r`

### 3.4 Observable Subspace Basis Construction

**Script/Module**: `python/part1/kalman_decomp_obsv.py`

**High-level approach**:
1. Extract observable subspace basis:
   - Option A: Use QR decomposition on `O^T`: `Q, R = np.linalg.qr(O.T, mode='reduced')`
   - Option B: Use SVD of `O`: Extract first `r` columns of `V` from `U, sigma, Vh = np.linalg.svd(O)`
   - Form `To` as basis for observable subspace (shape `(n, r)`)
2. Complete basis for unobservable subspace:
   - Option A: Use null space of `O`: `Tuo = null_space(O)` via SVD of `O.T`
   - Option B: Use Gram-Schmidt or QR to extend `To` to full basis
   - Form `Tuo` as basis for unobservable subspace (shape `(n, n-r)`)
3. Construct transformation: `T = np.hstack([To, Tuo])` (shape `(n, n)`)
4. Verify invertibility: Check condition number `cond(T)` and document

### 3.5 Similarity Transform and Partitioned Matrices

**Script/Module**: `python/part1/kalman_decomp_obsv.py` (same as above)

**High-level approach**:
1. Compute transformed matrices:
   - `Tinv = np.linalg.inv(T)`
   - `Abar = Tinv @ Ad @ T`
   - `Cbar = Cd @ T`
2. Verify partitioned structure:
   - Check that `Cbar[:, r:n]` is approximately zero (within tolerance)
   - Check that `Abar[r:n, 0:r]` may be nonzero (coupling term), but `Abar[r:n, r:n]` is unobservable block
   - Document block dimensions and structure
3. Extract blocks:
   - Observable block: `Aoo = Abar[0:r, 0:r]`
   - Unobservable block: `Auu = Abar[r:n, r:n]`
   - Observable output: `Cbar_o = Cbar[:, 0:r]`

### 3.6 Eigenvalue Extraction

**Script/Module**: `python/part1/kalman_decomp_obsv.py` (same as above)

**High-level approach**:
1. Compute eigenvalues of observable block: `eig_obs = np.linalg.eigvals(Aoo)`
2. Compute eigenvalues of unobservable block: `eig_unobs = np.linalg.eigvals(Auu)`
3. Optionally compute eigenvalues of full `Ad` for cross-check (should match union of `eig_obs` and `eig_unobs`)
4. Store results in structured format (dictionary or named tuple) for reporting

### 3.7 Results Compilation and Reporting

**Script/Module**: `python/part1/run_observability.py` or `python/part1/main.py`

**High-level approach**:
1. Orchestrate calls to `observability_rank.py` and `kalman_decomp_obsv.py`
2. Collect results:
   - Rank of `O`
   - Dimensions of observable/unobservable subspaces
   - Eigenvalues of observable and unobservable blocks
   - Condition number of transformation `T`
3. Save results table to `python/part1/outputs/observability_results.txt` or `.csv`
4. Optionally generate summary plots (if needed for report)

## 4. Required Inputs and Resources

### System Matrices (from Part 0)

- **Source**: `python/utils/build_model.py`
  - Continuous-time: `A` (12×12), `B` (12×3), `C` (1×12)
  - Discrete-time: `Ad` (12×12), `Bd` (12×3), `Cd` (1×12) at `Ts = 0.01`
  - Discretization method: Zero-order hold (ZOH) matching MATLAB `c2d()`

### Reference Files

- **`matlab/prep_final.m`**: Authoritative source for `C` matrix definition (line 32)
- **`docs/00_anchor.md`**: System conventions, state ordering, dimension expectations
- **`docs/sources/final_exam.pdf`**: Check for any Part 1-specific requirements or reporting format preferences (optional verification)

### Tolerance Policy (to be defined)

- **SVD rank tolerance**: Define relative tolerance (e.g., `1e-10`) or use machine-epsilon-based policy
- **Numerical equality checks**: Use `np.allclose()` for reconstruction verification (e.g., `rtol=1e-10, atol=1e-12`)
- **Zero-block checks**: Tolerance for verifying `Cbar[:, r:n] ≈ 0` (e.g., `max(abs(Cbar[:, r:n])) < 1e-10`)

### MATLAB Comparison Functions (for future cross-check)

Identify MATLAB functions for later validation (not implemented in this plan):
- `obsv(Cd, Ad)`: Construct observability matrix
- `rank(O, tol)`: Compute rank with tolerance
- `canon(sys, 'kalman')`: Kalman canonical form (may require observable/controllable variant)
- `obsvf(Ad, Cd)`: Observability staircase form (alternative decomposition method)

## 5. Deliverables

### Documentation

- **`docs/02_part1_observability/plan.md`**: This plan document
- **`docs/02_part1_observability/closeout.md`**: Results summary (to be created after implementation)

### Python Code Structure

**Directory**: `python/part1/`

1. **`observability_rank.py`** (or `obsv_rank.py`)
   - Function: `construct_observability_matrix(Ad, Cd)` → returns `O`
   - Function: `compute_observability_rank(O, tol=None)` → returns `rank, tol_used`
   - Function: `analyze_observability(Ad, Cd, tol=None)` → returns dictionary with rank, dimensions, and `O` matrix

2. **`kalman_decomp_obsv.py`** (or `obsv_decomp.py`)
   - Function: `construct_observable_basis(O, rank)` → returns `To` (basis for observable subspace)
   - Function: `construct_unobservable_basis(O, To, rank)` → returns `Tuo` (basis for unobservable subspace)
   - Function: `kalman_decomposition_observable(Ad, Cd, tol=None)` → returns dictionary with:
     - `T`: Transformation matrix
     - `Tinv`: Inverse of `T`
     - `Abar`: Transformed `Ad`
     - `Cbar`: Transformed `Cd`
     - `Aoo`: Observable block
     - `Auu`: Unobservable block
     - `Cbar_o`: Observable output block
     - `eig_obs`: Eigenvalues of observable block
     - `eig_unobs`: Eigenvalues of unobservable block
     - `cond_T`: Condition number of `T`

3. **`run_observability.py`** (or `main.py`)
   - Main runner script that:
     - Loads `Ad, Bd, Cd` using Part 0 utilities
     - Calls observability rank analysis
     - Calls Kalman decomposition
     - Saves results table to output directory
     - Prints summary to console

4. **`python/part1/outputs/`** (directory)
   - `observability_results.txt` or `.csv`: Table of key results (rank, dimensions, eigenvalue counts)
   - Optional: `O_matrix.txt`: Observability matrix (for reference)
   - Optional: `Abar_matrix.txt`: Transformed `Ad` matrix (for reference)
   - Optional: `eigenvalues_obs.txt`, `eigenvalues_unobs.txt`: Eigenvalue lists

### Results Table Format

The results table should include at minimum:
- Rank of observability matrix `O`
- Dimension of observable subspace
- Dimension of unobservable subspace
- Number of observable eigenvalues
- Number of unobservable eigenvalues
- Condition number of transformation `T`
- SVD tolerance used for rank computation

## 6. Validation Checklist

### 6.1 Dimension Checks

- [ ] `Ad` is 12×12
- [ ] `Cd` is 1×12
- [ ] Observability matrix `O` has shape `(12, 12)` (since `p=1`, `n=12`, so `n*p=12`)
- [ ] Transformation matrix `T` is 12×12
- [ ] `Abar` is 12×12
- [ ] `Cbar` is 1×12
- [ ] Observable block `Aoo` has shape `(r, r)` where `r = rank(O)`
- [ ] Unobservable block `Auu` has shape `((n-r), (n-r))` where `n-r = 12 - rank(O)`
- [ ] `Cbar_o` has shape `(1, r)`

### 6.2 Rank Result Reproducibility

- [ ] Rank computation uses documented tolerance policy
- [ ] Tolerance is explicitly stated in output/results
- [ ] Rank result is reproducible when tolerance is fixed (same input → same rank)
- [ ] Rank is less than `n = 12` (system is not fully observable)

### 6.3 Decomposition Sanity Checks

- [ ] `T` is invertible: `cond(T) < 1e12` (condition number is reasonable)
- [ ] Reconstruction check for `Ad`: `np.allclose(Ad, T @ Abar @ Tinv, rtol=1e-10, atol=1e-12)`
- [ ] Reconstruction check for `Cd`: `np.allclose(Cd, Cbar @ Tinv, rtol=1e-10, atol=1e-12)`
- [ ] Observable/unobservable partition dimensions consistent:
  - Sum of block dimensions equals `n`: `r + (n-r) = 12`
- [ ] Output coupling check: `max(abs(Cbar[:, r:n])) < 1e-10` (unobservable states do not appear in output)

### 6.4 Eigenvalue Consistency Checks

- [ ] Eigenvalues of `Ad` match union of eigenvalues of `Aoo` and `Auu` (within numerical tolerance)
- [ ] Total eigenvalue count: `len(eig_obs) + len(eig_unobs) = 12` (accounting for multiplicities if complex)
- [ ] Observable eigenvalues belong to `Aoo` block
- [ ] Unobservable eigenvalues belong to `Auu` block

### 6.5 Cross-Check Plan for MATLAB (Future Validation)

**MATLAB functions to use for comparison** (not implemented in this plan):
- `O_matlab = obsv(Cd, Ad)`: Construct observability matrix
- `rank_matlab = rank(O_matlab, tol)`: Compute rank with same tolerance
- `[Abar_matlab, Bbar_matlab, Cbar_matlab, T_matlab] = canon(ss(Ad, Bd, Cd, 0, Ts), 'kalman')`: Kalman canonical form
  - Note: `canon(..., 'kalman')` produces controllable/observable decomposition; verify it matches observability-only decomposition
- Alternative: `[Abar_matlab, Cbar_matlab, T_matlab] = obsvf(Ad, Cd)`: Observability staircase form

**Comparison checks**:
- Compare `O` matrices: `max(abs(O_python - O_matlab)) < tolerance`
- Compare ranks: `rank_python == rank_matlab`
- Compare transformed structures: Verify block partitioning matches (dimensions and zero-block locations)

## 7. Non-Goals

Part 1 explicitly does NOT include:

- **Observer Design**: No Luenberger observer, no pole placement, no observer gain computation
- **Controller Design**: No LQR, no pole placement, no feedback control
- **Noisy Simulations**: No process noise, no measurement noise, no Kalman filter design
- **Controllability Analysis**: Focus is on observability only (controllability may be analyzed in future parts)
- **Tuning Choices**: No design parameters to tune beyond:
  - SVD tolerance for rank computation (documented policy)
  - Numerical equality tolerances (documented policy)
- **Sensor Augmentation**: No analysis of alternative `C` matrices (Part 2 will add sensor at mass 6)
- **State Estimation**: No estimation algorithms, no comparison of estimated vs. true states

