---
name: System Audit Parts 0-6
overview: Comprehensive verification-first audit of Parts 0-6 with explicit PASS/FAIL gates. All results documented with evidence and reproducibility notes.
todos:
  - id: phase0-setup
    content: "PHASE 0: Read authoritative docs, record invariants, capture environment info (git hash, Python/NumPy/SciPy versions, OS)"
    status: completed
  - id: phase1-static-audit
    content: "PHASE 1: Static code audit - search for Ts/N, noise settings, cost definitions, Cmeas/x0/xhat0, process noise entry point. Document violations."
    status: completed
    dependencies:
      - phase0-setup
  - id: phase2-rerun-parts
    content: "PHASE 2: Re-run Parts 0-6 from scratch, validate gates, capture console outputs and evidence. Run reproducibility checks for Parts 5-6."
    status: completed
    dependencies:
      - phase1-static-audit
  - id: phase3-cross-consistency
    content: "PHASE 3: Cross-part consistency audit - verify Ts/N, indexing, Cmeas, x0/xhat0, K/Lk reuse across parts. Create consistency table."
    status: completed
    dependencies:
      - phase2-rerun-parts
  - id: phase4-documentation
    content: "PHASE 4: Create system audit report and evidence doc with all gates, evidence, and reproducibility notes. Add missing logging if needed."
    status: completed
    dependencies:
      - phase3-cross-consistency
---

# System Audit Plan - Parts 0-6

## Overview

This plan executes a skeptical, verification-first audit of Parts 0-6. Every claimed property must be evidenced by code inspection plus logged outputs from clean runs.

## PHASE 0: One-Time Setup and Repository Snapshot

### Step 0.1: Read Authoritative Documentation

**Files to read (in order):**

1. `docs/sources/final_exam_extract.md` - Exam requirements (already read)
2. `docs/00_anchor.md` - Project conventions (already read)
3. `docs/07_comprehensive_review_part0_to_part4/cross_part_invariants.md` - Frozen invariants (already read)
4. `docs/07_comprehensive_review_part0_to_part4/results_intake_template.md` - Results template (already read)
5. `docs/04_progress_audit/integration_readiness_check.md` - Prior checks (already read)

**Action:** Record exact formulas and constraints in scratch notes. Key invariants:

- Ts = 0.01, N = 1000 (frozen)
- x shape (12, N+1), u shape (m, N), cost sums k=0..N-1
- Cmeas (Part 2): measures x1 and x6, shape (2,12)
- x0 = [0,0,0,1,1,1,0,0,0,0,0,0]^T, xhat0 = [0,0,0,0,0,1,0,0,0,0,0,0]^T
- Noise: Qw = 0.05 * I3, Rv = 0.1 * I_p, seed = 42
- Process noise: w enters via Bd (x[k+1] = Ad x[k] + Bd u[k] + Bd w[k])
- Part 6 cost: J_true uses y_true = Cmeas x (official), J_meas optional

### Step 0.2: Capture Environment Information

**Commands to run:**

```bash
git rev-parse HEAD
python --version
python -c "import numpy, scipy, platform; print('numpy', numpy.__version__); print('scipy', scipy.__version__); print(platform.platform())"
```

**Action:** Save outputs to temporary file for later pasting into `docs/10_system_audit_evidence.md`.

## PHASE 1: Static Code Audit - Fast Search for Invariant Violations

### Step 1.1: Search for Ts and N Definitions

**Commands:**

```bash
rg -n "Ts\s*=" python/
rg -n "N\s*=" python/
```

**Gates:**

- All Ts assignments must be 0.01 (or derived from 0.01)
- All N assignments must be 1000 (or derived from 1000)
- No hardcoded duplicates with different values

**Action:** For each match, open file and verify:

- Ts and N defined once and reused, or duplicated consistently
- Arrays use N+1 for x and N for u exactly
- Cost sums exactly k=0..N-1

### Step 1.2: Search for Noise Settings

**Commands:**

```bash
rg -n "seed\s*=" python/part5 python/part6
rg -n "Qw|Rv|process noise|measurement noise" python/part5 python/part6
```

**Gates:**

- seed = 42 in Parts 5 and 6
- Qw = 0.05 * I3 (or equivalent scaling)
- Rv = 0.1 * I_p where p matches C matrix rows
- w enters only through Bd (verify x[k+1] = Ad x[k] + Bd u[k] + Bd w[k])

**Action:** Inspect `python/part5/run_kalman_filter.py` and `python/part6/run_lqg.py`:

- Verify noise generation uses seed 42
- Verify Qw and Rv definitions
- Verify process noise enters via Bd (not separate channel)

### Step 1.3: Search for Cost Definitions

**Commands:**

```bash
rg -n "J_true|J_meas|stage cost|cost" python/part3 python/part4 python/part6
```

**Gates:**

- Part 3 and 4: Cost sums k=0..N-1
- Part 6: J_true uses y_true = Cmeas x (official), J_meas optional and labeled non-official
- Cost uses y1 and y6 from Cmeas (or extracted from state)

**Action:** Inspect cost computation functions:

- Verify summation range
- Verify y_true vs y_meas usage in Part 6

### Step 1.4: Search for Cmeas, x0, xhat0 Consistency

**Commands:**

```bash
rg -n "xhat0|x0|Cmeas|Cy" python/part2 python/part3 python/part4 python/part5 python/part6
```

**Gates:**

- Cmeas identical across Parts 2-6 (shape (2,12), measures x1 and x6)
- x0 and xhat0 match invariants doc in Parts 2-6
- Cost uses Cy = Cmeas (or extracts y1, y6 from state)

**Action:** For each part, verify:

- Cmeas definition matches Part 2 exactly
- x0 and xhat0 match invariants doc
- Cost computation uses correct output selector

### Step 1.5: Search for Process Noise Entry Point

**Commands:**

```bash
rg -n "Bd\s*@\s*w|+ Bd.*w" python/part5 python/part6
```

**Gates:**

- Process noise w enters via Bd (x[k+1] = Ad x[k] + Bd u[k] + Bd w[k])
- No separate noise channel

**Action:** Inspect simulation loops in Parts 5 and 6 to verify noise model.

### Step 1.6: Document Violations

**Action:** Create preliminary list of violations in audit report structure. Each violation must:

- Point to exact file and line number
- State expected vs actual
- Classify as hard gate failure or warning

## PHASE 2: Re-Run and Validate Parts 0-6 from Scratch

### General Run Protocol for Each Part

1. Archive or delete only outputs for the part being re-run
2. Run the part's run script from repo root: `python python/partX/run_*.py`
3. Save console output to temporary file
4. After run: `ls -la python/partX/outputs/` and capture
5. Open `outputs/results.txt` and evaluate gates
6. Verify required plots exist

### Part 0: Baseline Verification

**Command:**

```bash
python python/part0/baseline_check.py
python python/part0/validate_matlab.py  # if exists
```

**Gates:**

- [GATE-P0-1] Discretization uses ZOH with Ts=0.01
- [GATE-P0-2] Simulation runs and produces plots: `output_plot.png`, `displacements_plot.png`
- [GATE-P0-3] MATLAB comparison (if used) matches documented procedure

**Evidence to capture:**

- Console output showing discretization method
- Plot file existence and timestamps
- Matrix shapes and fingerprints

### Part 1: Observability Analysis

**Command:**

```bash
python python/part1/run_observability.py
```

**Gates:**

- [GATE-P1-1] Observability rank computed and logged with tolerance and singular values
- [GATE-P1-2] Kalman decomposition outputs exist: `O_matrix.txt`, `O_matrix_summary.txt`, `eigenvalues_obs.txt`, `eigenvalues_unobs.txt`, `Abar_matrix.txt`, `observability_results.txt`

**Evidence to capture:**

- Full console output
- All output files listed above
- Directory listing

### Part 2: Observer Design

**Command:**

```bash
python python/part2/run_observer_sim.py
```

**Gates:**

- [GATE-P2-1] Uses Cmeas measuring x1 and x6 exactly (shape (2,12))
- [GATE-P2-2] x0 and xhat0 match invariants doc
- [GATE-P2-3] L has shape (12,2)
- [GATE-P2-4] Observer Aobs = Ad - L Cmeas stable (spectral radius < 1)
- [GATE-P2-5] Spectral radius approximately 0.8 (strong consistency check)
- [GATE-P2-6] results.txt prints desired poles and achieved poles

**Evidence to capture:**

- Full `results.txt`
- Console output
- Plots: `outputs_comparison.png`, `estimation_errors.png`, `all_state_errors.png`
- Directory listing

### Part 3: LQR with Observer

**Command:**

```bash
python python/part3/run_lqr_with_observer.py
```

**Gates:**

- [GATE-P3-1] PBH stabilizability checks pass for eigenvalues with |lambda| >= 1 - tol, tol logged
- [GATE-P3-2] Detectability check for DARE passes and is logged
- [GATE-P3-3] K shape (3,12), L shape (12,2)
- [GATE-P3-4] Closed-loop Acl = Ad - Bd K stable (spectral radius < 1)
- [GATE-P3-5] Controller uses xhat not x (explicit diagnostic in results.txt)
- [GATE-P3-6] Cost definition: J = sum_{k=0..N-1} (u[k]^T u[k] + y1[k]^2 + y6[k]^2) with y = Cmeas x
- [GATE-P3-7] J and max|u| values match cross_part_invariants.md baselines (consistency check)

**Evidence to capture:**

- Full `results.txt`
- Console output
- Plots: `outputs_y1_y6.png`, `inputs_u1_u2_u3.png`, `estimation_error_norm.png`
- K_matrix.npy and L_matrix.npy existence
- Directory listing

### Part 4: Reduced Input LQR

**Command:**

```bash
python python/part4/run_lqr_reduced_input.py
```

**Gates:**

- [GATE-P4-1] Removes u3, uses only 2 inputs consistently
- [GATE-P4-2] Same cost indexing and definition as Part 3 (but with reduced u)
- [GATE-P4-3] Observer and initial conditions consistent with required reuse
- [GATE-P4-4] Cost expected higher than Part 3 (consistency expectation)

**Evidence to capture:**

- Full `results.txt`
- Console output
- Plots: `outputs_y1_y6.png`, `inputs_u1_u2.png`, `estimation_error_norm.png`
- Directory listing

### Part 5: Kalman Filter

**Command:**

```bash
python python/part5/run_kalman_filter.py
python -c "import numpy as np; d=np.load('python/part5/outputs/traj.npz'); print('Keys:', d.files); print('Shapes:', {k: d[k].shape for k in d.files})"
```

**Gates:**

- [GATE-P5-1] Noise model: w enters via Bd, seed 42, Qw = 0.05 * I3, Rv = 0.1 * I2
- [GATE-P5-2] Lk computed and saved, shape (12,2)
- [GATE-P5-3] Estimator spectral radius < 1 and logged
- [GATE-P5-4] Innovation covariance S invertible, cond(S) <= 1e12
- [GATE-P5-5] traj.npz saved with keys: t, x, xhat, y_true, y_meas, yhat, w, v, errors

**Evidence to capture:**

- Full `results.txt`
- Console output
- Lk_matrix.npy existence
- traj.npz keys and shapes verification
- Plots: `outputs_y_vs_yhat.png`, `estimation_error_norm.png`, `estimation_error_x1_x6.png`, `per_state_rms_bar.png`
- Directory listing

### Part 6: LQG

**Command:**

```bash
python python/part6/run_lqg.py
python -c "import numpy as np; d=np.load('python/part6/outputs/traj.npz'); print('Keys:', d.files); print('Shapes:', {k: d[k].shape for k in d.files})"
```

**Gates:**

- [GATE-P6-1] K loaded from Part 3 outputs, matches exactly (max_abs_diff == 0 or hash match)
- [GATE-P6-2] Lk loaded from Part 5 outputs, matches exactly
- [GATE-P6-3] Controller uses xhat not x (explicit numeric diagnostic: ||u - (-K xhat)|| ≈ 0, ||u - (-K x)|| >> 0)
- [GATE-P6-4] Noise settings frozen: seed 42, Qw = 0.05 * I3, Rv = 0.1 * I2
- [GATE-P6-5] Cost reporting includes J_true based on y_true = Cmeas x (official), J_meas optional
- [GATE-P6-6] No-noise sanity check exists and quantified (differs from Part 3 if estimator differs, reason shown)

**Evidence to capture:**

- Full `results.txt`
- Console output
- K and Lk equality proofs (max_abs_diff values)
- traj.npz keys and shapes verification
- Plots: `outputs_y1_y6_comparison.png`, `outputs_y_meas_vs_yhat.png`, `inputs_u1_u2_u3.png`, `estimation_error_norm.png`
- Directory listing

### Reproducibility Check (Parts 5 and 6)

**Action:** Re-run Parts 5 and 6 twice without changing anything:

```bash
# Part 5 run 1
python python/part5/run_kalman_filter.py > /tmp/part5_run1.txt
# Part 5 run 2
python python/part5/run_kalman_filter.py > /tmp/part5_run2.txt
# Compare key numbers from results.txt
# Compare first 3 samples of w and v from traj.npz

# Part 6 run 1
python python/part6/run_lqg.py > /tmp/part6_run1.txt
# Part 6 run 2
python python/part6/run_lqg.py > /tmp/part6_run2.txt
# Compare key numbers from results.txt
# Compare first 3 samples of w and v from traj.npz
```

**Gates:**

- [GATE-REP-1] Part 5 results.txt key numbers identical between runs
- [GATE-REP-2] Part 5 traj.npz noise samples match (first 3 samples of w and v)
- [GATE-REP-3] Part 6 results.txt key numbers identical between runs
- [GATE-REP-4] Part 6 traj.npz noise samples match

**Evidence to capture:**

- Comparison of results.txt key metrics
- First 3 samples of w and v from each run

## PHASE 3: Cross-Part Consistency Audit

### Step 3.1: Create Consistency Check Table

**Checks to perform:**

1. **Ts and N consistency:** Extract from each part's results.txt or code
2. **x and u indexing:** Verify x shape (12, N+1), u shape (m, N) in all parts
3. **Cmeas consistency:** Verify Cmeas identical in Parts 2-6
4. **x0 and xhat0 consistency:** Verify match invariants doc in Parts 2-6
5. **Part 6 uses K from Part 3:** Prove exact match (max_abs_diff == 0)
6. **Part 6 uses Lk from Part 5:** Prove exact match
7. **Part 3 baseline J and max|u|:** Compare to cross_part_invariants.md values

**Action:** Create section in audit report with evidence quotes from each part.

## PHASE 4: Documentation and Result Report Assembly

### Step 4.1: Create System Audit Report

**File:** `docs/10_system_audit_report.md`

**Structure:**

1. **Title, commit hash, date, environment**

   - Git commit hash
   - Date of audit
   - Python version, NumPy version, SciPy version, OS

2. **Scope**

   - Parts 0 to 6 audited
   - Verification-first approach

3. **How to Reproduce**

   - Exact commands for each part
   - Environment setup notes

4. **Gate Summary Table**

   - Columns: Part, Gate ID, Description, PASS/FAIL, Evidence pointer
   - One row per gate from all parts

5. **Part-by-Part Sections**

   - Part 0: what checked, results, evidence links
   - Part 1: ...
   - Part 2: ...
   - Part 3: ...
   - Part 4: ...
   - Part 5: ...
   - Part 6: ...

6. **Cross-Part Invariants Section**

   - Table with checks and evidence
   - Ts/N consistency
   - Cmeas consistency
   - x0/xhat0 consistency
   - K/Lk reuse proofs

7. **Findings and Fixes Section**

   - Any violations found
   - Fixes applied (if any)
   - Re-run notes and result changes

8. **Open Issues** (if any)

### Step 4.2: Create System Audit Evidence Document

**File:** `docs/10_system_audit_evidence.md`

**Structure:**

1. **Environment Information**

   - Git commit hash
   - Python, NumPy, SciPy versions
   - OS information

2. **Part-by-Part Evidence**

   - For each part (0-6):
     - Console output (full or key excerpts)
     - Directory listing (`ls -la python/partX/outputs/`)
     - Key blocks from results.txt
     - Matrix hashes/fingerprints (K, L, Lk)
     - traj.npz verification (keys, shapes, first 3 noise samples)

3. **Reproducibility Evidence**

   - Part 5 and Part 6 re-run comparisons
   - Noise sample matching proof

### Step 4.3: Add Missing Logging (If Needed)

**Action:** If any required diagnostics are missing from code:

- Add logging for: Ts, N, shapes, cost summation range, x0, xhat0, Cmeas, spectral radii
- Add Part 6 diagnostic: ||u - (-K xhat)|| and ||u - (-K x)||
- Add Part 6 K equality proof: max_abs_diff
- Add Part 5-6 seed and first 3 noise samples logging

**Note:** Only add minimal logging needed for gates. Do not change algorithm logic.

## Deliverables Checklist

- [ ] `docs/10_system_audit_report.md` - Complete with all gates and evidence pointers
- [ ] `docs/10_system_audit_evidence.md` - Complete with console outputs, listings, hashes

## Stop Conditions

- If any hard gate fails: Mark FAIL in report, identify root cause (exact file/line), do not hand-wave
- If fix is small and clearly correct: Implement, document change, re-run impacted parts, update report
- Otherwise: Leave as documented failure with recommended next step

## Notes

- All gates must have explicit PASS or FAIL status
- Evidence must be self-contained (reader can reproduce with provided commands)
- Frozen invariants must be respected unless exam explicitly requires otherwise