# Progress Audit Plan - Parts 0-2

## Purpose

This document provides an executable audit procedure to verify that all results from Parts 0, 1, and 2 can be reproduced by running existing repository code. The audit validates source traceability to `docs/sources/final_exam.pdf`, checks pass/fail gates, and documents all artifacts.

**Critical rule**: Every result must be reproduced by running existing repo code. If a fact cannot be verified from files in the repo, write UNKNOWN and specify exactly what file, script, or excerpt is needed.

---

## A) Audit Preflight

### A.1 Environment Recording

Before running any code, record the audit environment:

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

5. **Package Dependencies**:
   ```bash
   pip freeze > docs/04_progress_audit/pip_freeze_audit.txt
   ```
   Record location: `docs/04_progress_audit/pip_freeze_audit.txt`

### A.2 Output Locations

- **Part 0 outputs**: `python/part0/` (plots: `output_plot.png`, `displacements_plot.png`)
- **Part 1 outputs**: `python/part1/outputs/`
- **Part 2 outputs**: `python/part2/outputs/`
- **Audit notes**: `docs/04_progress_audit/` (can store audit-specific logs and notes)

### A.3 Clean Run Instructions

Before running each part, clean or archive previous outputs:

1. **Option 1 (Archive)**:
   ```bash
   mkdir -p docs/04_progress_audit/archive_$(date +%Y%m%d_%H%M%S)
   # Archive if outputs exist
   [ -d python/part0/outputs ] && mv python/part0/outputs docs/04_progress_audit/archive_*/part0_outputs 2>/dev/null || true
   [ -f python/part0/*.png ] && mv python/part0/*.png docs/04_progress_audit/archive_*/ 2>/dev/null || true
   [ -d python/part1/outputs ] && mv python/part1/outputs docs/04_progress_audit/archive_*/part1_outputs 2>/dev/null || true
   [ -d python/part2/outputs ] && mv python/part2/outputs docs/04_progress_audit/archive_*/part2_outputs 2>/dev/null || true
   ```

2. **Option 2 (Delete)**:
   ```bash
   rm -rf python/part0/outputs python/part0/*.png 2>/dev/null || true
   rm -rf python/part1/outputs/* 2>/dev/null || true
   rm -rf python/part2/outputs/* 2>/dev/null || true
   ```

**Record which option was used**: `[RECORD HERE]`

---

## B) Entrypoint Discovery (Mandatory)

### B.1 Discovery Steps

Even though scripts are identified below, follow these discovery steps:

1. **List Part 0 directory contents**:
   ```bash
   ls -la python/part0/
   ```
   Record output: `[RECORD IN CLOSEOUT]`

2. **List Part 1 directory contents**:
   ```bash
   ls -la python/part1/
   ```
   Record output: `[RECORD IN CLOSEOUT]`

3. **List Part 2 directory contents**:
   ```bash
   ls -la python/part2/
   ```
   Record output: `[RECORD IN CLOSEOUT]`

4. **Identify runner scripts**:
   - Look for files with `if __name__ == '__main__':` pattern
   - Check for `run_*.py`, `main.py`, or documented entrypoints
   - Check for README files in each directory

### B.2 Identified Entrypoints

After discovery, record the exact commands used:

**Part 0**:
- Script: `python/part0/baseline_check.py`
- Command: `python python/part0/baseline_check.py`
- Status: `[CONFIRMED / UNKNOWN]`
- Notes: `[If UNKNOWN, state what is missing]`

**Part 1**:
- Script: `python/part1/run_observability.py`
- Command: `python python/part1/run_observability.py`
- Status: `[CONFIRMED / UNKNOWN]`
- Notes: `[If UNKNOWN, state what is missing]`

**Part 2**:
- Script: `python/part2/run_observer_sim.py`
- Command: `python python/part2/run_observer_sim.py`
- Status: `[CONFIRMED / UNKNOWN]`
- Notes: `[If UNKNOWN, state what is missing]`

### B.3 Working Directory

All commands assume execution from repository root: `/home/yunusdanabas/EE571_FinalProject/`

If execution from a different directory is required, record it: `[RECORD IF DIFFERENT]`

---

## C) Source Traceability Verification (Manual, Crucial)

### C.1 Procedure

This section requires manual inspection of `docs/sources/final_exam.pdf`.

**Steps**:
1. Open `docs/sources/final_exam.pdf`
2. Locate definitions for:
   - Baseline measurement definition (C or Cd measuring x1 only)
   - Augmented measurement definition (Cd_new measuring x1 and x6 for Part 2)
   - Initial conditions for Part 2 (x0 and xhat0 if specified)

### C.2 Traceability Table

Complete this table in the closeout document:

| Item | Expected Definition | Found (Yes/No) | Page | Section | Notes |
|------|---------------------|----------------|------|---------|-------|
| Baseline C (Part 0) | C is 1×12, measures x1 only | [ ] | [ ] | [ ] | [ ] |
| Cd_new (Part 2) | Cd_new is 2×12, measures x1 and x6 | [ ] | [ ] | [ ] | [ ] |
| x0 (Part 2) | Initial condition for actual state | [ ] | [ ] | [ ] | [ ] |
| xhat0 (Part 2) | Initial condition for observer state | [ ] | [ ] | [ ] | [ ] |

### C.3 Pass/Fail Gate

- **PASS**: All four items found in PDF with matching definitions
- **FAIL/UNVERIFIED**: Any item not found or definition mismatch
- **Action**: Mark UNVERIFIED items in closeout and do not claim correctness

---

## D) Part 0 Execution Checks

### D.1 Continuous Model Check

**Reference**: `matlab/prep_final.m`

**Check 1.1: Matrix Dimensions**
- Gate: A is 12×12, B is 12×3, baseline C is 1×12, Ts is 0.01
- Verification method: Inspect `python/utils/build_model.py` function `build_continuous_model()`
- Record:
  - A shape: `[RECORD]`
  - B shape: `[RECORD]`
  - C shape: `[RECORD]`
  - Ts: `[RECORD]`
- Pass/Fail: `[PASS / FAIL]`

**Check 1.2: Matrix Entrywise Match**
- Gate: Python continuous matrices match `prep_final.m` entrywise
- Verification method: Manual comparison or run validation script if available (`python/part0/validate_matlab.py`)
- Record: Max absolute difference: `[RECORD]`
- Tolerance: `max(abs(A_python - A_matlab)) < 1e-10`
- Pass/Fail: `[PASS / FAIL]`

### D.2 ZOH Discretization Check

**Check 2.1: Discretization Method**
- Gate: ZOH method confirmed
- Verification: Inspect `python/utils/build_model.py` function `discretize_zoh()`
- Check: Uses `scipy.signal.cont2discrete` with method='zoh' or equivalent
- Record: Method used: `[RECORD]`
- Pass/Fail: `[PASS / FAIL]`

**Check 2.2: Discrete Matrix Dimensions**
- Gate: Ad 12×12, Bd 12×3, Cd 1×12, Dd 1×3
- Verification: Run Part 0 script and check console output or inspect code
- Record:
  - Ad shape: `[RECORD]`
  - Bd shape: `[RECORD]`
  - Cd shape: `[RECORD]`
  - Dd shape: `[RECORD]`
- Pass/Fail: `[PASS / FAIL]`

**Check 2.3: Numerical Fingerprints**
- Record:
  - spectral_radius(Ad): `[RECORD]`
  - max(abs(Ad)): `[RECORD]`
  - Any saved matrix hashes (if repo provides): `[RECORD]`

**Check 2.4: Optional MATLAB Comparison**
- If MATLAB is available, compare Python Ad, Bd with MATLAB c2d ZOH:
  ```matlab
  % In MATLAB
  run('matlab/prep_final.m')
  % Compare Ad, Bd, Cd with Python outputs
  ```
- Record:
  - max(abs(Ad_python - Ad_matlab)): `[RECORD]`
  - Tolerance check: `[PASS / FAIL / NOT RUN]`

### D.3 Baseline Simulation Check

**Check 3.1: Initial Condition**
- Gate: x0 equals `prep_final.m` x0
- Expected: x0 = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0]
- Verification: Check `python/part0/baseline_check.py` line 37 or console output
- Record: x0 = `[RECORD]`
- Pass/Fail: `[PASS / FAIL]`

**Check 3.2: Input Signal**
- Gate: u is zero unless specified otherwise
- Verification: Check code (expected: `u = np.zeros((3, N))`)
- Record: u specification: `[RECORD]`
- Pass/Fail: `[PASS / FAIL]`

**Check 3.3: Simulation Dimensions**
- Gate: x shape (12, N) and y shape (rows(Cd), N)
- Expected: x shape (12, 1000), y shape (1, 1000)
- Verification: Check console output from Part 0 run
- Record:
  - x shape: `[RECORD]`
  - y shape: `[RECORD]`
  - N (simulation steps): `[RECORD]`
- Pass/Fail: `[PASS / FAIL]`

### D.4 Plot Policy Check

**Check 4.1: Output Plot Legend**
- Gate: If plotting y = Cd x with baseline Cd, legend must have 1 trace and must not be d1..d6
- Verification: Inspect `python/part0/output_plot.png`
- Check: Plot shows exactly 1 trace with label matching output dimension
- Record:
  - Number of traces: `[RECORD]`
  - Legend labels: `[RECORD]`
- Pass/Fail: `[PASS / FAIL]`

**Check 4.2: Displacements Plot**
- Gate: If plotting six displacements, require a selector such as C_disp = [I6, 0] and legend must match 6 traces
- Verification: Inspect `python/part0/displacements_plot.png` and code (line 115 in baseline_check.py)
- Check: Code uses `plot_displacements(t, x, ...)` which extracts first 6 states, not y = Cd x
- Record:
  - Number of traces: `[RECORD]`
  - Legend labels: `[RECORD]`
  - Code method: `[RECORD - should use states, not outputs]`
- Pass/Fail: `[PASS / FAIL]`

### D.5 Part 0 Artifacts

Record produced artifacts:

- `python/part0/output_plot.png`: `[EXISTS / MISSING]`
- `python/part0/displacements_plot.png`: `[EXISTS / MISSING]`
- Console output: `[SAVED TO: docs/04_progress_audit/part0_console.txt / NOT SAVED]`

---

## E) Part 1 Execution Checks

### E.1 Observability Matrix and Rank

**Check 1.1: Observability Matrix Dimensions**
- Gate: Observability matrix dimensions correct
- Expected: O shape (12, 12) for n=12, p=1 (since Cd is 1×12)
- Verification: Check console output or `python/part1/outputs/O_matrix_summary.txt`
- Record:
  - O shape: `[RECORD]`
  - n (states): `[RECORD]`
  - p (outputs): `[RECORD]`
- Pass/Fail: `[PASS / FAIL]`

**Check 1.2: Rank Computation**
- Gate: Rank computed with explicit SVD tolerance policy
- Expected: Rank < 12 (system is not fully observable with baseline Cd)
- Verification: Check `python/part1/run_observability.py` console output or `observability_results.txt`
- Record:
  - Rank: `[RECORD]`
  - Tolerance used: `[RECORD]`
  - SVD tolerance policy: `[RECORD - should be documented in observability_rank.py]`
- Pass/Fail: `[PASS / FAIL]`

### E.2 Kalman Decomposition

**Check 2.1: Transformation Matrix T**
- Gate: T invertible and cond(T) recorded
- Verification: Check console output or `observability_results.txt`
- Record:
  - T shape: `[RECORD]`
  - cond(T): `[RECORD]`
  - T invertible: `[YES / NO]`
- Pass/Fail: `[PASS / FAIL - cond(T) < 1e12]`

**Check 2.2: Reconstruction Residuals**
- Gate: Reconstruction residuals for Abar and Cbar recorded and under stated tolerance
- Verification: Check console output (should show "Ad reconstruction: ✓ PASS" and "Cd reconstruction: ✓ PASS")
- Expected tolerance: `rtol=1e-10, atol=1e-12` (from run_observability.py line 270-271)
- Record:
  - Ad reconstruction check: `[PASS / FAIL]`
  - Cd reconstruction check: `[PASS / FAIL]`
  - Residuals (if logged): `[RECORD]`
- Pass/Fail: `[PASS / FAIL]`

**Check 2.3: Block Structure Check**
- Gate: Block structure check that Cbar columns associated with unobservable subspace are near-zero
- Verification: Check console output (should show "Output coupling (Cbar[:, r:] ≈ 0): ✓ PASS")
- Expected: max|Cbar[:, r:]| < 1e-10 (from run_observability.py line 288)
- Record:
  - Output coupling check: `[PASS / FAIL]`
  - max|Cbar[:, r:]|: `[RECORD]`
- Pass/Fail: `[PASS / FAIL]`

**Check 2.4: Eigenvalue Set-Match**
- Gate: Eigenvalues set-match between Ad and Abar
- Verification: Check console output (should show "Eigenvalue consistency: ✓ PASS")
- Expected: Eigenvalues from Ad match union of eigenvalues from Aoo and Auu blocks
- Tolerance: max error < 1e-2 (from run_observability.py line 327)
- Record:
  - Eigenvalue consistency check: `[PASS / FAIL]`
  - Max eigenvalue error: `[RECORD]`
  - Observable eigenvalues count: `[RECORD]`
  - Unobservable eigenvalues count: `[RECORD]`
- Pass/Fail: `[PASS / FAIL]`

### E.3 Part 1 Artifacts

Record produced artifacts:

- `python/part1/outputs/observability_results.txt`: `[EXISTS / MISSING]`
- `python/part1/outputs/O_matrix.txt`: `[EXISTS / MISSING]`
- `python/part1/outputs/O_matrix_summary.txt`: `[EXISTS / MISSING]`
- `python/part1/outputs/Abar_matrix.txt`: `[EXISTS / MISSING]`
- `python/part1/outputs/eigenvalues_obs.txt`: `[EXISTS / MISSING]`
- `python/part1/outputs/eigenvalues_unobs.txt`: `[EXISTS / MISSING]`
- Console output: `[SAVED TO: docs/04_progress_audit/part1_console.txt / NOT SAVED]`

---

## F) Part 2 Execution Checks

### F.1 Cd_new Verification

**Check 1.1: Cd_new Definition**
- Gate: Cd_new definition matches final_exam.pdf citation (otherwise UNVERIFIED)
- Expected: Cd_new is 2×12, measures x1 (row 1) and x6 (row 2)
- Verification: 
  1. Check `python/part2/observer_design.py` function `get_part2_C_matrix()` (lines 26-43)
  2. Cross-reference with final_exam.pdf traceability table (Section C.2)
- Record:
  - Cd_new shape: `[RECORD]`
  - Cd_new definition (code): `[RECORD]`
  - Matches PDF: `[YES / NO / UNVERIFIED]`
- Pass/Fail: `[PASS / UNVERIFIED]`

### F.2 Observability Rank

**Check 2.1: Observability Rank for Cd_new**
- Gate: Rank computed with recorded tolerance
- Expected: Rank = 12 (system should be fully observable with Cd_new measuring x1 and x6)
- Verification: Check console output from Part 2 run or observer design results
- Record:
  - Rank: `[RECORD]`
  - n (states): `[RECORD]`
  - Is observable: `[YES / NO]`
  - Tolerance used: `[RECORD]`
- Pass/Fail: `[PASS / FAIL]`

### F.3 Observer Stability and Logs

**Check 3.1: Dual Controllability Check**
- Gate: Dual controllability check for (Ad^T, Cd_new^T) logged
- Verification: Check console output or observer design results
- Expected: Logs rank and minimum singular value
- Record:
  - Dual controllability rank: `[RECORD]`
  - Minimum singular value: `[RECORD]`
  - Is controllable (dual system): `[YES / NO]`
- Pass/Fail: `[PASS / FAIL]`

**Check 3.2: Observer Gain Dimensions**
- Gate: L shape is (12, 2)
- Verification: Check console output
- Record:
  - L shape: `[RECORD]`
- Pass/Fail: `[PASS / FAIL]`

**Check 3.3: Observer Stability Gate**
- Gate: spectral_radius(Ad - L Cd_new) < 1.0
- Verification: Check console output (should show "Spectral radius: [value]" and "Observer stable: True")
- Record:
  - Spectral radius: `[RECORD]`
  - Observer stable: `[YES / NO]`
- Pass/Fail: `[PASS / FAIL - must be < 1.0]`

**Check 3.4: Desired vs Achieved Poles**
- Gate: Desired poles and achieved poles both logged
- Verification: Check console output or `python/part2/outputs/results.txt`
- Expected: Compare as sets, record max desired magnitude and max achieved magnitude
- Record:
  - Design method: `[RECORD - pole_placement or dual_lqr]`
  - Desired poles (if pole placement): `[RECORD]`
  - Achieved poles: `[RECORD - eigenvalues of (Ad - L Cd_new)]`
  - Max desired magnitude: `[RECORD]`
  - Max achieved magnitude: `[RECORD]`
- Pass/Fail: `[PASS / FAIL]`

### F.4 Fallback Logic

**Check 4.1: Fallback Path Exists**
- Gate: Demonstrate fallback path can run and yields a stable observer
- Verification: 
  1. Inspect `python/part2/observer_design.py` function `design_observer()` (lines 390-420)
  2. Check if fallback is triggered: Look for RuntimeError handling and fallback_to_lqr=True
  3. If fallback occurs, check console output shows dual LQR method used
- Record:
  - Fallback mechanism: `[RECORD - automatic fallback to dual_lqr when pole placement fails]`
  - Fallback triggered in this run: `[YES / NO / UNKNOWN]`
  - How to trigger fallback: `[RECORD - pole placement must fail first]`
- Pass/Fail: `[PASS / FAIL / UNKNOWN]`

**Note**: The fallback logic is automatic: if pole placement fails with RuntimeError, it automatically calls `design_observer_dual_lqr()` when `fallback_to_lqr=True` (default). To test fallback, one would need to force pole placement to fail, which may not be possible with current system parameters.

### F.5 Metrics and Plots

**Check 5.1: RMS Metrics**
- Gate: RMS metrics include full-window RMS and steady-state RMS (last 20 percent)
- Verification: Check console output or `python/part2/outputs/results.txt`
- Expected: Both "Full window" and "Steady-state window (last 20%)" metrics reported
- Record:
  - Full-window RMS (displacements): `[RECORD]`
  - Steady-state RMS (displacements, last 20%): `[RECORD]`
  - Full-window RMS (all states): `[RECORD]`
  - Steady-state RMS (all states, last 20%): `[RECORD]`
- Pass/Fail: `[PASS / FAIL]`

**Check 5.2: Metric Assumptions**
- Gate: Metrics state assumptions (no-noise and float64 if applicable)
- Verification: Check `results.txt` for assumptions documentation
- Record:
  - Assumptions stated: `[YES / NO]`
  - Assumptions: `[RECORD]`
- Pass/Fail: `[PASS / FAIL]`

**Check 5.3: Plots Exist**
- Gate: Plots exist with exact filenames and paths
- Expected plots:
  - `python/part2/outputs/outputs_comparison.png`
  - `python/part2/outputs/estimation_errors.png`
  - `python/part2/outputs/all_state_errors.png`
- Record:
  - outputs_comparison.png: `[EXISTS / MISSING]`
  - estimation_errors.png: `[EXISTS / MISSING]`
  - all_state_errors.png: `[EXISTS / MISSING]`
- Pass/Fail: `[PASS / FAIL]`

### F.6 Part 2 Initial Conditions

**Check 6.1: x0 Verification**
- Gate: x0 matches final_exam.pdf (otherwise UNVERIFIED)
- Expected: x0 = [0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0] (from code, to be verified against PDF)
- Verification: 
  1. Check `python/part2/run_observer_sim.py` function `get_part2_initial_conditions()` (lines 22-40)
  2. Cross-reference with final_exam.pdf traceability table (Section C.2)
- Record:
  - x0 (code): `[RECORD]`
  - Matches PDF: `[YES / NO / UNVERIFIED]`
- Pass/Fail: `[PASS / UNVERIFIED]`

**Check 6.2: xhat0 Verification**
- Gate: xhat0 matches final_exam.pdf (otherwise UNVERIFIED)
- Expected: xhat0 = [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0] (from code, to be verified against PDF)
- Verification: 
  1. Check `python/part2/run_observer_sim.py` function `get_part2_initial_conditions()` (lines 22-40)
  2. Cross-reference with final_exam.pdf traceability table (Section C.2)
- Record:
  - xhat0 (code): `[RECORD]`
  - Matches PDF: `[YES / NO / UNVERIFIED]`
- Pass/Fail: `[PASS / UNVERIFIED]`

### F.7 Part 2 Artifacts

Record produced artifacts:

- `python/part2/outputs/results.txt`: `[EXISTS / MISSING]`
- `python/part2/outputs/outputs_comparison.png`: `[EXISTS / MISSING]`
- `python/part2/outputs/estimation_errors.png`: `[EXISTS / MISSING]`
- `python/part2/outputs/all_state_errors.png`: `[EXISTS / MISSING]`
- `python/part2/outputs/observer_design_results.txt` (if exists): `[EXISTS / MISSING / N/A]`
- Console output: `[SAVED TO: docs/04_progress_audit/part2_console.txt / NOT SAVED]`

---

## Execution Order

1. Complete Section A (Audit Preflight)
2. Complete Section B (Entrypoint Discovery)
3. Complete Section C (Source Traceability Verification) - **Manual PDF inspection**
4. Run Part 0 and complete Section D (Part 0 Execution Checks)
5. Run Part 1 and complete Section E (Part 1 Execution Checks)
6. Run Part 2 and complete Section F (Part 2 Execution Checks)

---

## Notes for Auditor

- **Never assume correctness**: If a result cannot be verified from repo files or execution, mark as UNKNOWN/UNVERIFIED
- **Save all console output**: Redirect stdout/stderr to files for each part run
- **Record exact commands**: Use the exact commands recorded in Section B.2
- **PDF verification is manual**: Section C requires opening and inspecting the PDF manually - this cannot be automated
- **Pass/Fail gates are strict**: Each gate must clearly pass or fail - no "maybe" results

---

## Completion Checklist

Before considering the audit complete, verify:

- [ ] All preflight information recorded (Section A)
- [ ] All entrypoints discovered and commands recorded (Section B)
- [ ] PDF traceability table completed (Section C)
- [ ] Part 0 executed and all checks completed (Section D)
- [ ] Part 1 executed and all checks completed (Section E)
- [ ] Part 2 executed and all checks completed (Section F)
- [ ] All artifacts documented
- [ ] Closeout document (`closeout.md`) completed with all results

