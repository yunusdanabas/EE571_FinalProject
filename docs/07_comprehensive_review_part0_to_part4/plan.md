# Comprehensive Review Plan: Parts 0-4

## Purpose

This document provides a strict step-by-step procedure for conducting a comprehensive, verification-first review of EE571 Final Project progress from Part 0 through Part 4. This is an audit and evaluation workflow. **Do not implement new features beyond minimal instrumentation needed to reproduce and record evidence. Do not start Part 5 or later.**

## Section 1: Preconditions and Environment Freeze

### 1.1 Environment Information Capture

Before running any parts, record the following environment information:

**Commands to run:**
```bash
# OS information
uname -a

# Python version
python --version

# NumPy version
python -c "import numpy; print(f'NumPy {numpy.__version__}')"

# SciPy version
python -c "import scipy; print(f'SciPy {scipy.__version__}')"

# Git commit hash
git rev-parse HEAD
```

**Record in evidence:**
- OS: [output of uname -a]
- Python version: [output of python --version]
- NumPy version: [output from numpy check]
- SciPy version: [output from scipy check]
- Git commit hash: [output of git rev-parse HEAD]

### 1.2 Clean Run Preparation

To ensure reproducible results, delete output folders for parts being run:

```bash
# Navigate to project root
cd /home/yunusdanabas/EE571_FinalProject

# Remove output directories (if they exist)
rm -rf python/part0/*.png
rm -rf python/part1/outputs/*
rm -rf python/part2/outputs/*
rm -rf python/part3/outputs/*
rm -rf python/part4/outputs/*
```

**Note:** Only delete output files, not source code or utility modules.

### 1.3 Verification Checklist

- [ ] Environment information captured
- [ ] Git commit hash recorded
- [ ] Output directories cleaned (if performing fresh runs)
- [ ] Python environment activated (if using conda/venv)

---

## Section 2: Source Traceability and Exam Mapping (Manual)

### 2.1 Verify Source Documents Exist

**Check:**
- [ ] `docs/sources/final_exam_extract.md` exists
- [ ] `docs/00_anchor.md` exists
- [ ] `docs/sources/final_exam.pdf` exists (optional, for page number verification)

### 2.2 Verify Citations in `docs/00_anchor.md`

For each item below, check if `docs/00_anchor.md` cites `docs/sources/final_exam_extract.md` or `docs/sources/final_exam.pdf`. If citation is missing, mark as **UNKNOWN** (do not infer).

**Items to verify:**

1. **Part 0 baseline C (x1-only)**
   - Expected: Citation to exam statement about baseline measurement
   - Status: [ ] VERIFIED / [ ] UNKNOWN
   - Evidence line: [quote from anchor.md or mark UNKNOWN]

2. **Part 2 C_part2 (x1 and x6)**
   - Expected: Citation to exam Question 2 with measurement matrix
   - Status: [ ] VERIFIED / [ ] UNKNOWN
   - Evidence line: [quote from anchor.md or mark UNKNOWN]

3. **Part 2 x0 and xhat0**
   - Expected: Citation to exam Question 2 with initial conditions
   - Status: [ ] VERIFIED / [ ] UNKNOWN
   - Evidence line: [quote from anchor.md or mark UNKNOWN]

4. **Part 3 cost definition J = Σ (u^T u + y1^2 + y6^2)**
   - Expected: Citation to exam Question 3 with cost function
   - Status: [ ] VERIFIED / [ ] UNKNOWN
   - Evidence line: [quote from anchor.md or mark UNKNOWN]

5. **Part 4 instruction to remove u3**
   - Expected: Citation to exam Question 4 with reduced input requirement
   - Status: [ ] VERIFIED / [ ] UNKNOWN
   - Evidence line: [quote from anchor.md or mark UNKNOWN]

**Record findings in closeout.md traceability table.**

---

## Section 3: Part-by-Part Execution and Gates

### Part 0: Baseline Verification

**Command to run:**
```bash
python python/part0/baseline_check.py
```

**Required output files:**
- `python/part0/output_plot.png`
- `python/part0/displacements_plot.png`

**Required log lines to capture:**
- Matrix shapes (A, B, C, Ad, Bd, Cd)
- Discretization method confirmation
- Simulation completion message
- Any error messages

**Explicit Pass/Fail Gates:**

| Gate | Pass Criteria | Evidence Location |
|------|---------------|-------------------|
| ZOH discretization | Uses `Ts = 0.01` and `scipy.signal.cont2discrete(method="zoh")` | Console log or code inspection |
| Matrix shapes | A (12×12), B (12×3), C (1×12) | Console log: "A shape: (12, 12)" etc. |
| Simulation runs | No errors, completes successfully | Console log: "Baseline verification complete!" |
| Output plot | Contains exactly 1 trace (matches C dimension) | Visual inspection of `output_plot.png` |
| Displacements plot | Contains exactly 6 traces (x1..x6) | Visual inspection of `displacements_plot.png` |

**Artifacts checklist:**
- [ ] `python/part0/output_plot.png` exists
- [ ] `python/part0/displacements_plot.png` exists
- [ ] Console log captured (if generated)

---

### Part 1: Observability Analysis

**Command to run:**
```bash
python python/part1/run_observability.py
```

**Required output files:**
- `python/part1/outputs/observability_results.txt`
- `python/part1/outputs/O_matrix.txt`
- `python/part1/outputs/O_matrix_summary.txt`
- `python/part1/outputs/Abar_matrix.txt`
- `python/part1/outputs/eigenvalues_obs.txt`
- `python/part1/outputs/eigenvalues_unobs.txt`

**Required log lines to capture:**
- Observability rank: "Rank of observability matrix: X"
- SVD tolerance used
- Kalman decomposition reconstruction errors
- Eigenvalue consistency check results

**Explicit Pass/Fail Gates:**

| Gate | Pass Criteria | Evidence Location |
|------|---------------|-------------------|
| Observability rank | Rank = 6/12 for (Ad, Cd_baseline) | Console: "Rank of observability matrix: 6" |
| Tolerance policy | SVD tolerance logged | Console: "SVD tolerance used: X.XXe-XX" |
| Kalman reconstruction | Ad and Cd reconstruction errors < 1e-10 | Console: "Ad reconstruction: ✓ PASS" |
| Eigenvalue consistency | Set-matched eigenvalues (not order-matched) | Console: "Eigenvalue consistency: ✓ PASS" |

**Artifacts checklist:**
- [ ] `python/part1/outputs/observability_results.txt` exists
- [ ] `python/part1/outputs/O_matrix.txt` exists
- [ ] `python/part1/outputs/O_matrix_summary.txt` exists
- [ ] `python/part1/outputs/Abar_matrix.txt` exists
- [ ] `python/part1/outputs/eigenvalues_obs.txt` exists
- [ ] `python/part1/outputs/eigenvalues_unobs.txt` exists
- [ ] Console log captured

---

### Part 2: Observer Design and Simulation

**Command to run:**
```bash
python python/part2/run_observer_sim.py
```

**Required output files:**
- `python/part2/outputs/results.txt`
- `python/part2/outputs/outputs_comparison.png`
- `python/part2/outputs/estimation_errors.png`
- `python/part2/outputs/all_state_errors.png`

**Required log lines to capture:**
- Observability rank for (Ad, C_part2): "Rank of observability matrix: 12"
- Observer design method used
- Spectral radius: "Spectral radius: X.XXXXXX"
- Observer stability confirmation
- RMS errors (full-window and steady-state)

**Explicit Pass/Fail Gates:**

| Gate | Pass Criteria | Evidence Location |
|------|---------------|-------------------|
| Observability rank | Rank = 12/12 for (Ad, C_part2) | Console or results.txt: "Rank of observability matrix: 12" |
| Observer stability | spectral_radius(Ad - L C_part2) < 1.0 | Console: "Spectral radius: X.XXXXXX" (must be < 1.0) |
| Pole matching | Desired vs achieved poles logged (max magnitude match) | Console or results.txt: "Requested max magnitude: X.XX" and "Achieved max magnitude: X.XX" |
| Fallback logic | Fallback logic present and documented | Code inspection or results.txt: "Method: dual_lqr" if fallback used, or "not demonstrated" if not |
| RMS metrics | Full-window and last-20% steady-state metrics present, labeled "no-noise" | results.txt: "RMS errors for displacements (x1..x6) - Full window:" and "Steady-state window" |

**Artifacts checklist:**
- [ ] `python/part2/outputs/results.txt` exists
- [ ] `python/part2/outputs/outputs_comparison.png` exists
- [ ] `python/part2/outputs/estimation_errors.png` exists
- [ ] `python/part2/outputs/all_state_errors.png` exists
- [ ] Console log captured

---

### Part 3: LQR Controller Design with Observer

**Command to run:**
```bash
python python/part3/run_lqr_with_observer.py
```

**Required output files:**
- `python/part3/outputs/results.txt`
- `python/part3/outputs/outputs_y1_y6.png`
- `python/part3/outputs/inputs_u1_u2_u3.png`
- `python/part3/outputs/estimation_error_norm.png`
- `python/part3/outputs/K_matrix.npy` (optional)
- `python/part3/outputs/L_matrix.npy` (optional)

**Required log lines to capture:**
- Indexing convention confirmation (x length N+1, u length N, y length N+1)
- PBH stabilizability check for (Ad, Bd): "is_stabilizable: True"
- Detectability check for (Ad, Cy): "is_detectable: True"
- DARE solver success: "DARE solved successfully"
- K matrix shape: "K shape: (3, 12)"
- Closed-loop spectral radius: "Spectral radius (Acl): X.XXXXXX" (must be < 1.0)
- Controller uses xhat diagnostic: "Controller uses: xhat" or similar
- Cost indexing convention: "Cost computed over k=0 to N-1" or similar

**Explicit Pass/Fail Gates:**

| Gate | Pass Criteria | Evidence Location |
|------|---------------|-------------------|
| Indexing convention | x length N+1, u length N, y length N+1 | Console or results.txt: explicit confirmation |
| PBH stabilizability | (Ad, Bd) is stabilizable, logged | Console or results.txt: "is_stabilizable: True" |
| Detectability | (Ad, Cy) is detectable, logged | Console or results.txt: "is_detectable: True" |
| DARE solver | DARE solved successfully | Console: "DARE solved successfully" |
| K shape | K matrix shape is (3, 12) | Console or results.txt: "K shape: (3, 12)" |
| Closed-loop stability | spectral_radius(Acl) < 1.0 | Console or results.txt: "Spectral radius (Acl): X.XXXXXX" (must be < 1.0) |
| Dominant eigenvalue | Dominant eigenvalue magnitude/angle logged | Console or results.txt: eigenvalue analysis |
| Controller uses xhat | Diagnostic confirms controller uses xhat, not x | Console or results.txt: explicit confirmation |
| Cost convention | Cost uses plant output y = Cy x, indexing logged | Console or results.txt: "Cost: sum(u^T u + y1^2 + y6^2)" |

**Artifacts checklist:**
- [ ] `python/part3/outputs/results.txt` exists
- [ ] `python/part3/outputs/outputs_y1_y6.png` exists
- [ ] `python/part3/outputs/inputs_u1_u2_u3.png` exists
- [ ] `python/part3/outputs/estimation_error_norm.png` exists
- [ ] Optional: `python/part3/outputs/K_matrix.npy` exists
- [ ] Optional: `python/part3/outputs/L_matrix.npy` exists
- [ ] Console log captured

---

### Part 4: Reduced Input LQR Controller Design

**Command to run:**
```bash
python python/part4/run_lqr_reduced_input.py
```

**Required output files:**
- `python/part4/outputs/results.txt`
- `python/part4/outputs/outputs_y1_y6.png`
- `python/part4/outputs/inputs_u1_u2.png`
- `python/part4/outputs/estimation_error_norm.png`

**Required log lines to capture:**
- Input reduction mapping: "Bd_red = Bd[:, [0,1]]" or "u_red = [u1, u2], removed u3"
- PBH stabilizability for (Ad, Bd_red): "is_stabilizable: True"
- Detectability for (Ad, Cy): "is_detectable: True"
- DARE solver success: "DARE solved successfully"
- K_red shape: "K_red shape: (2, 12)"
- Closed-loop spectral radius: "Spectral radius (Acl_red): X.XXXXXX" (must be < 1.0)
- Cost convention matches Part 3: explicit confirmation
- max_abs_u_overall: "max_abs_u_overall: X.XXXXXX"
- max_u_inf: "max_u_inf: X.XXXXXX"

**Explicit Pass/Fail Gates:**

| Gate | Pass Criteria | Evidence Location |
|------|---------------|-------------------|
| Input reduction | Bd_red = Bd[:, [0,1]], u_red = [u1, u2], u3 removed, mapping logged | Console or results.txt: explicit mapping |
| PBH stabilizability | (Ad, Bd_red) is stabilizable, PASS logged | Console or results.txt: "is_stabilizable: True" |
| Detectability | (Ad, Cy) is detectable, PASS logged | Console or results.txt: "is_detectable: True" |
| DARE solver | DARE solved successfully | Console: "DARE solved successfully" |
| K_red shape | K_red matrix shape is (2, 12) | Console or results.txt: "K_red shape: (2, 12)" |
| Closed-loop stability | spectral_radius(Acl_red) < 1.0, margin logged | Console or results.txt: "Spectral radius (Acl_red): X.XXXXXX" (must be < 1.0) |
| Cost convention | Matches Part 3 (indexing, N, Ts, plant output y = Cy x) | Console or results.txt: explicit confirmation |
| max_abs_u_overall | max(abs(u_red)) logged | Console or results.txt: "max_abs_u_overall: X.XXXXXX" |
| max_u_inf | max_u_inf logged | Console or results.txt: "max_u_inf: X.XXXXXX" |

**Artifacts checklist:**
- [ ] `python/part4/outputs/results.txt` exists
- [ ] `python/part4/outputs/outputs_y1_y6.png` exists
- [ ] `python/part4/outputs/inputs_u1_u2.png` exists
- [ ] `python/part4/outputs/estimation_error_norm.png` exists
- [ ] Console log captured

---

## Section 4: Cross-Part Consistency Checks

### 4.1 Sampling Time Consistency

**Check:** All parts use the same sampling time `Ts = 0.01` seconds.

**Evidence locations:**
- Part 0: Console log or code: `Ts = 0.01`
- Part 1: Console log or code: `Ts = 0.01`
- Part 2: Console log or results.txt: `Ts = 0.01`
- Part 3: Console log or results.txt: `Ts = 0.01`
- Part 4: Console log or results.txt: `Ts = 0.01`

**Status:** [ ] PASS / [ ] FAIL / [ ] UNKNOWN

**Notes:** [Record any discrepancies]

---

### 4.2 Part 2 C Matrix and Initial Conditions Consistency

**Check:** Parts 2, 3, and 4 all use the same C matrix (measuring x1 and x6) and the same initial conditions (x0 and xhat0).

**Evidence locations:**
- Part 2: results.txt: "Cd_new shape: (2, 12)" and initial conditions
- Part 3: results.txt: C matrix and initial conditions (should match Part 2)
- Part 4: results.txt: C matrix and initial conditions (should match Part 2)

**Status:** [ ] PASS / [ ] FAIL / [ ] UNKNOWN

**Notes:** [Record any discrepancies]

---

### 4.3 Cost Convention Consistency (Parts 3 and 4)

**Check:** Parts 3 and 4 use the same cost convention:
- Same indexing (k=0 to N-1)
- Same N (simulation horizon)
- Same Ts
- Same plant output definition (y = Cy x)

**Evidence locations:**
- Part 3: results.txt: cost definition and indexing
- Part 4: results.txt: cost definition and indexing (should match Part 3)

**Status:** [ ] PASS / [ ] FAIL / [ ] UNKNOWN

**Notes:** [Record any discrepancies]

---

### 4.4 Part 3 vs Part 4 Comparison Table

**Check:** Comparison table includes J (total cost) and max_abs_u_overall under aligned definition.

**Required comparison metrics:**
- Total cost J (Part 3 vs Part 4)
- max_abs_u_overall (Part 3 vs Part 4)
- If Part 3 baseline fields missing, mark as UNKNOWN

**Evidence locations:**
- Part 3: results.txt: "Total cost J: X.XXXXXX" and "max_abs_u_overall: X.XXXXXX"
- Part 4: results.txt: "Total cost J: X.XXXXXX" and "max_abs_u_overall: X.XXXXXX"

**Comparison table template:**

| Metric | Part 3 | Part 4 | Difference | Notes |
|--------|--------|--------|------------|-------|
| Total cost J | [value] | [value] | [difference] | [notes] |
| max_abs_u_overall | [value] | [value] | [difference] | [notes] |

**Status:** [ ] PASS / [ ] FAIL / [ ] UNKNOWN

**Notes:** [Record findings]

---

## Section 5: Packaging Evidence for ChatGPT Evaluation

### 5.1 Required Evidence Package

Create a package containing the following items for evaluation:

**1. Environment Information**
- OS, Python version, NumPy version, SciPy version
- Git commit hash

**2. Console Outputs**
- Part 0: Key console output (or summary if available)
- Part 1: Key console output (or summary if available)

**3. Full Results Files**
- Part 2: Complete `python/part2/outputs/results.txt`
- Part 3: Complete `python/part3/outputs/results.txt`
- Part 4: Complete `python/part4/outputs/results.txt`

**4. Directory Listings**
For each part's outputs folder:
```bash
ls -la python/part0/*.png
ls -la python/part1/outputs/
ls -la python/part2/outputs/
ls -la python/part3/outputs/
ls -la python/part4/outputs/
```

**5. Artifact Verification**
- Confirm all required plot files exist
- Confirm all required text output files exist

### 5.2 Evidence Package Checklist

- [ ] Environment information captured
- [ ] Part 0 console output captured
- [ ] Part 1 console output captured
- [ ] Part 2 results.txt captured (full file)
- [ ] Part 3 results.txt captured (full file)
- [ ] Part 4 results.txt captured (full file)
- [ ] Directory listings captured for all parts
- [ ] All required artifacts verified to exist

### 5.3 Submission Format

Use `results_intake_template.md` to format evidence for submission. Fill in all sections and paste the complete template into ChatGPT for evaluation.

---

## Execution Order

1. Complete Section 1 (Preconditions and Environment Freeze)
2. Complete Section 2 (Source Traceability) - manual verification
3. Run Part 0, verify gates, capture evidence
4. Run Part 1, verify gates, capture evidence
5. Run Part 2, verify gates, capture evidence
6. Run Part 3, verify gates, capture evidence
7. Run Part 4, verify gates, capture evidence
8. Complete Section 4 (Cross-Part Consistency Checks)
9. Complete Section 5 (Package Evidence)
10. Fill out `closeout.md` with all findings
11. Fill out `results_intake_template.md` with evidence
12. Submit `results_intake_template.md` for evaluation

---

## Notes

- This is an audit workflow. Do not modify existing code unless absolutely necessary for evidence capture.
- If a gate fails, document the failure clearly in closeout.md.
- Mark any missing information as UNKNOWN rather than inferring values.
- All evidence should be reproducible from the recorded git commit hash.
