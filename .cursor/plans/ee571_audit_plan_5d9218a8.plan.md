---
name: EE571 Audit Plan
overview: A verification-first audit plan for Parts 0-7 of the EE571 Final Project, covering gate criteria, cross-part invariant checks, results intake requirements, and Part 7 implementation guidance.
todos:
  - id: verify-parts-0-6
    content: Re-run Parts 0-6 and verify results match system audit report (38 gates)
    status: completed
  - id: implement-part7
    content: "Implement Part 7: run_part7.py with Case 1 (4 sensors) and Case 2 (6 sensors)"
    status: completed
  - id: validate-part7-gates
    content: Validate Part 7 against all P7-G1 through P7-G14 gates
    status: completed
  - id: update-invariants
    content: Update cross_part_invariants.md with Part 7 baseline metrics
    status: completed
  - id: final-audit-report
    content: Produce final audit report covering Parts 0-7 with all gate statuses
    status: completed
---

# EE571 Final Project - Revision and Audit Plan (Parts 0-7)

## Executive Summary

The existing system audit ([`docs/10_system_audit/system_audit_report.md`](docs/10_system_audit/system_audit_report.md)) shows **38 gates all PASS** for Parts 0-6 with commit `29a23a86`. Part 7 is **NOT started** and requires implementation. This plan extends the audit framework through Part 7.

---

## 1. File Inspection Order (Checklist)

### Phase 1: Foundation Verification (Already Complete - Re-verify if Changes Made)

1. [`docs/00_anchor.md`](docs/00_anchor.md) - Master conventions
2. [`docs/sources/final_exam_extract.md`](docs/sources/final_exam_extract.md) - Exam requirements (Part 7 in Section 9)
3. [`docs/07_comprehensive_review_part0_to_part4/cross_part_invariants.md`](docs/07_comprehensive_review_part0_to_part4/cross_part_invariants.md) - Frozen parameters
4. [`python/utils/build_model.py`](python/utils/build_model.py) - Discretization code

### Phase 2: Per-Part Results Inspection

5. `python/part0/baseline_check.py` + plots
6. `python/part1/outputs/observability_results.txt`
7. `python/part2/outputs/results.txt`
8. `python/part3/outputs/results.txt` + `K_matrix.npy`
9. `python/part4/outputs/results.txt`
10. `python/part5/outputs/results.txt` + `Lk_matrix.npy` + `traj.npz`
11. `python/part6/outputs/results.txt` + `traj.npz`

### Phase 3: Part 7 Implementation (NOT STARTED)

12. Create `python/part7/` directory
13. Implement `run_part7.py` for Cases 1 and 2

---

## 2. Explicit Gates Per Part

### Part 0: Baseline Verification

| Gate ID | Criterion | Threshold | Evidence Source |

|---------|-----------|-----------|-----------------|

| P0-G1 | ZOH discretization with Ts=0.01 | Exact | Console output |

| P0-G2 | Ad, Bd, Cd dimensions correct | Ad:(12,12), Bd:(12,3), Cd:(1,12) | Console output |

| P0-G3 | Plots generated | Files exist | `python/part0/outputs/` |

### Part 1: Observability Analysis

| Gate ID | Criterion | Threshold | Evidence Source |

|---------|-----------|-----------|-----------------|

| P1-G1 | Observability rank computed | rank = 6/12 (NOT fully observable) | `observability_results.txt` |

| P1-G2 | Kalman decomposition outputs exist | 6 files present | `python/part1/outputs/` |

| P1-G3 | Eigenvalue consistency | Observable eigenvalues in `eigenvalues_obs.txt` | Output files |

### Part 2: Observer Design

| Gate ID | Criterion | Threshold | Evidence Source |

|---------|-----------|-----------|-----------------|

| P2-G1 | Cmeas = [[1,0..0],[0,0,0,0,0,1,0..0]] (2x12) | Exact | `results.txt` |

| P2-G2 | x0 = [0,0,0,1,1,1,0,0,0,0,0,0]^T | Exact | `results.txt` |

| P2-G3 | xhat0 = [0,0,0,0,0,1,0,0,0,0,0,0]^T | Exact | `results.txt` |

| P2-G4 | L shape = (12, 2) | Exact | `results.txt` |

| P2-G5 | Observer spectral radius | ~0.8, < 1.0 | `results.txt` |

### Part 3: LQR with Observer

| Gate ID | Criterion | Threshold | Evidence Source |

|---------|-----------|-----------|-----------------|

| P3-G1 | PBH stabilizability check | PASS | Console output |

| P3-G2 | Detectability check for DARE | PASS | Console output |

| P3-G3 | K shape = (3, 12) | Exact | `results.txt` |

| P3-G4 | L shape = (12, 2) | Exact | `results.txt` |

| P3-G5 | Closed-loop spectral radius < 1.0 | ~0.9995 | `results.txt` |

| P3-G6 | Controller uses xhat, not x | max\|\|u - (-K@xhat)\|\| = 0 | Diagnostic |

| P3-G7 | Cost J | ~3.915420e+07 | `results.txt` |

| P3-G8 | max\|u\| | ~2.403429e+03 | `results.txt` |

| P3-G9 | Cost sum range k=0..N-1 | Verified | `results.txt` |

### Part 4: Reduced-Input LQR

| Gate ID | Criterion | Threshold | Evidence Source |

|---------|-----------|-----------|-----------------|

| P4-G1 | Bd_red shape = (12, 2) | u3 removed | `results.txt` |

| P4-G2 | K_red shape = (2, 12) | Exact | `results.txt` |

| P4-G3 | J_red > J_part3 | 5.838e+07 > 3.915e+07 | `results.txt` |

| P4-G4 | max\|u_red\| > max\|u_part3\| | 3.311e+03 > 2.403e+03 | `results.txt` |

| P4-G5 | Same Cmeas, x0, xhat0 as Part 3 | Exact | `results.txt` |

### Part 5: Kalman Filter (LQE)

| Gate ID | Criterion | Threshold | Evidence Source |

|---------|-----------|-----------|-----------------|

| P5-G1 | Qw = 0.05 * I_3 | Exact | `results.txt` |

| P5-G2 | Rv = 0.1 * I_2 | Exact | `results.txt` |

| P5-G3 | seed = 42 | Exact | `results.txt` |

| P5-G4 | Lk shape = (12, 2) | Exact | `results.txt` |

| P5-G5 | Estimator spectral radius < 1.0 | ~0.9995 | `results.txt` |

| P5-G6 | Noise enters via Bd | x[k+1]=Ad@x+Bd@u+Bd@w | Dynamics residual check |

| P5-G7 | traj.npz keys: x, xhat, y_true, y_meas, w, v | All present | File inspection |

| P5-G8 | First 3 noise samples reproducible | Match expected values | `results.txt` |

### Part 6: LQG Controller

| Gate ID | Criterion | Threshold | Evidence Source |

|---------|-----------|-----------|-----------------|

| P6-G1 | K loaded from Part 3 | max_abs_diff = 0 | `results.txt` |

| P6-G2 | Lk loaded from Part 5 | max_abs_diff = 0 | `results.txt` |

| P6-G3 | Controller uses xhat | max\|\|u-(-K@xhat)\|\| = 0 | Diagnostic |

| P6-G4 | Noise settings frozen | Qw, Rv, seed same as Part 5 | `results.txt` |

| P6-G5 | J_true reported (official metric) | ~4.26e+02 | `results.txt` |

| P6-G6 | J_meas reported (comparison) | ~6.34e+02 | `results.txt` |

| P6-G7 | No-noise sanity check | Mismatch detected and explained | `results.txt` |

| P6-G8 | Dynamics residual check | max_norm ~ 0 | Verification |

| P6-G9 | Measurement construction check | y_meas = Cmeas@x + v | Verification |

### Part 7: Sensor Augmentation (TO BE IMPLEMENTED)

| Gate ID | Criterion | Threshold | Evidence Source |

|---------|-----------|-----------|-----------------|

| P7-G1 | Case 1: C_case1 shape = (4, 12) | Measures x1, x2, x5, x6 | `results.txt` |

| P7-G2 | Case 2: C_case2 shape = (6, 12) | Measures x1..x6 | `results.txt` |

| P7-G3 | K reused from Part 3 (unchanged) | max_abs_diff = 0 | Verification |

| P7-G4 | Lk_case1 redesigned for C_case1 | Shape = (12, 4) | `results.txt` |

| P7-G5 | Lk_case2 redesigned for C_case2 | Shape = (12, 6) | `results.txt` |

| P7-G6 | Rv_case1 = 0.1 * I_4 | 4x4 covariance | `results.txt` |

| P7-G7 | Rv_case2 = 0.1 * I_6 | 6x6 covariance | `results.txt` |

| P7-G8 | Qw unchanged (0.05 * I_3) | Process noise unchanged | `results.txt` |

| P7-G9 | seed = 42 (reproducibility) | Same seed | `results.txt` |

| P7-G10 | Estimator spectral radius < 1 (both cases) | Stable | `results.txt` |

| P7-G11 | J_true reported for both cases | Numeric value | `results.txt` |

| P7-G12 | RMS estimation error for both cases | Should decrease vs Part 6 | Comparison |

| P7-G13 | Comparison table: Part 6 vs Part 7 Cases 1 and 2 | Quantitative | `results.txt` |

| P7-G14 | Answer: "Do more sensors help?" | Justified conclusion | Analysis |

---

## 3. Cross-Part Consistency Checks

| Invariant | Parts Affected | Check Method |

|-----------|----------------|--------------|

| Ts = 0.01 | All | Grep for `Ts` in each run script |

| N = 1000 | All | Grep for `N =` in each run script |

| x: (12, N+1), u: (m, N) | 2-7 | Shape logs in results.txt |

| Cmeas (2x12) measures x1, x6 | 2-6 | Matrix printed in results.txt |

| x0, xhat0 values | 2-7 | Printed in results.txt |

| Cost sum k=0..N-1 | 3, 4, 6, 7 | Cost implementation comments |

| seed = 42 | 5, 6, 7 | Reproducibility section |

| Qw = 0.05*I_3 | 5, 6, 7 | Noise covariance section |

| Rv = 0.1*I_p | 5, 6, 7 | Noise covariance section (p varies for Part 7) |

| K from Part 3 reused | 6, 7 | Matrix equality check |

| Process noise via Bd | 5, 6, 7 | Dynamics equation in code |

---

## 4. Results Intake Structure

### What I Need From You:

**A. Directory Listings (run these commands):**

```bash
ls -la python/part*/outputs/
```

**B. Per-Part results.txt Files:**

- `python/part2/outputs/results.txt` (already provided in my inspection)
- `python/part3/outputs/results.txt` (already provided)
- `python/part4/outputs/results.txt` (if different from cross_part_invariants.md)
- `python/part5/outputs/results.txt` (already provided)
- `python/part6/outputs/results.txt` (already provided)

**C. For Part 7 (when implemented):**

- `python/part7/outputs/results.txt` for both Case 1 and Case 2
- Key comparison metrics vs Part 6

**D. Key Verification Snippets:**

- Any console output showing gate-specific diagnostics
- Git commit hash for the run: `git rev-parse HEAD`

**E. Part 7 Task Statement:**

- Confirm the exam requirement excerpt matches Section 9 of `final_exam_extract.md`
- Any additional constraints not in the current extract

---

## 5. Top Likely Failure Modes and Detection Evidence

| Failure Mode | Detection Evidence | Severity |

|--------------|-------------------|----------|

| **Rv dimension mismatch (Part 7)** | Error during Kalman filter design; Rv shape != (p, p) | HIGH |

| **K matrix modified (Part 7)** | max_abs_diff(K_part7, K_part3) > 0 | HIGH |

| **Noise seed inconsistency** | First 3 noise samples mismatch vs Part 5/6 | MEDIUM |

| **Cost uses y_meas instead of y_true** | J_true != J_meas reported as same value | MEDIUM |

| **Process noise not via Bd** | Dynamics residual check fails | HIGH |

| **Spectral radius >= 1.0** | Unstable simulation, diverging trajectories | HIGH |

| **Cost sum range wrong** | J too small (missed samples) or too large (double-counted) | MEDIUM |

| **xhat used for plant dynamics** | Plant trajectory differs from expected | HIGH |

| **Cost definition wrong for Part 7** | Should still use y1^2 + y6^2, not all measured outputs | MEDIUM |

---

## 6. Part 7 Implementation Requirements

### Critical Design Decisions:

1. **Controller K remains unchanged** - The LQR design depends on plant dynamics and cost, not on sensors. Part 7 uses the same K from Part 3.

2. **Kalman gain Lk must be redesigned** for each sensor configuration:

   - Case 1: Solve DARE with C_case1 (4x12), Rv_case1 (4x4)
   - Case 2: Solve DARE with C_case2 (6x12), Rv_case2 (6x6)

3. **Rv dimension changes** (critical):

   - Part 6: Rv = 0.1 * I_2 (2 sensors)
   - Part 7 Case 1: Rv = 0.1 * I_4 (4 sensors)
   - Part 7 Case 2: Rv = 0.1 * I_6 (6 sensors)

4. **Cost function stays the same** - Cost uses y1^2 + y6^2 regardless of sensor count. The additional sensors improve estimation but don't change the regulation objective.

5. **Process noise unchanged** - Qw = 0.05 * I_3 (3 inputs, not affected by sensor count)

### Expected Outcomes:

- More sensors should **improve estimation** (lower RMS estimation error)
- Regulation performance should **improve or stay similar** (lower or equal J_true)
- Estimator spectral radius should **decrease** (faster convergence) or stay similar

---

## 7. Verification Commands (Cursor Actionable)

```bash
# Check all Ts values
grep -n "Ts\s*=" python/part*/run_*.py

# Check all N values
grep -n "N\s*=" python/part*/run_*.py

# Check noise settings
grep -n "seed\s*=" python/part*/run_*.py
grep -n "0.05" python/part*/run_*.py
grep -n "0.1" python/part*/run_*.py

# Verify traj.npz contents
python -c "import numpy as np; d=np.load('python/part6/outputs/traj.npz'); print(list(d.keys()))"

# Check K matrix equality
python -c "import numpy as np; K3=np.load('python/part3/outputs/K_matrix.npy'); print(f'K shape: {K3.shape}, max|K|: {np.max(np.abs(K3)):.6e}')"
```

---

## Next Steps

1. **Confirm Part 7 exam requirements** - Verify Section 9 of `final_exam_extract.md` is complete
2. **Run Parts 0-6** to confirm results match the system audit report
3. **Implement Part 7** with the design decisions above
4. **Validate Part 7** against the P7-G1 through P7-G14 gates
5. **Update cross_part_invariants.md** with Part 7 baselines