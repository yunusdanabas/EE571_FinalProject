# System Audit Report - Parts 0-6

**Audit Date:** 2025-01-07 (verified correct)  
**Git Commit Hash:** 29a23a865bd33aef1eff8e5920b3df85e2ea5b0d  
**Auditor:** System Verification Audit

## Environment Information

- **Python Version:** 3.12.11
- **NumPy Version:** 2.3.3
- **SciPy Version:** 1.16.2
- **Operating System:** Linux-6.8.0-90-generic-x86_64-with-glibc2.39
- **Platform:** Linux

## Scope

This audit verifies Parts 0-6 of the EE571 Final Project with a verification-first approach. Every claimed property is evidenced by code inspection plus logged outputs from clean runs.

## How to Reproduce

All parts were run from the repository root with the following commands:

```bash
# Part 0
python python/part0/baseline_check.py

# Part 1
python python/part1/run_observability.py

# Part 2
python python/part2/run_observer_sim.py

# Part 3
python python/part3/run_lqr_with_observer.py

# Part 4
python python/part4/run_lqr_reduced_input.py

# Part 5
python python/part5/run_kalman_filter.py

# Part 6
python python/part6/run_lqg.py
```

Full console outputs are captured in `docs/10_system_audit/system_audit_evidence.md`.

## Gate Summary Table

**Total Gates:** 38 (29 original + 9 additional verification gates)

| Part | Gate ID | Description | Status | Evidence |
|------|---------|-------------|--------|----------|
| P0 | GATE-P0-1 | Discretization uses ZOH with Ts=0.01 | PASS | Part 0 console output |
| P0 | GATE-P0-2 | Simulation runs and produces plots | PASS | output_plot.png, displacements_plot.png exist |
| P1 | GATE-P1-1 | Observability rank computed and logged | PASS | observability_results.txt |
| P1 | GATE-P1-2 | Kalman decomposition outputs exist | PASS | All 6 required files present |
| P2 | GATE-P2-1 | Uses Cmeas measuring x1 and x6 exactly | PASS | results.txt lines 4-8 |
| P2 | GATE-P2-2 | x0 and xhat0 match invariants doc | PASS | results.txt lines 11-12 |
| P2 | GATE-P2-3 | L has shape (12,2) | PASS | results.txt line 26 |
| P2 | GATE-P2-4 | Observer Aobs stable (spectral radius < 1) | PASS | results.txt line 27: 0.800000 |
| P2 | GATE-P2-5 | Spectral radius approximately 0.8 | PASS | results.txt line 27: 0.800000 |
| P2 | GATE-P2-6 | results.txt prints desired and achieved poles | PASS | results.txt lines 20-27 |
| P3 | GATE-P3-1 | PBH stabilizability checks pass | PASS | Part 3 console output |
| P3 | GATE-P3-2 | Detectability check for DARE passes | PASS | Part 3 console output |
| P3 | GATE-P3-3 | K shape (3,12), L shape (12,2) | PASS | results.txt |
| P3 | GATE-P3-4 | Closed-loop Acl stable | PASS | Spectral radius: 0.999463 |
| P3 | GATE-P3-5 | Controller uses xhat not x | PASS | Diagnostic in console output |
| P3 | GATE-P3-6 | Cost definition correct | PASS | results.txt: J = sum_{k=0..N-1} |
| P3 | GATE-P3-7 | J and max\|u\| match baselines | PASS | J=3.915420e+07, max\|u\|=2.403429e+03 |
| P3 | GATE-P3-8 | J recomputed from traj.npz matches logged value | PASS | abs(diff)=1.50e-03, rel=3.83e-11 |
| P3 | GATE-P3-9 | Observer update residual: xhat[k+1] = Ad@xhat[k] + Bd@u[k] + L@(y_true[k] - Cmeas@xhat[k]) | PASS | max_norm=8.88e-16, max_abs=8.88e-16, dtype=float64 |
| P4 | GATE-P4-1 | Removes u3, uses only 2 inputs | PASS | Part 4 console output |
| P4 | GATE-P4-2 | Same cost indexing as Part 3 | PASS | results.txt |
| P4 | GATE-P4-5 | J recomputed from traj.npz matches logged value | PASS | abs(diff)=2.94e+00, rel=5.04e-08 |
| P4 | GATE-P4-3 | Observer and initial conditions consistent | PASS | results.txt |
| P4 | GATE-P4-4 | Cost higher than Part 3 | PASS | J_red=5.838118e+07 > J=3.915420e+07 |
| P5 | GATE-P5-1 | Noise model correct | PASS | Qw=0.05*I3, Rv=0.1*I2, seed=42 |
| P5 | GATE-P5-2 | Lk computed and saved, shape (12,2) | PASS | Lk_matrix.npy exists |
| P5 | GATE-P5-3 | Estimator spectral radius < 1 | PASS | 0.999547 < 1 |
| P5 | GATE-P5-4 | Innovation covariance S invertible | PASS | cond(S)=1.00 |
| P5 | GATE-P5-5 | traj.npz saved with required keys | PASS | All keys present, shapes correct |
| P6 | GATE-P6-1 | K loaded from Part 3, matches exactly | PASS | max_abs_diff=0.0 |
| P6 | GATE-P6-2 | Lk loaded from Part 5, matches exactly | PASS | max_abs_diff=0.0 |
| P6 | GATE-P6-3 | Controller uses xhat not x | PASS | Diagnostic: ||u - (-K xhat)||=0 |
| P6 | GATE-P6-4 | Noise settings frozen | PASS | seed=42, Qw=0.05*I3, Rv=0.1*I2 |
| P6 | GATE-P6-5 | Cost reporting includes J_true | PASS | J_true=4.260967e+02 (official) |
| P6 | GATE-P6-6 | No-noise sanity check exists and correctly interpreted | PASS | Mismatch detected and explained (Lk≠L) |
| P6 | GATE-P6-7 | J_true recomputed from traj.npz matches logged value | PASS | abs(diff)=4.18e-05, rel=9.82e-08 |
| P5 | GATE-P5-6 | Dynamics residual: x[k+1] = Ad x[k] + Bd u[k] + Bd w[k] | PASS | max_norm=0.0, max_abs=0.0, dtype=float64 |
| P6 | GATE-P6-8 | Dynamics residual: x[k+1] = Ad x[k] + Bd u[k] + Bd w[k] | PASS | max_norm=0.0, max_abs=0.0, dtype=float64 |
| P6 | GATE-P6-9 | Measurement: y_true = Cmeas@x, y_meas = y_true+v | PASS | max_diff=0.0 (k=0..N-1, y_true[N] not checked) |
| P6 | GATE-P6-10 | Kalman update residual: xhat[k+1] = Ad@xhat[k] + Bd@u[k] + Lk@(y_meas[k] - Cmeas@xhat[k]) | PASS | max_norm=0.0, max_abs=0.0, dtype=float64 |
| REP | GATE-REP-1 | Part 5 results.txt identical between runs | PASS | Noise samples match |
| REP | GATE-REP-2 | Part 5 traj.npz noise samples match | PASS | First 3 samples identical |
| REP | GATE-REP-3 | Part 6 results.txt identical between runs | PASS | Noise samples match |
| REP | GATE-REP-4 | Part 6 traj.npz noise samples match | PASS | First 3 samples identical |

## Part-by-Part Results

### Part 0: Baseline Verification

**What was checked:**
- Discretization method (ZOH with Ts=0.01)
- Simulation execution and plot generation
- Matrix dimensions and validation
- MATLAB validation (optional, requires exported .mat file)

**Results:**
- All gates PASS
- Plots generated: output_plot.png, displacements_plot.png
- Discretization confirmed: ZOH method, Ts=0.01
- MATLAB validation: Script exists but requires matlab/export_matrices.m to be run first (optional check)

**Evidence:** See `docs/10_system_audit/system_audit_evidence.md` Part 0 section.

### Part 1: Observability Analysis

**What was checked:**
- Observability rank computation
- Kalman decomposition outputs
- Eigenvalue consistency

**Results:**
- All gates PASS
- Observability rank: 6/12 (system not fully observable)
- All 6 required output files generated

**Evidence:** See `docs/10_system_audit/system_audit_evidence.md` Part 1 section.

### Part 2: Observer Design

**What was checked:**
- Cmeas matrix (measures x1 and x6)
- Initial conditions (x0, xhat0)
- Observer gain L shape and stability
- Spectral radius consistency

**Results:**
- All gates PASS
- Cmeas shape: (2, 12), matches invariants
- x0 and xhat0 match invariants doc exactly
- L shape: (12, 2)
- Spectral radius: 0.800000 (stable, matches expectation)

**Evidence:** See `docs/10_system_audit/system_audit_evidence.md` Part 2 section.

### Part 3: LQR with Observer

**What was checked:**
- PBH stabilizability and detectability
- K and L matrix shapes
- Closed-loop stability
- Controller uses xhat (not x)
- Cost definition and computation
- Baseline metrics consistency
- Cost recomputation from traj.npz (independent verification)
- Observer update residual check (xhat[k+1] = Ad@xhat[k] + Bd@u[k] + L@(y_true[k] - Cmeas@xhat[k]))

**Results:**
- All gates PASS
- K shape: (3, 12), L shape: (12, 2)
- Closed-loop spectral radius: 0.999463 (stable)
- Controller diagnostic confirms u uses xhat
- Cost J: 3.915420e+07 (matches cross_part_invariants.md)
- Max |u|: 2.403429e+03 (matches cross_part_invariants.md)
- J recomputation: abs(diff)=1.50e-03, rel=3.83e-11 (matches logged value)
- Observer update residual: max_norm=8.88e-16, max_abs=8.88e-16, dtype=float64 (perfect match, confirms Bd@u[k] term included)

**Evidence:** See `docs/10_system_audit/system_audit_evidence.md` Part 3 section.

### Part 4: Reduced Input LQR

**What was checked:**
- u3 removal and 2-input consistency
- Cost indexing consistency with Part 3
- Observer and initial conditions reuse
- Cost comparison with Part 3
- Cost recomputation from traj.npz (independent verification)

**Results:**
- All gates PASS
- Bd_red shape: (12, 2) (u3 removed)
- Cost J_red: 5.838118e+07 > Part 3 J (expected)
- Observer and initial conditions match Part 2/3
- J recomputation: abs(diff)=2.94e+00, rel=5.04e-08 (matches logged value)

**Evidence:** See `docs/10_system_audit/system_audit_evidence.md` Part 4 section.

### Part 5: Kalman Filter

**What was checked:**
- Noise model (w via Bd, seed 42, Qw, Rv)
- Lk computation and shape
- Estimator stability
- Innovation covariance invertibility
- traj.npz keys and shapes
- Dynamics residual check (x[k+1] = Ad x[k] + Bd u[k] + Bd w[k])

**Results:**
- All gates PASS
- Noise model: Qw=0.05*I3, Rv=0.1*I2, seed=42
- Process noise enters via Bd (verified in code)
- Lk shape: (12, 2)
- Estimator spectral radius: 0.999547 (stable)
- Innovation covariance cond(S)=1.00 (invertible)
- traj.npz contains all required keys
- Dynamics residual: max_norm=0.0, max_abs=0.0, dtype=float64 (perfect match, confirms w via Bd)

**Evidence:** See `docs/10_system_audit/system_audit_evidence.md` Part 5 section.

### Part 6: LQG

**What was checked:**
- K loaded from Part 3 (exact match)
- Lk loaded from Part 5 (exact match)
- Controller uses xhat (not x)
- Noise settings frozen
- Cost reporting (J_true official, J_meas optional)
- No-noise sanity check (mismatch detected and correctly interpreted)
- J_true recomputation from traj.npz (independent verification)
- Dynamics residual check (x[k+1] = Ad x[k] + Bd u[k] + Bd w[k])
- Measurement construction check (y_true = Cmeas@x, y_meas = y_true+v)
- Kalman filter update residual check (xhat[k+1] = Ad@xhat[k] + Bd@u[k] + Lk@(y_meas[k] - Cmeas@xhat[k]))

**Results:**
- All gates PASS
- K equality: max_abs_diff=0.0 (exact match)
- Lk equality: max_abs_diff=0.0 (exact match)
- Controller diagnostic: ||u - (-K xhat)||=0, ||u - (-K x)||>0
- Noise settings: seed=42, Qw=0.05*I3, Rv=0.1*I2
- J_true=4.260967e+02 (official), J_meas=6.343417e+02 (comparison)
- J_true recomputation: abs(diff)=4.18e-05, rel=9.82e-08 (matches logged value)
- Dynamics residual: max_norm=0.0, max_abs=0.0, dtype=float64 (perfect match)
- Measurement construction: max_diff=0.0 for k=0..N-1 (perfect match, y_true[N] not checked)
- Kalman filter update residual: max_norm=0.0, max_abs=0.0, dtype=float64 (perfect match, confirms Bd@u[k] term included)
- No-noise sanity check: Mismatch detected and correctly explained (Lk≠L, not an error)

**Evidence:** See `docs/10_system_audit/system_audit_evidence.md` Part 6 section.

## Cross-Part Consistency Audit

### Ts and N Consistency

| Part | Ts | N | Evidence |
|------|----|---|----------|
| 0 | 0.01 | 1000 | baseline_check.py line 33 |
| 1 | 0.01 | 1000 | run_observability.py line 232 |
| 2 | 0.01 | 1000 | run_observer_sim.py lines 195-196, 246 |
| 3 | 0.01 | 1000 | run_lqr_with_observer.py lines 412-413, 605 |
| 4 | 0.01 | 1000 | run_lqr_reduced_input.py lines 474-475, 670 |
| 5 | 0.01 | 1000 | run_kalman_filter.py lines 339-340, 416 |
| 6 | 0.01 | 1000 | run_lqg.py lines 388-389, 474 |

**Status:** PASS - All parts use Ts=0.01 and N=1000 consistently.

### x and u Indexing Convention

| Part | x shape | u shape | Evidence |
|------|---------|---------|----------|
| 2 | (12, N+1) | (m, N) | results.txt |
| 3 | (12, N+1) | (3, N) | results.txt |
| 4 | (12, N+1) | (2, N) | results.txt |
| 5 | (12, N+1) | (3, N) | traj.npz verification |
| 6 | (12, N+1) | (3, N) | traj.npz verification |

**Status:** PASS - All parts follow frozen indexing convention.

### Cmeas Consistency

| Part | Cmeas shape | Measures | Evidence |
|------|-------------|----------|----------|
| 2 | (2, 12) | x1, x6 | results.txt lines 4-8 |
| 3 | (2, 12) | x1, x6 | results.txt |
| 4 | (2, 12) | x1, x6 | results.txt |
| 5 | (2, 12) | x1, x6 | results.txt |
| 6 | (2, 12) | x1, x6 | results.txt |

**Status:** PASS - Cmeas identical across Parts 2-6.

### x0 and xhat0 Consistency

| Part | x0 | xhat0 | Evidence |
|------|----|-------|----------|
| 2 | [0,0,0,1,1,1,0,0,0,0,0,0] | [0,0,0,0,0,1,0,0,0,0,0,0] | results.txt lines 11-12 |
| 3 | [0,0,0,1,1,1,0,0,0,0,0,0] | [0,0,0,0,0,1,0,0,0,0,0,0] | results.txt |
| 4 | [0,0,0,1,1,1,0,0,0,0,0,0] | [0,0,0,0,0,1,0,0,0,0,0,0] | results.txt |
| 5 | [0,0,0,1,1,1,0,0,0,0,0,0] | [0,0,0,0,0,1,0,0,0,0,0,0] | results.txt |
| 6 | [0,0,0,1,1,1,0,0,0,0,0,0] | [0,0,0,0,0,1,0,0,0,0,0,0] | results.txt |

**Status:** PASS - x0 and xhat0 match invariants doc exactly.

### Part 6 K and Lk Reuse

| Matrix | Source | Loaded in Part 6 | Equality Proof |
|--------|--------|------------------|----------------|
| K | Part 3 outputs | Yes | max_abs_diff=0.0 |
| Lk | Part 5 outputs | Yes | max_abs_diff=0.0 |

**Status:** PASS - Part 6 uses K and Lk from Parts 3 and 5 exactly.

### Part 3 Baseline Metrics Comparison

| Metric | Part 3 (Audit) | cross_part_invariants.md | Match |
|--------|----------------|--------------------------|-------|
| J | 3.915420e+07 | 3.915420e+07 | YES |
| max\|u1\| | 1.228057e+03 | 1.228057e+03 | YES |
| max\|u2\| | 2.700929e+01 | 2.700929e+01 | YES |
| max\|u3\| | 2.403429e+03 | 2.403429e+03 | YES |
| max\|u\| overall | 2.403429e+03 | 2.403429e+03 | YES |

**Status:** PASS - All metrics match cross_part_invariants.md exactly.

## Findings and Fixes

### Static Code Audit Findings

1. **Ts and N definitions:** All parts consistently use Ts=0.01 and N=1000. No violations found.

2. **Noise settings:** Parts 5 and 6 correctly use seed=42, Qw=0.05*I3, Rv=0.1*I_p. No violations found.

3. **Process noise entry:** Verified in code that w enters via Bd (x[k+1] = Ad x[k] + Bd u[k] + Bd w[k]) in both Parts 5 and 6. No violations found.

4. **Cost definitions:** Parts 3 and 4 correctly sum k=0..N-1. Part 6 correctly uses J_true (official) and J_meas (optional). No violations found.

5. **Cmeas, x0, xhat0 consistency:** All parts use identical values. No violations found.

### Runtime Findings

1. **Part 2 observer warning:** UserWarning about convergence tolerance in pole placement. This is a warning, not an error, and the observer design succeeds with spectral radius 0.8 as expected.

2. **Part 6 no-noise sanity check:** Expected difference between Part 3 and Part 6 (no noise) due to Lk≠L. This is documented and explained in results.txt.

### Fixes Applied

No fixes were needed. All gates passed on first run.

### Re-run Notes

All parts were re-run from scratch. Parts 5 and 6 were run twice to verify reproducibility (seed 42). All results were identical between runs.

## Open Issues

None. All gates passed and all invariants are satisfied.

## Additional Verification Checks

### Cost Recomputation (Parts 3, 4, 6)

**Gates:** GATE-P3-8, GATE-P4-5, GATE-P6-7 - Independent recomputation from traj.npz

**Check:**
```python
# Independent recomputation from traj.npz using exact same formula
J_recomputed = sum_{k=0}^{N-1} (u[k]^T u[k] + y1[k]^2 + y6[k]^2)
where y = Cmeas @ x[k] (for Parts 3, 4) or y_true[k] (for Part 6)
```

**Results:**

| Part | J_recomputed | J_logged | abs(diff) | rel(diff) | Status |
|------|--------------|----------|-----------|-----------|--------|
| 3 | 3.915420e+07 | 3.915420e+07 | 1.50e-03 | 3.83e-11 | PASS |
| 4 | 5.838118e+07 | 5.838118e+07 | 2.94e+00 | 5.04e-08 | PASS |
| 6 | 4.260967e+02 | 4.260967e+02 | 4.18e-05 | 9.82e-08 | PASS |

**Status:** All PASS (well within tolerance, confirms cost computation consistency)

### Dynamics Residual Checks

**Gates:** GATE-P5-6, GATE-P6-8 - Verify x[k+1] = Ad x[k] + Bd u[k] + Bd w[k]

**Check:**
```python
residual[k] = x[k+1] - Ad @ x[k] - Bd @ u[k] - Bd @ w[k]
max_norm(residual) <= 1e-9
max_abs(residual) <= 1e-9
```

**Results:**
- Part 5: max_norm = 0.0, max_abs = 0.0, dtype = float64 (perfect match)
- Part 6: max_norm = 0.0, max_abs = 0.0, dtype = float64 (perfect match)
- **Status:** PASS (confirms w enters via Bd correctly, no broadcasting issues)

### Measurement Construction Check

**Gate:** GATE-P6-9 - Verify y_true = Cmeas@x and y_meas = y_true + v

**Check:**
```python
# Check domain: k=0..N-1 only (y_true[N] may not be set)
max_abs(y_true - Cmeas@x) <= 1e-10  (for k=0..N-1)
max_abs(y_meas - (y_true + v)) <= 1e-10  (for k=0..N-1)
```

**Result:**
- Check domain: k=0..999 (N=1000, y_true[N] not checked)
- max_abs(y_true - Cmeas@x) = 0.0 (for k=0..N-1)
- max_abs(y_meas - (y_true + v)) = 0.0 (for k=0..N-1)
- Data types: float64 (consistent)
- **Status:** PASS (perfect match)

**Note:** y_true[N] is not set in the simulation loop (by design), but this does not affect cost computation (cost sums k=0..N-1).

## Conclusions

All Parts 0-6 have been verified with explicit PASS gates (38 total gates). The system demonstrates:
- Consistent use of frozen invariants (Ts, N, Cmeas, x0, xhat0)
- Correct noise model implementation (w via Bd, seed 42) - **numerically verified**
- Proper matrix reuse (K from Part 3, Lk from Part 5 in Part 6)
- Reproducible results (Parts 5 and 6 with seed 42)
- Correct cost definitions and indexing conventions - **independently verified**
- Dynamics equations satisfied exactly - **numerically verified**
- Measurement construction correct - **numerically verified**

The codebase is ready for Part 7 implementation (if needed) with confidence in the foundation established by Parts 0-6.
