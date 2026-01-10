# Part 6 Updates Summary

## Completed Updates (Pre-Part 7)

### 1. Cost Convention Locked
- **Location**: `python/part6/run_lqg.py`, `compute_lqg_metrics()` function
- **Convention**:
  - Official cost `J_true` uses `y_true = Cmeas @ x` (does not penalize uncontrollable measurement noise)
  - Comparison cost `J_meas` uses `y_meas = Cmeas @ x + v` (penalizes noise, for comparison only)
  - Cost formula: `J = Σ_{k=0}^{N-1} (u[k]^T u[k] + y1[k]^2 + y6[k]^2)`
  - **Frozen for Parts 6-7**

### 2. Cost Breakdown Added
- **Purpose**: Explain the huge cost drop in Part 6 vs Part 3
- **Components**:
  - `Σ u^T u` (control effort): 4.379727e+01
  - `Σ (y1^2 + y6^2)` (output penalty): 3.822994e+02
- **Explanation**: Part 6 uses much smaller inputs than Part 3, so `Σ u^T u` dominates the drop
- **Location**: Logged in `python/part6/outputs/results.txt`

### 3. Trajectory Saving Consistency
- **Part 3**: Saves `t, x, xhat, u, y, e`
- **Part 5**: Saves `x, xhat, y_true, y_meas, yhat, u, w, v, innovations, t`
- **Part 6**: Saves `t, x, xhat, u, y, e` (same as Part 3) **plus** `y_true, y_meas, yhat, w, v`
- **Status**: Consistent for automated comparisons

### 4. Diagnostics Verified
All new diagnostics are present and working:
- ✅ No-noise sanity check (w=0, v=0 comparison with Part 3)
- ✅ Composite closed-loop fingerprint (augmented system eigenvalues)
- ✅ Dual cost logging (J_true vs J_meas)
- ✅ K matrix fingerprint (||K||_F, max(|K|), hash)
- ✅ Early time control magnitudes

### 5. Artifact Naming Note
**For exports/sharing**: When exporting or attaching files, rename to avoid collisions:
- `results.txt` → `part6_results.txt`
- `plan.md` → `part6_plan.md`
- `closeout.md` → `part6_closeout.md`
- Or keep files under `python/partX/outputs/` and attach the folder

## Ready for Part 7

All hygiene items completed. Part 6 is ready and Part 7 can proceed with:
- Update `Cmeas` to Case 1 and Case 2
- Set `Rv = 0.1 * I_p` with correct dimension `p`
- Redesign steady-state Kalman gain for each case
- Run LQG loop and compare estimation RMS and closed-loop output metrics
