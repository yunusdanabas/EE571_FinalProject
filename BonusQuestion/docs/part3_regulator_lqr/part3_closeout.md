# Part 3 Closeout

## Summary of Work

Implemented the discrete-time infinite-horizon LQR regulator for vehicle trajectory tracking. Designed weighting matrices Q and R, computed the feedback gain K_LQR using the discrete algebraic Riccati equation, integrated the controller into the simulation loop, and validated performance with baseline case (scale=1).

**Work performed**:
1. Designed Q matrix (5×5, positive semi-definite) with higher weights on tracking errors
2. Designed R matrix (2×2, positive definite) to balance control effort
3. Computed K_LQR using `scipy.linalg.solve_discrete_are`
4. Verified K_LQR dimensions (2×5) and closed-loop stability
5. Integrated LQR controller into simulation loop
6. Ran baseline simulation (scale=1) and verified error state convergence
7. Generated and saved trajectory, error, and input plots

## Files Changed/Added

- **Modified**: `code/vehicle_tracking.py`
  - Lines 143-169: Added LQR controller design (Q, R, K_LQR computation)
  - Line 175: Updated `controller_type` default to 'LQR'
  - Lines 409-420: Added `compute_metrics()` function for numerical analysis
  - Lines 421-500: Updated main block to run LQR simulation, save plots to both locations, and save numerical outputs

- **Added**: `results/plots/lqr_scale1_trajectory.png`
- **Added**: `results/plots/lqr_scale1_errors.png`
- **Added**: `results/plots/lqr_scale1_inputs.png`
- **Added**: `output/part3/lqr_scale1_trajectory.png` (copy of trajectory plot)
- **Added**: `output/part3/lqr_scale1_errors.png` (copy of errors plot)
- **Added**: `output/part3/lqr_scale1_inputs.png` (copy of inputs plot)
- **Added**: `output/part3/lqr_scale1_outputs.txt` (numerical outputs and metrics)

## How to Run / Reproduce

1. Navigate to project directory:
   ```bash
   cd /home/yunusdanabas/EE571_FinalProject/BonusQuestion
   ```

2. Run the simulation:
   ```bash
   python code/vehicle_tracking.py
   ```

3. The code will:
   - Compute discrete model (Ad, Bd) from Part 2
   - Design LQR controller (Q, R, K_LQR)
   - Run baseline simulation (scale=1) with LQR controller
   - Generate and save plots to `results/plots/`

4. View plots:
   - `results/plots/lqr_scale1_trajectory.png` or `output/part3/lqr_scale1_trajectory.png`: Vehicle trajectory vs reference
   - `results/plots/lqr_scale1_errors.png` or `output/part3/lqr_scale1_errors.png`: Tracking errors (ey, epsi, ev)
   - `results/plots/lqr_scale1_inputs.png` or `output/part3/lqr_scale1_inputs.png`: Control inputs (steering, acceleration)
   - `output/part3/lqr_scale1_outputs.txt`: Numerical outputs including Q/R matrices, K_LQR, eigenvalues, and performance metrics

**Note**: The `controller_type` variable (line 175) controls which controller is used. Set to 'LQR' for LQR controller (default).

## Checks Performed

### Q Matrix Verification
- ✅ Q is 5×5 matrix (diagonal)
- ✅ Q is positive semi-definite (all diagonal elements ≥ 0)
- ✅ Q values: `diag([5.0, 5.0, 50.0, 50.0, 30.0])`
  - `Q[0,0] = 5.0`: Weight on lateral velocity (vy)
  - `Q[1,1] = 5.0`: Weight on yaw rate (r)
  - `Q[2,2] = 50.0`: Weight on cross-track error (ey)
  - `Q[3,3] = 50.0`: Weight on heading error (epsi)
  - `Q[4,4] = 30.0`: Weight on speed error (ev)

### R Matrix Verification
- ✅ R is 2×2 matrix (diagonal)
- ✅ R is positive definite (all diagonal elements > 0)
- ✅ R values: `diag([2.0, 1.0])`
  - `R[0,0] = 2.0`: Weight on steering effort (delta_reg)
  - `R[1,1] = 1.0`: Weight on acceleration effort (ax_reg)

### K_LQR Verification
- ✅ K_LQR shape is (2, 5) (verified with assertion)
- ✅ No NaN or Inf values in K_LQR (verified with assertion)
- ✅ K_LQR values:
  ```
  K_LQR = [[ 3.02452260e-01,  6.48540402e-01,  2.05084928e+00,  7.80061059e+00,  4.91619369e-15],
           [-3.66126783e-16,  1.31551779e-15,  7.63512377e-15,  4.26464551e-14,  5.18543526e+00]]
  ```
  - Row 1: Steering regulation gains for [vy, r, ey, epsi, ev]
  - Row 2: Acceleration regulation gains for [vy, r, ey, epsi, ev]

### Closed-Loop Stability
- ✅ All closed-loop eigenvalues inside unit circle (verified)
- ✅ Closed-loop eigenvalues: `[0.16205, 0.93595±0.08550j, 0.85955, 0.89629]`
- ✅ Maximum eigenvalue magnitude: 0.940 (well inside unit circle)

### Simulation Integration
- ✅ Error state vector constructed correctly: `x_e = [vy, r, ey, epsi, ev]` (line 294)
- ✅ Regulation input computed: `u_reg = -K_LQR @ x_e` (line 301)
- ✅ Feedforward and regulation combined correctly (lines 306-307)
- ✅ Saturations applied: steering ±25°, acceleration [-6, +3] m/s² (lines 310-311)

### Code Execution
- ✅ Code runs without errors
- ✅ Controller stabilizes error states (errors converge to zero)
- ✅ Plots generated and saved successfully

## Outputs Produced

### LQR Controller Design Location

**File**: `code/vehicle_tracking.py`  
**Lines**: 143-169

**Code added**:
```python
# LQR Controller Design (Part 3)
# Q matrix: Penalize error states [vy, r, ey, epsi, ev]
# Higher weights on tracking errors (ey, epsi, ev) than internal states (vy, r)
Q = np.diag([5.0, 5.0, 50.0, 50.0, 30.0])  # 5x5, positive semi-definite

# R matrix: Penalize control effort [delta_reg, ax_reg]
# Moderate weights to allow reasonable control authority
R = np.diag([2.0, 1.0])  # 2x2, positive definite

# Solve discrete algebraic Riccati equation
P = solve_discrete_are(Ad, Bd, Q, R)

# Compute LQR feedback gain
K_LQR = np.linalg.inv(R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)
```

### Q and R Matrix Rationale

**Q Matrix Design**:
- Higher weights on tracking errors (`ey`, `epsi`, `ev`) since these directly affect path following performance
- Lower weights on internal states (`vy`, `r`) as they are intermediate variables
- Values chosen: `[5.0, 5.0, 50.0, 50.0, 30.0]` to emphasize tracking accuracy while allowing reasonable control authority

**R Matrix Design**:
- Moderate weights to balance control effort with tracking performance
- Steering weight (2.0) slightly higher than acceleration (1.0) to prevent excessive steering
- Values chosen to allow sufficient control authority while penalizing excessive inputs

### Simulation Results

**Baseline case (scale=1)**:
- Initial conditions: X(0) = X_ref(0) - 2.0 m, Y(0) = Y_ref(0) + 1.0 m, ψ(0) = ψ_ref(0) + 8°, v_x(0) = 15 - 5 = 10 m/s
- Controller successfully stabilizes all error states
- Tracking errors converge to near zero
- Control inputs remain within saturation limits

### Plots Saved

**In `results/plots/` and `output/` folders:**

1. **`lqr_scale1_trajectory.png`**
   - Vehicle trajectory (red dashed) vs reference path (blue solid)
   - Shows path following performance

2. **`lqr_scale1_errors.png`**
   - Cross-track error (ey) over time
   - Heading error (epsi) over time
   - Speed error (ev) over time
   - All errors converge to near zero

3. **`lqr_scale1_inputs.png`**
   - Steering angle (delta) with ±25° saturation limits
   - Longitudinal acceleration (ax) with [-6, +3] m/s² saturation limits
   - Shows control effort and saturation behavior

### Numerical Outputs Saved

**`output/part3/lqr_scale1_outputs.txt`** contains:
- Controller design parameters (Q, R matrices)
- K_LQR matrix (2×5)
- Closed-loop eigenvalues with magnitudes
- Tracking error metrics:
  - RMS errors (ey, epsi, ev)
  - Maximum absolute errors
  - Final errors
- Control effort metrics:
  - RMS steering angle and acceleration
  - Saturation counts
- Simulation parameters (Ts, duration, etc.)

This file provides a complete numerical summary of the LQR controller performance for the baseline case.

## Issues / TODOs for Next Part

### Handoff to Part 4 (Pole Placement Regulator)

**LQR Implementation Status**:
- ✅ LQR controller fully implemented and tested
- ✅ Baseline case (scale=1) completed
- ⏳ Additional cases (scale=2, scale=3) will be run in Part 5

**For Part 4 (Pole Placement)**:

**Tasks**:
- Choose 5 real poles inside unit circle (no complex poles allowed)
- Compute K_PP using `scipy.signal.place_poles(Ad, Bd, desired_poles)`
- Verify K_PP is 2×5
- Verify closed-loop eigenvalues match desired poles (all real)
- Use same integration pattern as LQR (lines 293-311)
- Run baseline simulation (scale=1)
- Generate and save plots

**Code Structure**:
- Controller design location: Lines 171-173 (TODO comments)
- Simulation integration: Same pattern as LQR (lines 293-311)
- Main block: Add pole placement option alongside LQR

**Important Notes**:
1. **Same discretization**: Both regulators use identical `Ad` and `Bd` from Part 2
2. **Same feedforward**: Both use same feedforward terms (lines 290-291)
3. **Same saturations**: Both apply same input limits (lines 310-311)
4. **Fair comparison**: Ensure same initial conditions and simulation parameters

**Controller Selection**:
- Current default: `controller_type = 'LQR'` (line 175)
- Part 4 should add 'PP' option
- Part 5 will run both controllers for comparison

### Code Structure Summary

**Current state**:
- Lines 143-169: LQR controller design (✅ complete)
- Lines 171-173: Pole placement placeholder (⏳ for Part 4)
- Lines 293-311: Controller integration in simulation loop (✅ works for both LQR and PP)
- Lines 391-420: Main block with LQR implementation (✅ complete)

**Next steps**:
- Part 4: Implement pole placement controller at lines 171-173
- Part 5: Run all 6 cases (2 regulators × 3 scales) and generate comparison plots

### Verification Checklist

- [x] Q is 5×5, positive semi-definite
- [x] R is 2×2, positive definite
- [x] K_LQR is 2×5 (verified with assertion)
- [x] Code runs without errors
- [x] Controller stabilizes error states
- [x] Feedforward and regulation properly combined
- [x] Saturations applied correctly
- [x] Baseline plots saved to `results/plots/`
- [x] Closeout document complete with Q/R values documented

### Notes

- **Q/R Tuning**: Initial values provide good tracking performance. May need adjustment in Part 5 for different scales.
- **Closed-loop Stability**: All eigenvalues well inside unit circle, ensuring stable closed-loop system.
- **Control Authority**: R matrix values allow sufficient control authority without excessive saturation.
- **Error Convergence**: All tracking errors converge to near zero, demonstrating effective regulation.
