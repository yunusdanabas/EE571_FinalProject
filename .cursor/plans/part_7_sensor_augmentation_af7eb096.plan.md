---
name: Part 7 Sensor Augmentation
overview: Create a simplified implementation of Part 7 sensor augmentation analysis that compares 2-sensor (Part 6 baseline), 4-sensor (Case 1), and 6-sensor (Case 2) configurations, using utility functions and following the final/ code style guidelines.
todos:
  - id: add-sim-util
    content: Add simulate_lqg_augmented() to final/utils/simulation.py
    status: completed
  - id: create-sensor-aug
    content: Create simplified sensor_augmentation.py in final/part7/
    status: completed
    dependencies:
      - add-sim-util
  - id: run-and-verify
    content: Run the script to generate outputs
    status: completed
    dependencies:
      - create-sensor-aug
  - id: create-report
    content: Create part7_report.md with results
    status: completed
    dependencies:
      - run-and-verify
---

# Part 7: Sensor Augmentation Implementation

## Overview

Simplify the original ~1000-line Part 7 implementation to a clean, readable script that analyzes how adding more sensors affects estimation and regulation performance.

## Key Design Decisions

1. **Controller K unchanged** - LQR gain from Part 3 is reused (depends on cost function, not sensors)
2. **Kalman gain Lk redesigned** - Must be computed for each sensor configuration
3. **Noise parameters frozen** - Qw = 0.05 * I_3, Rv = 0.1 * I_p (p varies per case)
4. **Cost function fixed** - J = sum(u'u + y1^2 + y6^2) regardless of sensor count

## Files to Modify/Create

### 1. Add Utility Function to `simulation.py`

Add `simulate_lqg_augmented()` to [`final/utils/simulation.py`](final/utils/simulation.py) to handle the separation between:

- Cmeas (measurement matrix, varies: 2/4/6 rows)
- Cy (cost output selector, fixed: 2 rows for x1, x6)

The existing `simulate_lqg()` doesn't separate these, so a new function is needed.

### 2. Create `sensor_augmentation.py`

Create simplified [`final/part7/sensor_augmentation.py`](final/part7/sensor_augmentation.py):

```python
# Structure (simplified from original 1000+ lines):
def main():
    # 1. Load model, K from Part 3, Lk from Part 5
    # 2. Define Case 1 C matrix (4x12: x1, x2, x5, x6)
    # 3. Define Case 2 C matrix (6x12: x1-x6)
    # 4. Design Kalman filter for each case (DARE)
    # 5. Run LQG simulation for each case
    # 6. Compute metrics and compare against Part 6
    # 7. Generate plots and save results
```

Key simplifications:

- Remove verbose validation checks
- Use utils functions for simulation
- Keep math/algorithms unchanged
- Minimal humanized comments

### 3. Create `part7_report.md`

Create [`final/part7/part7_report.md`](final/part7/part7_report.md) with:

- Objective: analyze if more sensors help estimation/regulation
- Approach: redesign Kalman filter for each sensor configuration
- Key results: comparison table (J, RMS error, spectral radius)
- Findings: conclusions about sensor augmentation benefits

## Expected Outputs in `final/part7/outputs/`

- `results.txt` - Numerical results and comparison table
- `Lk_case1_matrix.npy` - Kalman gain for 4-sensor case
- `Lk_case2_matrix.npy` - Kalman gain for 6-sensor case
- `traj_case1.npz`, `traj_case2.npz` - Simulation trajectories
- `estimation_error_comparison.png` - Error norm vs time
- `outputs_comparison.png` - y1, y6 comparison
- `inputs_comparison.png` - u1, u2, u3 comparison
- `per_state_rms_comparison.png` - Bar chart of per-state RMS

## Dependencies

- Load K from [`final/part3/outputs/K_matrix.npy`](final/part3/outputs/K_matrix.npy)
- Load Part 6 baseline from [`final/part6/outputs/traj.npz`](final/part6/outputs/traj.npz)
- Use `get_part2_C()` and `get_part2_initial_conditions()` from [`final/part2/observer_design.py`](final/part2/observer_design.py)