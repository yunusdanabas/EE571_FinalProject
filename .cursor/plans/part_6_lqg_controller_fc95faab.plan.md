---
name: Part 6 LQG Controller
overview: Create a simplified LQG controller implementation that combines LQR (Part 3) with Kalman filter (Part 5) to control the noisy system, reusing utils functions and following the established code style.
todos:
  - id: add-lqg-sim
    content: Add simulate_lqg function to final/utils/simulation.py
    status: completed
  - id: create-lqg-code
    content: Create simplified final/part6/lqg_controller.py
    status: completed
    dependencies:
      - add-lqg-sim
  - id: run-generate
    content: Run code to generate outputs in final/part6/outputs/
    status: completed
    dependencies:
      - create-lqg-code
  - id: create-report
    content: Create final/part6/part6_report.md with results
    status: completed
    dependencies:
      - run-generate
---

# Part 6: LQG Controller Implementation

## Overview

Combine the LQR controller gain K (Part 3) with the Kalman filter gain Lk (Part 5) to implement an LQG controller for the noisy 6-mass system. The original implementation (`python/part6/run_lqg.py`) is 1,069 lines with excessive validation. We will simplify to ~150 lines.

## Key Dependencies

- **Part 3**: Load K matrix from `final/part3/outputs/K_matrix.npy`
- **Part 5**: Load Lk matrix from `final/part5/outputs/Lk_matrix.npy`
- **Part 2**: `get_part2_C()` and `get_part2_initial_conditions()` from `final/part2/observer_design.py`

## Implementation Strategy

### 1. Add LQG Simulation Function to Utils

Add `simulate_lqg` function to [`final/utils/simulation.py`](final/utils/simulation.py):

```python
def simulate_lqg(Ad, Bd, Cd, K, Lk, x0, xhat0, N, Ts, Qw, Rv, seed=42):
    """LQG closed-loop: u[k] = -K @ xhat[k] with noisy measurements."""
```

This is similar to existing `simulate_kalman_noisy` but:

- Adds control input u[k] = -K @ xhat[k]
- Updates state with both control and noise: x[k+1] = Ad @ x[k] + Bd @ u[k] + Bd @ w[k]
- Updates estimator with control: xhat[k+1] = Ad @ xhat[k] + Bd @ u[k] + Lk @ innovation[k]

### 2. Create Simplified [`final/part6/lqg_controller.py`](final/part6/lqg_controller.py)

Structure following Part 5's pattern:

```python
# Imports and dependencies
# design_lqr() - reuse from Part 3
# design_kalman_filter() - reuse from Part 5
# compute_lqg_cost() - simplified cost function
# main() - orchestration
```

Key simplifications vs original:

- Remove fingerprinting and hash computation
- Remove no-noise sanity checks
- Remove verbose console output
- Remove 20+ validation blocks
- Use utils for simulation

### 3. Generate Outputs

Save to `final/part6/outputs/`:

- `results.txt` - Key metrics (J, spectral radii, RMS errors)
- `K_matrix.npy`, `Lk_matrix.npy` - Gain matrices
- `traj.npz` - Trajectories (x, xhat, u, y_true, y_meas, yhat)
- `outputs_comparison.png` - Part 3 vs Part 6 outputs
- `inputs_u1_u2_u3.png` - Control inputs
- `estimation_error_norm.png` - Error convergence

### 4. Create `part6_report.md`

Sections: Objective, Approach, Key Results (J, comparison with Part 3), Findings (separation principle, noise impact).

## Files Modified/Created

| File | Action |

|------|--------|

| [`final/utils/simulation.py`](final/utils/simulation.py) | Add `simulate_lqg` function at end |

| [`final/part6/lqg_controller.py`](final/part6/lqg_controller.py) | Create simplified implementation |

| `final/part6/outputs/*` | Generate via code execution |

| `final/part6/part6_report.md` | Create report |

## Key Algorithm (Preserved from Original)

LQG dynamics (same math, simplified code):

```
y_true[k] = Cmeas @ x[k]
y_meas[k] = y_true[k] + v[k]
u[k] = -K @ xhat[k]
x[k+1] = Ad @ x[k] + Bd @ u[k] + Bd @ w[k]
xhat[k+1] = Ad @ xhat[k] + Bd @ u[k] + Lk @ (y_meas[k] - Cmeas @ xhat[k])
```

Cost function: `J = sum(u[k]'u[k] + y1[k]^2 + y6[k]^2)` using y_true (not y_meas).