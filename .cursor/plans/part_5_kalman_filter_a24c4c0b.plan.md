---
name: Part 5 Kalman Filter
overview: Design and implement a simplified discrete-time steady-state Kalman filter for the 6-mass spring system with process and measurement noise. The implementation will follow the established patterns from Parts 3-4 while adding noisy simulation utilities.
todos:
  - id: add-sim-util
    content: Add simulate_kalman_noisy() function to final/utils/simulation.py
    status: completed
  - id: create-kalman
    content: Create simplified kalman_filter.py with DARE design and main()
    status: completed
    dependencies:
      - add-sim-util
  - id: run-and-generate
    content: Run kalman_filter.py to generate outputs in final/part5/outputs/
    status: completed
    dependencies:
      - create-kalman
  - id: create-report
    content: Create part5_report.md with objective, approach, results, and figures
    status: completed
    dependencies:
      - run-and-generate
---

# Part 5: Kalman Filter Implementation Plan

## Objective

Create a simplified Kalman filter implementation in `final/part5/kalman_filter.py` based on the original 780-line implementation in `python/part5/run_kalman_filter.py`. The goal is to reduce code complexity while preserving mathematical correctness.

## Key Algorithm (Unchanged)

1. **Noise covariances**: Qw = 0.05*I_3 (actuator), Rv = 0.1*I_2 (sensor)
2. **Process noise transformation**: Qx = Bd @ Qw @ Bd.T (12x12)
3. **DARE for estimator**: `P = solve_discrete_are(Ad.T, Cmeas.T, Qx, Rv)`
4. **Kalman gain**: `Lk = P @ Cmeas.T @ inv(Cmeas @ P @ Cmeas.T + Rv)`
5. **Stochastic simulation** with zero input (open-loop)

## Implementation Steps

### 1. Add Utility Function to `simulation.py`

Add `simulate_kalman_noisy()` function to [`final/utils/simulation.py`](final/utils/simulation.py):

```python
def simulate_kalman_noisy(Ad, Bd, Cd, Lk, x0, xhat0, N, Ts, Qw, Rv, seed=42):
    """Simulate stochastic system with Kalman filter (zero input)."""
```

This function will:

- Generate process noise w ~ N(0, Qw) and measurement noise v ~ N(0, Rv)
- True system: x[k+1] = Ad @ x[k] + Bd @ w[k]
- Measured output: y_meas[k] = Cd @ x[k] + v[k]
- Kalman update: xhat[k+1] = Ad @ xhat[k] + Lk @ (y_meas[k] - yhat[k])

### 2. Create `kalman_filter.py`

Create simplified main file [`final/part5/kalman_filter.py`](final/part5/kalman_filter.py) with:

- **Imports**: Use `final.utils.model`, `final.utils.simulation`, `final.part2.observer_design`
- **`design_kalman_filter()`**: Solve DARE and compute Kalman gain Lk
- **`compute_rms_metrics()`**: Simplified RMS computation (overall, per-state, steady-state)
- **`main()`**: Build model, design Kalman filter, simulate, plot, save results

Target: ~150-200 lines (vs 780 in original)

### 3. Generate Outputs

Run the code to produce in `final/part5/outputs/`:

- `results.txt` - Key metrics and design parameters
- `Lk_matrix.npy` - Kalman gain matrix
- `traj.npz` - Trajectories (x, xhat, y_true, y_meas, yhat)
- `outputs_y_vs_yhat.png` - True vs estimated outputs
- `estimation_error_norm.png` - Error norm over time
- `estimation_error_x1_x6.png` - Per-output estimation errors
- `per_state_rms_bar.png` - Bar chart of per-state RMS

### 4. Create Report

Create [`final/part5/part5_report.md`](final/part5/part5_report.md) with:

- Objective: Kalman filter for stochastic system
- Approach: DARE-based steady-state Kalman gain
- Key results: Lk shape, spectral radius, RMS errors
- Plots with figure references

## Key Simplifications

| Original | Simplified |

|----------|------------|

| Extensive PSD validation | Trust numpy, remove checks |

| Verbose innovation consistency | Brief comment only |

| Detailed noise sample logging | Omit or minimal |

| 700+ lines with comments | ~150-200 lines |

## Dependencies

- Reuse `get_part2_C()` and `get_part2_initial_conditions()` from [`final/part2/observer_design.py`](final/part2/observer_design.py)
- Reuse `build_continuous_model()` and `discretize_zoh()` from [`final/utils/model.py`](final/utils/model.py)