---
name: Part 3 LQR Simplification
overview: Simplify the Part 3 LQR controller code from 1050+ lines to a clean ~150-200 line implementation that reuses Part 2 utilities and adds a closed-loop simulation function to utils.
todos:
  - id: utils-sim
    content: Add simulate_lqr_with_observer() to final/utils/simulation.py
    status: completed
  - id: lqr-code
    content: Create simplified final/part3/lqr_controller.py
    status: completed
    dependencies:
      - utils-sim
  - id: run-code
    content: Run lqr_controller.py to generate outputs
    status: completed
    dependencies:
      - lqr-code
  - id: report
    content: Create final/part3/part3_report.md with results
    status: completed
    dependencies:
      - run-code
---

# Part 3: LQR Controller Simplification

## Overview

The original `python/part3/run_lqr_with_observer.py` contains ~1050 lines with excessive validation, verbose logging, and redundant checks. The simplified version will keep the same math/algorithms but use existing utilities and reduce code to ~150-200 lines.

## Analysis of Original Code

**Functions to Remove/Simplify**:

- `check_detectability()` (~70 lines) - remove, excessive validation
- `check_stabilizability()` (~110 lines) - remove, excessive validation
- `design_lqr()` - keep core logic, remove verbose logging
- `simulate_closed_loop_with_observer()` - move to utils, simplify
- `compute_cost_metrics()` - keep inline, simplify

**Key Math to Preserve**:

- DARE solver: `P = solve_discrete_are(Ad, Bd, Q, R)`, `K = solve(R + Bd'PBd, Bd'PAd)`
- Cost: `J = sum(u[k]'u[k] + y1[k]^2 + y6[k]^2) `where `y = Cy @ x`
- Closed-loop: `u[k] = -K @ xhat[k]` (uses estimated states from observer)

## Dependencies

**From [`final/part2/observer_design.py`](final/part2/observer_design.py)**:

- `get_part2_C()` - measurement matrix C
- `get_part2_initial_conditions()` - x0, xhat0  
- `design_observer_gain()` - observer gain L

**From [`final/utils/model.py`](final/utils/model.py)**:

- `build_continuous_model()`, `discretize_zoh()`

## Implementation Plan

### 1. Add to [`final/utils/simulation.py`](final/utils/simulation.py)

Add a `simulate_lqr_with_observer()` function:

```python
def simulate_lqr_with_observer(Ad, Bd, Cd, K, L, x0, xhat0, N, Ts):
    """Closed-loop simulation with observer-based LQR control."""
    # u[k] = -K @ xhat[k], observer updates xhat
    ...
    return {'x': x, 'xhat': xhat, 'u': u, 'y': y, 'e': e, 't': t}
```

### 2. Create [`final/part3/lqr_controller.py`](final/part3/lqr_controller.py)

Simplified structure (~150 lines):

```python
# Imports from final.utils and final.part2
# design_lqr() function - minimal DARE wrapper
# compute_cost() function - compact cost computation
# main() function:
#   - Build model using utils
#   - Get C, L, x0, xhat0 from Part 2
#   - Define Q = Cy'Cy, R = I3
#   - Design LQR (DARE)
#   - Simulate closed-loop
#   - Compute cost
#   - Plot and save results
```

### 3. Run and Generate Outputs

Execute script to produce in `final/part3/outputs/`:

- `outputs_y1_y6.png` - output trajectories
- `inputs_u1_u2_u3.png` - input trajectories  
- `estimation_error_norm.png` - error convergence
- `results.txt` - key metrics (K, cost J, spectral radius)
- `K_matrix.npy`, `L_matrix.npy`, `traj.npz` - for later parts

### 4. Create [`final/part3/part3_report.md`](final/part3/part3_report.md)

Concise report with:

- Objective: LQR controller using estimated states
- Approach: DARE solver, closed-loop with observer
- Key results: K gain, cost J, max inputs, spectral radius
- Figures: reference output plots

## Key Code Simplifications

| Original | Simplified |

|----------|------------|

| 70-line `check_detectability()` | Removed |

| 110-line `check_stabilizability()` | Removed |

| 90-line `simulate_closed_loop_with_observer()` | Moved to utils (~40 lines) |

| 400+ lines of verbose main block | Compact ~80-line main() |

| Duplicate model building code | Reuse `final.utils.model` |

| Duplicate observer design | Import from `final.part2` |