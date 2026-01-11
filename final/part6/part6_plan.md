# Part 6: LQG Controller

## Objective

Combine the LQR controller (Part 3) with the Kalman filter (Part 5) to implement an LQG controller for the noisy system.

## Source Files

- Original implementation: `python/part6/run_lqg.py`
- Dependencies: Part 3 (LQR gain K), Part 5 (Kalman gain Lk)
- Reference: `docs/00_anchor.md` (LQG definitions, cost computation)

## Key Tasks

1. Load K from Part 3 and Lk from Part 5
2. Simulate closed-loop system with noise (LQG)
3. Compute cost metrics (J_true using y_true, not y_meas)
4. Compare with Part 3 baseline (no noise)
5. Plot comparisons

## Simplification Targets

- Simplify LQG simulation code
- Streamline cost computation
- Use utils for comparison plots
- Remove excessive validation

## Utils Functions to Check/Add

Check `final/utils/` for:
- `model.py`: Model functions
- `simulation.py`: LQG simulation (closed-loop with Kalman filter and noise)
- `plotting.py`: Comparison plots (Part 3 vs Part 6 overlays)

Add LQG-specific functions if needed.

## Expected Outputs

- LQG closed-loop trajectories
- Cost comparison with Part 3
- Plots: outputs comparison, inputs, estimation errors

## Report Content

The `part6_report.md` should include:
- Objective: LQG controller (LQR + Kalman filter)
- Approach: combine Part 3 controller with Part 5 estimator
- Key results: cost J, comparison with Part 3, noise impact
- Findings: performance under noise, separation principle
