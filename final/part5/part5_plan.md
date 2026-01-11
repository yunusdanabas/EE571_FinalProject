# Part 5: Kalman Filter Design

## Objective

Design a steady-state Kalman filter for the system with process and measurement noise.

## Source Files

- Original implementation: `python/part5/run_kalman_filter.py`
- Reference: `docs/00_anchor.md` (noise parameters, Part 5 definitions)

## Key Tasks

1. Define noise covariances (Qw = 0.05*I3, Rv = 0.1*I2)
2. Design Kalman filter (solve DARE for estimator)
3. Simulate noisy system with Kalman filter
4. Compute estimation metrics (RMS errors)

## Simplification Targets

- Simplify Kalman filter design code
- Streamline noisy simulation
- Use utils for plotting
- Simplify metrics computation

## Utils Functions to Check/Add

Check `final/utils/` for:
- `model.py`: Model functions
- `simulation.py`: Noisy simulation, Kalman filter simulation
- `plotting.py`: Noisy signal plots, estimation comparisons

Add noise-related functions if needed.

## Expected Outputs

- Kalman gain Lk
- Noisy trajectories and estimates
- Estimation error metrics (RMS)
- Plots: noisy measurements, estimates, errors

## Report Content

The `part5_report.md` should include:
- Objective: Kalman filter for noisy system
- Approach: DARE for estimator, steady-state Kalman gain
- Key results: gain Lk, RMS estimation errors
- Findings: filter performance in presence of noise
