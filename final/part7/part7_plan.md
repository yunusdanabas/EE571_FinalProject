# Part 7: Sensor Augmentation Analysis

## Objective

Analyze the impact of adding more sensors by comparing two augmented sensor configurations (4 sensors and 6 sensors) against the Part 6 baseline (2 sensors).

## Source Files

- Original implementation: `python/part7/run_part7.py`
- Reference: `docs/00_anchor.md` (Part 7 sensor configurations)

## Key Tasks

1. Define Case 1 C matrix (4×12, measures x1, x2, x5, x6)
2. Define Case 2 C matrix (6×12, measures all positions x1-x6)
3. Redesign Kalman filter for each case (controller K unchanged)
4. Simulate LQG for each case
5. Compare estimation and regulation performance

## Simplification Targets

- Simplify sensor configuration setup
- Streamline multi-case simulation
- Use utils for comparison plots
- Simplify performance comparison

## Utils Functions to Check/Add

Check `final/utils/` for:
- `model.py`: Model functions
- `simulation.py`: LQG simulation
- `plotting.py`: Multi-case comparison plots

Add comparison utilities if needed.

## Expected Outputs

- Kalman gains for Case 1 and Case 2
- Performance metrics for each case
- Comparison plots (2-sensor vs 4-sensor vs 6-sensor)
- Analysis of sensor augmentation benefits

## Report Content

The `part7_report.md` should include:
- Objective: analyze sensor augmentation impact
- Approach: two augmented configurations, LQG design
- Key results: performance metrics for each case
- Findings: whether more sensors help estimation/regulation, trade-offs
