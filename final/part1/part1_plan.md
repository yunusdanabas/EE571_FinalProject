# Part 1: Observability Analysis

## Objective

Analyze observability of the discrete-time system and perform Kalman decomposition to separate observable and unobservable subspaces.

## Source Files

- Original implementation: `python/part1/run_observability.py`
- Supporting modules: `python/part1/observability_rank.py`, `python/part1/kalman_decomp_obsv.py`

## Key Tasks

1. Build and discretize system (reuse Part 0 approach)
2. Compute observability matrix and rank
3. Perform Kalman decomposition
4. Analyze eigenvalues of observable/unobservable blocks

## Simplification Targets

- Simplify rank computation (remove excessive SVD details)
- Streamline Kalman decomposition code
- Remove verbose validation checks
- Keep only essential results

## Utils Functions to Check/Add

Check `final/utils/` for:
- `model.py`: `build_continuous_model()`, `discretize_zoh()` (from Part 0)

Add new functions if needed (matrix analysis, eigenvalue formatting, etc.).

## Expected Outputs

- Observability rank and analysis
- Kalman decomposition results
- Eigenvalue analysis

## Report Content

The `part1_report.md` should include:
- Objective: observability analysis with single sensor
- Approach: observability matrix rank, Kalman decomposition
- Key results: rank value, observable/unobservable dimensions
- Findings: system is not fully observable with single sensor
