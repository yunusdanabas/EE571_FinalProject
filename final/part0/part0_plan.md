# Part 0: Baseline Verification

## Objective

Verify the system model by discretizing the continuous-time matrices and running a baseline open-loop simulation.

## Source Files

- Original implementation: `python/part0/baseline_check.py`
- Reference: `docs/00_anchor.md` (baseline definitions)

## Key Tasks

1. Build continuous-time model (A, B, C matrices)
2. Discretize using zero-order hold (ZOH) at Ts=0.01
3. Simulate open-loop system with zero input
4. Generate plots for outputs and displacements

## Simplification Targets

- Remove excessive validation and dimension checks
- Simplify plotting code (use utils functions)
- Keep only essential sanity checks
- Reduce verbose print statements

## Utils Functions to Check/Add

Check `final/utils/` for:
- `model.py`: `build_continuous_model()`, `discretize_zoh()`
- `simulation.py`: `simulate_discrete()` or similar
- `plotting.py`: `plot_outputs()`, `plot_states()` or generic `plot_signals()`

Add functions to utils if missing (at end of file).

## Expected Outputs

- `output_plot.png` - System output (y = Cx, single trace)
- `displacements_plot.png` - All 6 mass displacements

## Report Content

The `part0_report.md` should include:
- Brief explanation: baseline verification of discretized model
- Key results: matrix dimensions, discretization validation
- Figures: reference the plots in `outputs/`
- Findings: confirm system is stable in open-loop
