# Part 3: LQR Controller Design

## Objective

Design a discrete-time LQR controller that uses estimated states from the Part 2 observer and minimizes the cost J = Σ(u^T u + y1^2 + y6^2).

## Source Files

- Original implementation: `python/part3/run_lqr_with_observer.py`
- Reference: `docs/00_anchor.md` (LQR cost function, Part 2 dependencies)

## Key Tasks

1. Load Part 2 C matrix and observer design
2. Design LQR controller (solve DARE)
3. Simulate closed-loop system with observer
4. Compute cost metrics
5. Plot inputs, outputs, and estimation errors

## Simplification Targets

- Simplify LQR design (remove excessive stabilizability checks)
- Streamline closed-loop simulation
- Use utils for plotting
- Simplify cost computation

## Utils Functions to Check/Add

Check `final/utils/` for:
- `model.py`: Model functions
- `simulation.py`: Closed-loop simulation with observer
- `plotting.py`: Input plots, output plots, error plots

Add functions if missing (cost computation, DARE solver wrapper, etc.).

## Expected Outputs

- LQR gain K
- Closed-loop trajectories (states, inputs, outputs)
- Cost J and input magnitudes
- Plots: inputs, outputs, estimation errors

## Report Content

The `part3_report.md` should include:
- Objective: LQR controller using estimated states
- Approach: DARE solver, closed-loop simulation
- Key results: gain K, cost J, max input magnitudes
- Findings: controller stabilizes system, performance metrics
