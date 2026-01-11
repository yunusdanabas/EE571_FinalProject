# Part 2: Observer Design

## Objective

Design an observer using the augmented sensor matrix (measuring x1 and x6) and simulate the plant-observer system.

## Source Files

- Original implementation: `python/part2/run_observer_sim.py`
- Observer design: `python/part2/observer_design.py`
- Reference: `docs/00_anchor.md` (Part 2 C matrix and initial conditions)

## Key Tasks

1. Use Part 2 C matrix (2×12, measures x1 and x6)
2. Design observer using pole placement or LQR
3. Simulate plant-observer system with Part 2 initial conditions
4. Plot estimation errors

## Simplification Targets

- Simplify observer design code (remove excessive pole placement details)
- Streamline simulation loop
- Use utils functions for plotting
- Remove verbose validation

## Utils Functions to Check/Add

Check `final/utils/` for:
- `model.py`: Model functions (from Parts 0-1)
- `simulation.py`: Observer simulation function
- `plotting.py`: Error plotting, comparison plots

Add functions if missing.

## Expected Outputs

- Observer gain L
- Estimation error plots
- State estimation comparison

## Report Content

The `part2_report.md` should include:
- Objective: observer design with two sensors
- Approach: pole placement or LQR observer design
- Key results: observer gain L, convergence behavior
- Findings: observer successfully estimates states
