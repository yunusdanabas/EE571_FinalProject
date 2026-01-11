# Part 4: Reduced Input LQR

## Objective

Redesign the LQR controller with only two inputs (u1, u2), removing u3, and compare performance with Part 3.

## Source Files

- Original implementation: `python/part4/run_lqr_reduced_input.py`
- Reference: Part 3 for baseline comparison

## Key Tasks

1. Remove third column from B matrix (B_reduced)
2. Redesign LQR with reduced input matrix
3. Simulate closed-loop system
4. Compare cost and performance with Part 3

## Simplification Targets

- Simplify reduced input setup
- Remove redundant validation
- Use utils for comparison plots
- Streamline cost comparison

## Utils Functions to Check/Add

Check `final/utils/` for:
- `model.py`: Model functions
- `simulation.py`: Closed-loop simulation
- `plotting.py`: Comparison plots (overlay Part 3 vs Part 4)

Add comparison/utility functions if needed.

## Expected Outputs

- Reduced LQR gain K_red
- Closed-loop trajectories
- Cost comparison with Part 3
- Plots comparing Part 3 vs Part 4

## Report Content

The `part4_report.md` should include:
- Objective: LQR with reduced inputs
- Approach: remove u3, redesign controller
- Key results: cost comparison, performance degradation
- Findings: impact of removing one input on control performance
