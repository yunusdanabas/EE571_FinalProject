---
name: Part 0 Workspace Setup
overview: Establish repository structure, create Python utilities for model discretization and simulation, verify numerical equivalence with MATLAB reference, and set up baseline open-loop simulation with dimension-aware plotting conventions.
todos:
  - id: setup_repo_structure
    content: "Create directory structure: python/utils/, python/part0/, docs/01_part0_workspace/, docs/sources/"
    status: completed
  - id: move_exam_pdf
    content: Move final_exam.pdf to docs/sources/final_exam.pdf and verify Part 2+ requirements
    status: completed
  - id: build_model_util
    content: Implement build_model.py with build_continuous_model() (extract A,B,C from prep_final.m) and discretize_zoh() (scipy.signal.cont2discrete with ZOH)
    status: completed
  - id: sim_util
    content: Implement sim.py with simulate_discrete() function for discrete-time simulation loop
    status: completed
  - id: plots_util
    content: Implement plots.py with dimension-aware plot_outputs() and plot_states() functions
    status: completed
  - id: metrics_util
    content: Implement metrics.py with check_dimensions() and placeholder functions for future parts
    status: completed
  - id: baseline_script
    content: "Create python/part0/baseline_check.py: load model, discretize, simulate open-loop, validate, plot"
    status: completed
  - id: matlab_reference
    content: Run matlab/prep_final.m and capture/export discrete matrices for comparison
    status: completed
  - id: validation
    content: "Run validation checklist: dimensions, numerical equivalence (rtol=1e-10, atol=1e-12), simulation sanity, plot sanity"
    status: completed
  - id: closeout_doc
    content: Create docs/01_part0_workspace/closeout.md with results summary and validation outcomes
    status: completed
---

# Part 0: Workspace Setup and Baseline Verification

## Overview

Part 0 establishes the foundation for the EE571 Final Project by creating reproducible model definitions, verification tools, and baseline simulations. This is infrastructure setup only—no control design work.

## Key Deliverables

### Python Utilities (`python/utils/`)

- **`build_model.py`**: Extract A, B, C from `matlab/prep_final.m`; implement ZOH discretization using `scipy.signal.cont2discrete()` to match MATLAB `c2d`
- **`sim.py`**: Discrete-time simulation loop `simulate_discrete(Ad, Bd, Cd, x0, u, N)` returning states, outputs, and time vector
- **`plots.py`**: Dimension-aware plotting utilities that match legend entries to actual `C` matrix dimensions (critical fix for `prep_final.m` legend mismatch)
- **`metrics.py`**: Initial dimension validation; placeholders for cost computation, input metrics, and estimation errors (to be extended in later parts)

### Baseline Verification (`python/part0/baseline_check.py`)

- Load continuous model, discretize with ZOH at `Ts=0.01`, simulate open-loop with `x0=[0,0,0,0,0,1,0,0,0,0,0,0]`, `N=1000`, zero input
- Validate dimensions and numerical equivalence with MATLAB reference
- Generate plots consistent with actual output dimensions

## Validation Framework

**Numerical Tolerance Policy:**

- Relative tolerance: `rtol = 1e-10`
- Absolute tolerance: `atol = 1e-12`
- Use `np.allclose()` for matrix comparisons (Ad, Bd, Cd)

**Key Checks:**

- Dimension validation (A: 12×12, B: 12×3, C: 1×12 for Part 0)
- Numerical equivalence against MATLAB `c2d` output
- Simulation sanity (bounded states, reasonable outputs)
- Plot sanity (legend entries match `size(C, 1)`)

## Critical Conventions

- **Plot Policy**: Legends must match actual `C` dimensions. For Part 0, `C` is 1×12, so plots show 1 output trace, not 6. Use `plot_states()` for displacement visualization if needed.
- **Known Issue**: `prep_final.m` line 77 has legend for 6 outputs but `C` defines 1 output. Repository code will avoid this inconsistency.
- **Random Seed**: `seed = 42` for all noisy simulations (to be used in Part 5+)

## Repository Structure

Matches `docs/00_anchor.md` conventions:

- `python/utils/` for shared utilities
- `python/part0/` for baseline verification
- `docs/01_part0_workspace/plan.md` and `closeout.md`

## TODOs

- Move `final_exam.pdf` to `docs/sources/final_exam.pdf`
- Verify Part 2+ requirements (C matrices, initial conditions, noise covariances) against exam PDF

## Non-Goals