---
name: Final Project Cleanup
overview: Create a clean final version of the EE571 project with simplified code, shared utilities, and documentation for each part to support report writing.
todos:
  - id: create-structure
    content: Create final/ directory and part subdirectories
    status: pending
  - id: create-anchor
    content: Create anchor.md with master instructions
    status: pending
    dependencies:
      - create-structure
  - id: create-utils
    content: Create utils/ with empty plotting.py, model.py, simulation.py (agents add functions as needed)
    status: pending
    dependencies:
      - create-structure
  - id: create-plan-part0
    content: Create plan.md for Part 0 (baseline)
    status: pending
    dependencies:
      - create-anchor
  - id: create-plan-part1
    content: Create plan.md for Part 1 (observability)
    status: pending
    dependencies:
      - create-anchor
  - id: create-plan-part2
    content: Create plan.md for Part 2 (observer)
    status: pending
    dependencies:
      - create-anchor
  - id: create-plan-part3
    content: Create plan.md for Part 3 (LQR)
    status: pending
    dependencies:
      - create-anchor
  - id: create-plan-part4
    content: Create plan.md for Part 4 (reduced LQR)
    status: pending
    dependencies:
      - create-anchor
  - id: create-plan-part5
    content: Create plan.md for Part 5 (Kalman)
    status: pending
    dependencies:
      - create-anchor
  - id: create-plan-part6
    content: Create plan.md for Part 6 (LQG)
    status: pending
    dependencies:
      - create-anchor
  - id: create-plan-part7
    content: Create plan.md for Part 7 (sensors)
    status: pending
    dependencies:
      - create-anchor
---

# Final Project Organization and Simplification

## Overview

Create a new `final/` directory with clean, simplified code for all 8 parts (Part 0-7), shared utilities, and documentation files that can be used for report writing. Each part will have a plan file for dedicated Cursor agents.

## Folder Structure

```
final/
  anchor.md                  # Master document explaining simplification process
  utils/
    plotting.py              # Shared plotting functions
    model.py                 # System model (matrices, discretization)
    simulation.py            # Common simulation functions
  part0/
    plan.md                  # Instructions for agent
    baseline_check.py        # Simplified code
    report.md                # Results + explanation for report
    outputs/                 # Figures
  part1/
    plan.md
    observability.py
    report.md
    outputs/
  part2/
    plan.md
    observer_design.py
    report.md
    outputs/
  part3/
    plan.md
    lqr_controller.py
    report.md
    outputs/
  part4/
    plan.md
    lqr_reduced.py
    report.md
    outputs/
  part5/
    plan.md
    kalman_filter.py
    report.md
    outputs/
  part6/
    plan.md
    lqg_controller.py
    report.md
    outputs/
  part7/
    plan.md
    sensor_augmentation.py
    report.md
    outputs/
```

## Key Files to Create Now

### 1. `final/anchor.md` - Master Anchor Document

Explains:

- Purpose of final version (simplified, documented code)
- How agents should approach each part
- Code style guidelines (minimal humanized comments)
- Utils usage instructions
- Report file format

### 2. `final/utils/` - Shared Utilities (Incremental Build)

Start with **empty Python files** - agents add functions as they work on their parts:

- **plotting.py**: Empty initially. Agents add plotting functions as needed.
- **model.py**: Empty initially. Agents add model/discretization functions as needed.
- **simulation.py**: Empty initially. Agents add simulation functions as needed.

**Incremental Build Rules:**

1. Before writing a new function, check if a similar one already exists in utils
2. Use parameterized functions (e.g., one `plot_trajectory()` with options, not separate functions for each plot type)
3. Add new functions at the end of the file
4. Keep functions generic and reusable across parts
5. Reference existing [python/utils/](python/utils/) for inspiration but simplify

### 3. Part Plan Files (8 total)

Each `plan.md` includes:

- Brief objective
- Source files to reference in `python/partX/`
- Key simplification targets
- What the `report.md` should contain

## Part Summary Table

| Part | Objective | Source File | Key Output |

|------|-----------|-------------|------------|

| 0 | Baseline verification | [python/part0/baseline_check.py](python/part0/baseline_check.py) | Discretization + plots |

| 1 | Observability analysis | [python/part1/run_observability.py](python/part1/run_observability.py) | Rank, Kalman decomposition |

| 2 | Observer design | [python/part2/run_observer_sim.py](python/part2/run_observer_sim.py) | Observer gain L, error plots |

| 3 | LQR controller | [python/part3/run_lqr_with_observer.py](python/part3/run_lqr_with_observer.py) | Gain K, cost J, inputs |

| 4 | Reduced input LQR | [python/part4/run_lqr_reduced_input.py](python/part4/run_lqr_reduced_input.py) | Cost comparison |

| 5 | Kalman filter | [python/part5/run_kalman_filter.py](python/part5/run_kalman_filter.py) | Gain Lk, estimation RMS |

| 6 | LQG controller | [python/part6/run_lqg.py](python/part6/run_lqg.py) | Noisy comparison |

| 7 | Sensor augmentation | [python/part7/run_part7.py](python/part7/run_part7.py) | 4-sensor vs 6-sensor analysis |

## Code Style Guidelines

1. **Minimal comments** - only explain "why", not "what"
2. **Humanized tone** - avoid robotic phrasing
3. **No excessive validation gates** - trust the math
4. **Single main function per file** - easy to follow
5. **Import utils at top** - `from final.utils.plotting import ...`

## Agent Workflow

For each part, the assigned agent should:

1. Read the `plan.md` in that part folder
2. Read original source code from `python/partX/`
3. **Check `final/utils/` files** - see what functions already exist
4. If a needed function is missing, add it to the appropriate utils file (at the end)
5. Create simplified version in `final/partX/` using utils functions
6. Run the code to generate outputs
7. Create `report.md` with results and explanations
8. Never modify original `python/` files

## Utils File Guidelines

**Before adding a new function to utils:**

- Check if an existing function can handle your case with parameters
- Prefer one flexible function over multiple specific ones
- Example: `plot_signals(t, signals, labels, title)` instead of separate `plot_outputs()`, `plot_inputs()`, `plot_errors()`

**When adding a new function:**

- Add at the end of the file (don't reorganize existing code)
- Use clear parameter names
- Keep docstrings minimal (one line if possible)
- Make it generic enough for other parts to reuse

**Utils file structure:**

```python
# Each utils file grows incrementally:
# - Part 0 agent adds first functions
# - Part 1 agent checks, adds if needed
# - Part 2 agent checks, adds if needed
# - ... and so on
```

## What I Will Create Now

1. Create `final/` directory structure (all folders)
2. Create `final/anchor.md` with full instructions
3. Create **empty** `final/utils/*.py` files (agents will populate incrementally)
4. Create `plan.md` for each part (Part 0-7) with utils instructions