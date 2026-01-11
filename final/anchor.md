# Final Project - Anchor Document

## Purpose

This `final/` directory contains a cleaned-up, simplified version of the EE571 Final Project code. The goal is to:

1. **Simplify code** - Remove excessive validation, verbose comments, and complexity
2. **Improve readability** - Clean, minimal code with humanized comments
3. **Create documentation** - Each part has a `partX_report.md` file explaining the approach and results
4. **Shared utilities** - Common functions in `utils/` for plotting, modeling, and simulation

## Code Style Guidelines

1. **Minimal comments** - Only explain "why", not "what"
2. **Humanized tone** - Write comments as if explaining to a colleague, avoid robotic phrasing
3. **No excessive validation** - Trust the math, remove redundant checks
4. **Single main function** - Each Python file should have one clear `main()` function
5. **Import utils** - Use `from final.utils.plotting import ...` at the top

## Utils Files - Incremental Build

The `utils/` directory starts with **empty Python files**. Each agent working on a part should:

1. **Check existing functions** - Before writing a new function, check if something similar already exists
2. **Use parameterized functions** - Prefer one flexible function over multiple specific ones
   - Example: `plot_signals(t, signals, labels, title)` instead of separate `plot_outputs()`, `plot_inputs()`, `plot_errors()`
3. **Add at the end** - When adding a new function, append it to the end of the file (don't reorganize)
4. **Keep it generic** - Functions should be reusable across multiple parts
5. **Minimal docstrings** - One line is usually enough

**Utils file structure:**
- `plotting.py` - All plotting functions (trajectories, errors, comparisons)
- `model.py` - System model construction and discretization
- `simulation.py` - Common simulation routines

## Agent Workflow

When working on a part (e.g., `final/part3/`):

1. Read the `partX_plan.md` file in that part's directory
2. Read the original source code from `python/partX/`
3. **Check `final/utils/` files** - See what functions already exist
4. If a needed function is missing, add it to the appropriate utils file (at the end)
5. Create a simplified version in `final/partX/` using utils functions
6. Run the code to generate outputs (save to `outputs/` directory)
7. Create `partX_report.md` with:
   - Brief explanation of what the part does
   - Key approach/methodology
   - Results and findings
   - Figures and plots (reference files in `outputs/`)
8. **Never modify** the original `python/` files

## Report File Format

Each `partX_report.md` should include:

1. **Objective** - What this part accomplishes
2. **Approach** - Brief methodology (e.g., "Uses LQR design with DARE solver")
3. **Key Results** - Important numbers, gains, costs, etc.
4. **Figures** - Reference plots in `outputs/` directory
5. **Findings** - What was learned or observed

Keep reports concise but informative - they're meant to help with report writing.

## Part Dependencies

Reference the main project anchor at `docs/00_anchor.md` for:
- System model definitions (A, B, C matrices)
- Initial conditions for each part
- Noise parameters (Parts 5-7)
- Cost function definitions
- Array indexing conventions

## File Naming

- Part code: `baseline_check.py`, `observability.py`, `lqr_controller.py`, etc.
- Utils: `plotting.py`, `model.py`, `simulation.py`
- Plans: `partX_plan.md` (e.g., `part0_plan.md`, `part1_plan.md`, etc.)
- Reports: `partX_report.md` (e.g., `part0_report.md`, `part1_report.md`, etc.)

## Original Code Reference

The original implementations are in `python/partX/`. When simplifying:
- Keep the core algorithm/math the same
- Remove verbose validation and error checking
- Simplify plotting code (use utils functions)
- Reduce comments to essentials
- Don't change numerical results (only code structure)
