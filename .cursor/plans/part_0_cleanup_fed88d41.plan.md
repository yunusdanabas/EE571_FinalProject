---
name: Part 0 Cleanup
overview: Create simplified Part 0 baseline verification code in final/part0/, populate utils with needed functions, generate outputs, and write the report.
todos:
  - id: utils-model
    content: Add build_continuous_model() and discretize_zoh() to final/utils/model.py
    status: completed
  - id: utils-sim
    content: Add simulate_discrete() to final/utils/simulation.py
    status: completed
  - id: utils-plot
    content: Add plot_signals() to final/utils/plotting.py
    status: completed
  - id: baseline-code
    content: Create simplified final/part0/baseline_check.py
    status: completed
    dependencies:
      - utils-model
      - utils-sim
      - utils-plot
  - id: run-code
    content: Run baseline_check.py and generate outputs in final/part0/outputs/
    status: completed
    dependencies:
      - baseline-code
  - id: write-report
    content: Create final/part0/part0_report.md with results
    status: completed
    dependencies:
      - run-code
---

# Part 0: Baseline Verification Cleanup

## Overview

Simplify the baseline verification code from `python/part0/baseline_check.py`, add needed utility functions to `final/utils/`, and generate outputs with documentation.

## Key Files

| Purpose | File |

|---------|------|

| Original code | [`python/part0/baseline_check.py`](python/part0/baseline_check.py) |

| Original utils | [`python/utils/build_model.py`](python/utils/build_model.py), [`sim.py`](python/utils/sim.py), [`plots.py`](python/utils/plots.py) |

| Target code | [`final/part0/baseline_check.py`](final/part0/baseline_check.py) |

| Target utils | [`final/utils/model.py`](final/utils/model.py), [`simulation.py`](final/utils/simulation.py), [`plotting.py`](final/utils/plotting.py) |

## Implementation Steps

### 1. Populate `final/utils/model.py`

Add simplified versions of:

- `build_continuous_model()` - returns A, B, C matrices (hardcoded from prep_final.m)
- `discretize_zoh(A, B, C, Ts)` - uses scipy's cont2discrete with ZOH method

Keep minimal docstrings (one line each).

### 2. Populate `final/utils/simulation.py`

Add:

- `simulate_discrete(Ad, Bd, Cd, x0, u, N, Ts)` - discrete-time simulation loop

Returns state trajectory x, output y, and time vector t.

### 3. Populate `final/utils/plotting.py`

Add a generic parameterized function:

- `plot_signals(t, signals, labels, title, ylabel, save_path=None)` - plots multiple signals with legend

This replaces the separate `plot_outputs()` and `plot_displacements()` functions. Per anchor guidelines, prefer one flexible function over multiple specific ones.

### 4. Create `final/part0/baseline_check.py`

Simplified structure:

```python
import numpy as np
from final.utils.model import build_continuous_model, discretize_zoh
from final.utils.simulation import simulate_discrete
from final.utils.plotting import plot_signals

def main():
    # Build and discretize model
    # Simulate open-loop with zero input
    # Generate two plots: output_plot.png, displacements_plot.png
    # Print minimal summary

if __name__ == '__main__':
    main()
```

Key simplifications:

- Remove dimension validation (trust the math)
- Remove verbose shape printing
- Remove NaN/Inf checks
- Keep only essential summary output
- Use parameterized `plot_signals()` for both plots

### 5. Run and Generate Outputs

Execute the script to generate:

- `final/part0/outputs/output_plot.png` - single output y = Cx
- `final/part0/outputs/displacements_plot.png` - all 6 mass displacements

### 6. Create `final/part0/part0_report.md`

Include:

- **Objective**: Verify discretized model baseline
- **Approach**: Build continuous model, discretize with ZOH at Ts=0.01
- **Key Results**: Matrix dimensions (A: 12x12, B: 12x3, C: 1x12)
- **Figures**: Reference output_plot.png and displacements_plot.png
- **Findings**: System is marginally stable (oscillatory response)