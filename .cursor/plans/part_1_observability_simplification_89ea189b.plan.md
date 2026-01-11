---
name: Part 1 Observability Simplification
overview: Simplify the Part 1 observability analysis code by merging three separate Python files into one clean script, removing verbose validation, and using existing utils functions for model building.
todos:
  - id: create-observability-py
    content: Create simplified observability.py with inline observability matrix and Kalman decomposition
    status: completed
  - id: run-code
    content: Run observability.py to generate outputs/observability_results.txt
    status: completed
    dependencies:
      - create-observability-py
  - id: create-report
    content: Create part1_report.md with objective, approach, key results, and findings
    status: completed
    dependencies:
      - run-code
---

# Part 1: Observability Analysis Simplification

## Summary

The original Part 1 implementation spans 3 Python files (~490 lines total) with extensive validation, formatting helpers, and verbose outputs. The simplified version will be a single clean script (~80-100 lines) that produces the same numerical results.

## Key Files

**Source (read-only):**

- [`python/part1/run_observability.py`](python/part1/run_observability.py) - main runner with output formatting
- [`python/part1/observability_rank.py`](python/part1/observability_rank.py) - observability matrix and rank
- [`python/part1/kalman_decomp_obsv.py`](python/part1/kalman_decomp_obsv.py) - Kalman decomposition

**Destination:**

- [`final/part1/observability.py`](final/part1/observability.py) - simplified implementation
- [`final/part1/part1_report.md`](final/part1/part1_report.md) - analysis report
- `final/part1/outputs/observability_results.txt` - analysis results

## Utils Check

Existing functions in [`final/utils/model.py`](final/utils/model.py):

- `build_continuous_model()` - builds A, B, C matrices
- `discretize_zoh()` - discretizes with ZOH

No new utils needed - observability analysis is specialized to this part.

## Simplification Targets

| Original | Simplified |

|----------|------------|

| 3 separate Python files | 1 file with inline functions |

| `format_eigenvalues()` helper | Simple inline formatting |

| 6 output files (matrices, eigenvalues) | 1 summary text file |

| Extensive dimension/reconstruction checks | Removed (trust the math) |

| Verbose SVD singular value output | Removed |

| Checkmark symbols in output | Removed |

## Code Structure for `observability.py`

```python
def build_observability_matrix(Ad, Cd):
    # Construct O = [C; CA; CA^2; ...; CA^(n-1)]

def kalman_decomposition(Ad, Cd):
    # Compute rank, basis, and transformed system

def main():
    # 1. Build and discretize system (reuse model.py)
    # 2. Compute observability matrix and rank
    # 3. Perform Kalman decomposition
    # 4. Save results to outputs/observability_results.txt
    # 5. Print summary
```

## Expected Output File Content

```
Part 1: Observability Analysis
==============================
System dimension: 12
Observability rank: 6 / 12
Observable subspace: 6 states
Unobservable subspace: 6 states

Observable eigenvalues: [list of 6]
Unobservable eigenvalues: [list of 6]
```

## Report Content for `part1_report.md`

- **Objective:** Analyze observability with single sensor (mass 1 displacement)
- **Approach:** Observability matrix rank + Kalman decomposition
- **Key Results:** Rank = 6, meaning 6 observable and 6 unobservable states
- **Findings:** Single sensor observes only half the system (one from each pair of symmetric/antisymmetric modes)