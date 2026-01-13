---
name: Baseline Setup Plan
overview: Create repository structure and planning documents for the EE571 vehicle tracking project without implementing any controllers. This sets up infrastructure for future agents to solve parts 0-6.
todos:
  - id: create-folders
    content: Create docs/, code/, results/ folder structure with all subfolders
    status: completed
  - id: write-explanation
    content: Write docs/01_project_explanation.md with requirements and constraints
    status: completed
  - id: write-anchor
    content: Write docs/00_anchor.md master plan with Parts 0-6 definitions
    status: completed
  - id: write-part-plans
    content: Write partX_plan.md for each of the 7 parts (Part 0-6)
    status: completed
  - id: write-closeout-templates
    content: Write partX_closeout.md templates for each part folder
    status: completed
  - id: create-code-skeleton
    content: Create code/run_all_cases.m skeleton with config struct, no controller
    status: completed
---

# Baseline Setup for Vehicle Reference Tracking Project

This plan creates the documentation infrastructure and folder structure for the EE571 Final Exam Bonus project. **No controller implementation or simulation will occur** - only planning documents and code skeletons.

---

## 1. Repository Structure Creation

Create the following folder hierarchy:

```
docs/
  part0_baseline/
  part1_model_review/
  part2_discretization/
  part3_regulator_lqr/
  part4_regulator_poleplacement/
  part5_experiments_comparison/
  part6_report_packaging/
code/
  utils/
results/
  plots/
  logs/
```

---

## 2. Project Explanation Document

Create [`docs/01_project_explanation.md`](docs/01_project_explanation.md) with:

- **Purpose**: Track a time-parameterized reference path using linearized error model control on nonlinear bicycle-model plant
- **Given Files**: `vehicle_dlqr.m` (starter code), `VehcileTracking_Document.md` (spec)
- **What to Implement**:
  - Discretize continuous error model `(Ac, Bc)` using ZOH
  - Regulator 1: Discrete LQR via `dlqr(Ad, Bd, Q, R)`
  - Regulator 2: Discrete pole placement with real poles only
- **Experiment Matrix**: 6 runs (2 regulators x 3 scales)
- **Deliverables**: Comparison plots, PDF report, runnable code
- **Constraints**: Saturation limits (steering +/-25 deg, accel [-6, 3] m/s^2), Ts=0.02s, Tend=25s

Key code locations to reference:

- Controller placeholder: lines 72-77 in [`vehicle_dlqr.m`](vehicle_dlqr.m)
- Input placeholder: lines 113-118
- Initial condition: line 87 (scale offsets by 2x, 3x for experiments)

---

## 3. Master Anchor Document

Create [`docs/00_anchor.md`](docs/00_anchor.md) defining Parts 0-6:

| Part | Objective | Key Deliverables |

|------|-----------|------------------|

| 0 | Baseline setup, requirements freeze | Frozen interpretation in closeout |

| 1 | Model/signals review | Variable mapping table |

| 2 | Discretization of error model | `Ad`, `Bd` matrices |

| 3 | LQR regulator implementation | `K_lqr`, baseline run |

| 4 | Pole placement implementation | `K_pp`, baseline run |

| 5 | 6-run experiments and comparison | Plots, metrics table |

| 6 | Report packaging | PDF, clean code folder |

Each part section includes: Objective, Inputs, Tasks (checkbox), Deliverables, Acceptance checks, Handoff notes.

---

## 4. Part Plan Documents

Create `partX_plan.md` for each part (7 files total):

- **part0_plan.md**: Read spec and code, confirm requirements, freeze interpretation
- **part1_plan.md**: Map code variables to math symbols, identify signals
- **part2_plan.md**: Use `c2d_zoh_exact(Ac, Bc, Ts)`, verify `Ad`, `Bd` dimensions
- **part3_plan.md**: Choose Q/R, compute `K_lqr`, integrate into loop
- **part4_plan.md**: Choose real poles inside unit circle, compute `K_pp`
- **part5_plan.md**: Run 6 cases, save plots to `results/plots/`, compute metrics
- **part6_plan.md**: Assemble PDF report, verify code runs clean

---

## 5. Closeout Templates

Create `partX_closeout.md` template in each part folder with headings:

```markdown
# Part X Closeout

## Summary of Work
(To be filled by executing agent)

## Files Changed/Added
- 

## How to Run / Reproduce
- 

## Checks Performed
- 

## Outputs Produced
- 

## Issues / TODOs for Next Part
- 
```

---

## 6. Code Skeleton

Create [`code/run_all_cases.m`](code/run_all_cases.m) with:

```matlab
%% run_all_cases.m - Skeleton for 6-run experiment matrix
% DO NOT SOLVE YET - Controller gains not implemented

% Define 6 run configurations
configs = struct('regulator', {}, 'scale', {});
configs(1) = struct('regulator', 'LQR', 'scale', 1);
configs(2) = struct('regulator', 'LQR', 'scale', 2);
configs(3) = struct('regulator', 'LQR', 'scale', 3);
configs(4) = struct('regulator', 'PP',  'scale', 1);
configs(5) = struct('regulator', 'PP',  'scale', 2);
configs(6) = struct('regulator', 'PP',  'scale', 3);

% TODO: Implement runner function calls
% TODO: Save plots to ../results/plots/
% TODO: Save metrics to ../results/logs/
```

Optionally copy `vehicle_dlqr.m` into `code/` as a working copy (original stays in root).

---

## Files to Create (Summary)

| File | Purpose |

|------|---------|

| `docs/00_anchor.md` | Master step-by-step plan with checkboxes |

| `docs/01_project_explanation.md` | Project overview and requirements |

| `docs/part0_baseline/part0_plan.md` | Plan for Part 0 |

| `docs/part0_baseline/part0_closeout.md` | Closeout template |

| `docs/part1_model_review/part1_plan.md` | Plan for Part 1 |

| `docs/part1_model_review/part1_closeout.md` | Closeout template |

| `docs/part2_discretization/part2_plan.md` | Plan for Part 2 |

| `docs/part2_discretization/part2_closeout.md` | Closeout template |

| `docs/part3_regulator_lqr/part3_plan.md` | Plan for Part 3 |

| `docs/part3_regulator_lqr/part3_closeout.md` | Closeout template |

| `docs/part4_regulator_poleplacement/part4_plan.md` | Plan for Part 4 |

| `docs/part4_regulator_poleplacement/part4_closeout.md` | Closeout template |

| `docs/part5_experiments_comparison/part5_plan.md` | Plan for Part 5 |

| `docs/part5_experiments_comparison/part5_closeout.md` | Closeout template |

| `docs/part6_report_packaging/part6_plan.md` | Plan for Part 6 |

| `docs/part6_report_packaging/part6_closeout.md` | Closeout template |

| `code/run_all_cases.m` | Skeleton for experiment runner |

| `code/utils/.gitkeep` | Placeholder for utilities |

| `results/plots/.gitkeep` | Placeholder for output plots |

| `results/logs/.gitkeep` | Placeholder for metrics logs |

---

## Stop Condition

After creating all folders and documents listed above, **STOP**. Do not proceed to implement any controller, discretization, or simulation.