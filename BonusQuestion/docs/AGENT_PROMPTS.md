# Agent Prompts for Vehicle Tracking Project

This file provides brief prompts for agents working on each part. Full details are in `docs/00_anchor.md`, `docs/01_project_explanation.md`, and individual `partX_plan.md` files.

**Implementation Language: Python** (numpy, scipy, matplotlib)

---

## General Instructions for All Agents

1. **Read first**: `docs/01_project_explanation.md` (project overview) and `docs/00_anchor.md` (master plan)
2. **Focus on your part**: Read `docs/partX_*/partX_plan.md` for detailed steps
3. **Review prior closeouts**: Check `docs/partY_*/partY_closeout.md` for parts before yours
4. **Complete closeout**: Fill `docs/partX_*/partX_closeout.md` before finishing

---

## Part 0 Agent Prompt (COMPLETED)

Baseline setup and requirements freeze completed.

---

## Part 1 Agent Prompt (COMPLETED)

Model and signals review completed.

---

## Part 2 Agent Prompt (COMPLETED)

Discretization of error model completed.

---

## Part 3 Agent Prompt

**Your task**: Implement discrete LQR regulator in Python.

**Read**: `docs/part2_discretization/part2_closeout.md`, `code/vehicle_tracking.py`

**Do**:
- Choose `Q` (5x5) and `R` (2x2) matrices
- Compute K_LQR using:
  ```python
  from scipy.linalg import solve_discrete_are
  P = solve_discrete_are(Ad, Bd, Q, R)
  K_LQR = np.linalg.inv(R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)
  ```
- In simulation loop: build `x_e = np.array([vy, r, ey, epsi, ev])`, compute `u_reg = -K_LQR @ x_e`, combine with feedforward
- Run baseline (scale=1), save plot
- Document in `docs/part3_regulator_lqr/part3_closeout.md`

**Deliver**: LQR working, baseline plot saved, Q/R values documented.

---

## Part 4 Agent Prompt

**Your task**: Implement discrete pole placement regulator (real poles only) in Python.

**Read**: `docs/part3_regulator_lqr/part3_closeout.md`, `code/vehicle_tracking.py`

**Do**:
- Choose real poles inside unit circle (5 poles for 5 states)
- Compute K_PP using:
  ```python
  from scipy.signal import place_poles
  result = place_poles(Ad, Bd, desired_poles)
  K_PP = result.gain_matrix
  ```
- Verify closed-loop eigenvalues are all real
- Use same integration pattern as LQR
- Run baseline (scale=1), save plot
- Document in `docs/part4_regulator_poleplacement/part4_closeout.md`

**Deliver**: Pole placement working, baseline plot saved, poles documented.

---

## Part 5 Agent Prompt

**Your task**: Run 6 experiments (2 regulators x 3 scales) and generate comparisons.

**Read**: `docs/part3_regulator_lqr/part3_closeout.md`, `docs/part4_regulator_poleplacement/part4_closeout.md`

**Do**:
- Implement initial condition scaling (scale offsets by 1x, 2x, 3x)
- Run all 6 cases: LQR/PP x scale 1/2/3
- Generate comparison plots (trajectory, errors, inputs)
- Compute metrics (RMS errors, max errors)
- Save plots to `results/plots/`, metrics to `results/logs/`
- Document in `docs/part5_experiments_comparison/part5_closeout.md`

**Deliver**: All 6 runs complete, comparison plots saved, metrics table created.

---

## Part 6 Agent Prompt

**Your task**: Assemble PDF report and ensure code runs cleanly.

**Read**: `docs/part5_experiments_comparison/part5_closeout.md`, all plots in `results/plots/`

**Do**:
- Write report sections: problem, error model, LQR method, pole placement method, results, conclusion
- Include all required plots
- Create requirements.txt
- Verify code runs from clean Python environment
- Add README with run instructions
- Generate PDF report
- Document in `docs/part6_report_packaging/part6_closeout.md`

**Deliver**: PDF report complete, code folder organized, submission ready.

---

## Key Constraints

- **Real poles only**: Part 4 must use real poles (no complex poles)
- **Same discretization**: Part 3 and Part 4 use same `Ad`, `Bd` from Part 2
- **Fair comparison**: Both regulators use same feedforward, saturations, initial conditions
- **6 runs required**: Part 5 must run all 2x3 combinations

---

## Python Key Functions

```python
# Discretization (already done in Part 2)
from scipy.linalg import expm
def c2d_zoh_exact(Ac, Bc, Ts):
    n, m = Ac.shape[0], Bc.shape[1]
    M = np.block([[Ac, Bc], [np.zeros((m, n)), np.zeros((m, m))]])
    Md = expm(M * Ts)
    return Md[:n, :n], Md[:n, n:n+m]

# LQR
from scipy.linalg import solve_discrete_are
P = solve_discrete_are(Ad, Bd, Q, R)
K_LQR = np.linalg.inv(R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)

# Pole Placement
from scipy.signal import place_poles
result = place_poles(Ad, Bd, desired_poles)
K_PP = result.gain_matrix

# Wrap angle to [-pi, pi]
def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi
```

---

## File Locations Reference

- Project overview: `docs/01_project_explanation.md`
- Master plan: `docs/00_anchor.md`
- Part plans: `docs/partX_*/partX_plan.md`
- Closeout templates: `docs/partX_*/partX_closeout.md`
- Python code: `code/vehicle_tracking.py`
- Results: `results/plots/`, `results/logs/`
