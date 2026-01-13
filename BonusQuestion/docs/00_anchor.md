# Master Step-by-Step Plan: Vehicle Reference Tracking Project

This document defines the complete project workflow broken into 7 parts (Part 0 through Part 6). Each part has specific objectives, inputs, tasks, deliverables, and acceptance checks.

**Implementation Language: Python** (numpy, scipy, matplotlib)

---

## Closeout Protocol

**Every part agent must:**
1. Complete all tasks in their `partX_plan.md`
2. Fill out `partX_closeout.md` with:
   - Summary of work performed
   - Files changed/added
   - How to run/reproduce
   - Checks performed
   - Outputs produced (plots, logs, etc.)
   - Issues/TODOs for next part
3. Verify acceptance checks are met
4. Hand off to next part agent with clear status

---

## Part 0: Baseline Setup and Requirements Freeze

**Status: COMPLETED**

**Objective:** Establish stable repository structure and freeze interpretation of requirements.

**Deliverables:**
- Complete folder structure
- All planning documents
- Frozen requirements interpretation in `part0_closeout.md`

---

## Part 1: Model and Signals Review

**Status: COMPLETED**

**Objective:** Understand what is simulated and what signals are available for control and plotting.

**Deliverables:**
- Variable mapping table (code variable to mathematical meaning)
- Notes on signal computation locations
- Notes on where to add controller integration

---

## Part 2: Discretization of Error Model

**Status: COMPLETED**

**Objective:** Obtain the discrete-time error model `(Ad, Bd)` that both regulators will use.

**Deliverables:**
- Python implementation with discretization code
- `Ad` (5x5) and `Bd` (5x2) computed via ZOH
- Dimension verification

---

## Part 3: Regulator 1 Implementation (Discrete LQR)

**Objective:** Implement the discrete-time infinite-horizon LQR regulator using Python.

**Inputs:**
- `code/vehicle_tracking.py` (with discretization from Part 2)
- `docs/part2_discretization/part2_closeout.md`
- `Ad`, `Bd` matrices (from Part 2)

**Tasks:**
- [ ] Choose `Q` matrix (5x5, positive semi-definite)
- [ ] Choose `R` matrix (2x2, positive definite)
- [ ] Compute LQR gain using `scipy.linalg.solve_discrete_are`:
  ```python
  P = solve_discrete_are(Ad, Bd, Q, R)
  K_LQR = np.linalg.inv(R + Bd.T @ P @ Bd) @ (Bd.T @ P @ Ad)
  ```
- [ ] Verify `K_LQR` is 2x5
- [ ] In simulation loop:
  - [ ] Build `x_e = np.array([vy, r, ey, epsi, ev])`
  - [ ] Compute `u_reg = -K_LQR @ x_e`
  - [ ] Combine with feedforward
- [ ] Run baseline simulation (scale = 1)
- [ ] Generate and save plots

**Deliverables:**
- Modified Python code with LQR controller
- `docs/part3_regulator_lqr/part3_closeout.md` with Q, R values and rationale
- Baseline plot saved to `results/plots/`

**Acceptance Checks:**
- [ ] `Q` is 5x5, positive semi-definite
- [ ] `R` is 2x2, positive definite
- [ ] `K_LQR` is 2x5
- [ ] Code runs without errors
- [ ] Controller stabilizes error states
- [ ] Plots show reasonable tracking performance

---

## Part 4: Regulator 2 Implementation (Pole Placement)

**Objective:** Implement the discrete-time pole placement regulator with real poles only.

**Inputs:**
- `code/vehicle_tracking.py` (with LQR from Part 3)
- `docs/part3_regulator_lqr/part3_closeout.md`
- `Ad`, `Bd` matrices (from Part 2)

**Tasks:**
- [ ] Choose real poles inside unit circle (5 poles for 5 states)
  - **Constraint: Real poles only** (no complex poles)
- [ ] Compute gain using `scipy.signal.place_poles`:
  ```python
  result = place_poles(Ad, Bd, desired_poles)
  K_PP = result.gain_matrix
  ```
- [ ] Verify `K_PP` is 2x5
- [ ] Verify closed-loop eigenvalues match desired poles (all real)
- [ ] Use same integration pattern as LQR
- [ ] Run baseline simulation (scale = 1)
- [ ] Generate and save plots

**Deliverables:**
- Modified Python code with pole placement controller
- `docs/part4_regulator_poleplacement/part4_closeout.md` with poles documented
- Baseline plot saved to `results/plots/`

**Acceptance Checks:**
- [ ] All chosen poles are real (no complex poles)
- [ ] All chosen poles are inside unit circle (stable)
- [ ] `K_PP` is 2x5
- [ ] Closed-loop eigenvalues match desired poles
- [ ] Code runs without errors
- [ ] Plots show reasonable tracking performance

---

## Part 5: Experiments and Comparisons (6 Required Runs)

**Objective:** Run all required cases and generate comparison plots and metrics.

**Inputs:**
- `code/vehicle_tracking.py` (with both regulators)
- Closeouts from Part 3 and Part 4

**Tasks:**
- [ ] Implement run matrix: 2 regulators x 3 scales
- [ ] Implement initial condition scaling
- [ ] For each of 6 runs:
  - [ ] Run simulation
  - [ ] Compute metrics (RMS errors, max errors)
  - [ ] Generate plots
- [ ] Create comparison plots (trajectory, errors, inputs overlay)
- [ ] Save plots to `results/plots/`
- [ ] Create metrics summary table in `results/logs/`

**Deliverables:**
- All 6 runs executed
- Comparison plots saved to `results/plots/`
- Metrics table in `results/logs/metrics_summary.md`
- `docs/part5_experiments_comparison/part5_closeout.md`

**Acceptance Checks:**
- [ ] All 6 runs complete without errors
- [ ] All required plots are generated
- [ ] Metrics table is complete
- [ ] Plots clearly show comparison between regulators

---

## Part 6: Report Packaging and Submission Readiness

**Objective:** Assemble a clear PDF report and ensure code runs cleanly.

**Inputs:**
- All plots from Part 5
- Metrics from Part 5
- Final Python code

**Tasks:**
- [ ] Write report sections:
  - [ ] Problem statement and setup
  - [ ] Error model and discretization
  - [ ] Regulator 1 method (LQR)
  - [ ] Regulator 2 method (Pole placement)
  - [ ] Results: 6-case comparisons with plots
  - [ ] Conclusion
- [ ] Verify code runs from clean environment
- [ ] Create requirements.txt
- [ ] Add README with run instructions
- [ ] Generate PDF report

**Deliverables:**
- PDF report
- Clean code folder with requirements.txt
- `docs/part6_report_packaging/part6_closeout.md`

**Acceptance Checks:**
- [ ] PDF report contains all required sections
- [ ] All plots are included
- [ ] Code runs from clean Python environment
- [ ] README is present and clear

---

## Project Completion Checklist

- [x] Part 0: Baseline setup complete
- [x] Part 1: Model review complete
- [x] Part 2: Discretization complete
- [ ] Part 3: LQR regulator complete
- [ ] Part 4: Pole placement regulator complete
- [ ] Part 5: All 6 experiments run and compared
- [ ] Part 6: Report and code folder ready

---

## Notes

- **Python implementation**: Use numpy, scipy, matplotlib
- **Use same discretization for both regulators**: Part 2 creates `Ad`, `Bd` used by both Part 3 and Part 4
- **Real poles only**: Pole placement must use real poles (no complex poles allowed)
- **Fair comparison**: Same feedforward, same saturations, same initial conditions
