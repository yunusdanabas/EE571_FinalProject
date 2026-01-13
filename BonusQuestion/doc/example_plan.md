# Step-by-step solution guide (high level)

This file explains what to do in each project part. It is intentionally not detailed. Each part should be executed by a separate agent using the corresponding `docs/partX_*/partX_plan.md`. After completing a part, the agent must write `docs/partX_*/partX_closeout.md` so the next agent can continue.

Repo inputs assumed:
- `VehcileTracking_Document.md` (primary spec)
- `vehicle_dlqr.m` (starter code)

Outputs expected by the end:
- Runnable code folder.
- A PDF report with required plots and comparisons.
- Results for 6 runs (2 regulators × 3 initial error scales).

---

## Part 0. Baseline setup and requirements freeze
**Goal:** Ensure the repo has a stable structure and an agreed interpretation of the requirements.

**Do:**
- Read `VehcileTracking_Document.md` and identify deliverables and constraints.
- Read `vehicle_dlqr.m` to locate:
  - the controller design placeholder
  - the “apply inputs” placeholder
  - the initial condition definition (for error scaling)
  - existing plotting code and available signals
- Confirm the required experiment matrix: LQR vs pole placement, each with scale 1, 2, 3.

**Produce:**
- Confirm folder structure exists.
- Update `docs/01_project_explanation.md` if needed.
- Fill `docs/part0_baseline/part0_closeout.md` with the frozen interpretation.

---

## Part 1. Model and signals review (no design yet)
**Goal:** Make sure you understand what is simulated and what signals are available for control and plotting.

**Do:**
- Identify plant state, input, reference signals, and error signals in `vehicle_dlqr.m`.
- Identify what the “error state vector” is in the code (the 5-state vector for control).
- Confirm feedforward terms are already present in the loop and how they are combined with regulation.

**Produce:**
- A short mapping table in `docs/part1_model_review/part1_closeout.md`:
  - “variable name in code” → “meaning”
- Notes on where to compute and log signals.

---

## Part 2. Discretization of the error model
**Goal:** Obtain the discrete-time error model that both regulators will use.

**Do:**
- Confirm continuous-time `Ac`, `Bc` dimensions and contents.
- Use the provided discretization helper (ZOH exact) to compute `Ad`, `Bd`.
- Add a small verification step that checks matrix sizes and basic sanity (no solving, just validation).

**Produce:**
- A minimal log or printed summary:
  - `Ts`
  - sizes of `Ad`, `Bd`
  - eigenvalues of `Ad` (optional but useful)
- Closeout with where `Ad`, `Bd` are computed and how they will be reused.

---

## Part 3. Regulator 1 implementation (discrete LQR)
**Goal:** Implement the LQR-based regulator and integrate it into the simulation loop.

**Do:**
- Choose `Q`, `R` (keep rationale simple, state penalties vs input effort).
- Compute `K_lqr` using `dlqr(Ad, Bd, Q, R)`.
- In the simulation loop:
  - build `x_e` each step
  - compute `u_reg = -K_lqr * x_e`
  - combine with existing feedforward terms
  - keep existing saturations
- Add consistent logging (errors, inputs, key states).

**Produce:**
- One runnable configuration for LQR.
- A saved plot set for a baseline run (scale 1) to confirm it executes.
- Closeout with:
  - where `Q`, `R`, `K_lqr` are defined
  - how to run LQR mode

---

## Part 4. Regulator 2 implementation (pole placement)
**Goal:** Implement the pole placement regulator and integrate it identically to LQR.

**Do:**
- Pick real, stable discrete-time pole locations (inside unit circle).
- Compute a feedback gain `K_pp` using an appropriate MATLAB method for state feedback.
- In the simulation loop:
  - same `x_e`
  - `u_reg = -K_pp * x_e`
  - same feedforward and saturations
- Ensure logging is identical so comparisons are fair.

**Produce:**
- One runnable configuration for pole placement.
- A baseline plot set (scale 1) to confirm it executes.
- Closeout with:
  - chosen poles (listed)
  - where `K_pp` is computed
  - how to run PP mode

---

## Part 5. Experiments and comparisons (6 required runs)
**Goal:** Run the required cases and generate comparison plots and metrics.

**Do:**
- Implement a “run matrix”:
  - Regulator: LQR, PP
  - Error scale: 1, 2, 3
- Modify initial conditions using a clean scaling approach:
  - scale only the intended initial offsets relative to reference.
- For each run, save:
  - trajectory plot (vehicle path vs reference)
  - time histories: `e_y`, `e_psi`, `e_v`
  - control inputs: steering and acceleration, with saturation visible
- Compute a small set of metrics per run:
  - RMS and max of `e_y`, `e_psi`, `e_v`
  - max |steering| and max/min acceleration
  - count or duration of saturation (optional)

**Produce:**
- All plots saved under `results/plots/` with consistent naming.
- A single summary table (CSV or Markdown) under `results/logs/`.
- Closeout listing the exact run commands and produced artifacts.

---

## Part 6. Report packaging and submission readiness
**Goal:** Assemble a clear report that matches the project deliverables.

**Do:**
- Write the report sections:
  - Problem and setup
  - Error model and discretization (brief)
  - Regulator 1 method (brief)
  - Regulator 2 method (brief)
  - Results: 6-case comparisons with plots
  - Conclusion: which regulator is more robust as initial error increases
- Ensure every required plot is present and labeled.
- Ensure the code runs from a clean MATLAB session.

**Produce:**
- Final PDF report.
- A clean code folder with instructions.
- Closeout confirming final deliverables and run steps.

---

## Handoff rules (for every part)
Each part’s agent must write `partX_closeout.md` including:
- What changed (files, functions).
- How to run what was added.
- What was verified.
- What artifacts were generated (plots, logs).
- Any known issues and TODOs for the next part.
